/**
 *******************************************************************************
 * @file  foc_obs.c
 * @brief FOC 观察模块实现 — 观察变量定义 + 状态历史 + 主循环打印/事件处理。
 *
 *        本文件代码全部从 foc_iq_pi.c / foc_lock_iq_pi.c / main.c 原样
 *        迁移而来（变量名、打印格式、处理逻辑均未改动）：
 *          - g_iqpi_*  观察量与状态历史   <- foc_iq_pi.c
 *          - g_lockiq_* 观察量与事件快照  <- foc_lock_iq_pi.c
 *          - Foc_Obs_Task() 六段打印/事件 <- main.c 主循环
 *
 *        模块说明见 foc_obs.h 文件头。
 *******************************************************************************
 */

#include "foc_obs.h"
#include "foc_core.h"
#include "foc_zizeng.h"
#include "foc_lock_iq_pi.h"
#include "foc_align.h"
#include "foc_cal.h"
#include "foc_cal_angle.h"
#include "foc_olf.h"
#include "foc_dcl.h"
#include "dev_comm_runner.h"   /* CommRunner_SetMode（mode 20 完成自动回 mode 0） */
#include "encoder.h"
#include "motor_config.h"
#include "rtt_log.h"
#include "TickTimer.h"

/*******************************************************************************
 * mode 31 观察量定义（原 foc_iq_pi.c）
 ******************************************************************************/
volatile iqpi_step_t g_iqpi_step_hist[IQPI_HISTORY_LEN];   /* 状态历史, [0]最旧 */
volatile uint8_t g_iqpi_step_hist_cnt  = 0;                /* 历史有效条数 0..10 */
volatile uint8_t g_iqpi_flip_cnt       = 0;                /* 框架180°自动翻转次数 */

/* --- 转向诊断观测量（ISR 更新，Foc_Obs_Task 周期打印 / Watch 直接看） --- */
volatile int32_t g_iqpi_enc_pos        = 0;   /* ISR 内累积的编码器计数镜像 */
volatile int32_t g_iqpi_win_moved      = 0;   /* 最近一次完成的500ms窗口位移(counts,带符号) */
volatile uint32_t g_iqpi_win_evals     = 0;   /* 已完成的窗口评估次数(0=方向检查从未运行) */
volatile int8_t  g_iqpi_cur_dir        = 0;   /* 最近窗口实测方向 +1/-1 */
volatile int8_t  g_iqpi_expect_dir     = 0;   /* 预期方向(+1/-1) */
volatile int8_t  g_iqpi_ref_dir        = 0;   /* 启动时捕获的 mode30 拖动方向基准 */

/* 翻转事件快照（ISR 置 flag，Foc_Obs_Task 打印后清 flag） */
volatile uint8_t g_iqpi_evt_flag       = 0;
volatile uint8_t g_iqpi_evt_seq        = 0;   /* 第几次翻转 (1,2,...) */
volatile int32_t g_iqpi_evt_pos        = 0;   /* 翻转时编码器计数 */
volatile int32_t g_iqpi_evt_iq_ma      = 0;   /* 翻转时 iq (mA) */
volatile int32_t g_iqpi_evt_vq_mv      = 0;   /* 翻转时 vq (mV) */
volatile int32_t g_iqpi_evt_off_deg    = 0;   /* 翻转后偏移基线 (deg) */

/*******************************************************************************
 * mode 32 观察量定义（原 foc_lock_iq_pi.c）
 ******************************************************************************/
volatile int32_t g_lockiq_win_moved      = 0; /* 最近完成窗口的平均位置位移 (counts) */
volatile uint32_t g_lockiq_win_evals     = 0; /* 已完成窗口评估次数 */
volatile int32_t g_lockiq_track_err_cnts = 0; /* VERIFY 跟踪误差 (counts) */

/* 锁定/失败事件快照（ISR 置 flag，Foc_Obs_Task 处理后清 flag） */
volatile uint8_t g_lockiq_evt_flag     = 0;
volatile uint8_t g_lockiq_evt_code     = 0;
volatile int32_t g_lockiq_evt_off_deg  = 0;   /* 锁定的注入偏移 (deg) */

/*******************************************************************************
 * Foc_Obs_IqpiHistClear - 清空 mode 31 状态历史（新的一次运行重新记录）
 ******************************************************************************/
void Foc_Obs_IqpiHistClear(void)
{
    uint8_t i;

    for (i = 0; i < IQPI_HISTORY_LEN; i++) {
        g_iqpi_step_hist[i] = IQPI_STEP_IDLE;
    }
    g_iqpi_step_hist_cnt = 0;
}

/*******************************************************************************
 * Foc_Obs_IqpiRecordStep - 记录一条状态历史
 *   hist[0] 最旧、hist[cnt-1] 最新；记满后挤掉最旧一条（滑动窗口）
 ******************************************************************************/
void Foc_Obs_IqpiRecordStep(iqpi_step_t s)
{
    uint8_t i;

    if (g_iqpi_step_hist_cnt < IQPI_HISTORY_LEN) {
        g_iqpi_step_hist[g_iqpi_step_hist_cnt] = s;
        g_iqpi_step_hist_cnt++;
    } else {
        for (i = 1; i < IQPI_HISTORY_LEN; i++) {
            g_iqpi_step_hist[i - 1] = g_iqpi_step_hist[i];
        }
        g_iqpi_step_hist[IQPI_HISTORY_LEN - 1u] = s;
    }
}

/*******************************************************************************
 * Foc_Obs_Task - 观察任务（主循环每圈一次；ISR 内禁止调用）
 *   以下六段均自 main.c 原样迁移，打印格式与处理逻辑未改动。
 ******************************************************************************/
void Foc_Obs_Task(void)
{
#if MOTOR_FOC_ENABLE
    /* ---- 角度实时观测（读 TMRA_1 原始计数 + 扣 offset） ----
     * mode 0 下维持 g_foc_if_rotor_rad / g_foc_mech_rad 实时更新；
     * 活跃模式下 ISR 会覆盖 g_foc_if_rotor_rad，此写入无害。 */
    Foc_Core_UpdateAngleObs();

    /* ---- FOC 故障打印 ---- */
    {
        static uint8_t s_foc_fault_printed = 0;

        if (g_foc_fault != 0u) {
            if (!s_foc_fault_printed) {
                s_foc_fault_printed = 1;
                OBS_DBG("[FOC] FAULT oc=%d stage=%d i=%d mA",
                        (int)g_foc_fault, (int)g_foc_fault_stage, (int)g_foc_fault_i_ma);
            }
        } else {
            s_foc_fault_printed = 0;
        }
    }

    /* ---- mode 20 校准事件（ISR 置 evt，此处打印；完成/过流自动回 mode 0） ---- */
    if (g_cal_evt != 0u) {
        uint8_t evt = g_cal_evt;
        g_cal_evt = 0u;
        switch (evt) {
        case CAL_EVT_BETA_DONE:
            CAL_DBG("BETA done hw=%d -> ALPHA 0deg", (int)g_cal_beta_hw);
            break;
        case CAL_EVT_LOCKED:
            CAL_DBG("ALPHA done hw=%d moved=%d offset=%d cnts (%d deg)",
                    (int)g_cal_alpha_hw, (int)g_cal_moved, (int)g_cal_offset,
                    (int)((g_cal_offset * 360) / (int32_t)ENCODER_CPR));
            CommRunner_SetMode(COMM_RUNNER_STOP);   /* 校准完成自动回 mode 0 */
            break;
        case CAL_EVT_OC:
            CAL_DBG("FAULT_OC i=%d mA", (int)g_foc_fault_i_ma);
            CommRunner_SetMode(COMM_RUNNER_STOP);
            break;
        default:
            break;
        }
    }

    /* ---- mode 25 手动角度吸附事件（ISR 置 evt，此处打印；仅 OC 自动回 mode 0，
     *      LOCKED / DONE 均保持在 mode 25 等下一次输入） ---- */
    if (g_calang_evt != 0u) {
        uint8_t evt = g_calang_evt;
        g_calang_evt = 0u;
        switch (evt) {
        case CALANG_EVT_BETA_DONE:
            CALANG_DBG("BETA done -> ALPHA 0deg");
            break;
        case CALANG_EVT_LOCKED:
            CALANG_DBG("LOCKED offset=%d deg",
                    (int)((g_calang_offset * 360) / (int32_t)ENCODER_CPR));
            break;
        case CALANG_EVT_DONE_OK:
            CALANG_DBG("HOLD ok target=%d meas=%d err=%d x0.1deg",
                    (int)g_calang_target_deg, (int)g_calang_meas_deg,
                    (int)g_calang_err_deg);
            break;
        case CALANG_EVT_DONE_FAIL:
            CALANG_DBG("HOLD FAIL moved=%d cnts target=%d meas=%d x0.1deg",
                    (int)g_calang_win_moved, (int)g_calang_target_deg,
                    (int)g_calang_meas_deg);
            break;
        case CALANG_EVT_OC:
            CALANG_DBG("FAULT_OC i=%d mA", (int)g_foc_fault_i_ma);
            CommRunner_SetMode(COMM_RUNNER_STOP);
            break;
        default:
            break;
        }
    }

    /* ---- mode 26 开环 VF 事件（ISR 置 evt，此处打印；仅 OC 自动回 mode 0） ---- */
    if (g_olf_evt != 0u) {
        uint8_t evt = g_olf_evt;
        g_olf_evt = 0u;
        switch (evt) {
        case OLF_EVT_BETA_DONE:
            OLF_DBG("BETA done -> ALPHA 0deg");
            break;
        case OLF_EVT_LOCKED:
            OLF_DBG("LOCKED offset=%d deg -> drag f=%d mHz",
                    (int)((g_olf_offset * 360) / (int32_t)ENCODER_CPR),
                    (int)(g_olf_freq_hz * 1000.0f));
            break;
        case OLF_EVT_OC:
            OLF_DBG("FAULT_OC i=%d mA", (int)g_foc_fault_i_ma);
            CommRunner_SetMode(COMM_RUNNER_STOP);
            break;
        default:
            break;
        }
    }

    /* ---- mode 26 拖动实验数据（200ms 周期，全整型；diff=负载角 delta） ---- */
    if (g_olf_running && (g_olf_state == OLF_STEP_DRAG)) {
        static uint32_t s_last_olf_dbg = 0u;
        uint32_t now = tickTimer_GetCount();
        if ((now - s_last_olf_dbg) >= 200u) {
            s_last_olf_dbg = now;
            OLF_DBG("f=%d mHz fld=%d deg rot=%d deg diff=%d deg id=%d iq=%d mA",
                    (int)(g_olf_freq_hz * 1000.0f),   /* 磁场转速 (mHz) */
                    (int)g_olf_field_deg,             /* 磁场电角度 (deg, 0~359) */
                    (int)g_olf_rotor_deg,             /* 转子电角度 (deg, 0~359) */
                    (int)g_olf_diff_deg,              /* 负载角 delta (deg, -180~180) */
                    (int)g_olf_id_ma,                 /* 真实转子系 id (mA) */
                    (int)g_olf_iq_ma);                /* 真实转子系 iq (mA) */
        }
    }

    /* ---- mode 27 功角闭环事件（ISR 置 evt，此处打印；仅 OC 自动回 mode 0） ---- */
    if (g_dcl_evt != 0u) {
        uint8_t evt = g_dcl_evt;
        g_dcl_evt = 0u;
        switch (evt) {
        case DCL_EVT_BETA_DONE:
            DCL_DBG("BETA done -> ALPHA 0deg");
            break;
        case DCL_EVT_LOCKED:
            DCL_DBG("LOCKED off=%d deg -> run dlt %d->%d deg tr=%d ms",
                    (int)((g_dcl_offset * 360) / (int32_t)ENCODER_CPR),
                    (int)g_dcl_dlt_init_deg, (int)g_dcl_dlt_targ_deg,
                    (int)g_dcl_dlt_tr_ms);
            break;
        case DCL_EVT_RAMP_DONE:
            DCL_DBG("ramp done dlt=%d deg spd=%d mHz",
                    (int)g_dcl_dlt_now_deg, (int)(g_dcl_speed_hz * 1000.0f));
            break;
        case DCL_EVT_OC:
            DCL_DBG("FAULT_OC i=%d mA", (int)g_foc_fault_i_ma);
            CommRunner_SetMode(COMM_RUNNER_STOP);
            break;
        default:
            break;
        }
    }

    /* ---- mode 27 功角闭环运行数据（200ms 周期，全整型） ---- */
    if (g_dcl_running && (g_dcl_state == DCL_STEP_RUN)) {
        static uint32_t s_last_dcl_dbg = 0u;
        uint32_t now = tickTimer_GetCount();
        if ((now - s_last_dcl_dbg) >= 200u) {
            s_last_dcl_dbg = now;
            DCL_DBG("dlt=%d deg spd=%d mHz fld=%d deg rot=%d deg diff=%d deg id=%d iq=%d mA",
                    (int)g_dcl_dlt_now_deg,           /* 当前 delta 指令 (deg) */
                    (int)(g_dcl_speed_hz * 1000.0f),  /* 实测电频率 (mHz, 带符号) */
                    (int)g_dcl_field_deg,             /* 磁场电角度 (deg, 0~359) */
                    (int)g_dcl_rotor_deg,             /* 转子电角度 (deg, 0~359) */
                    (int)g_dcl_diff_deg,              /* 功角实测 (deg, 应≈dlt) */
                    (int)g_dcl_id_ma,                 /* 真实转子系 id (mA) */
                    (int)g_dcl_iq_ma);                /* 真实转子系 iq (mA) */
        }
    }

    /* ---- 对齐校准事件打印 ---- */
    if (g_foc_align_evt != 0u) {
        uint8_t evt = g_foc_align_evt;
        g_foc_align_evt = 0u;
        switch (evt) {
        case 1u:
            OBS_DBG("[ALIGN] start volt=%d mV", (int)g_foc_align_evt_v1);
            break;
        case 2u:
            OBS_DBG("[ALIGN] beta done -> alpha");
            break;
        case 3u:
            OBS_DBG("[ALIGN] locked offset=%d id=%d iq=%d",
                    (int)g_foc_align_evt_v1, (int)g_foc_align_evt_v2,
                    (int)g_foc_align_evt_v3);
            break;
        case 4u:
            OBS_DBG("[ALIGN] done offset=%d", (int)g_foc_align_evt_v1);
            break;
        case 5u:
            OBS_DBG("[ALIGN] FAULT code=%d i=%d mA",
                    (int)g_foc_align_evt_v1, (int)g_foc_align_evt_v2);
            break;
        default:
            break;
        }
    }

    /* ---- ZIZENG 状态打印（200ms 高频调试） ---- */
    if (g_zizeng_running) {
        static uint32_t s_last_zz_dbg = 0u;
        uint32_t now = tickTimer_GetCount();
        if ((now - s_last_zz_dbg) >= 200u) {
            s_last_zz_dbg = now;
            OBS_DBG("[ZIZENG_DBG] cnt=%d rpm=%d enc_dir=%d theta=%d deg rotor=%d deg diff=%d deg",
                    (int)g_enc_count,                     /* 编码器累积计数(counts,4倍频,带符号) */
                    (int)g_enc_speed_rpm,                 /* 机械转速估算(RPM) */
                    (int)g_foc_enc_dir,                   /* 编码器方向符号(+1/-1) */
                    (int)(g_foc_theta_rad * 57.2958f),    /* 磁场角theta(deg,拖动角,递增) */
                    (int)(g_foc_if_rotor_rad * 57.2958f), /* 转子电角度(deg,已扣mode30偏移,0-360) */
                    (int)(g_foc_if_diff_rad * 57.2958f)); /* rotor-theta(deg): 锁定后应≈0 */
        }
    }

    /* ---- mode 31 运行监视（200ms 节流，全部整型缩放） ---- */
    if (g_iqpi_running) {
        static uint32_t s_last_iqpi_dbg = 0u;
        uint32_t now = tickTimer_GetCount();
        if ((now - s_last_iqpi_dbg) >= 200u) {
            s_last_iqpi_dbg = now;
            OBS_DBG("[IQPI_MON] st=%d iq=%d id=%d vq=%d vd=%d rr=%d win=%d ev=%d cd=%d ed=%d rd=%d flip=%d pos=%d ",
                    (int)g_iqpi_step,                   /* 状态: 2=零矢量校准 3=闭环 4=vq饱和 5=堵转 6=过流 7=已翻转 */
                    (int)g_foc_iq_ma, (int)g_foc_id_ma, /* 控制系电流 iq/id (mA), 应跟随 rr/0 */
                    (int)(g_foc_vq * 1000.0f),          /* q轴电压指令(mV), ±3500=±限幅(顶格=饱和) */
                    (int)(g_foc_vd * 1000.0f),          /* d轴电压指令(mV) */
                    (int)g_iqpi_iq_ref_ramp_ma,         /* 斜坡后的iq参考 rr (mA) */
                    (int)g_iqpi_win_moved,              /* 最近完成的500ms窗口位移(counts,带符号,0=尚无) */
                    (int)g_iqpi_win_evals,              /* 已完成方向评估次数(0=方向检查从未运行) */
                    (int)g_iqpi_cur_dir,                /* 最近窗口实测转向(+1/-1) */
                    (int)g_iqpi_expect_dir,             /* 预期转向(由rd和rr符号决定) */
                    (int)g_iqpi_ref_dir,                /* mode30记录的拖动方向基准(+1/-1,0=未测到) */
                    (int)g_iqpi_flip_cnt,               /* 本次运行180°框架翻转次数 */
                    (int)g_iqpi_enc_pos);               /* ISR累积编码器计数(看转向和速率) */
        }
    }

    /* ---- mode 31 翻转事件（ISR 置 flag，此处打印一次后清零） ---- */
    if (g_iqpi_evt_flag) {
        g_iqpi_evt_flag = 0u;
        OBS_DBG("[IQPI_FLIP] n=%d pos=%d iq=%d vq=%d off=%d deg cd=%d ed=%d rd=%d",
                (int)g_iqpi_evt_seq, (int)g_iqpi_evt_pos,
                (int)g_iqpi_evt_iq_ma, (int)g_iqpi_evt_vq_mv,
                (int)g_iqpi_evt_off_deg,
                (int)g_iqpi_cur_dir, (int)g_iqpi_expect_dir,
                (int)g_iqpi_ref_dir);
    }

    /* ---- mode 32 锁定/失败事件（ISR 置 flag，此处打印并交接/停机） ---- */
    if (g_lockiq_evt_flag) {
        uint8_t lock_code = g_lockiq_evt_code;
        g_lockiq_evt_flag = 0u;
        if ((lock_code == LOCKIQ_EVT_LOCKED) && g_lockiq_running) {
            OBS_DBG("[LOCKIQ] locked off=%d deg dir=%d -> handoff iqpi",
                    (int)g_lockiq_evt_off_deg, (int)g_foc_enc_dir);
            g_lockiq_running = 0u;   /* 交接: ISR 停发 mode 32 */
            Foc_StartIqPi();         /* mode 31 接管（偏移/方向已注入） */
        } else if (lock_code == LOCKIQ_EVT_LOCKED) {
            /* 事件置位后、处理前模式已被切走（如已进 mode 0）：跳过交接 */
            OBS_DBG("[LOCKIQ] locked off=%d deg but mode moved, skip handoff",
                    (int)g_lockiq_evt_off_deg);
        } else {
            OBS_DBG("[LOCKIQ] FAIL code=%d step=%d moved=%d track_err=%d",
                    (int)lock_code, (int)g_lockiq_step,
                    (int)g_lockiq_win_moved, (int)g_lockiq_track_err_cnts);
            Foc_LockIqPi_Stop();
        }
    }
#endif /* MOTOR_FOC_ENABLE */
}
