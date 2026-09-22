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
#include "foc_dci.h"
#include "foc_dcal24.h"
#include "foc_drun29.h"
#include "foc_drun41.h"
#include "foc_smo45.h"
#include "foc_smo.h"
#include "foc_pll.h"
#include "foc_calib.h"
#include "dev_comm_runner.h"   /* CommRunner_SetMode（mode 20 完成自动回 mode 0） */
#include "encoder.h"
#include "motor_config.h"
#include "rtt_log.h"
#include <stdio.h>
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
 * Pll_Fmt1000 - float 转 "SIII.DDD" 定点字符串（×1000，负数带符号）
 * 工程 printf 不支持 %f，PLL 数据打印统一走此转换。buf 需 ≥14 字节。
 ******************************************************************************/
static const char *Pll_Fmt1000(char *buf, float v)
{
    int32_t x = (int32_t)(v * 1000.0f + ((v >= 0.0f) ? 0.5f : -0.5f));
    int32_t a = (x < 0) ? -x : x;

    (void)snprintf(buf, 14, "%s%d.%03d",
                   (x < 0) ? "-" : "", (int)(a / 1000), (int)(a % 1000));
    return buf;
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

    /* ---- mode45 SMO 旁观诊断（sqrtf/atan2f 只在主循环，不进 ISR） ----
     * omega_e = 机械 rpm -> 电角速度 rad/s（×2π/60×极对数） */
    if (g_smo45_running) {
        float smo_omega_e = g_smo45_speed_filt_rpm * 0.10472f
                          * (float)FOC_POLE_PAIRS;
        Foc_Smo_Diag((float)g_smo45_rotor_deg, smo_omega_e);

        /* ---- mode45 PLL 旁观诊断（只滤波；角度比较已在 ISR 同拍完成） ---- */
        Foc_Pll_Diag();

        /* ---- mode45 PLL 数据快照 RTT 打印（5ms 节流，供复制粘贴判读） ----
         * 工程 printf 不支持 %f：全部经 Pll_Fmt1000 转"整型.整型"定点串。
         * e=补偿后 Park 角−编码器角（同拍）；sE=同拍 SMO atan2 角差；
         * d=当前补偿角 δ̂；sl=无感锁存状态（1=Park/速度反馈已用 PLL）。
         * 补偿开启后 e 均值应 ≈0（残差=锁点偏置+bias）。 */
        {
            static uint32_t s_pll_print_t0 = 0;
            uint32_t now = (uint32_t)tickTimer_GetCount();
            char b1[14], b2[14], b3[14], b4[14], b5[14], b6[14], b7[14], b8[14], b9[14], b10[14];

            if ((uint32_t)(now - s_pll_print_t0) >= 5u) {
                s_pll_print_t0 = now;
                OBS_DBG("[PLL] t=%u e=%s sE=%s th=%s ec=%s wE=%s enc=%s iq=%s d=%s sl=%u",
                        (unsigned)now,
                        Pll_Fmt1000(b1, g_pll_diag_theta_err_deg),
                        Pll_Fmt1000(b2, g_pll_diag_smo_err_deg),
                        Pll_Fmt1000(b3, g_pll_theta_rotor_deg),
                        Pll_Fmt1000(b4, (float)g_smo45_rotor_deg),
                        Pll_Fmt1000(b5, g_pll_omega_hat_rpm),
                        Pll_Fmt1000(b6, g_smo45_speed_filt_rpm),
                        Pll_Fmt1000(b7, (float)g_smo45_iq_filt_ma),
                        Pll_Fmt1000(b8, g_pll_comp_deg_out),
                        (unsigned)g_smo45_sl_active);
            }
        }

        /* ---- SMO 反电动势判定结果跳变事件（ok/fail 由 foc_smo.c 维护，
         *      mode45 每次 Start 经 Foc_Smo_Reset 重新判定） ---- */
        {
            static uint8_t s_smo_emf_ok_prev = 0u;

            if (g_smo_emf_ok != s_smo_emf_ok_prev) {
                s_smo_emf_ok_prev = g_smo_emf_ok;
                if (g_smo_emf_ok) {
                    OBS_DBG("SMO EMF judge: OK (ratio=%d%% phase=%ddeg hold=%dms)",
                            (int)(g_smo_jdg_ratio_filt * 100.0f),
                            (int)g_smo_jdg_phase_filt_deg,
                            (int)g_smo_jdg_hold_ms);
                } else if (g_smo_jdg_fail != 0u) {
                    OBS_DBG("SMO EMF judge: FAIL mask=0x%02X%s%s%s",
                            (int)g_smo_jdg_fail,
                            ((g_smo_jdg_fail & 0x01u) ? " [rpm<min]" : ""),
                            ((g_smo_jdg_fail & 0x02u) ? " [ratio]" : ""),
                            ((g_smo_jdg_fail & 0x04u) ? " [phase]" : ""));
                }
                /* fail==0 的掉零 = Stop/Reset 路径，不打印 */
            }
        }
    }

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
            OLF_DBG("LOCKED offset=%d deg -> drag f %d->%d mHz tr=%d ms",
                    (int)((g_olf_offset * 360) / (int32_t)ENCODER_CPR),
                    (int)(g_olf_freq_init_hz * 1000.0f),
                    (int)(g_olf_freq_targ_hz * 1000.0f),
                    (int)g_olf_freq_tr_ms);
            break;
        case OLF_EVT_RAMP_DONE:
            OLF_DBG("freq ramp done f=%d mHz diff=%d deg",
                    (int)(g_olf_freq_hz * 1000.0f), (int)g_olf_diff_deg);
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

    /* ---- mode 26 峰峰值记录（foc_olf 内部 5s 窗口刷新，此处检测变化打印） ---- */
    if (g_olf_running && (g_olf_state == OLF_STEP_DRAG)) {
        static float s_last_olf_id_pp = -1.0f;
        static float s_last_olf_iq_pp = -1.0f;
        if ((g_olf_id_pp_ma != s_last_olf_id_pp)
                || (g_olf_iq_pp_ma != s_last_olf_iq_pp)) {
            s_last_olf_id_pp = g_olf_id_pp_ma;
            s_last_olf_iq_pp = g_olf_iq_pp_ma;
            OLF_DBG("idPP=%d iqPP=%d mA (%d s window)",
                    (int)g_olf_id_pp_ma, (int)g_olf_iq_pp_ma,
                    (int)(OLF_PP_WIN_MS / 1000u));
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

    /* ---- mode 27 峰峰值记录（foc_dcl 内部 5s 窗口刷新，此处检测变化打印） ---- */
    if (g_dcl_running && (g_dcl_state == DCL_STEP_RUN)) {
        static float s_last_dcl_id_pp = -1.0f;
        static float s_last_dcl_iq_pp = -1.0f;
        if ((g_dcl_id_pp_ma != s_last_dcl_id_pp)
                || (g_dcl_iq_pp_ma != s_last_dcl_iq_pp)) {
            s_last_dcl_id_pp = g_dcl_id_pp_ma;
            s_last_dcl_iq_pp = g_dcl_iq_pp_ma;
            DCL_DBG("idPP=%d iqPP=%d mA (%d s window)",
                    (int)g_dcl_id_pp_ma, (int)g_dcl_iq_pp_ma,
                    (int)(DCL_PP_WIN_MS / 1000u));
        }
    }

    /* ---- mode 24 独立校准事件（ISR 置 evt；完成/过流自动回 mode 0） ---- */
    if (g_dcal24_evt != 0u) {
        uint8_t evt = g_dcal24_evt;
        g_dcal24_evt = 0u;
        switch (evt) {
        case DCAL24_EVT_ZERO_DONE:
            DCAL24_LOG("zero-offset locked iu=%d iv=%d iw=%d mA",
                       (int)g_dcal24_zero_u_ma, (int)g_dcal24_zero_v_ma,
                       (int)g_dcal24_zero_w_ma);
            break;
        case DCAL24_EVT_BETA_DONE:
            DCAL24_LOG("BETA done hw=%d -> ALPHA 0deg", (int)g_dcal24_beta_hw);
            break;
        case DCAL24_EVT_DONE:
            DCAL24_LOG("done: beta=%d alpha=%d moved=%d offset=%d cnts (%d deg) -> mode 0",
                       (int)g_dcal24_beta_hw, (int)g_dcal24_alpha_hw,
                       (int)g_dcal24_moved, (int)g_dcal24_offset,
                       (int)((g_dcal24_offset * 360) / (int32_t)ENCODER_CPR));
            CommRunner_SetMode(COMM_RUNNER_STOP);
            break;
        case DCAL24_EVT_OC:
            DCAL24_LOG("FAULT_OC i=%d mA", (int)g_foc_fault_i_ma);
            CommRunner_SetMode(COMM_RUNNER_STOP);
            break;
        default:
            break;
        }
    }

    /* ---- mode 28 功角电流闭环事件（ISR 置 evt，此处打印；仅 OC 自动回 mode 0） ---- */
    if (g_dci_evt != 0u) {
        uint8_t evt = g_dci_evt;
        g_dci_evt = 0u;
        switch (evt) {
        case DCI_EVT_CALIB_DONE:
            DCI_DBG("zero-offset locked iu=%d iv=%d iw=%d mA",
                    (int)g_calib_iu_off_ma, (int)g_calib_iv_off_ma,
                    (int)g_calib_iw_off_ma);
            break;
        case DCI_EVT_BETA_DONE:
            DCI_DBG("BETA done -> ALPHA 0deg");
            break;
        case DCI_EVT_LOCKED:
            DCI_DBG("LOCKED off=%d deg -> run dlt %d->%d deg tr=%d ms",
                    (int)((g_dci_offset * 360) / (int32_t)ENCODER_CPR),
                    (int)g_dci_dlt_init_deg, (int)g_dci_dlt_targ_deg,
                    (int)g_dci_dlt_tr_ms);
            break;
        case DCI_EVT_RAMP_DONE:
            DCI_DBG("ramp done dlt=%d deg spd=%d mHz",
                    (int)g_dci_dlt_now_deg, (int)(g_dci_speed_hz * 1000.0f));
            break;
        case DCI_EVT_OC:
            DCI_DBG("FAULT_OC i=%d mA", (int)g_foc_fault_i_ma);
            CommRunner_SetMode(COMM_RUNNER_STOP);
            break;
        default:
            break;
        }
    }

    /* ---- mode 28 功角闭环运行数据（200ms 周期，全整型；电流环版） ---- */
    if (g_dci_running && (g_dci_state == DCI_STEP_RUN)) {
        static uint32_t s_last_dci_dbg = 0u;
        uint32_t now = tickTimer_GetCount();
        if ((now - s_last_dci_dbg) >= 200u) {
            s_last_dci_dbg = now;
            DCI_DBG("dlt=%d deg spd=%d mHz diff=%d deg idRef=%d iqRef=%d mA id=%d iq=%d mA vd=%d mV vq=%d mV sat=%d",
                    (int)g_dci_dlt_now_deg,           /* 当前 delta 指令 (deg) */
                    (int)(g_dci_speed_hz * 1000.0f),  /* 实测电频率 (mHz, 带符号) */
                    (int)g_dci_diff_deg,              /* 参考功角 (deg, =dlt) */
                    (int)g_dci_id_ref_ma,             /* d 轴电流参考 (mA) */
                    (int)g_dci_iq_ref_ma,             /* q 轴电流参考 (mA) */
                    (int)g_dci_id_ma,                 /* id 反馈 (mA, 零偏校正后) */
                    (int)g_dci_iq_ma,                 /* iq 反馈 (mA, 零偏校正后) */
                    (int)(g_foc_vd * 1000.0f),        /* PI 输出 vd (mV) */
                    (int)(g_foc_vq * 1000.0f),        /* PI 输出 vq (mV) */
                    (int)g_dci_vsat);                 /* 电压饱和标志 */
        }
    }

    /* ---- mode 28 峰峰值记录（foc_dci 内部 5s 窗口刷新，此处检测变化打印） ---- */
    if (g_dci_running && (g_dci_state == DCI_STEP_RUN)) {
        static float s_last_dci_id_pp = -1.0f;
        static float s_last_dci_iq_pp = -1.0f;
        if ((g_dci_id_pp_ma != s_last_dci_id_pp)
                || (g_dci_iq_pp_ma != s_last_dci_iq_pp)) {
            s_last_dci_id_pp = g_dci_id_pp_ma;
            s_last_dci_iq_pp = g_dci_iq_pp_ma;
            DCI_DBG("idPP=%d iqPP=%d mA (%d s window)",
                    (int)g_dci_id_pp_ma, (int)g_dci_iq_pp_ma,
                    (int)(DCI_PP_WIN_MS / 1000u));
        }
    }

    /* ---- mode 28 均值记录（foc_dci 内部 3s 窗口刷新，此处检测变化打印） ---- */
    if (g_dci_running && (g_dci_state == DCI_STEP_RUN)) {
        static float s_last_dci_id_mean = 1e9f;
        static float s_last_dci_iq_mean = 1e9f;
        if ((g_dci_id_mean_ma != s_last_dci_id_mean)
                || (g_dci_iq_mean_ma != s_last_dci_iq_mean)) {
            s_last_dci_id_mean = g_dci_id_mean_ma;
            s_last_dci_iq_mean = g_dci_iq_mean_ma;
            DCI_DBG("idMean=%d iqMean=%d mA (%d s window)",
                    (int)g_dci_id_mean_ma, (int)g_dci_iq_mean_ma,
                    (int)(DCI_MEAN_WIN_MS / 1000u));
        }
    }

    /* ---- mode 28 误差统计（foc_dci 内部 5s 窗口刷新，P 调参主判据） ---- */
    if (g_dci_running && (g_dci_state == DCI_STEP_RUN)) {
        static float s_last_dci_ed = 1e9f;
        static float s_last_dci_eq = 1e9f;
        if ((g_dci_ed_mean_ma != s_last_dci_ed)
                || (g_dci_eq_mean_ma != s_last_dci_eq)) {
            s_last_dci_ed = g_dci_ed_mean_ma;
            s_last_dci_eq = g_dci_eq_mean_ma;
            DCI_DBG("edMean=%d eqMean=%d edPP=%d eqPP=%d mA (%d s win)%s",
                    (int)g_dci_ed_mean_ma, (int)g_dci_eq_mean_ma,
                    (int)g_dci_ed_pp_ma, (int)g_dci_eq_pp_ma,
                    (int)(DCI_ERR_WIN_MS / 1000u),
                    g_dci_vsat ? " [SAT]" : "");
        }
    }

    /* ---- mode 29 独立电流环事件 ---- */
    if (g_drun29_evt != 0u) {
        uint8_t evt = g_drun29_evt;
        g_drun29_evt = 0u;
        switch (evt) {
        case DRUN29_EVT_RAMP_DONE:
            DRUN29_LOG("ramp done dlt=%d deg spd=%d mHz rot=%d cnt",
                       (int)g_drun29_dlt_now_deg,
                       (int)(g_drun29_speed_hz * 1000.0f),
                       (int)g_drun29_rotor_count);
            break;
        case DRUN29_EVT_OC:
            DRUN29_LOG("FAULT_OC i=%d mA", (int)g_foc_fault_i_ma);
            CommRunner_SetMode(COMM_RUNNER_STOP);
            break;
        default:
            break;
        }
    }

    /* ---- mode 29 独立电流环运行数据（200ms 周期） ---- */
    if (g_drun29_running && (g_drun29_state == DRUN29_STEP_RUN)) {
        static uint32_t s_last_drun29_dbg = 0u;
        uint32_t now = tickTimer_GetCount();
        if ((now - s_last_drun29_dbg) >= 200u) {
            s_last_drun29_dbg = now;
            DRUN29_LOG("dlt=%d deg spd=%d mHz rot=%d cnt idRef=%d iqRef=%d mA id=%d iq=%d mA vd=%d mV vq=%d mV sat=%d",
                       (int)g_drun29_dlt_now_deg,
                       (int)(g_drun29_speed_hz * 1000.0f),
                       (int)g_drun29_rotor_count,
                       (int)g_drun29_id_ref_ma,
                       (int)g_drun29_iq_ref_ma,
                       (int)g_drun29_id_ma,
                       (int)g_drun29_iq_ma,
                       (int)(g_foc_vd * 1000.0f),
                       (int)(g_foc_vq * 1000.0f),
                       (int)g_drun29_vsat);
        }
    }

    /* ---- mode 29 独立误差统计 ---- */
    if (g_drun29_running && (g_drun29_state == DRUN29_STEP_RUN)) {
        static float s_last_drun29_ed = 1e9f;
        static float s_last_drun29_eq = 1e9f;
        if ((g_drun29_ed_mean_ma != s_last_drun29_ed)
                || (g_drun29_eq_mean_ma != s_last_drun29_eq)) {
            s_last_drun29_ed = g_drun29_ed_mean_ma;
            s_last_drun29_eq = g_drun29_eq_mean_ma;
            DRUN29_LOG("edMean=%d eqMean=%d edPP=%d eqPP=%d mA (%d s win)%s",
                       (int)g_drun29_ed_mean_ma, (int)g_drun29_eq_mean_ma,
                       (int)g_drun29_ed_pp_ma, (int)g_drun29_eq_pp_ma,
                       (int)(DRUN29_ERR_WIN_MS / 1000u),
                       g_drun29_vsat ? " [SAT]" : "");
        }
    }

    /* ---- mode 41 current feed-forward loop events ---- */
    if (g_drun41_evt != 0u) {
        uint8_t evt = g_drun41_evt;
        g_drun41_evt = 0u;
        switch (evt) {
        case DRUN41_EVT_RAMP_DONE:
            DRUN41_LOG("ramp done dlt=%d deg spd=%d mHz rot=%d cnt",
                       (int)g_drun41_dlt_now_deg,
                       (int)(g_drun41_speed_hz * 1000.0f),
                       (int)g_drun41_rotor_count);
            break;
        case DRUN41_EVT_OC:
            DRUN41_LOG("FAULT_OC i=%d mA", (int)g_foc_fault_i_ma);
            CommRunner_SetMode(COMM_RUNNER_STOP);
            break;
        default:
            break;
        }
    }

    if (g_drun41_running && (g_drun41_state == DRUN41_STEP_RUN)) {
        static uint32_t s_last_drun41_dbg = 0u;
        uint32_t now = tickTimer_GetCount();
        if ((now - s_last_drun41_dbg) >= 200u) {
            s_last_drun41_dbg = now;
            DRUN41_LOG("dlt=%d deg spd=%d mHz idRef=%d iqRef=%d mA id=%d iq=%d mA vdPi=%d vqPi=%d vdFF=%d vqFF=%d mV sat=%d ff=%u",
                       (int)g_drun41_dlt_now_deg,
                       (int)(g_drun41_speed_hz * 1000.0f),
                       (int)g_drun41_id_ref_ma,
                       (int)g_drun41_iq_ref_ma,
                       (int)g_drun41_id_ma,
                       (int)g_drun41_iq_ma,
                       (int)(g_drun41_vd_pi_v * 1000.0f),
                       (int)(g_drun41_vq_pi_v * 1000.0f),
                       (int)(g_drun41_vd_ff_v * 1000.0f),
                       (int)(g_drun41_vq_ff_v * 1000.0f),
                       (int)g_drun41_vsat,
                       (unsigned)g_drun41_ff_enable);
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
