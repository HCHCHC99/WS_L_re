/**
 *******************************************************************************
 * @file  foc_iq_pi.c
 * @brief FOC 模式31 — 编码器转子角度 PI 电流环实现。
 *
 *        转子电角度：直接读取 TIMERA_1 硬件计数（不依赖主循环
 *        Encoder_Update()），并扣除 mode 30 (ZIZENG) 锁定的基线偏移，
 *        得到绝对化转子电角度用于 Park/InvPark。
 *
 *        电流路径与 ZIZENG 一致：foc_calib 残余零偏扣除 (dataCal 副本)
 *        -> Clarke/Park -> EMA -> PI；过流保护仍用原始 pData。
 *******************************************************************************
 */

#include "foc_iq_pi.h"
#include "foc_math.h"
#include "foc_calib.h"
#include "foc_zizeng.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "rtt_log.h"
#include "hc32_ll_tmra.h"   /* 直接读取 TIMERA_1 计数器 */

/* ISR period in us */
#define FOC_ISR_DT_US  (1000000u / FOC_ISR_HZ)

/*******************************************************************************
 * Keil Watch 可调变量 / 观测量
 ******************************************************************************/
volatile uint8_t g_iqpi_running        = 0;
volatile iqpi_step_t g_iqpi_step       = IQPI_STEP_IDLE;   /* 当前/最后状态，Watch 看枚举名 */
volatile iqpi_step_t g_iqpi_step_hist[IQPI_HISTORY_LEN];   /* 状态历史, [0]最旧 */
volatile uint8_t g_iqpi_step_hist_cnt  = 0;                /* 历史有效条数 0..10 */
volatile uint8_t g_iqpi_flip_cnt       = 0;                /* 框架180°自动翻转次数 */
volatile float   g_iqpi_iq_ref_ma      = IQPI_IQ_REF_MA;
volatile float   g_iqpi_iq_ref_ramp_ma = 0.0f;
volatile float   g_iqpi_iq_ramp_ma_s   = IQPI_IQ_RAMP_MA_S;
volatile float   g_iqpi_theta_rad      = 0.0f;

/* --- 转向诊断观测量（ISR 更新，main.c 周期打印 / Watch 直接看） --- */
volatile int32_t g_iqpi_enc_pos        = 0;   /* ISR 内累积的编码器计数镜像 */
volatile int32_t g_iqpi_win_moved      = 0;   /* 最近一次完成的500ms窗口位移(counts,带符号) */
volatile uint32_t g_iqpi_win_evals     = 0;   /* 已完成的窗口评估次数(0=方向检查从未运行) */
volatile int8_t  g_iqpi_cur_dir        = 0;   /* 最近窗口实测方向 +1/-1 */
volatile int8_t  g_iqpi_expect_dir     = 0;   /* 预期方向(+1/-1) */
volatile int8_t  g_iqpi_ref_dir        = 0;   /* 启动时捕获的 mode30 拖动方向基准 */

/* 翻转事件快照（ISR 置 flag，main.c 打印后清 flag） */
volatile uint8_t g_iqpi_evt_flag       = 0;
volatile uint8_t g_iqpi_evt_seq        = 0;   /* 第几次翻转 (1,2,...) */
volatile int32_t g_iqpi_evt_pos        = 0;   /* 翻转时编码器计数 */
volatile int32_t g_iqpi_evt_iq_ma      = 0;   /* 翻转时 iq (mA) */
volatile int32_t g_iqpi_evt_vq_mv      = 0;   /* 翻转时 vq (mV) */
volatile int32_t g_iqpi_evt_off_mrad   = 0;   /* 翻转后偏移基线 (mrad) */

/*******************************************************************************
 * d/q 轴 PI 配置（初值来自 foc_iq_pi.h 宏，字段 volatile 可 Watch 实时修改）
 ******************************************************************************/
pid_config_t g_iqpi_pid_id_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = IQPI_PI_KP,
    .ki           = IQPI_PI_KI,
    .kd           = 0.0f,
    .output_min   = -IQPI_PI_UMAX_V,
    .output_max   =  IQPI_PI_UMAX_V,
    .integral_max = IQPI_INTEGRAL_MAX,
    .i_term_max   = 0.0f,
    .update_ms    = 0,
};

pid_config_t g_iqpi_pid_iq_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = IQPI_PI_KP,
    .ki           = IQPI_PI_KI,
    .kd           = 0.0f,
    .output_min   = -IQPI_PI_UMAX_V,
    .output_max   =  IQPI_PI_UMAX_V,
    .integral_max = IQPI_INTEGRAL_MAX,
    .i_term_max   = 0.0f,
    .update_ms    = 0,
};

/* PI runtime states */
static pid_state_t s_pid_id;
static pid_state_t s_pid_iq;

/* 编码器硬件计数跟踪（与 foc_zizeng 相同的回绕处理，独立状态） */
static int32_t s_prev_hw_cnt = 0;
static int32_t s_enc_pos = 0;
static uint8_t s_enc_initialized = 0;

/* 启动时捕获的 ZIZENG 偏移基线 (rad) */
static float s_zizeng_off_rad = 0.0f;

/* mode 30 拖动方向基准 (+1/-1，0=未知)：mode 31 运行转向与之相反
 * 即判定为 180° 框架误差，自动翻转修正 */
static int8_t s_ref_dir = 0;

/* 堵转检测（每 500ms 窗口评估一次机械位移） */
static uint16_t s_stall_tick    = 0;
static int32_t  s_stall_enc_ref = 0;
static uint8_t  s_stall_flag    = 0;

/*******************************************************************************
 * Iqpi_SetStep - 更新当前状态并记录历史（状态变化才记录一条）
 *   hist[0] 最旧、hist[cnt-1] 最新；连续相同状态不重复记录，
 *   来回抖动（如 闭环<->饱和 反复切换）会按顺序如实记下
 ******************************************************************************/
static void Iqpi_SetStep(iqpi_step_t s)
{
    uint8_t i;

    if (s == g_iqpi_step) {
        return;
    }
    g_iqpi_step = s;

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
 * Foc_IqPi_InitPids - 绑定 PI 实例与配置（Foc_Init 调用一次）
 ******************************************************************************/
void Foc_IqPi_InitPids(void)
{
    PID_Init(&s_pid_id, &g_iqpi_pid_id_cfg);
    PID_Init(&s_pid_iq, &g_iqpi_pid_iq_cfg);
}

/*******************************************************************************
 * Foc_StartIqPi - mode 31 启动（需 mode 30 已锁定偏移）
 ******************************************************************************/
void Foc_StartIqPi(void)
{
    if (!Foc_Zizeng_GetOffsetRad(&s_zizeng_off_rad)) {
        IQPI_DBG("ERROR: ZIZENG offset not locked, run mode 30 first");
        Iqpi_SetStep(IQPI_STEP_ERR_NO_OFFSET);   /* 不清历史，Watch 可查 */
        return;
    }

    Foc_Core_ClearFault();
    g_iqpi_running      = 1;
    g_iqpi_theta_rad    = 0.0f;
    g_iqpi_iq_ref_ramp_ma = 0.0f;

    /* 新的一次运行：清空历史重新记录 */
    {
        uint8_t i;
        for (i = 0; i < IQPI_HISTORY_LEN; i++) {
            g_iqpi_step_hist[i] = IQPI_STEP_IDLE;
        }
        g_iqpi_step_hist_cnt = 0;
    }
    Iqpi_SetStep(IQPI_STEP_PWM_ZERO_VECTOR);

    s_enc_initialized = 0;
    s_enc_pos = 0;
    s_stall_tick    = 0;
    s_stall_enc_ref = 0;
    s_stall_flag    = 0;
    g_iqpi_flip_cnt = 0;
    /* 诊断观测量启动清零: 避免上次运行的残值混进本次日志 */
    g_iqpi_enc_pos    = 0;
    g_iqpi_win_moved  = 0;
    g_iqpi_win_evals  = 0;
    g_iqpi_cur_dir    = 0;
    g_iqpi_expect_dir = 0;
    g_iqpi_evt_flag   = 0;
    g_iqpi_evt_seq    = 0;
    s_ref_dir = Foc_Zizeng_GetDragDir();   /* mode 30 实测拖动方向基准 */
    g_iqpi_ref_dir = s_ref_dir;

    TMR4_PWM_SetFocMode(FOC_DEADTIME_NS);
    g_foc_du = 50.0f;
    g_foc_dv = 50.0f;
    g_foc_dw = 50.0f;
    TMR4_PWM_SetDuty3Phase(50.0f, 50.0f, 50.0f);
    TMR4_PWM_StartOutput();
    g_foc_active = 1u;
    g_foc_mode   = FOC_MODE_NONE;

    /* 相电流 DC 零偏自校准：本函数返回后的 ~210ms 内强制零矢量，
     * 锁定后 iq_ref 从 0 斜坡启动 */
    Foc_Calib_Start();

    Foc_Core_ResetEma();
    PID_Reset(&s_pid_id);
    PID_Reset(&s_pid_iq);

    IQPI_DBG("Started: off=%d mrad, iq_ref=%d mA, kp=%d m, ki=%d, ref_dir=%d",
             (int)(s_zizeng_off_rad * 1000.0f), (int)g_iqpi_iq_ref_ma,
             (int)(g_iqpi_pid_iq_cfg.kp * 1000.0f),
             (int)g_iqpi_pid_iq_cfg.ki,
             (int)s_ref_dir);
}

/*******************************************************************************
 * Foc_StopIqPi - mode 31 停止
 ******************************************************************************/
void Foc_StopIqPi(void)
{
    /* 停止不清状态：g_iqpi_step 保持最后状态（如 FAULT_OC / VQ_SAT），
     * 切到 mode 0 后 Watch 仍可查看停机原因；下次成功启动时才复位 */

    g_iqpi_running = 0;
    g_foc_active   = 0u;
    TMR4_PWM_EmergencyStop();
    g_foc_du = 0.0f;
    g_foc_dv = 0.0f;
    g_foc_dw = 0.0f;
    IQPI_DBG("Stopped");
}

/*******************************************************************************
 * Foc_IqPi_Step - 模式31 单步运算（20 kHz ISR 中调用）
 ******************************************************************************/
void Foc_IqPi_Step(const stc_i_data_t *pData)
{
    float enc_elec, id, iq, iq_ref_a, vd, vq, valpha, vbeta, du, dv, dw;
    float off_u, off_v, off_w;
    uint32_t hw_cnt;
    int16_t delta;
    stc_i_data_t dataCal;   /* 零偏校正后的电流采样副本 */

    if (!g_iqpi_running) {
        return;
    }

    /* 过流保护（原始 pData） */
    if (Foc_Core_OverCurrent(pData)) {
        Foc_Core_FaultStop(1u);
        g_iqpi_running = 0;
        Iqpi_SetStep(IQPI_STEP_FAULT_OC);   /* 停机后保持，Watch 可查 */
        return;
    }

    /* ===== 0. 编码器硬件计数（与 ZIZENG 相同的回绕处理） ===== */
    hw_cnt = TMRA_GetCountValue(CM_TMRA_1);
    if (!s_enc_initialized) {
        s_prev_hw_cnt = (int32_t)hw_cnt;
        s_enc_pos = 0;
        s_enc_initialized = 1;
    }
    delta = (int16_t)((uint16_t)hw_cnt - (uint16_t)s_prev_hw_cnt);
    s_prev_hw_cnt = (int32_t)hw_cnt;
    s_enc_pos += (int32_t)delta;
    g_enc_count = s_enc_pos;
    g_enc_count_f = (float)s_enc_pos;
    g_iqpi_enc_pos = s_enc_pos;   /* 诊断镜像 */

    /* ===== 1. 相电流 DC 零偏自校准（零矢量窗口，非阻塞） =====
     * 锁定前三相 50% 占空比（零电流），锁定后才开始闭环。 */
    if (!Foc_Calib_IsLocked()) {
        Foc_Calib_Feed(pData);
        TMR4_PWM_SetDuty3Phase(50.0f, 50.0f, 50.0f);
        g_foc_du = 50.0f;
        g_foc_dv = 50.0f;
        g_foc_dw = 50.0f;
        g_foc_id_ma = 0.0f;
        g_foc_iq_ma = 0.0f;
        g_foc_vd = 0.0f;    /* 校准窗口 PI 未运行, 清掉上次运行残值 */
        g_foc_vq = 0.0f;
        Iqpi_SetStep(IQPI_STEP_PWM_ZERO_VECTOR);
        return;
    }

    /* ===== 2. 转子电角度（扣 ZIZENG 偏移基线，绝对化） ===== */
    enc_elec = (float)Foc_Core_ModPos(s_enc_pos * (int32_t)g_foc_enc_dir,
                                      (int32_t)ENCODER_CPR)
             * (FOC_MATH_2PI * (float)FOC_POLE_PAIRS / (float)ENCODER_CPR);
    enc_elec -= (float)((int32_t)(enc_elec * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
    if (enc_elec < 0.0f) {
        enc_elec += FOC_MATH_2PI;
    }
    enc_elec -= s_zizeng_off_rad;
    /* ZIZENG q 轴拖动锁定瞬间：电流矢量在 theta+90°，转子 d 轴滞后一个负载
     * 角 delta，故锁定基线 offset = c + 90° - delta，减基线后角度
     * = 转子角 - 90° + delta。这里补回 90° 使 Park 角对准真实转子 d 轴
     * （残余误差仅为 delta 几度）。若不补，iq 电流矢量仅超前转子 delta，
     * 转矩不足以克服静摩擦 -> 堵转（iq 有读数但转子不动）。 */
    enc_elec += FOC_MATH_HALF_PI;
    enc_elec -= (float)((int32_t)(enc_elec * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
    if (enc_elec < 0.0f) {
        enc_elec += FOC_MATH_2PI;
    }
    g_iqpi_theta_rad = enc_elec;
    g_foc_theta_rad    = enc_elec;
    g_foc_if_rotor_rad = enc_elec;

    /* ===== 3. 相电流零偏扣除 -> Clarke/Park -> EMA =====
     * 过流保护等仍使用原始 pData，不受影响。 */
    dataCal = *pData;
    Foc_Calib_GetOffsetsMa(&off_u, &off_v, &off_w);
    dataCal.i16IU_mA = (int16_t)((float)pData->i16IU_mA - off_u);
    dataCal.i16IV_mA = (int16_t)((float)pData->i16IV_mA - off_v);
    dataCal.i16IW_mA = (int16_t)((float)pData->i16IW_mA - off_w);

    Foc_Core_GetDq(&dataCal, enc_elec, &id, &iq);
    Foc_Core_EmaFilter(&id, &iq);
    g_foc_id_ma = id * 1000.0f;
    g_foc_iq_ma = iq * 1000.0f;

    /* ===== 4. iq 软启动斜坡（Watch 改 g_iqpi_iq_ref_ma 实时生效） ===== */
    {
        float step = g_iqpi_iq_ramp_ma_s / (float)FOC_ISR_HZ;
        if (g_iqpi_iq_ref_ramp_ma < g_iqpi_iq_ref_ma) {
            g_iqpi_iq_ref_ramp_ma += step;
            if (g_iqpi_iq_ref_ramp_ma > g_iqpi_iq_ref_ma) {
                g_iqpi_iq_ref_ramp_ma = g_iqpi_iq_ref_ma;
            }
        } else if (g_iqpi_iq_ref_ramp_ma > g_iqpi_iq_ref_ma) {
            g_iqpi_iq_ref_ramp_ma -= step;
            if (g_iqpi_iq_ref_ramp_ma < g_iqpi_iq_ref_ma) {
                g_iqpi_iq_ref_ramp_ma = g_iqpi_iq_ref_ma;
            }
        }
    }

    /* ===== 5. 双 PI 闭环：vd = PI_id(0, id)，vq = PI_iq(iq_ref, iq) ===== */
    iq_ref_a = g_iqpi_iq_ref_ramp_ma * 0.001f;
    vd = PID_UpdateUs(&s_pid_id, 0.0f,     id, FOC_ISR_DT_US);
    vq = PID_UpdateUs(&s_pid_iq, iq_ref_a, iq, FOC_ISR_DT_US);
    g_foc_vd = vd;
    g_foc_vq = vq;

    /* ===== 5b. 运行状态标记 g_iqpi_step（诊断用，优先级: 电压饱和 > 堵转 > 正常） ===== */
    {
        float ref_abs, iq_abs;
        ref_abs = g_iqpi_iq_ref_ramp_ma;
        if (ref_abs < 0.0f) {
            ref_abs = -ref_abs;
        }
        iq_abs = g_foc_iq_ma;
        if (iq_abs < 0.0f) {
            iq_abs = -iq_abs;
        }

        /* 电压饱和: vd/vq 任一顶到限幅 98% 以上 */
        {
            float vq_lim = 0.98f * g_iqpi_pid_iq_cfg.output_max;
            float vd_lim = 0.98f * g_iqpi_pid_id_cfg.output_max;
            if ((vq >= vq_lim) || (vq <= -vq_lim) ||
                (vd >= vd_lim) || (vd <= -vd_lim)) {
                Iqpi_SetStep(IQPI_STEP_RUNNING_VQ_SAT);
            } else {
                Iqpi_SetStep(IQPI_STEP_CLOSED_LOOP);
            }
        }

        /* 堵转疑似: 电流已建立但 500ms 窗口内机械位移过小
         * (静摩擦/框架角度错误/电压饱和连带)。未饱和时才评估。 */
        if (g_iqpi_step == IQPI_STEP_CLOSED_LOOP) {
            if ((ref_abs > IQPI_STALL_IQ_MIN_MA) && (iq_abs > IQPI_STALL_IQ_MIN_MA)) {
                s_stall_tick++;
                if (s_stall_tick >= IQPI_STALL_WIN_MS * (FOC_ISR_HZ / 1000u)) {
                    int32_t moved = s_enc_pos - s_stall_enc_ref;
                    int32_t moved_abs = (moved < 0) ? -moved : moved;

                    s_stall_enc_ref = s_enc_pos;
                    s_stall_tick = 0;
                    g_iqpi_win_evals++;          /* 诊断: 完成一次窗口评估 */
                    g_iqpi_win_moved = moved;

                    if (moved_abs < (int32_t)IQPI_STALL_MIN_CNTS) {
                        s_stall_flag = 1u;              /* 没动 -> 疑似堵转 */
                    } else {
                        s_stall_flag = 0u;
                        /* 方向比对: 编码器位移符号与 mode 30 拖动方向相反
                         * => 180° 框架误差(180°误差在 dq 系里隐形, 唯一暴露
                         * 就是转向) => 偏移基线 +180° 使框架翻转, 转矩反向。
                         * 负 iq_ref 时预期方向同样取反。 */
                        if (s_ref_dir != 0) {
                            int8_t cur_dir = (moved > 0) ? 1 : -1;
                            int8_t expect = (g_iqpi_iq_ref_ramp_ma >= 0.0f)
                                          ? s_ref_dir : (int8_t)-s_ref_dir;
                            g_iqpi_cur_dir = cur_dir;
                            g_iqpi_expect_dir = expect;
                            if (cur_dir != expect) {
                                s_zizeng_off_rad += FOC_MATH_PI;
                                if (s_zizeng_off_rad >= FOC_MATH_2PI) {
                                    s_zizeng_off_rad -= FOC_MATH_2PI;
                                }
                                g_iqpi_flip_cnt++;
                                Iqpi_SetStep(IQPI_STEP_DIR_FLIPPED);
                                /* 翻转事件快照: main.c 检测 flag 后打印一次 */
                                g_iqpi_evt_seq    = g_iqpi_flip_cnt;
                                g_iqpi_evt_pos    = s_enc_pos;
                                g_iqpi_evt_iq_ma  = (int32_t)g_foc_iq_ma;
                                g_iqpi_evt_vq_mv  = (int32_t)(vq * 1000.0f);
                                g_iqpi_evt_off_mrad = (int32_t)(s_zizeng_off_rad
                                                                * 1000.0f);
                                g_iqpi_evt_flag   = 1u;
                            }
                        }
                    }
                }
                if (s_stall_flag) {
                    Iqpi_SetStep(IQPI_STEP_RUNNING_STALL);
                }
            } else {
                /* 电流未建立(斜坡初期/参考为0)不评估，避免误报 */
                s_stall_tick = 0;
                s_stall_flag = 0;
            }
        } else {
            s_stall_tick = 0;
            s_stall_flag = 0;
        }
    }

    /* ===== 6. InvPark -> SVPWM 输出 ===== */
    Foc_InvPark(vd, vq, enc_elec, &valpha, &vbeta);
    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);

    g_foc_valpha = valpha;
    g_foc_vbeta  = vbeta;
    g_foc_du = du;
    g_foc_dv = dv;
    g_foc_dw = dw;
}
