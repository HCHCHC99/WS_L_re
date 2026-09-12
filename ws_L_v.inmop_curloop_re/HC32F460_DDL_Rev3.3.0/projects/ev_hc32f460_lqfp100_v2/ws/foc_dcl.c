/**
 *******************************************************************************
 * @file  foc_dcl.c
 * @brief FOC 模式27 — 功角闭环拖动实现。
 *
 *        流程：自动校准 BETA(2s, 90°) -> ALPHA(2s, 0°, 锁零点 offset)
 *              -> RUN：磁场角 = 转子实测电角 + delta（功角闭环核心），
 *                 delta 从 g_dcl_dlt_init_deg 线性爬坡到 g_dcl_dlt_targ_deg
 *                 （历时 g_dcl_dlt_tr_ms，0 = 立即），到点置 RAMP_DONE 一次。
 *                 转速不由指令决定：电压定转速（BEMF 平衡，直流电机式）。
 *
 *        输出约定（q 轴电压，与 mode 26/30 一致）：
 *          磁场角 fa 直接作为控制变量：valpha=V·cos(fa), vbeta=V·sin(fa)，
 *          等效 d 轴角 theta = fa - 90°（g_foc_theta_rad 语义统一），
 *          锁相稳态控制系电流 g_foc_id_ma≈0, g_foc_iq_ma≈I。
 *
 *        角度框架（增量累积式，编码器升级为控制路径）：
 *          每拍 wrap-safe 差分 -> ±DCL_ENC_DELTA_MAX(32) counts 限幅
 *          （物理极限 7800rpm 单拍上限 26.6 counts，超过必是毛刺，
 *          限幅把单次毛刺对功角框架的永久污染封顶 ±2.8° 电角）
 *          -> s_enc_pos 累加 -> 转子电角 =
 *             mod((s_enc_pos - s_off_rel) × 编码器方向, CPR) × 360°×极对数/CPR
 *          s_off_rel = ALPHA 锁零点瞬间的 s_enc_pos（锁相瞬间转子角=0）。
 *
 *        电流观测用原始 pData（与 mode 25/26 同策略：吸附式校准无 DC
 *        零偏窗口，传感器残余零偏相对 5A 拖动电流可忽略）。
 *        真实转子系电流 id≈I·cosδ, iq≈I·sinδ（δ=当前功角）。
 *
 *        ISR 内禁止打印：事件经 g_dcl_evt、周期数据经 Watch 观测量
 *        由 foc_obs 在主循环完成。
 *******************************************************************************
 */

#include "foc_dcl.h"
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "hc32_ll_tmra.h"       /* TMRA_GetCountValue(CM_TMRA_1) */

/* 阶段时长/窗口 -> ISR tick 数 */
#define DCL_BETA_TICKS    ((uint32_t)DCL_BETA_MS    * (uint32_t)FOC_ISR_HZ / 1000u)
#define DCL_ALPHA_TICKS   ((uint32_t)DCL_ALPHA_MS   * (uint32_t)FOC_ISR_HZ / 1000u)
#define DCL_SPEED_WIN_TICKS ((uint32_t)DCL_SPEED_WIN_MS * (uint32_t)FOC_ISR_HZ / 1000u)

/* 角度换算（观测/控制用） */
#define DCL_RAD2DEG   57.2958f
#define DCL_DEG2RAD   0.0174533f

/*******************************************************************************
 * Watch 观测量 / 可调变量
 ******************************************************************************/
volatile float    g_dcl_dlt_init_deg = 5.0f;   /* 功角爬坡起点（软吸附起步） */
volatile float    g_dcl_dlt_targ_deg = 45.0f;  /* 功角爬坡终点 */
volatile uint32_t g_dcl_dlt_tr_ms   = 2000u;   /* 功角爬坡过渡时间 */
volatile float    g_dcl_volt_v      = 0.6f;    /* 电压（=调速旋钮），Start 复位 */
volatile uint8_t  g_dcl_running     = 0u;
volatile uint8_t  g_dcl_state       = DCL_STEP_IDLE;
volatile uint8_t  g_dcl_evt         = 0u;
volatile int32_t  g_dcl_offset      = 0;
volatile float    g_dcl_dlt_now_deg = 0.0f;
volatile float    g_dcl_speed_hz    = 0.0f;
volatile int32_t  g_dcl_field_deg   = 0;
volatile int32_t  g_dcl_rotor_deg   = 0;
volatile int32_t  g_dcl_diff_deg    = 0;
volatile float    g_dcl_id_ma       = 0.0f;
volatile float    g_dcl_iq_ma       = 0.0f;
volatile int32_t  g_dcl_enc_pos     = 0;
volatile float    g_dcl_du          = 50.0f;
volatile float    g_dcl_dv          = 50.0f;
volatile float    g_dcl_dw          = 50.0f;

/* 内部状态 */
static uint32_t s_phase_tick  = 0u;    /* 校准阶段计时（tick） */
static uint32_t s_run_tick    = 0u;    /* RUN 态计时（tick），爬坡时基 */
static uint8_t  s_ramp_done   = 0u;    /* 爬坡完成事件已置位（每次运行只置一次） */
static uint16_t s_enc_prev_hw = 0u;    /* 上拍编码器硬件计数 */
static int32_t  s_enc_pos     = 0;     /* 相对计数累积（限幅后） */
static uint8_t  s_enc_init    = 0u;    /* 首拍基准初始化标志 */
static int32_t  s_off_rel     = 0;     /* 锁零点瞬间的相对计数（控制帧零点） */
static uint32_t s_speed_tick  = 0u;    /* 转速窗口计时（tick） */
static int32_t  s_speed_acc   = 0;     /* 转速窗口内计数累积 */

/*******************************************************************************
 * 内部助手：输出指定磁场电角度 + 刷新观测量（ISR 内调用）
 *   入参 fa = 磁场角（本模式的主控制变量 = 转子角 + delta）。
 *   valpha=V·cos(fa), vbeta=V·sin(fa)；
 *   等效 d 轴角 theta = fa - 90°（g_foc_theta_rad 语义与 mode 26/30 统一），
 *   锁相稳态时控制系电流 g_foc_id_ma≈0, g_foc_iq_ma≈I。
 ******************************************************************************/
static void Dcl_OutputField(const stc_i_data_t *pData, float fa)
{
    float valpha, vbeta, du, dv, dw, id, iq, th;

    valpha = g_dcl_volt_v * Foc_Math_Cos(fa);
    vbeta  = g_dcl_volt_v * Foc_Math_Sin(fa);
    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);

    th = fa - FOC_MATH_HALF_PI;
    th -= (float)((int32_t)(th * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
    if (th < 0.0f) {
        th += FOC_MATH_2PI;
    }

    g_foc_valpha    = valpha;
    g_foc_vbeta     = vbeta;
    g_foc_vd        = valpha;
    g_foc_vq        = vbeta;
    g_foc_theta_rad = th;
    g_foc_du        = du;
    g_foc_dv        = dv;
    g_foc_dw        = dw;
    g_dcl_du        = du;
    g_dcl_dv        = dv;
    g_dcl_dw        = dw;

    /* 控制系（theta d 轴框架）电流观察：锁相稳态 id≈0, iq≈I */
    Foc_Core_GetDq(pData, th, &id, &iq);
    g_foc_id_ma = id * 1000.0f;
    g_foc_iq_ma = iq * 1000.0f;
}

/*******************************************************************************
 * Foc_Dcl_Start - mode 27 入口（主循环上下文，允许打印）
 *   自动先跑 mode 25/26 式校准锁零点，随后进入功角闭环拖动。
 *   电压复位 0.6V（防残留）；功角三参数不复位（便于预设后启动）。
 ******************************************************************************/
void Foc_Dcl_Start(void)
{
    Foc_Core_ClearFault();
    g_foc_mode        = FOC_MODE_ALIGN;   /* 复用 ALIGN 分发路径（按 g_dcl_running 区分） */
    g_foc_phase       = 4u;
    g_foc_theta_rad   = FOC_MATH_HALF_PI;
    g_foc_id_ma       = 0.0f;
    g_foc_iq_ma       = 0.0f;
    g_foc_vd          = 0.0f;
    g_foc_vq          = 0.0f;

    Foc_Core_SetStateMachine(FOC_STATE_ALIGN);
    Foc_Core_ResetVoltageEnvelope();
    Foc_Core_ResetEma();
    g_foc_align_state = 1u;

    g_dcl_running     = 1u;
    g_dcl_state       = DCL_STEP_CAL_BETA;
    g_dcl_evt         = 0u;
    s_phase_tick      = 0u;
    s_run_tick        = 0u;
    s_ramp_done       = 0u;
    s_enc_init        = 0u;
    s_enc_pos         = 0;
    s_off_rel         = 0;
    s_speed_tick      = 0u;
    s_speed_acc       = 0;
    g_dcl_dlt_now_deg = g_dcl_dlt_init_deg;
    g_dcl_speed_hz    = 0.0f;
    g_dcl_field_deg   = 0;
    g_dcl_rotor_deg   = 0;
    g_dcl_diff_deg    = 0;
    g_dcl_id_ma       = 0.0f;
    g_dcl_iq_ma       = 0.0f;
    g_dcl_enc_pos     = 0;
    g_dcl_volt_v      = 0.6f;
    /* g_dcl_offset 保留上次锁定值（校准完成后覆盖刷新） */

    Foc_Core_PwmStart();   /* 零矢量起 PWM（g_foc_active=1），下一拍开始吸附 */

    DCL_DBG("start calib BETA 90deg dlt %d->%d deg tr=%d ms volt=%d mV",
            (int)g_dcl_dlt_init_deg, (int)g_dcl_dlt_targ_deg,
            (int)g_dcl_dlt_tr_ms, (int)(g_dcl_volt_v * 1000.0f));
}

/*******************************************************************************
 * Foc_Dcl_Step - 步进（20 kHz ISR 中调用，禁止打印）
 ******************************************************************************/
void Foc_Dcl_Step(const stc_i_data_t *pData)
{
    float rot_rad, fa, id, iq, w, dlt;
    uint16_t hw, hw0;
    int32_t delta, rel, rot_deg, fld_deg, dfd, dir;
    uint32_t tr_ticks;

    /* ===== OC 保护（原始 pData，去抖在 Foc_Core_OverCurrent 内） ===== */
    if (Foc_Core_OverCurrent(pData)) {
        g_dcl_state   = DCL_STEP_FAULT_OC;
        g_dcl_evt     = DCL_EVT_OC;
        g_dcl_running = 0u;
        Foc_Core_FaultStop(1u);   /* 置故障 + 关 PWM + active=0 + IDLE */
        return;
    }

    switch (g_dcl_state) {
    /* ===== 校准 BETA：磁场定 90°，2s ===== */
    case DCL_STEP_CAL_BETA:
        Dcl_OutputField(pData, FOC_MATH_HALF_PI);
        if (++s_phase_tick >= DCL_BETA_TICKS) {
            s_phase_tick = 0u;
            g_dcl_state  = DCL_STEP_CAL_ALPHA;
            g_dcl_evt    = DCL_EVT_BETA_DONE;
        }
        break;

    /* ===== 校准 ALPHA：磁场定 0°，2s，结束锁零点 -> 功角闭环 ===== */
    case DCL_STEP_CAL_ALPHA:
        Dcl_OutputField(pData, 0.0f);
        if (++s_phase_tick >= DCL_ALPHA_TICKS) {
            /* 锁零点（双帧）：
             *  - 控制帧：s_off_rel = 此刻相对计数，RUN 态转子电角 =
             *    mod((s_enc_pos - s_off_rel) × dir, CPR)，锁相瞬间 = 0
             *  - 显示帧：g_dcl_offset = hw 绝对帧（与 mode 20/25/26 同框架，
             *    便于跨模式/跨次对比） */
            hw0 = TMRA_GetCountValue(CM_TMRA_1);
            dir = (int32_t)g_foc_enc_dir;
            g_dcl_offset = Foc_Core_ModPos((int32_t)hw0 * dir,
                                           (int32_t)ENCODER_CPR);
            Foc_Core_SetAlignOffset(g_dcl_offset);   /* core 共享角度观测同帧（与 mode 26 一致） */
            s_off_rel     = s_enc_pos;
            s_enc_prev_hw = hw0;      /* 编码器基准锚定在锁零点瞬间 */
            s_enc_init    = 1u;

            s_phase_tick = 0u;
            s_run_tick   = 0u;
            s_ramp_done  = 0u;
            g_dcl_state  = DCL_STEP_RUN;
            g_dcl_evt    = DCL_EVT_LOCKED;
        }
        break;

    /* ===== RUN：磁场 = 转子 + delta（功角闭环核心） ===== */
    case DCL_STEP_RUN:
        /* 1. delta 爬坡：value = init + (targ-init)×w，w = elapsed/tr 线性，
         *    init/targ/tr 每拍实时读 Watch（运行中改 = 按新值重算轨迹）。
         *    tr=0 立即到目标；到点置 RAMP_DONE 一次。 */
        tr_ticks = g_dcl_dlt_tr_ms * (FOC_ISR_HZ / 1000u);
        if (tr_ticks == 0u) {
            w = 1.0f;
        } else {
            w = (float)s_run_tick / (float)tr_ticks;
            if (w > 1.0f) {
                w = 1.0f;
            }
        }
        dlt = g_dcl_dlt_init_deg
            + (g_dcl_dlt_targ_deg - g_dcl_dlt_init_deg) * w;
        g_dcl_dlt_now_deg = dlt;
        if ((s_ramp_done == 0u) && (s_run_tick >= tr_ticks)) {
            s_ramp_done = 1u;
            g_dcl_evt   = DCL_EVT_RAMP_DONE;
        }
        s_run_tick++;

        /* 2. 编码器增量读取：wrap-safe 差分 + ±DCL_ENC_DELTA_MAX 限幅
         *    （毛刺单拍污染封顶 ±32 counts ≈ ±2.8° 电角） */
        hw = TMRA_GetCountValue(CM_TMRA_1);
        if (!s_enc_init) {
            s_enc_prev_hw = hw;
            s_enc_init    = 1u;
        }
        delta = (int32_t)(int16_t)((uint16_t)hw - s_enc_prev_hw);
        s_enc_prev_hw = hw;
        if (delta > DCL_ENC_DELTA_MAX) {
            delta = DCL_ENC_DELTA_MAX;
        }
        if (delta < -DCL_ENC_DELTA_MAX) {
            delta = -DCL_ENC_DELTA_MAX;
        }
        s_enc_pos += delta;
        g_dcl_enc_pos = s_enc_pos;

        /* 3. 转子真实电角度（相对帧 - 锁零点，机械圈 -> 电角度） */
        rel = Foc_Core_ModPos((s_enc_pos - s_off_rel) * (int32_t)g_foc_enc_dir,
                              (int32_t)ENCODER_CPR);
        rot_rad = (float)rel * (FOC_MATH_2PI * (float)FOC_POLE_PAIRS
                                / (float)ENCODER_CPR);
        rot_rad -= (float)((int32_t)(rot_rad * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
        if (rot_rad < 0.0f) {
            rot_rad += FOC_MATH_2PI;
        }

        /* 4. 磁场角 = 转子 + delta（锁相核心），输出 */
        fa = rot_rad + dlt * DCL_DEG2RAD;
        fa -= (float)((int32_t)(fa * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
        if (fa < 0.0f) {
            fa += FOC_MATH_2PI;
        }
        Dcl_OutputField(pData, fa);

        /* 5. 真实转子系 id/iq（锁相稳态: id≈I·cosδ, iq≈I·sinδ） */
        Foc_Core_GetDq(pData, rot_rad, &id, &iq);
        g_dcl_id_ma = id * 1000.0f;
        g_dcl_iq_ma = iq * 1000.0f;

        /* 6. 转速测量（200ms 窗口，带符号）：Hz = counts×极对数×窗口率/CPR */
        s_speed_acc += delta;
        if (++s_speed_tick >= DCL_SPEED_WIN_TICKS) {
            g_dcl_speed_hz = (float)s_speed_acc * (float)FOC_POLE_PAIRS
                             * (1000.0f / (float)DCL_SPEED_WIN_MS)
                             / (float)ENCODER_CPR;
            s_speed_tick = 0u;
            s_speed_acc  = 0;
        }

        /* 7. 角度观测（整型电角度 deg）+ 功角折叠（应≈dlt_now，验证用） */
        fld_deg = (int32_t)(fa * DCL_RAD2DEG);
        if (fld_deg >= 360) {
            fld_deg -= 360;
        }
        rot_deg = (int32_t)(rot_rad * DCL_RAD2DEG);
        dfd     = fld_deg - rot_deg;
        dfd     = 180 - Foc_Core_ModPos(180 - dfd, 360);

        g_dcl_field_deg = fld_deg;
        g_dcl_rotor_deg = rot_deg;
        g_dcl_diff_deg  = dfd;
        break;

    case DCL_STEP_FAULT_OC:
    default:
        /* 故障/未知状态：不发波，等待主循环切模式 */
        break;
    }
}

/*******************************************************************************
 * Foc_Dcl_Stop - 停止（用户中途切模式，主循环上下文）
 *   自带 g_foc_active 清零，保证任何退出路径 ISR 都不会再进入本模块步进。
 ******************************************************************************/
void Foc_Dcl_Stop(void)
{
    if (g_dcl_running) {
        g_dcl_running     = 0u;
        g_foc_align_state = 0u;
        if (g_foc_active) {
            g_foc_active = 0u;
            Foc_Core_PwmStop();
            Foc_Core_SetStateMachine(FOC_STATE_IDLE);
        }
        DCL_DBG("stopped");
    }
}
