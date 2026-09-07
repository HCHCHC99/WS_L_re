/**
 *******************************************************************************
 * @file  foc_cal_angle.c
 * @brief FOC 模式25 — 手动角度吸附实现（SVPWM 教学）。
 *
 *        流程：自动校准 BETA(2s, 90°) -> ALPHA(2s, 0°, 锁零点 offset)
 *              -> 刹车 50/50/50 等输入
 *              -> 检测 g_foc_angle_input 改值 -> 磁场指向目标角吸附 2s
 *              -> 500ms 校验窗（磁场保持不动）判吸稳
 *              -> 无论成败回刹车，停在 mode 25 等下一次输入。
 *              仅过流自动退回 mode 0（经 foc_obs 事件）。
 *
 *        角度框架说明（与 mode 20 / mode 0 观测一致）：
 *          offset = mod(ALPHA 结束时 TMRA_1 原始计数 × 编码器方向, CPR)，
 *          表示"转子 d 轴在静止系电角度 0°"时的原始计数。实测电角度：
 *            diff = mod(原始计数 × dir - offset, CPR)     （机械圈内位置）
 *            elec = diff × 360 × 极对数 / CPR  (mod 360)
 *          与 Foc_Core_UpdateAngleObs 同一框架，零点天然一致。
 *
 *        校验窗位移测量：int16 回绕差值逐拍累加到 s_rel（无 wrap 风险，
 *        50µs 一拍内位移远小于 ±32767），窗口内跟踪 s_min/s_max，
 *        moved = max-min <= g_calang_stable_cnts 判吸稳。
 *
 *        ISR 内禁止打印：所有打印经 g_calang_evt 事件由 foc_obs 完成。
 *******************************************************************************
 */

#include "foc_cal_angle.h"
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "hc32_ll_tmra.h"       /* TMRA_GetCountValue(CM_TMRA_1) */

/* 阶段时长 -> ISR tick 数 */
#define CALANG_BETA_TICKS    ((uint32_t)CALANG_BETA_MS    * (uint32_t)FOC_ISR_HZ / 1000u)
#define CALANG_ALPHA_TICKS   ((uint32_t)CALANG_ALPHA_MS   * (uint32_t)FOC_ISR_HZ / 1000u)
#define CALANG_HOLD_TICKS    ((uint32_t)CALANG_HOLD_MS    * (uint32_t)FOC_ISR_HZ / 1000u)
#define CALANG_VERIFY_TICKS  ((uint32_t)CALANG_VERIFY_MS  * (uint32_t)FOC_ISR_HZ / 1000u)

/* deg -> rad（目标角为电角度，静止系直接输出） */
#define CALANG_DEG2RAD  (FOC_MATH_2PI / 360.0f)

/*******************************************************************************
 * Watch 观测量 / 可调变量
 ******************************************************************************/
volatile int32_t  g_foc_angle_input    = 0;                          /* 用户输入目标电角度 */
volatile float    g_calang_volt_v      = (float)FOC_ALIGN_VOLT_V;    /* 吸附电压 0.4V */
volatile uint8_t  g_calang_running     = 0u;
volatile uint8_t  g_calang_state       = CALANG_STEP_IDLE;
volatile uint8_t  g_calang_evt         = 0u;
volatile int32_t  g_calang_stable_cnts = 8;                          /* 校验窗静止判据 */
volatile int32_t  g_calang_target_deg  = 0;
volatile int32_t  g_calang_meas_deg    = 0;
volatile int32_t  g_calang_err_deg     = 0;
volatile int32_t  g_calang_win_moved   = 0;
volatile int32_t  g_calang_offset      = 0;
volatile float    g_calang_du          = 50.0f;
volatile float    g_calang_dv          = 50.0f;
volatile float    g_calang_dw          = 50.0f;

/* 内部状态 */
static uint32_t s_phase_tick = 0u;    /* 当前阶段计时（tick） */
static int32_t  s_last_input = 0;     /* 上次输入快照（改值检测基准） */
static float    s_target_rad = 0.0f;  /* 锁存的目标电角度 (rad) */
static uint16_t s_hw_prev    = 0u;    /* 校验窗上一拍编码器原始计数 */
static int32_t  s_rel        = 0;     /* 校验窗内相对位移累积 (counts) */
static int32_t  s_min        = 0;     /* 窗口内最小相对位移 */
static int32_t  s_max        = 0;     /* 窗口内最大相对位移 */

/*******************************************************************************
 * 内部助手：输出指定电角度的固定磁场 + 刷新观测量（ISR 内调用）
 ******************************************************************************/
static void CalAng_OutputField(const stc_i_data_t *pData, float theta)
{
    float valpha, vbeta, du, dv, dw, id, iq;

    valpha = g_calang_volt_v * Foc_Math_Cos(theta);
    vbeta  = g_calang_volt_v * Foc_Math_Sin(theta);
    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);

    g_foc_valpha    = valpha;
    g_foc_vbeta     = vbeta;
    g_foc_vd        = valpha;
    g_foc_vq        = vbeta;
    g_foc_theta_rad = theta;
    g_foc_du        = du;
    g_foc_dv        = dv;
    g_foc_dw        = dw;
    g_calang_du     = du;
    g_calang_dv     = dv;
    g_calang_dw     = dw;

    /* 吸附期电流观察（无控制意义，仅 Watch / 故障参考） */
    Foc_Core_GetDq(pData, theta, &id, &iq);
    g_foc_id_ma = id * 1000.0f;
    g_foc_iq_ma = iq * 1000.0f;
}

/*******************************************************************************
 * 内部助手：刹车 50/50/50 零矢量（不出力）
 ******************************************************************************/
static void CalAng_OutputBrake(void)
{
    TMR4_PWM_SetDuty3Phase(50.0f, 50.0f, 50.0f);

    g_foc_valpha = 0.0f;
    g_foc_vbeta  = 0.0f;
    g_foc_du     = 50.0f;
    g_foc_dv     = 50.0f;
    g_foc_dw     = 50.0f;
    g_calang_du  = 50.0f;
    g_calang_dv  = 50.0f;
    g_calang_dw  = 50.0f;
    g_foc_id_ma  = 0.0f;
    g_foc_iq_ma  = 0.0f;
}

/*******************************************************************************
 * 内部助手：锁存目标角（wrap 到 [0,360)）+ 阶段计时清零
 ******************************************************************************/
static void CalAng_LatchTarget(void)
{
    int32_t t = Foc_Core_ModPos(g_foc_angle_input, 360);

    g_calang_target_deg = t;
    s_target_rad        = (float)t * CALANG_DEG2RAD;
    s_phase_tick        = 0u;
}

/*******************************************************************************
 * Foc_CalAngle_Start - mode 25 入口（主循环上下文，允许打印）
 *   自动先跑 mode 20 式校准，锁零点后进入刹车等待。
 ******************************************************************************/
void Foc_CalAngle_Start(void)
{
    Foc_Core_ClearFault();
    g_foc_mode        = FOC_MODE_ALIGN;   /* 复用 ALIGN 分发路径（按 g_calang_running 区分） */
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

    g_calang_running = 1u;
    g_calang_state   = CALANG_STEP_CAL_BETA;
    g_calang_evt     = 0u;
    s_phase_tick     = 0u;
    /* Start 时刻快照：校准 4s 期间预设的 g_foc_angle_input 会在
     * 校准完成后立即被检测到并执行 */
    s_last_input     = g_foc_angle_input;
    s_target_rad     = 0.0f;
    s_rel            = 0;
    s_min            = 0;
    s_max            = 0;
    s_hw_prev        = TMRA_GetCountValue(CM_TMRA_1);

    Foc_Core_PwmStart();   /* 零矢量起 PWM（g_foc_active=1），下一拍开始吸附 */

    CALANG_DBG("start calib BETA 90deg volt=%d mV",
               (int)(g_calang_volt_v * 1000.0f));
}

/*******************************************************************************
 * Foc_CalAngle_Step - 步进（20 kHz ISR 中调用，禁止打印）
 ******************************************************************************/
void Foc_CalAngle_Step(const stc_i_data_t *pData)
{
    uint16_t hw;
    int16_t  d;
    int32_t  diff;
    int32_t  e;

    /* ===== OC 保护（原始 pData，去抖 4 拍） ===== */
    if (Foc_Core_OverCurrent(pData)) {
        g_calang_state   = CALANG_STEP_FAULT_OC;
        g_calang_evt     = CALANG_EVT_OC;
        g_calang_running = 0u;
        Foc_Core_FaultStop(1u);   /* 置故障 + 关 PWM + active=0 + IDLE */
        return;
    }

    switch (g_calang_state) {
    /* ===== 校准 BETA：磁场定 90°，2s ===== */
    case CALANG_STEP_CAL_BETA:
        CalAng_OutputField(pData, FOC_MATH_HALF_PI);
        if (++s_phase_tick >= CALANG_BETA_TICKS) {
            s_phase_tick   = 0u;
            g_calang_state = CALANG_STEP_CAL_ALPHA;
            g_calang_evt   = CALANG_EVT_BETA_DONE;
        }
        break;

    /* ===== 校准 ALPHA：磁场定 0°，2s，结束锁零点 -> 刹车等待 ===== */
    case CALANG_STEP_CAL_ALPHA:
        CalAng_OutputField(pData, 0.0f);
        if (++s_phase_tick >= CALANG_ALPHA_TICKS) {
            int32_t off;

            /* 零点：ALPHA 结束时转子 d 轴在静止系 0°。
             * offset 用 TMRA_1 原始计数 mod CPR 表达（与 mode 20 同框架） */
            hw  = TMRA_GetCountValue(CM_TMRA_1);
            off = Foc_Core_ModPos((int32_t)hw * (int32_t)g_foc_enc_dir,
                                  (int32_t)ENCODER_CPR);
            Foc_Core_SetAlignOffset(off);
            g_calang_offset = off;

            s_hw_prev = hw;              /* 位移基准复位 */
            s_phase_tick   = 0u;
            g_calang_state = CALANG_STEP_BRAKE_WAIT;
            g_calang_evt   = CALANG_EVT_LOCKED;
            CalAng_OutputBrake();        /* 立即切刹车，模式停在 25 */
        }
        break;

    /* ===== 刹车等待：50/50/50，监视 g_foc_angle_input 改值 ===== */
    case CALANG_STEP_BRAKE_WAIT:
        CalAng_OutputBrake();
        if (g_foc_angle_input != s_last_input) {
            s_last_input = g_foc_angle_input;
            CalAng_LatchTarget();        /* 锁存目标 + 计时清零 */
            g_calang_state = CALANG_STEP_HOLD_ATTRACT;
        }
        break;

    /* ===== 吸附：磁场指向目标角 2s（中途改值 -> 重新锁存重新计时） ===== */
    case CALANG_STEP_HOLD_ATTRACT:
        if (g_foc_angle_input != s_last_input) {
            s_last_input = g_foc_angle_input;
            CalAng_LatchTarget();
        }
        CalAng_OutputField(pData, s_target_rad);
        if (++s_phase_tick >= CALANG_HOLD_TICKS) {
            /* 进入 500ms 校验窗：位移窗口清零（磁场保持不动，用户决定） */
            s_hw_prev      = TMRA_GetCountValue(CM_TMRA_1);
            s_rel          = 0;
            s_min          = 0;
            s_max          = 0;
            s_phase_tick   = 0u;
            g_calang_state = CALANG_STEP_HOLD_VERIFY;
        }
        break;

    /* ===== 校验：磁场保持 y，500ms 窗口跟踪位移，判吸稳 ===== */
    case CALANG_STEP_HOLD_VERIFY:
        CalAng_OutputField(pData, s_target_rad);
        hw = TMRA_GetCountValue(CM_TMRA_1);
        d  = (int16_t)((uint16_t)hw - (uint16_t)s_hw_prev);
        s_hw_prev = hw;
        s_rel    += d;
        if (s_rel < s_min) { s_min = s_rel; }
        if (s_rel > s_max) { s_max = s_rel; }

        if (++s_phase_tick >= CALANG_VERIFY_TICKS) {
            g_calang_win_moved = s_max - s_min;

            /* 实测电角度 (0~360)：与 mode 0 观测同框架（原始计数 - offset） */
            diff = Foc_Core_ModPos((int32_t)hw * (int32_t)g_foc_enc_dir
                                   - (int32_t)g_calang_offset,
                                   (int32_t)ENCODER_CPR);
            g_calang_meas_deg = (diff * 360 * (int32_t)FOC_POLE_PAIRS
                                 / (int32_t)ENCODER_CPR) % 360;

            /* err = meas - target，折叠到 (-180,180] */
            e = g_calang_meas_deg - g_calang_target_deg;
            g_calang_err_deg = 180 - Foc_Core_ModPos(180 - e, 360);

            s_phase_tick   = 0u;
            g_calang_state = CALANG_STEP_BRAKE_WAIT;   /* 无论成败回刹车 */
            g_calang_evt   = (g_calang_win_moved <= g_calang_stable_cnts)
                             ? CALANG_EVT_DONE_OK : CALANG_EVT_DONE_FAIL;
        }
        break;

    case CALANG_STEP_FAULT_OC:
    default:
        /* 故障/未知状态：不发波，等待主循环切模式 */
        break;
    }
}

/*******************************************************************************
 * Foc_CalAngle_Stop - 停止（用户中途切模式，主循环上下文）
 *   自带 g_foc_active 清零，保证任何退出路径 ISR 都不会再进入本模块步进。
 ******************************************************************************/
void Foc_CalAngle_Stop(void)
{
    if (g_calang_running) {
        g_calang_running     = 0u;
        g_foc_align_state    = 0u;
        if (g_foc_active) {
            g_foc_active = 0u;
            Foc_Core_PwmStop();
            Foc_Core_SetStateMachine(FOC_STATE_IDLE);
        }
        CALANG_DBG("stopped");
    }
}
