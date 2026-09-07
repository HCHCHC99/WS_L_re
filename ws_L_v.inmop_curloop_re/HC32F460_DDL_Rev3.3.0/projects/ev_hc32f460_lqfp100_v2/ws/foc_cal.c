/**
 *******************************************************************************
 * @file  foc_cal.c
 * @brief FOC 模式20 — 编码器零点校准实现。
 *
 *        流程：BETA(2s, θ=90°) -> ALPHA(2s, θ=0°) -> 锁 offset -> 关 PWM
 *              -> foc_obs 打印事件并自动回 mode 0。
 *
 *        offset 框架说明（为什么用原始硬件计数）：
 *          TMRA_1 是 16 位 free-run 计数器（0..65535），CPR=4096 整除
 *          65536，所以"原始计数 mod CPR"就是连续合法的圈内位置，回绕
 *          无缝。offset 与 mode 0 角度观测（Foc_Core_UpdateAngleObs）
 *          都用同一原始计数框架，不存在"各模块相对计数零点不一致"的
 *          框架错位问题（mode 30/31/32 的教训）。
 *
 *        ISR 内禁止打印：所有打印经 g_cal_evt 事件由 foc_obs 完成。
 *******************************************************************************
 */

#include "foc_cal.h"
#include "foc_math.h"
#include "foc_lock_iq_pi.h"     /* g_lockiq_align_volt_v（复用 mode 32 电压） */
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "hc32_ll_tmra.h"       /* TMRA_GetCountValue(CM_TMRA_1) */

/* 阶段时长 -> ISR tick 数 */
#define CAL_BETA_TICKS   ((uint32_t)CAL_BETA_MS   * (uint32_t)FOC_ISR_HZ / 1000u)
#define CAL_ALPHA_TICKS  ((uint32_t)CAL_ALPHA_MS  * (uint32_t)FOC_ISR_HZ / 1000u)

/*******************************************************************************
 * Watch 观测量
 ******************************************************************************/
volatile uint8_t  g_cal_running   = 0u;
volatile uint8_t  g_cal_phase     = CAL_PHASE_BETA;
volatile uint8_t  g_cal_evt       = 0u;
volatile uint16_t g_cal_beta_hw   = 0u;
volatile uint16_t g_cal_alpha_hw  = 0u;
volatile int32_t  g_cal_moved     = 0;
volatile int32_t  g_cal_offset    = 0;

/* 内部状态 */
static uint32_t s_phase_tick = 0u;

/*******************************************************************************
 * Foc_Cal_Start - mode 20 入口（主循环上下文，允许打印）
 ******************************************************************************/
void Foc_Cal_Start(void)
{
    Foc_Core_ClearFault();
    g_foc_mode        = FOC_MODE_ALIGN;   /* 复用 ALIGN 分发路径（按 g_cal_running 区分） */
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

    g_cal_running     = 1u;
    g_cal_phase       = CAL_PHASE_BETA;
    g_cal_evt         = 0u;
    s_phase_tick      = 0u;

    Foc_Core_PwmStart();   /* 零矢量起 PWM（g_foc_active=1），下一拍开始吸附 */

    CAL_DBG("start BETA 90deg volt=%d mV",
            (int)(g_lockiq_align_volt_v * 1000.0f));
}

/*******************************************************************************
 * Foc_Cal_Step - 校准步进（20 kHz ISR 中调用，禁止打印）
 ******************************************************************************/
void Foc_Cal_Step(const stc_i_data_t *pData)
{
    float    theta, valpha, vbeta, du, dv, dw, id, iq;
    uint16_t hw;

    /* ===== OC 保护（原始 pData，去抖 4 拍） ===== */
    if (Foc_Core_OverCurrent(pData)) {
        g_cal_phase   = CAL_PHASE_FAULT_OC;
        g_cal_evt     = CAL_EVT_OC;
        g_cal_running = 0u;
        Foc_Core_FaultStop(1u);   /* 置故障 + 关 PWM + active=0 + IDLE */
        return;
    }

    /* ===== 输出固定磁场：BETA=静止系90° / ALPHA=静止系0° ===== */
    theta  = (g_cal_phase == CAL_PHASE_BETA) ? FOC_MATH_HALF_PI : 0.0f;
    valpha = g_lockiq_align_volt_v * Foc_Math_Cos(theta);
    vbeta  = g_lockiq_align_volt_v * Foc_Math_Sin(theta);
    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);
    g_foc_valpha   = valpha;
    g_foc_vbeta    = vbeta;
    g_foc_vd       = valpha;
    g_foc_vq       = vbeta;
    g_foc_du       = du;
    g_foc_dv       = dv;
    g_foc_dw       = dw;
    g_foc_theta_rad = theta;

    /* 吸附期电流观察（无控制意义，仅 Watch / 故障参考） */
    Foc_Core_GetDq(pData, theta, &id, &iq);
    g_foc_id_ma = id * 1000.0f;
    g_foc_iq_ma = iq * 1000.0f;

    s_phase_tick++;

    /* ===== BETA 阶段：计时 2s -> 记录计数 -> 切 ALPHA ===== */
    if (g_cal_phase == CAL_PHASE_BETA) {
        if (s_phase_tick >= CAL_BETA_TICKS) {
            g_cal_beta_hw = TMRA_GetCountValue(CM_TMRA_1);
            g_cal_phase   = CAL_PHASE_ALPHA;
            s_phase_tick  = 0u;
            g_cal_evt     = CAL_EVT_BETA_DONE;
        }
        return;
    }

    /* ===== ALPHA 阶段：计时 2s -> 锁 offset -> 关 PWM 停发波 ===== */
    if (s_phase_tick >= CAL_ALPHA_TICKS) {
        int32_t off;

        hw = TMRA_GetCountValue(CM_TMRA_1);
        g_cal_alpha_hw = hw;
        /* 16 位回绕安全差值（BETA->ALPHA 位移，期望 ±CPR/(4×PP)≈±102） */
        g_cal_moved = (int16_t)((uint16_t)hw - (uint16_t)g_cal_beta_hw);

        /* 零点：ALPHA 结束时转子 d 轴在静止系 0°。
         * offset 用 TMRA_1 原始计数 mod CPR 表达，mode 0 观测同框架 */
        off = Foc_Core_ModPos((int32_t)hw * (int32_t)g_foc_enc_dir,
                              (int32_t)ENCODER_CPR);
        Foc_Core_SetAlignOffset(off);
        g_cal_offset = off;

        g_cal_phase       = CAL_PHASE_DONE;
        g_cal_evt         = CAL_EVT_LOCKED;
        g_cal_running     = 0u;
        g_foc_align_state = 0u;
        g_foc_active      = 0u;      /* 停发波：防止 ISR 回退到 Foc_Align_Step */
        Foc_Core_PwmStop();          /* EmergencyStop + duty 清零 */
        Foc_Core_SetStateMachine(FOC_STATE_IDLE);
        /* g_foc_mode 保持 ALIGN：obs 处理事件时经 CommRunner STOP 统一清理 */
    }
}

/*******************************************************************************
 * Foc_Cal_Stop - 停止校准（用户中途切模式，主循环上下文）
 *   自带 g_foc_active 清零，保证任何退出路径 ISR 都不会再进校准/
 *   对齐步进（Bug A 防护）。
 ******************************************************************************/
void Foc_Cal_Stop(void)
{
    if (g_cal_running) {
        g_cal_running     = 0u;
        g_foc_align_state = 0u;
        if (g_foc_active) {
            g_foc_active = 0u;
            Foc_Core_PwmStop();
            Foc_Core_SetStateMachine(FOC_STATE_IDLE);
        }
        CAL_DBG("stopped");
    }
}
