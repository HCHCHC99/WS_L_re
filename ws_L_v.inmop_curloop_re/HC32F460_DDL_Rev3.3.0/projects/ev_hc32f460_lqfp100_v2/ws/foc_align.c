/**
 *******************************************************************************
 * @file  foc_align.c
 * @brief FOC 模式23 — 静止电角度对齐校准实现（自 foc.c 原样迁移）。
 *******************************************************************************
 */

#include "foc_align.h"
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"

/* Align 阶段计数换算（FOC_ISR_HZ tick） */
#define FOC_ALIGN_BETA_CNT     ((uint32_t)FOC_ALIGN_BETA_MS * FOC_ISR_HZ / 1000u)
#define FOC_ALIGN_STABLE_CNT  ((uint32_t)FOC_ALIGN_STABLE_MS * FOC_ISR_HZ / 1000u)
#define FOC_ALIGN_TIMEOUT_CNT ((uint32_t)FOC_ALIGN_TIMEOUT_MS * FOC_ISR_HZ / 1000u)
#define FOC_ALIGN_HOLD_CNT    ((uint32_t)FOC_ALIGN_HOLD_MS * FOC_ISR_HZ / 1000u)

/* Align calibration (mode 23) */
volatile float   g_foc_align_volt_v = FOC_ALIGN_VOLT_V;
volatile uint8_t  g_foc_align_evt    = 0u;
volatile int32_t  g_foc_align_evt_v1 = 0;
volatile int32_t  g_foc_align_evt_v2 = 0;
volatile int32_t  g_foc_align_evt_v3 = 0;

/* Align 内部状态 */
static int32_t  s_align_last_cnt   = 0;
static uint32_t s_align_stable_cnt = 0u;
static uint32_t s_align_hold_cnt   = 0u;
static uint8_t  s_align_phase      = 0u;
static uint32_t s_align_phase_tick = 0u;

/*******************************************************************************
 * Foc_StartAlign - mode 23: standstill electrical alignment
 ******************************************************************************/
void Foc_StartAlign(void)
{
    Foc_Core_ClearFault();
    g_foc_mode        = FOC_MODE_ALIGN;
    g_foc_phase       = 4u;
    g_foc_theta_rad   = 0.0f;
    g_foc_id_ma       = 0.0f;
    g_foc_iq_ma       = 0.0f;
    g_foc_vd          = 0.0f;
    g_foc_vq          = 0.0f;

    Foc_Core_SetStateMachine(FOC_STATE_ALIGN);
    s_align_phase      = 0u;
    s_align_phase_tick = 0u;
    s_align_last_cnt   = (int32_t)g_enc_count;
    s_align_stable_cnt = 0u;
    s_align_hold_cnt   = 0u;
    Foc_Core_ResetVoltageEnvelope();
    Foc_Core_ResetEma();
    g_foc_align_state  = 1u;

    /* 注：对齐模式不使用 PI；id/iq/spd PI 由 Foc_StartCurrentLoop() 统一复位，
     * 因此本模块不依赖 foc_curloop。 */
}

/*******************************************************************************
 * Foc_Align_Step - 对齐校准步进（20 kHz ISR 中调用）
 ******************************************************************************/
void Foc_Align_Step(const stc_i_data_t *pData)
{
    float theta = (s_align_phase == 0u) ? FOC_MATH_HALF_PI : 0.0f;
    float valpha, vbeta, du, dv, dw;
    float id, iq;
    int32_t cnt;

    if (Foc_Core_OverCurrent(pData)) {
        g_foc_align_evt    = 5u;
        g_foc_align_evt_v1 = 1;
        g_foc_align_evt_v2 = (int32_t)g_foc_fault_i_ma;
        Foc_Core_FaultStop(1u);
        return;
    }

    valpha = g_foc_align_volt_v * Foc_Math_Cos(theta);
    vbeta  = g_foc_align_volt_v * Foc_Math_Sin(theta);
    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);
    g_foc_valpha = valpha;
    g_foc_vbeta  = vbeta;
    g_foc_vd = valpha;
    g_foc_vq = vbeta;
    g_foc_du = du;
    g_foc_dv = dv;
    g_foc_dw = dw;

    Foc_Core_GetDq(pData, theta, &id, &iq);
    g_foc_id_ma = id * 1000.0f;
    g_foc_iq_ma = iq * 1000.0f;

    s_align_phase_tick++;

    if (s_align_phase == 0u) {
        if (s_align_phase_tick >= FOC_ALIGN_BETA_CNT) {
            s_align_phase      = 1u;
            s_align_phase_tick = 0u;
            s_align_last_cnt   = (int32_t)g_enc_count;
            s_align_stable_cnt = 0u;
            g_foc_align_state  = 1u;
            g_foc_align_evt    = 2u;
            g_foc_align_evt_v1 = 0;
            g_foc_align_evt_v2 = 0;
            g_foc_align_evt_v3 = 0;
        }
        return;
    }

    if (g_foc_align_state == 1u) {
        cnt = (int32_t)g_enc_count;
        if (cnt == s_align_last_cnt) {
            if (++s_align_stable_cnt >= FOC_ALIGN_STABLE_CNT) {
                int32_t off = Foc_Core_ModPos((int32_t)g_enc_count * (int32_t)g_foc_enc_dir,
                                              (int32_t)ENCODER_CPR);
                Foc_Core_SetAlignOffset(off);
                g_foc_align_state  = 2u;
                s_align_hold_cnt   = 0u;
                g_foc_align_evt    = 3u;
                g_foc_align_evt_v1 = off;
                g_foc_align_evt_v2 = (int32_t)g_foc_id_ma;
                g_foc_align_evt_v3 = (int32_t)g_foc_iq_ma;
            }
        } else {
            s_align_last_cnt   = cnt;
            s_align_stable_cnt = 0u;
        }
        if (s_align_phase_tick >= FOC_ALIGN_TIMEOUT_CNT) {
            g_foc_align_evt    = 5u;
            g_foc_align_evt_v1 = 3;
            g_foc_align_evt_v2 = (int32_t)g_foc_id_ma;
            g_foc_align_evt_v3 = (int32_t)g_foc_iq_ma;
            Foc_Core_FaultStop(3u);
            return;
        }
    } else if (g_foc_align_state == 2u) {
        if (++s_align_hold_cnt >= FOC_ALIGN_HOLD_CNT) {
            g_foc_align_state = 3u;
            g_foc_active      = 0u;
            Foc_Core_PwmStop();
            Foc_Core_SetStateMachine(FOC_STATE_IDLE);
            g_foc_align_evt    = 4u;
            g_foc_align_evt_v1 = Foc_Core_GetAlignOffset();
            g_foc_align_evt_v2 = (int32_t)(g_foc_valpha * 1000.0f);
            g_foc_align_evt_v3 = (int32_t)(g_foc_vbeta * 1000.0f);
        }
    }
}
