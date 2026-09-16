/**
 *******************************************************************************
 * @file  foc_dcal24.c
 * @brief FOC mode 24 - independent calibration-only state machine.
 *
 * ZERO measures private phase-current offsets at 50/50/50.  BETA aligns the
 * rotor to 90 degrees, ALPHA aligns it to zero and records the live encoder
 * offset.  The completed snapshot stays valid after mode 24 stops and the
 * rotor is moved in mode 0.
 *******************************************************************************
 */

#include "foc_dcal24.h"
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "hc32_ll_tmra.h"
#include <string.h>

#define DCAL24_ZERO_TOTAL  (DCAL24_ZERO_SKIP_SAMPLES + DCAL24_ZERO_AVG_SAMPLES)

volatile uint8_t  g_dcal24_running   = 0u;
volatile uint8_t  g_dcal24_state     = DCAL24_STEP_IDLE;
volatile uint8_t  g_dcal24_evt       = 0u;
volatile uint16_t g_dcal24_beta_hw   = 0u;
volatile uint16_t g_dcal24_alpha_hw  = 0u;
volatile int32_t  g_dcal24_moved     = 0;
volatile int32_t  g_dcal24_offset    = 0;
volatile float    g_dcal24_zero_u_ma = 0.0f;
volatile float    g_dcal24_zero_v_ma = 0.0f;
volatile float    g_dcal24_zero_w_ma = 0.0f;
volatile float    g_dcal24_volt_v    = 0.6f;

static foc_dcal24_result_t s_result;
static uint32_t s_zero_tick;
static int64_t  s_zero_sum_u;
static int64_t  s_zero_sum_v;
static int64_t  s_zero_sum_w;
static uint32_t s_phase_tick;

static void Dcal24_ClearRunState(void)
{
    g_dcal24_running = 0u;
    g_dcal24_state   = DCAL24_STEP_IDLE;
    g_dcal24_evt     = 0u;
    g_dcal24_beta_hw  = 0u;
    g_dcal24_alpha_hw = 0u;
    g_dcal24_moved    = 0;
    g_dcal24_offset   = 0;
    g_dcal24_zero_u_ma = 0.0f;
    g_dcal24_zero_v_ma = 0.0f;
    g_dcal24_zero_w_ma = 0.0f;
    g_dcal24_volt_v    = 0.6f;

    memset(&s_result, 0, sizeof(s_result));
    s_zero_tick  = 0u;
    s_zero_sum_u = 0;
    s_zero_sum_v = 0;
    s_zero_sum_w = 0;
    s_phase_tick = 0u;
}

static void Dcal24_ZeroVector(void)
{
    g_foc_du = 50.0f;
    g_foc_dv = 50.0f;
    g_foc_dw = 50.0f;
    TMR4_PWM_SetDuty3Phase(50.0f, 50.0f, 50.0f);
}

static stc_i_data_t Dcal24_CorrectedData(const stc_i_data_t *pData)
{
    stc_i_data_t data = *pData;
    data.i16IU_mA = (int16_t)((float)pData->i16IU_mA - s_result.zero_u_ma);
    data.i16IV_mA = (int16_t)((float)pData->i16IV_mA - s_result.zero_v_ma);
    data.i16IW_mA = (int16_t)((float)pData->i16IW_mA - s_result.zero_w_ma);
    return data;
}

static void Dcal24_OutputField(const stc_i_data_t *pData, float field_angle)
{
    stc_i_data_t data = Dcal24_CorrectedData(pData);
    float valpha = g_dcal24_volt_v * Foc_Math_Cos(field_angle);
    float vbeta  = g_dcal24_volt_v * Foc_Math_Sin(field_angle);
    float du, dv, dw, id, iq, theta;

    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);

    theta = field_angle - FOC_MATH_HALF_PI;
    theta -= (float)((int32_t)(theta * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
    if (theta < 0.0f) {
        theta += FOC_MATH_2PI;
    }

    Foc_Core_GetDq(&data, theta, &id, &iq);
    g_foc_valpha = valpha;
    g_foc_vbeta  = vbeta;
    g_foc_vd     = valpha;
    g_foc_vq     = vbeta;
    g_foc_theta_rad = theta;
    g_foc_du = du;
    g_foc_dv = dv;
    g_foc_dw = dw;
    g_foc_id_ma = id * 1000.0f;
    g_foc_iq_ma = iq * 1000.0f;
}

static void Dcal24_StopOutput(void)
{
    if (g_foc_active) {
        g_foc_active = 0u;
        Foc_Core_PwmStop();
        Foc_Core_SetStateMachine(FOC_STATE_IDLE);
    }
    g_foc_align_state = 0u;
}

void Foc_Dcal24_Start(void)
{
    Foc_Core_ClearFault();
    Dcal24_ClearRunState();

    g_foc_mode        = FOC_MODE_ALIGN;
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

    g_dcal24_running = 1u;
    g_dcal24_state   = DCAL24_STEP_ZERO;
    Dcal24_ZeroVector();
    Foc_Core_PwmStart();
    DCAL24_LOG("start zero=%ums beta=%ums alpha=%ums volt=%dmV",
               (unsigned)DCAL24_ZERO_TOTAL * 1000u / (unsigned)FOC_ISR_HZ,
               (unsigned)DCAL24_BETA_MS, (unsigned)DCAL24_ALPHA_MS,
               (int)(g_dcal24_volt_v * 1000.0f));
}

void Foc_Dcal24_Stop(void)
{
    if (!g_dcal24_running) {
        return;
    }

    g_dcal24_running = 0u;
    Dcal24_StopOutput();
    DCAL24_LOG("stopped");
}

uint8_t Foc_Dcal24_GetResult(foc_dcal24_result_t *result)
{
    if ((result == NULL) || (s_result.valid == 0u)) {
        return 0u;
    }

    *result = s_result;
    return 1u;
}

void Foc_Dcal24_Step(const stc_i_data_t *pData)
{
    int32_t encoder_dir;

    if (Foc_Core_OverCurrent(pData)) {
        g_dcal24_state = DCAL24_STEP_FAULT_OC;
        g_dcal24_evt   = DCAL24_EVT_OC;
        g_dcal24_running = 0u;
        Foc_Core_FaultStop(1u);
        return;
    }

    switch (g_dcal24_state) {
    case DCAL24_STEP_ZERO:
        Dcal24_ZeroVector();
        if (s_zero_tick >= DCAL24_ZERO_SKIP_SAMPLES) {
            s_zero_sum_u += pData->i16IU_mA;
            s_zero_sum_v += pData->i16IV_mA;
            s_zero_sum_w += pData->i16IW_mA;
        }
        if (++s_zero_tick >= DCAL24_ZERO_TOTAL) {
            s_result.valid     = 0u;
            s_result.offset    = 0;
            s_result.zero_u_ma = (float)s_zero_sum_u / (float)DCAL24_ZERO_AVG_SAMPLES;
            s_result.zero_v_ma = (float)s_zero_sum_v / (float)DCAL24_ZERO_AVG_SAMPLES;
            s_result.zero_w_ma = (float)s_zero_sum_w / (float)DCAL24_ZERO_AVG_SAMPLES;
            g_dcal24_zero_u_ma = s_result.zero_u_ma;
            g_dcal24_zero_v_ma = s_result.zero_v_ma;
            g_dcal24_zero_w_ma = s_result.zero_w_ma;
            s_zero_tick = 0u;
            s_phase_tick = 0u;
            g_dcal24_state = DCAL24_STEP_BETA;
            g_dcal24_evt   = DCAL24_EVT_ZERO_DONE;
        }
        break;

    case DCAL24_STEP_BETA:
        Dcal24_OutputField(pData, FOC_MATH_HALF_PI);
        if (++s_phase_tick >= ((uint32_t)DCAL24_BETA_MS * FOC_ISR_HZ / 1000u)) {
            g_dcal24_beta_hw = TMRA_GetCountValue(CM_TMRA_1);
            s_phase_tick = 0u;
            g_dcal24_state = DCAL24_STEP_ALPHA;
            g_dcal24_evt   = DCAL24_EVT_BETA_DONE;
        }
        break;

    case DCAL24_STEP_ALPHA:
        Dcal24_OutputField(pData, 0.0f);
        if (++s_phase_tick >= ((uint32_t)DCAL24_ALPHA_MS * FOC_ISR_HZ / 1000u)) {
            g_dcal24_alpha_hw = TMRA_GetCountValue(CM_TMRA_1);
            g_dcal24_moved = (int16_t)(g_dcal24_alpha_hw - g_dcal24_beta_hw);
            encoder_dir = (int32_t)g_foc_enc_dir;
            g_dcal24_offset = Foc_Core_ModPos((int32_t)g_dcal24_alpha_hw * encoder_dir,
                                              (int32_t)ENCODER_CPR);
            g_dcal24_evt = DCAL24_EVT_DONE;

            s_result.valid  = 1u;
            s_result.offset = g_dcal24_offset;
            Foc_Core_SetAlignOffset(g_dcal24_offset);
            g_dcal24_state = DCAL24_STEP_DONE;
            g_dcal24_running = 0u;
            Dcal24_StopOutput();
        }
        break;

    case DCAL24_STEP_DONE:
    case DCAL24_STEP_FAULT_OC:
    case DCAL24_STEP_IDLE:
    default:
        break;
    }
}
