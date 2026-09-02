/**
 *******************************************************************************
 * @file  foc_openloop.c
 * @brief FOC 模式21 — 电压开环运行实现（自 foc.c 原样迁移）。
 *******************************************************************************
 */

#include "foc_openloop.h"
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "motor_config.h"

/*******************************************************************************
 * Foc_StartOpenLoop - mode 21: voltage open loop
 ******************************************************************************/
void Foc_StartOpenLoop(void)
{
    g_foc_theta_rad = 0.0f;
    Foc_Core_ClearFault();
    if (g_foc_openloop_volt_v <= 0.0f) {
        g_foc_openloop_volt_v = FOC_OPENLOOP_VOLT_V;
    }
    g_foc_mode        = FOC_MODE_OPENLOOP;
    g_foc_phase       = 0u;
    Foc_Core_SetStateMachine(FOC_STATE_IDLE);
    g_foc_align_state = 0u;

    Foc_Core_PwmStart();
}

/*******************************************************************************
 * Foc_OpenLoop_Step - theta 积分 + SVPWM 输出（20 kHz）
 ******************************************************************************/
void Foc_OpenLoop_Step(void)
{
    float theta, valpha, vbeta;
    float du, dv, dw;

    theta = g_foc_theta_rad
          + (FOC_MATH_2PI * g_foc_openloop_freq_hz / (float)FOC_ISR_HZ);
    if (theta >= FOC_MATH_2PI) {
        theta -= FOC_MATH_2PI;
    }
    g_foc_theta_rad = theta;

    valpha = g_foc_openloop_volt_v * Foc_Math_Cos(theta);
    vbeta  = g_foc_openloop_volt_v * Foc_Math_Sin(theta);

    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);

    g_foc_valpha = valpha;
    g_foc_vbeta  = vbeta;
    g_foc_du = du;
    g_foc_dv = dv;
    g_foc_dw = dw;
}
