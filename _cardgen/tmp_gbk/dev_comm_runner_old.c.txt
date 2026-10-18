#include "dev_comm_runner.h"
#include "motor_config.h"
#include "dev_commutation.h"
#include "tmr4_pwm.h"
#include "timer6_timebase.h"
#include "rtt_log.h"
#include "TickTimer.h"
#include <string.h>
#include "foc.h"

/*=============================================================================
 * State
 *=============================================================================*/
static comm_runner_config_t s_cfg;
static comm_runner_mode_t   s_mode     = COMM_RUNNER_STOP;
static float                s_duty     = 80.0f;
static uint8_t              s_initialized = 0;

/*=============================================================================
 * CommRunner_Init
 *=============================================================================*/
void CommRunner_Init(const comm_runner_config_t *cfg)
{
    int ch;
    if (!cfg) return;

    memcpy(&s_cfg, cfg, sizeof(comm_runner_config_t));
    s_duty = cfg->default_duty_pct;

    /* ---- PWM ---- */
    tmr4_pwm_config_t pwm_cfg = {
        .output_type_u = TMR4_OUTPUT_SYNC,
        .output_type_v = TMR4_OUTPUT_SYNC,
        .output_type_w = TMR4_OUTPUT_SYNC,
        .freq_hz       = cfg->pwm_freq_hz,
        .dead_time_ns  = 0,
        .active_high   = true,
    };
    TMR4_PWM_Config(&pwm_cfg);
    TMR4_PWM_StartOutput();

    /* ---- Timebase ---- */
    Timer6_Timebase_Init();
    Timer6_Timebase_Start();

    /* ---- 上电默认：全关（待机安全） ---- */
    for (ch = 0; ch < 3; ch++) {
        TMR4_PWM_SetChannelMode((tmr4_pwm_channel_t)ch, TMR4_MODE_OFF, 0.0f);
    }

    if (cfg->on_init_done) {
        cfg->on_init_done();
    }

    s_mode = COMM_RUNNER_STOP;
    s_initialized = 1;

    MAIN_D("[CommRunner] Init done (MINIMAL: only mode 23 and 30), freq=%u Hz", cfg->pwm_freq_hz);
}

/*=============================================================================
 * CommRunner_SetMode
 *=============================================================================*/
void CommRunner_SetMode(comm_runner_mode_t mode)
{
    if (!s_initialized) return;
    if (mode == s_mode) return;

    s_mode = mode;

    switch (mode) {
    case COMM_RUNNER_STOP:
        Commutation_Stop();
        if (g_foc_mode == FOC_MODE_ALIGN) {
            Foc_Stop();
        }
        if (g_zizeng_running) {
            Foc_StopZizeng();
        }
        MAIN_D("[CommRunner] STOP");
        break;

    case COMM_RUNNER_FOC_ALIGN:
        if (g_zizeng_running) {
            Foc_StopZizeng();
        }
        Commutation_Stop();
        Foc_StartAlign();
        MAIN_D("[CommRunner] FOC_ALIGN (mode 23)");
        break;

    case COMM_RUNNER_ZIZENG:
        if (g_foc_mode == FOC_MODE_ALIGN) {
            Foc_Stop();
        }
        Commutation_Stop();
        Foc_StartZizeng();
        MAIN_D("[CommRunner] ZIZENG (mode 30)");
        break;

    default:
        break;
    }
}

/*=============================================================================
 * CommRunner_GetMode
 *=============================================================================*/
comm_runner_mode_t CommRunner_GetMode(void)
{
    return s_mode;
}

/*=============================================================================
 * CommRunner_GetPwmFreqHz
 *=============================================================================*/
uint32_t CommRunner_GetPwmFreqHz(void)
{
    return s_cfg.pwm_freq_hz;
}

/*=============================================================================
 * CommRunner_SetDuty
 *=============================================================================*/
void CommRunner_SetDuty(float duty_pct)
{
    if (duty_pct < 2.0f) duty_pct = 2.0f;
    if (duty_pct > 98.0f) duty_pct = 98.0f;
    s_duty = duty_pct;
}

/*=============================================================================
 * CommRunner_GetDuty
 *=============================================================================*/
float CommRunner_GetDuty(void)
{
    return s_duty;
}

/*=============================================================================
 * CommRunner_Update — 几乎空操作（FOC ISR 驱动模式23和30）
 *=============================================================================*/
void CommRunner_Update(void)
{
    if (!s_initialized) return;
    /* 模式23和30完全由 Foc_Isr 驱动，主循环无事可做 */
}

/*=============================================================================
 * 桩函数（保留接口，供外部调用）
 *=============================================================================*/
float CommRunner_GetRPM(void) { return 0.0f; }
uint8_t CommRunner_IsRunning(void) { return 0; }
uint8_t CommRunner_IsStalled(void) { return 0; }
uint8_t CommRunner_CurLoopActive(void) { return 0; }
void CommRunner_SetTargetRPM(float rpm) { (void)rpm; }
float CommRunner_GetTargetRPM(void) { return 0.0f; }
