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

    RUNNER_DBG("Init done, freq=%u Hz", cfg->pwm_freq_hz);
}

/*=============================================================================
 * Stop every FOC mode owned by CommRunner. Stop calls are no-ops for modules
 * that are not active, so each mode switch can use the same explicit cleanup.
 *=============================================================================*/
static void CommRunner_StopFocModes(void)
{
    if (g_cal_running) {
        Foc_Cal_Stop();
    }
    if (g_calang_running) {
        Foc_CalAngle_Stop();
    }
    if (g_olf_running) {
        Foc_Olf_Stop();
    }
    if (g_dcl_running) {
        Foc_Dcl_Stop();
    }
    if (g_dcal24_running) {
        Foc_Dcal_Stop();
    }
    if (g_dci_running) {
        Foc_Dci_Stop();
    }
    if (g_drun29_running) {
        Foc_Drun_Stop();
    }
    if (g_drun41_running) {
        Foc_Drun41_Stop();
    }
    if (g_speed40_running) {
        Foc_Speed_Stop();
    }
    if (g_smo45_running) {
        Foc_Smo45_Stop();
    }
    if (g_foc_mode == FOC_MODE_ALIGN) {
        Foc_Stop();
    }
    if (g_zizeng_running) {
        Foc_Ramp_Stop();
    }
    if (g_lockiq_running) {
        Foc_LockIqPi_Stop();
    }
    if (g_iqpi_running) {
        Foc_IqPi_Stop();
    }
    if (g_rs51_running) {
        Foc_RsId_Stop();
    }
    if (g_ldlq52_running) {
        Foc_LdLqId_Stop();
    }
    if (g_flx53_running) {
        Foc_FlxId_Stop();
    }
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
        CommRunner_StopFocModes();
        Commutation_Stop();
        TMR4_PWM_StartOutput();   /* keep VALLEY/PEAK ADC trigger alive in mode 0 */
        RUNNER_DBG("STOP");
        break;

    case COMM_RUNNER_CAL:
        CommRunner_StopFocModes();
        Commutation_Stop();
        Foc_Cal_Start();
        RUNNER_DBG("CAL (mode 20)");
        break;

    case COMM_RUNNER_CAL_ANGLE:
        CommRunner_StopFocModes();
        Commutation_Stop();
        Foc_CalAngle_Start();
        RUNNER_DBG("CAL_ANGLE (mode 25)");
        break;

    case COMM_RUNNER_OLF:
        CommRunner_StopFocModes();
        Commutation_Stop();
        Foc_Olf_Start();
        RUNNER_DBG("OLF (mode 26)");
        break;

    case COMM_RUNNER_ZIZENG:
        CommRunner_StopFocModes();
        Commutation_Stop();
        Foc_Ramp_Start();
        RUNNER_DBG("ZIZENG (mode 30)");
        break;

    case COMM_RUNNER_IQ_PI:
        CommRunner_StopFocModes();
        Commutation_Stop();
        Foc_IqPi_Start();
        RUNNER_DBG("IQ_PI (mode 31)");
        break;

    case COMM_RUNNER_LOCK_IQ_PI:
        CommRunner_StopFocModes();
        Commutation_Stop();
        Foc_LockIqPi_Start();
        RUNNER_DBG("LOCK_IQ_PI (mode 32)");
        break;

    case COMM_RUNNER_DCL:
        CommRunner_StopFocModes();
        Commutation_Stop();
        Foc_Dcl_Start();
        RUNNER_DBG("DCL (mode 27)");
        break;

    case COMM_RUNNER_DCI:
        CommRunner_StopFocModes();
        Commutation_Stop();
        Foc_Dci_Start();
        RUNNER_DBG("DCI (mode 28)");
        break;

    case COMM_RUNNER_DCAL:
        CommRunner_StopFocModes();
        Commutation_Stop();
        Foc_Dcal_Start();
        RUNNER_DBG("DCAL (mode 24, cal-only)");
        break;

    case COMM_RUNNER_DRUN:
        CommRunner_StopFocModes();
        Commutation_Stop();
        Foc_Drun_Start();
        RUNNER_DBG("DRUN (mode 29, run-only)");
        break;

    case COMM_RUNNER_DRUN41:
        CommRunner_StopFocModes();
        Commutation_Stop();
        Foc_Drun41_Start();
        RUNNER_DBG("DRUN41 (mode 41, current FF)");
        break;

    case COMM_RUNNER_SPEED_FOC:
        CommRunner_StopFocModes();
        Commutation_Stop();
        Foc_Speed_Start();
        RUNNER_DBG("SPEED_FOC (mode 40)");
        break;

    case COMM_RUNNER_SMO45:
        CommRunner_StopFocModes();
        Commutation_Stop();
        Foc_Smo45_Start();
        RUNNER_DBG("SMO45 (mode 45)");
        break;

    case COMM_RUNNER_RS51:
        CommRunner_StopFocModes();
        Commutation_Stop();
        Foc_RsId_Start();
        RUNNER_DBG("RS51 (mode 51, Rs identification)");
        break;

    case COMM_RUNNER_LDLQ52:
        CommRunner_StopFocModes();
        Commutation_Stop();
        Foc_LdLqId_Start();
        RUNNER_DBG("LDLQ52 (mode 52, Ld/Lq identification)");
        break;

    case COMM_RUNNER_FLX53:
        CommRunner_StopFocModes();
        Commutation_Stop();
        Foc_FlxId_Start();
        RUNNER_DBG("FLX53 (mode 53, flux identification)");
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
 * CommRunner_ReleaseMode — 只把模式号放回 STOP（供自终止模式跑完后调用）
 *   动机：SetMode() 开头有 "同模式早退"，跑完后 s_mode 仍停在原模式号，
 *   于是 Keil 里再写同一个号不会重新启动。把它放回 STOP 即可重进；
 *   不动 FOC 模块状态，所以 VOFA 布局与观测量都保留。
 *=============================================================================*/
void CommRunner_ReleaseMode(void)
{
    s_mode = COMM_RUNNER_STOP;
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
 * CommRunner_Update — 几乎空操作（FOC 模式与 mode 30 都由 Foc_Isr 驱动）
 *=============================================================================*/
void CommRunner_Update(void)
{
    if (!s_initialized) return;
    /* FOC 模式与 mode 30 完全由 Foc_Isr 驱动，主循环无事可做 */
}

/*=============================================================================
 * 桩函数（保留接口，供外部调用）
 *=============================================================================*/
float CommRunner_GetRPM(void) { return 0.0f; }
uint8_t CommRunner_IsRunning(void) { return 0; }
uint8_t CommRunner_IsStalled(void) { return 0; }
uint8_t CommRunner_CurLoopActive(void) { return 0; }
void CommRunner_SetTargetRPM(float rpm)
{
    Foc_Speed_SetTargetRPM(rpm);
}

float CommRunner_GetTargetRPM(void)
{
    return g_speed40_speed_target_rpm;
}
