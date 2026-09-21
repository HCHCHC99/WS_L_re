/**
 *******************************************************************************
 * @file  foc_drun41.h
 * @brief FOC mode 41 - current loop with dq feed-forward experiment.
 *
 * Mode 41 copies mode 29 and adds runtime-switchable BEMF/cross-coupling
 * feed-forward.  Disable the feed-forward flags for a mode29-equivalent A/B
 * baseline.  It reuses the mode 24 calibration result.
 *******************************************************************************
 */

#ifndef __FOC_DRUN41_H__
#define __FOC_DRUN41_H__

#include <stdint.h>
#include "foc_core.h"
#include "../Utils/dev_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DRUN41_DBG   1
#if DRUN41_DBG
#define DRUN41_LOG(fmt, ...)  MAIN_D("[DRUN41] " fmt, ##__VA_ARGS__)
#else
#define DRUN41_LOG(fmt, ...)  ((void)0)
#endif

#define DRUN41_PI_KP             0.5f
#define DRUN41_PI_KI             300.0f
#define DRUN41_PI_UMAX_V         3.5f
#define DRUN41_ITERM_MAX_V       3.2f
#define DRUN41_VMAX_DEFAULT_V    6.0f
#define DRUN41_IQ_FILT_ALPHA     0.10f
#define DRUN41_I_REF_MA          500.0f
#define DRUN41_I_RAMP_MA_S       0.0f
#define DRUN41_ENC_DELTA_MAX     32

#define DRUN41_SPEED_WIN_MS      200u
#define DRUN41_PP_WIN_MS         5000u
#define DRUN41_MEAN_WIN_MS       3000u
#define DRUN41_ERR_WIN_MS        5000u

#define DRUN41_STEP_IDLE         0u
#define DRUN41_STEP_RUN          1u
#define DRUN41_STEP_FAULT_OC     2u

#define DRUN41_EVT_RAMP_DONE     1u
#define DRUN41_EVT_OC            2u

#define DRUN41_STEP_DEADBAND_MA  50.0f
#define DRUN41_STEP_MIN_MA       100.0f
#define DRUN41_STEP_TIMEOUT_US   200000u
#define DRUN41_STEP_CONFIRM_TICK 3u
#define DRUN41_STEP_TIME_TIMEOUT 0xFFFFFFFFu

#define DRUN41_STEP_ST_IDLE      0u
#define DRUN41_STEP_ST_WAIT      1u
#define DRUN41_STEP_ST_DONE      2u
#define DRUN41_STEP_ST_TIMEOUT   3u

extern volatile uint8_t  g_drun41_running;
extern volatile uint8_t  g_drun41_state;
extern volatile uint8_t  g_drun41_evt;
extern volatile float    g_drun41_dlt_init_deg;
extern volatile float    g_drun41_dlt_targ_deg;
extern volatile uint32_t g_drun41_dlt_tr_ms;
extern volatile float    g_drun41_dlt_now_deg;
extern volatile float    g_drun41_speed_hz;
extern volatile int32_t  g_drun41_field_deg;
extern volatile int32_t  g_drun41_rotor_deg;
extern volatile int32_t  g_drun41_diff_deg;
extern volatile int32_t  g_drun41_rotor_count;
extern volatile float    g_drun41_id_ma;
extern volatile float    g_drun41_iq_ma;
extern volatile float    g_drun41_iq_filt_ma;
extern volatile float    g_drun41_iq_filt_alpha;
extern volatile float    g_drun41_id_pp_ma;
extern volatile float    g_drun41_iq_pp_ma;
extern volatile float    g_drun41_id_mean_ma;
extern volatile float    g_drun41_iq_mean_ma;
extern volatile float    g_drun41_i_ref_ma;
extern volatile float    g_drun41_id_ref_ma;
extern volatile float    g_drun41_iq_ref_ma;
extern volatile float    g_drun41_i_ramp_ma_s;
extern volatile uint8_t  g_drun41_vsat;
extern volatile float    g_drun41_ed_mean_ma;
extern volatile float    g_drun41_eq_mean_ma;
extern volatile float    g_drun41_ed_pp_ma;
extern volatile float    g_drun41_eq_pp_ma;
extern volatile float    g_drun41_du;
extern volatile float    g_drun41_dv;
extern volatile float    g_drun41_dw;
extern volatile uint8_t  g_drun41_ff_enable;
extern volatile float    g_drun41_ff_bemf_gain;
extern volatile float    g_drun41_ff_cross_gain;
extern volatile float    g_drun41_ff_res_gain;
extern volatile float    g_drun41_omega_e_rad_s;
extern volatile float    g_drun41_vd_pi_v;
extern volatile float    g_drun41_vq_pi_v;
extern volatile float    g_drun41_vd_ff_v;
extern volatile float    g_drun41_vq_ff_v;
extern volatile float    g_drun41_vmax_v;

/* Step-response instrumentation.  Times are measured from the first ISR where
 * the selected axis reference differs from its previous value by more than
 * DRUN41_STEP_DEADBAND_MA.  A crossing must remain present for
 * DRUN41_STEP_CONFIRM_TICK samples before it is accepted. */
extern volatile uint32_t g_drun41_time_us;
extern volatile float    g_drun41_step_frac_pct;
extern volatile uint8_t  g_drun41_id_step_state;
extern volatile uint8_t  g_drun41_iq_step_state;
extern volatile float    g_drun41_id_step_target_ma;
extern volatile float    g_drun41_iq_step_target_ma;
extern volatile uint32_t g_drun41_id_step_t90_us;
extern volatile uint32_t g_drun41_iq_step_t90_us;

extern pid_config_t g_drun41_pid_id_cfg;
extern pid_config_t g_drun41_pid_iq_cfg;

void Foc_Drun41_InitPids(void);
void Foc_Drun41_Start(void);
void Foc_Drun41_Stop(void);
void Foc_Drun41_Step(const stc_i_data_t *pData);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_DRUN41_H__ */
