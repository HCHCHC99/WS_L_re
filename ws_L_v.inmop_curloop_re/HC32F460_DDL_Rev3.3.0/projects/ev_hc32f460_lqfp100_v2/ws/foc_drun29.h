/**
 *******************************************************************************
 * @file  foc_drun29.h
 * @brief FOC mode 29 - independent run-only current loop.
 *
 * Mode 29 copies the calibration result produced by mode 24, samples the live
 * rotor angle when it starts, and then owns its entire 20 kHz current loop.
 *******************************************************************************
 */

#ifndef __FOC_DRUN29_H__
#define __FOC_DRUN29_H__

#include <stdint.h>
#include "foc_core.h"
#include "../Utils/dev_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DRUN29_DBG   1
#if DRUN29_DBG
#define DRUN29_LOG(fmt, ...)  MAIN_D("[DRUN29] " fmt, ##__VA_ARGS__)
#else
#define DRUN29_LOG(fmt, ...)  ((void)0)
#endif

#define DRUN29_PI_KP             0.5f
#define DRUN29_PI_KI             300.0f
#define DRUN29_PI_UMAX_V         3.5f
#define DRUN29_ITERM_MAX_V       3.2f
#define DRUN29_I_REF_MA          500.0f
#define DRUN29_I_RAMP_MA_S       0.0f
#define DRUN29_ENC_DELTA_MAX     32

#define DRUN29_SPEED_WIN_MS      200u
#define DRUN29_PP_WIN_MS         5000u
#define DRUN29_MEAN_WIN_MS       3000u
#define DRUN29_ERR_WIN_MS        5000u

#define DRUN29_STEP_IDLE         0u
#define DRUN29_STEP_RUN          1u
#define DRUN29_STEP_FAULT_OC     2u

#define DRUN29_EVT_RAMP_DONE     1u
#define DRUN29_EVT_OC            2u

extern volatile uint8_t  g_drun29_running;
extern volatile uint8_t  g_drun29_state;
extern volatile uint8_t  g_drun29_evt;
extern volatile float    g_drun29_dlt_init_deg;
extern volatile float    g_drun29_dlt_targ_deg;
extern volatile uint32_t g_drun29_dlt_tr_ms;
extern volatile float    g_drun29_dlt_now_deg;
extern volatile float    g_drun29_speed_hz;
extern volatile int32_t  g_drun29_field_deg;
extern volatile int32_t  g_drun29_rotor_deg;
extern volatile int32_t  g_drun29_diff_deg;
extern volatile int32_t  g_drun29_rotor_count;
extern volatile float    g_drun29_id_ma;
extern volatile float    g_drun29_iq_ma;
extern volatile float    g_drun29_id_pp_ma;
extern volatile float    g_drun29_iq_pp_ma;
extern volatile float    g_drun29_id_mean_ma;
extern volatile float    g_drun29_iq_mean_ma;
extern volatile float    g_drun29_i_ref_ma;
extern volatile float    g_drun29_id_ref_ma;
extern volatile float    g_drun29_iq_ref_ma;
extern volatile float    g_drun29_i_ramp_ma_s;
extern volatile uint8_t  g_drun29_vsat;
extern volatile float    g_drun29_ed_mean_ma;
extern volatile float    g_drun29_eq_mean_ma;
extern volatile float    g_drun29_ed_pp_ma;
extern volatile float    g_drun29_eq_pp_ma;
extern volatile float    g_drun29_du;
extern volatile float    g_drun29_dv;
extern volatile float    g_drun29_dw;

extern pid_config_t g_drun29_pid_id_cfg;
extern pid_config_t g_drun29_pid_iq_cfg;

void Foc_Drun29_InitPids(void);
void Foc_Drun29_Start(void);
void Foc_Drun29_Stop(void);
void Foc_Drun29_Step(const stc_i_data_t *pData);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_DRUN29_H__ */
