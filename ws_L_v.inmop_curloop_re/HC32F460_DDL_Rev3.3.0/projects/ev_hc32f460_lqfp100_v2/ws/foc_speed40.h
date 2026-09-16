/**
 *******************************************************************************
 * @file  foc_speed40.h
 * @brief FOC mode 40 - encoder FOC with cascade speed/current loops.
 *
 * The speed PI runs on a fixed 5 ms tick and generates the q-axis current
 * reference.  The id reference is zero.  The independent id/iq current PIs
 * run every current-sample ISR.  Mode 40 requires a valid mode 24 calibration
 * before start.
 *******************************************************************************
 */

#ifndef __FOC_SPEED40_H__
#define __FOC_SPEED40_H__

#include <stdint.h>
#include "foc_core.h"
#include "../Utils/dev_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPEED40_DBG   1
#if SPEED40_DBG
#define SPEED40_LOG(fmt, ...)  MAIN_D("[SPEED40] " fmt, ##__VA_ARGS__)
#else
#define SPEED40_LOG(fmt, ...)  ((void)0)
#endif

/* Inner current PI: start from the verified mode 29 values. */
#define SPEED40_PI_KP             0.5f
#define SPEED40_PI_KI             300.0f
#define SPEED40_PI_UMAX_V         3.5f
#define SPEED40_ITERM_MAX_V       3.2f

/* Outer speed PI output is a signed q-axis current reference in mA. */
#define SPEED40_SPD_KP_MA_PER_RPM       0.0005f
#define SPEED40_SPD_KI_MA_PER_RPM_S     0.020f
#define SPEED40_SPD_IQ_LIMIT_MA         ((float)FOC_MOTOR_RATED_CURRENT_A \
                                        * 1000.0f * 0.20f)

/* Safety envelope derived from motor_config.h. */
#define SPEED40_SPEED_REF_LIMIT_RPM     ((float)FOC_MOTOR_MAX_SPEED_RPM)
#define SPEED40_ACCEL_LIMIT_RPM_S       ((float)FOC_MOTOR_MAX_SPEED_RPM \
                                        * 0.25f)

#define SPEED40_SPD_WIN_MS              5u
#define SPEED40_SPD_WIN_US              (SPEED40_SPD_WIN_MS * 1000u)
#define SPEED40_SPD_FILT_ALPHA          0.25f
#define SPEED40_ENC_DELTA_MAX           32

#define SPEED40_STEP_IDLE               0u
#define SPEED40_STEP_RUN                1u
#define SPEED40_STEP_FAULT_OC           2u

#define SPEED40_EVT_OC                  1u

extern volatile uint8_t  g_speed40_running;
extern volatile uint8_t  g_speed40_state;
extern volatile uint8_t  g_speed40_evt;
extern volatile float    g_speed40_speed_target_rpm;
extern volatile float    g_speed40_speed_ramp_rpm;
extern volatile float    g_speed40_speed_meas_rpm;
extern volatile float    g_speed40_speed_filt_rpm;
extern volatile float    g_speed40_speed_err_rpm;
extern volatile float    g_speed40_speed_out_ma;
extern volatile int32_t  g_speed40_rotor_count;
extern volatile int32_t  g_speed40_rotor_deg;
extern volatile float    g_speed40_id_ref_ma;
extern volatile float    g_speed40_iq_ref_ma;
extern volatile float    g_speed40_id_ma;
extern volatile float    g_speed40_iq_ma;
extern volatile float    g_speed40_vd;
extern volatile float    g_speed40_vq;
extern volatile uint8_t  g_speed40_vsat;
extern volatile float    g_speed40_du;
extern volatile float    g_speed40_dv;
extern volatile float    g_speed40_dw;

extern pid_config_t g_speed40_pid_speed_cfg;
extern pid_config_t g_speed40_pid_id_cfg;
extern pid_config_t g_speed40_pid_iq_cfg;

void Foc_Speed40_InitPids(void);
void Foc_Speed40_SetTargetRPM(float target_rpm);
void Foc_Speed40_Start(void);
void Foc_Speed40_Stop(void);
void Foc_Speed40_Step(const stc_i_data_t *pData);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_SPEED40_H__ */
