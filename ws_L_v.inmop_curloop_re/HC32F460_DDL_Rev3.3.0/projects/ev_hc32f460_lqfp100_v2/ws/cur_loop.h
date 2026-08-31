/**
 *******************************************************************************
 * @file  cur_loop.h
 * @brief Current PI loop — 精简版：保留接口，但功能已禁用
 *        （电流采集由 I.c 继续工作，电流环 PI 控制被禁用）
 *******************************************************************************
 */

#ifndef __CUR_LOOP_H__
#define __CUR_LOOP_H__

#include "../Utils/dev_pid.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Keil Watch: current setpoint (mA, signed) */
extern volatile float g_i_ref_ma;

/* J-Scope observability (updated in ADC ISR at control rate) */
extern volatile float g_scope_i_ref;
extern volatile float g_scope_i_fb;
extern volatile float g_scope_i_fb_raw;
extern volatile float g_scope_i_duty;
extern volatile float g_scope_i_err;
extern volatile float g_scope_i_ol;
extern volatile uint32_t g_scope_i_dt_us;

/* Feedback smoothing (Keil Watch tunable): 1.0 = off, 0.1 = heavy */
extern volatile float g_cur_fb_alpha;

/* Debug print interval (ms) for [CURLOOP] log; 0 = off */
extern volatile uint32_t g_cur_dbg_ms;

/* Current-loop PID config (volatile, Keil Watch tunable) */
extern pid_config_t g_cur_pid_cfg;

/* Register ADC EOCB callback. Call once after I_Init(). */
void CurLoop_Init(void);

void  CurLoop_SetRef(float ma);
float CurLoop_GetRef(void);

/* Cascade mode: current setpoint comes from g_cur_ref_ext_ma (no soft-start ramp).
 * Handshake: write g_cur_ref_ext_ma first, then CurLoop_SetExternalRef(true);
 * to exit, call CurLoop_SetExternalRef(false) first, then restore g_i_ref_ma. */
void CurLoop_SetExternalRef(bool enable);
extern volatile float g_cur_ref_ext_ma;

#ifdef __cplusplus
}
#endif

#endif /* __CUR_LOOP_H__ */
