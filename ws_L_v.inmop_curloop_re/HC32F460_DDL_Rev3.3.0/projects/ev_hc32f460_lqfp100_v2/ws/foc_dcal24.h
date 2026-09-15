/**
 *******************************************************************************
 * @file  foc_dcal24.h
 * @brief FOC mode 24 - independent calibration-only DCAL24.
 *
 * Mode 24 owns its own zero-current window, BETA alignment, ALPHA alignment,
 * over-current path, and completion event.  The finished calibration snapshot
 * is copied by value to mode 29; no run-time mode state is shared.
 *******************************************************************************
 */

#ifndef __FOC_DCAL24_H__
#define __FOC_DCAL24_H__

#include <stdint.h>
#include "foc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DCAL24_DBG   1
#if DCAL24_DBG
#define DCAL24_LOG(fmt, ...)  MAIN_D("[DCAL24] " fmt, ##__VA_ARGS__)
#else
#define DCAL24_LOG(fmt, ...)  ((void)0)
#endif

#define DCAL24_ZERO_SKIP_SAMPLES  200u
#define DCAL24_ZERO_AVG_SAMPLES   4000u
#define DCAL24_BETA_MS            2000u
#define DCAL24_ALPHA_MS           2000u

#define DCAL24_STEP_IDLE       0u
#define DCAL24_STEP_ZERO       1u
#define DCAL24_STEP_BETA       2u
#define DCAL24_STEP_ALPHA      3u
#define DCAL24_STEP_DONE       4u
#define DCAL24_STEP_FAULT_OC   5u

#define DCAL24_EVT_ZERO_DONE   1u
#define DCAL24_EVT_BETA_DONE   2u
#define DCAL24_EVT_DONE        3u
#define DCAL24_EVT_OC          4u

typedef struct {
    uint8_t valid;
    int32_t offset;
    float   zero_u_ma;
    float   zero_v_ma;
    float   zero_w_ma;
} foc_dcal24_result_t;

extern volatile uint8_t  g_dcal24_running;
extern volatile uint8_t  g_dcal24_state;
extern volatile uint8_t  g_dcal24_evt;
extern volatile uint16_t g_dcal24_beta_hw;
extern volatile uint16_t g_dcal24_alpha_hw;
extern volatile int32_t  g_dcal24_moved;
extern volatile int32_t  g_dcal24_offset;
extern volatile float    g_dcal24_zero_u_ma;
extern volatile float    g_dcal24_zero_v_ma;
extern volatile float    g_dcal24_zero_w_ma;
extern volatile float    g_dcal24_volt_v;

void Foc_Dcal24_Start(void);
void Foc_Dcal24_Stop(void);
void Foc_Dcal24_Step(const stc_i_data_t *pData);
uint8_t Foc_Dcal24_GetResult(foc_dcal24_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_DCAL24_H__ */
