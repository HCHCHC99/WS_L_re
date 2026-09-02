/**
 *******************************************************************************
 * @file  foc_align.h
 * @brief FOC 模式23 — 静止电角度对齐校准 (comm_mode 23)。
 *
 *        以小电流将转子锁定到 d 轴，等待编码器稳定后记录电零点 offset，
 *        保持一段时间后释放。事件通过 g_foc_align_evt + v1/v2/v3 上报，
 *        由主循环打印（ISR 内不做打印）。
 *******************************************************************************
 */

#ifndef __FOC_ALIGN_H__
#define __FOC_ALIGN_H__

#include <stdint.h>
#include "foc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Align calibration (mode 23) */
extern volatile float   g_foc_align_volt_v; /* fixed align voltage (V), Watch tunable */

/* Align calibration events (main loop prints; ISR only sets flag+payload) */
extern volatile uint8_t g_foc_align_evt;    /* 1=start 2=beta done 3=locked 4=done 5=fault */
extern volatile int32_t g_foc_align_evt_v1;
extern volatile int32_t g_foc_align_evt_v2;
extern volatile int32_t g_foc_align_evt_v3;

/* Start standstill electrical alignment (comm_mode 23): lock rotor to the
 * d-axis with a small current, record the encoder electrical zero, release. */
void Foc_StartAlign(void);

/* 模式23 单步运算（20 kHz ISR 中由 Foc_Isr 分发调用） */
void Foc_Align_Step(const stc_i_data_t *pData);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_ALIGN_H__ */
