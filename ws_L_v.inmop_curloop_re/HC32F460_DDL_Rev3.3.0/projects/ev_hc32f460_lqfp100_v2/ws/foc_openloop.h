/**
 *******************************************************************************
 * @file  foc_openloop.h
 * @brief FOC 模式21 — 电压开环运行 (comm_mode 21)。
 *
 *        theta 每个 ISR 按 g_foc_openloop_freq_hz 积分，电压矢量
 *        (g_foc_openloop_volt_v) 经 SVPWM 变换为 U/V/W duty 输出。
 *******************************************************************************
 */

#ifndef __FOC_OPENLOOP_H__
#define __FOC_OPENLOOP_H__

#include <stdint.h>
#include "foc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start open-loop FOC (comm_mode 21): reset theta, reconfigure TMR4 to
 * complementary, enable output, set active. */
void Foc_StartOpenLoop(void);

/* 模式21 单步运算（20 kHz ISR 中由 Foc_Isr 分发调用） */
void Foc_OpenLoop_Step(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_OPENLOOP_H__ */
