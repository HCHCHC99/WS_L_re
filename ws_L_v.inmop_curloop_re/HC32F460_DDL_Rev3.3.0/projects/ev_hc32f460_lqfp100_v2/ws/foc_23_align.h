/**
 *******************************************************************************
 * @file  foc_23_align.h
 * @brief FOC 模式23 — 静止电角度对齐校准 (comm_mode 23)。
 *
 *        以小电流将转子锁定到 d 轴，等待编码器稳定后记录电零点 offset，
 *        保持一段时间后释放。事件通过 g_foc_align_evt + v1/v2/v3 上报，
 *        由主循环打印（ISR 内不做打印）。
 *
 * ============================ 模式速览卡（唯一事实源）========================
 * 模式：23 = 静止电角度对齐校准（老版；新流程请用 mode 20 / mode 24）
 * 入口：comm_mode = 23
 * 前置：无（自带对齐过程）
 * 结束：保持一段时间后自动释放
 *
 * 【Watch 可调变量】（名称 = 单位）
 *   g_foc_align_volt_v   V    对齐固定电压（锁定力矩大小）
 *
 * 【关键观察变量】
 *   g_foc_align_evt      事件：1=start 2=beta done 3=locked 4=done 5=fault
 *   g_foc_align_evt_v1/v2/v3  事件附带数据（含义随事件类型）
 *   g_foc_elec_deg       对齐后转子电角度（deg，[0, 360×极对数)）
 *
 * 【VOFA 通道】mode 23 **无专属布局，回落通用布局（20ch）**：
 *   通道定义见 main.c 的 Foc_Common_VofaFill（ch0~2 三相电流 / ch3~5 零偏 /
 *   ch6~7 静止系 / ch8~11 id-iq-vd-vq / ch12 幅值 / ch13~14 角度 /
 *   ch15 零点 / ch16 故障 / ch17~18 ZIZENG 兼容 / ch19 OC 阈值）。
 *   ⚠ 本模式无专属语义，判读请用 g_foc_align_evt 事件 + Watch 变量。
 * ===========================================================================
 */

#ifndef __FOC_23_ALIGN_H__
#define __FOC_23_ALIGN_H__

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

#endif /* __FOC_23_ALIGN_H__ */
