/**
 *******************************************************************************
 * @file  foc_iq_pi.h
 * @brief FOC 模式31 — 编码器转子角度 PI 电流环 (comm_mode 31)。
 *
 *        前置条件：先运行 mode 30 (ZIZENG) 完成偏移锁定，使编码器电角度
 *        绝对化（扣除 ZIZENG 锁定的基线偏移），本模式即可直接闭环。
 *
 *        控制流程（20 kHz ISR）：
 *          TIMERA_1 硬件计数 -> 转子电角度（扣 ZIZENG 偏移） -> Park
 *          -> id PI(id_ref=0) / iq PI(iq_ref) -> InvPark -> SVPWM
 *
 *        启动时复用 foc_calib 零偏自校准窗口（~210ms 零矢量），
 *        锁定后 iq_ref 从 0 软启动斜坡至目标值。
 *
 *        ISR 约束：短小、无阻塞、无打印、无 malloc。
 *******************************************************************************
 */

#ifndef __FOC_IQ_PI_H__
#define __FOC_IQ_PI_H__

#include <stdint.h>
#include "foc_core.h"
#include "../Utils/dev_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * PI 初始参数（编译期宏，Keil Watch 中可运行时实时覆盖）
 *   kp 参考模式22 电流环整定值（Rs=0.1Ω, Ls=42.3µH, 20kHz -> Kp≈0.1）；
 *   ki 初始为 0（纯 P 起步），稳定后经 Watch 逐渐加大
 * ==========================================================================*/
#define IQPI_PI_KP           1.00f     /* d/q 轴 PI 比例增益 */
#define IQPI_PI_KI           0.0f      /* d/q 轴 PI 积分增益（初始 0，纯 P 控制起步） */
#define IQPI_PI_UMAX_V       1.0f      /* PI 输出限幅 (V)，|vd/vq| ≤ UMAX */
#define IQPI_INTEGRAL_MAX    0.5f      /* 积分抗饱和限幅 */

/* iq 目标初值 (mA) 与软启动斜率 (mA/s)，Watch 运行时可改 g_iqpi_iq_ref_ma */
#define IQPI_IQ_REF_MA       500.0f
#define IQPI_IQ_RAMP_MA_S    1000.0f

/* ============================================================================
 * Keil Watch 可调变量 / 观测量（定义见 foc_iq_pi.c）
 * ==========================================================================*/
extern volatile uint8_t g_iqpi_running;        /* 1 = 模式31 运行中 */
extern volatile float   g_iqpi_iq_ref_ma;      /* iq 目标 (mA)，Watch 可实时修改 */
extern volatile float   g_iqpi_iq_ref_ramp_ma; /* 斜坡后的实际 iq 参考 (mA) */
extern volatile float   g_iqpi_iq_ramp_ma_s;   /* iq 斜率 (mA/s)，Watch 可调 */
extern volatile float   g_iqpi_theta_rad;      /* 当前使用的转子电角度 (rad) */

/* d/q 轴 PI 配置（字段 volatile，Watch 可实时改 kp/ki/限幅） */
extern pid_config_t g_iqpi_pid_id_cfg;
extern pid_config_t g_iqpi_pid_iq_cfg;

/*******************************************************************************
 * API
 ******************************************************************************/

/* PI 实例绑定配置（Foc_Init 调用一次） */
void Foc_IqPi_InitPids(void);

/* 启动模式31：需先跑过 mode 30 锁定 ZIZENG 偏移，否则拒绝启动。
 * 内部：清故障 -> PWM 启动（零矢量）-> foc_calib 校准窗口 -> 闭环。 */
void Foc_StartIqPi(void);

/* 停止模式31 */
void Foc_StopIqPi(void);

/* 模式31 单步运算（20 kHz ISR 中由 Foc_Isr 分发调用） */
void Foc_IqPi_Step(const stc_i_data_t *pData);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_IQ_PI_H__ */
