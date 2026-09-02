/**
 *******************************************************************************
 * @file  foc_scope.h
 * @brief MotorScope RTT 遥测 — FOC 状态变化/阈值触发的心跳上报（主循环调用）。
 *
 *        非实时路径：由主循环周期调用 Foc_RttSend()，仅当状态/观测量
 *        变化超过阈值或保活超时才发送，避免刷爆 RTT 通道。
 *******************************************************************************
 */

#ifndef __FOC_SCOPE_H__
#define __FOC_SCOPE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MotorScope RTT 总开关：0=关 1=开（Watch 可改） */
extern volatile int32_t g_motor_scope;

/* MotorScope RTT 心跳，主循环调用（us 时间戳） */
void Foc_RttSend(uint64_t now_us);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_SCOPE_H__ */
