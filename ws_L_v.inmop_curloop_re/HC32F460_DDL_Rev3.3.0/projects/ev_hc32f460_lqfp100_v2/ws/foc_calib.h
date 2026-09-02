/**
 *******************************************************************************
 * @file  foc_calib.h
 * @brief FOC 相电流 DC 零偏自校准（非阻塞，ISR 喂样式）。
 *
 *        背景：I.c 的 I_Calibrate() 仅在上电时执行一次，电流传感器
 *        （霍尔类）零点随温度漂移，运行数分钟后每相可漂移数百 mA。
 *        三相残余零偏经 Clarke/Park 后在 dq 系表现为 1x 电频率的
 *        正弦摆动，且 PI 无法消除，必须在每次模式启动时重新校准。
 *
 *        用法：在保证零电流的窗口（三相 50% 占空比零矢量）内，
 *        每 20kHz ISR 调用一次 Foc_Calib_Feed()，锁定后用
 *        Foc_Calib_GetOffsetsMa() 取偏移并在 Clarke 前减除。
 *        全程非阻塞，可在 ISR 上下文安全调用。
 *******************************************************************************
 */

#ifndef __FOC_CALIB_H__
#define __FOC_CALIB_H__

#include <stdint.h>
#include "I.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 参数（编译期宏）
 * ==========================================================================*/
#define FOC_CALIB_SKIP_SAMPLES   200u   /* 使能后先丢弃 200 样本(10ms)瞬态 */
#define FOC_CALIB_AVG_SAMPLES   4000u   /* 平均 4000 样本(200ms)后锁定 */

/* ============================================================================
 * Keil Watch 观测量
 * ==========================================================================*/
extern volatile uint8_t g_calib_state;          /* 0=idle 1=skip 2=sampling 3=locked */
extern volatile float   g_calib_iu_off_ma;      /* U 相残余零偏 (mA) */
extern volatile float   g_calib_iv_off_ma;      /* V 相残余零偏 (mA) */
extern volatile float   g_calib_iw_off_ma;      /* W 相残余零偏 (mA) */

/* 启动一轮校准（复位状态机，进入 SKIP 阶段） */
void Foc_Calib_Start(void);

/* 每 ISR 喂入一次当前采样；返回 1 表示已锁定（调用方随后可停止喂样） */
uint8_t Foc_Calib_Feed(const stc_i_data_t *pData);

/* 1 = 偏移已锁定，可直接使用 */
uint8_t Foc_Calib_IsLocked(void);

/* 取三相残余零偏 (mA)，未锁定时返回 0 */
void Foc_Calib_GetOffsetsMa(float *pU, float *pV, float *pW);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_CALIB_H__ */
