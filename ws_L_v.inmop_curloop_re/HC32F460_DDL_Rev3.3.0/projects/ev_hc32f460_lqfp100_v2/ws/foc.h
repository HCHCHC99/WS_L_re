/**
 *******************************************************************************
 * @file  foc.h
 * @brief FOC 门面头文件 — 对外 API 与全部子模块的统一入口。
 *
 *        拆分后的模块结构（ws 目录）：
 *          foc_core.h     公共核心：共享状态/观测量、过流保护、PWM 启停、
 *                         公共助手（GetDq/EMA/电压包络/编码器电角度）
 *          foc_openloop.h 模式21 开环 V/f
 *          foc_curloop.h  模式22 编码器 FOC 电流环（I-F 启动 + RUN）
 *          foc_align.h    模式23 静止电角度对齐校准
 *          foc_cal.h      模式20 编码器零点校准（BETA 2s + ALPHA 2s -> 锁 offset）
 *          foc_cal_angle.h 模式25 手动角度吸附（自动校准 -> 刹车等待输入 -> 吸附+校验）
 *          foc_olf.h      模式26 开环 VF 负载角实验（校准 -> 磁场自增拖动 -> delta/失步观测）
 *          foc_zizeng.h   模式30 磁场角度自增拖动（ZIZENG）
 *          foc_iq_pi.h    模式31 PI 电流环（编码器角度 + ZIZENG 偏移）
 *          foc_lock_iq_pi.h 模式32 自锁偏移（直流对齐）+ 自动交接 mode 31
 *          foc_calib.h    相电流 DC 零偏自校准（各模式启动时复用）
 *          foc_scope.h    MotorScope RTT 遥测
 *
 *        依赖方向：各模式模块 -> foc_core；本文件仅聚合头文件与
 *        Foc_Init/Foc_Isr 分发入口。main.c / dev_comm_runner.c 等
 *        外部调用方只需包含 foc.h，接口与拆分前完全一致。
 *******************************************************************************
 */

#ifndef __FOC_H__
#define __FOC_H__

#include "I.h"
#include "../Utils/dev_pid.h"
#include "foc_core.h"
#include "foc_openloop.h"
#include "foc_curloop.h"
#include "foc_align.h"
#include "foc_cal.h"
#include "foc_cal_angle.h"
#include "foc_olf.h"
#include "foc_zizeng.h"
#include "foc_iq_pi.h"
#include "foc_lock_iq_pi.h"
#include "foc_calib.h"
#include "foc_scope.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/

/* Init: build math LUT, bind current-loop PIs, register Foc_Isr as the 2nd
 * current callback. Does NOT start any PWM output. Call once after
 * I_Init()/I_Calibrate(). */
void Foc_Init(void);

/* 20 kHz ISR callback (registered via I_RegisterFocCallback). Must be short:
 * no blocking, no prints, no malloc. 按模式分发：
 *   FOC_MODE_OPENLOOP -> Foc_OpenLoop_Step
 *   FOC_MODE_ALIGN    -> g_cal_running ? Foc_Cal_Step
 *                        : (g_calang_running ? Foc_CalAngle_Step
 *                        : (g_olf_running ? Foc_Olf_Step : Foc_Align_Step))
 *   FOC_MODE_CURLOOP  -> Foc_CurLoop_Step（内部再按状态机分派）
 *   g_zizeng_running  -> Foc_Zizeng_Step
 *   g_lockiq_running  -> Foc_LockIqPi_Step（锁定后由 main.c 交接 mode 31）
 *   g_iqpi_running    -> Foc_IqPi_Step
 */
void Foc_Isr(const stc_i_data_t *pData);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_H__ */
