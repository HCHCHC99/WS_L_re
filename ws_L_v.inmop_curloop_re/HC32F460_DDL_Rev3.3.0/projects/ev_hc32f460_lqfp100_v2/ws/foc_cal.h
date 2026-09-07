/**
 *******************************************************************************
 * @file  foc_cal.h
 * @brief FOC 模式20 — 编码器零点校准 (comm_mode 20)。
 *
 * ============================================================================
 * 【傻瓜式讲解：这个模式是干什么的】
 *
 * 一句话：让磁场先在 90° 吸住转子 2 秒，再切到 0° 吸住 2 秒，然后记住
 *         "转子在 0° 时编码器读数是多少"，这个读数就是"零点"（offset）。
 *         以后所有模式都能用它从编码器读数反推转子真实角度。
 *
 * 为什么需要它？
 *   编码器只能告诉你"轴转过了多少"，不知道电机里磁铁的 N 极朝哪。
 *   FOC 必须知道 N 极位置才能正确发力。mode 20 把磁铁吸到已知的 0°，
 *   记下此刻编码器读数当零点，以后角度计算都扣掉它。
 *
 * 和 mode 23（foc_align）的区别：
 *   mode 23 是老版校准，"稳定判据"依赖 g_enc_count，但 ISR 里没人更新
 *   它，判据形同虚设，锁出的 offset 不可靠。mode 20 改为纯定时
 *   （BETA 2s + ALPHA 2s，全程自动），并且 offset 直接用 TMRA_1 原始
 *   硬件计数表达，mode 0 观测也读同一原始计数，零点框架天然一致。
 *
 * 工作过程（按时间顺序，全程自动，共 4 秒）：
 *   1. BETA 校准（2 秒）：在静止系 90° 电角度方向打固定磁场
 *      （幅值 = g_lockiq_align_volt_v，复用 mode 32 的电压），转子
 *      N 极被吸到 90° 位置。结束时记下编码器原始计数 g_cal_beta_hw。
 *   2. ALPHA 校准（2 秒）：磁场切到静止系 0°，转子被吸到 0°。
 *      结束时记下 g_cal_alpha_hw，并算出：
 *        offset = mod(原始计数 × 编码器方向, CPR)   ← 这一刻电角度 = 0°
 *   3. 完成：把 offset 写入共享对齐零点（Foc_Core_SetAlignOffset），
 *      关 PWM，竖起完成事件 g_cal_evt，由 foc_obs 自动跳回 mode 0。
 *
 * 校准后的"零点"定义（重要）：
 *   ALPHA 结束位置 = 电角度 0° = 机械角度 0°。之后 mode 0 下
 *   Foc_Core_UpdateAngleObs() 实时更新：
 *     g_foc_if_rotor_rad : 转子电角度 [0, 2π)        （rad，控制框架）
 *     g_foc_mech_rad     : 机械角度   [0, 2π)        （rad，控制框架）
 *     g_foc_mech_deg     : 机械角度   [0, 360)       （deg，Watch/显示）
 *     g_foc_elec_deg     : 电角度     [0, 360*极对数)（deg，Watch/显示）
 *   四者共用同一个 offset，随手转轴都能实时看到角度变化。
 *
 * Watch 常用变量：
 *   g_cal_running : 1 = 正在运行
 *   g_cal_phase   : 0=BETA 1=ALPHA 2=DONE 3=FAULT_OC
 *   g_cal_beta_hw / g_cal_alpha_hw : 两阶段结束时的编码器原始计数
 *   g_cal_moved   : BETA->ALPHA 编码器位移（带符号 counts，期望
 *                   ±CPR/(4×极对数) = ±102 counts 左右）
 *   g_cal_offset  : 锁定的零点（counts）
 *   g_cal_evt     : 1=BETA完成 2=锁定完成 3=过流（ISR 置，obs 清）
 *
 * ISR 约束：短小、无阻塞、无打印、无 malloc。打印全部由 foc_obs 在
 * 主循环完成（事件快照模式，与 mode 23/31/32 一致）。
 *******************************************************************************
 */

#ifndef __FOC_CAL_H__
#define __FOC_CAL_H__

#include <stdint.h>
#include "foc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * Debug macros（RTT打印规范：0/1 赋值式开关，定义在 .h，.c 只调封装宏）
 *=============================================================================*/
/* 1 = RTT prints on, 0 = off */
#define FOC_CAL_DBG   1
#if FOC_CAL_DBG
    #define CAL_DBG(fmt, ...)   MAIN_D("[CAL] " fmt, ##__VA_ARGS__)
#else
    #define CAL_DBG(fmt, ...)   ((void)0)
#endif

/*=============================================================================
 * 校准时长（编译期常量，FOC_ISR_HZ tick 换算在 .c 内完成）
 *=============================================================================*/
#define CAL_BETA_MS     2000u   /* BETA 吸附时长 */
#define CAL_ALPHA_MS    2000u   /* ALPHA 吸附时长 */

/*=============================================================================
 * 校准阶段（g_cal_phase）
 *=============================================================================*/
#define CAL_PHASE_BETA      0u  /* 吸附在静止系 90° */
#define CAL_PHASE_ALPHA     1u  /* 吸附在静止系 0° */
#define CAL_PHASE_DONE      2u  /* 完成（offset 已锁定） */
#define CAL_PHASE_FAULT_OC  3u  /* 过流停机 */

/*=============================================================================
 * 事件码（g_cal_evt，ISR 置位，Foc_Obs_Task 打印后清零）
 *=============================================================================*/
#define CAL_EVT_BETA_DONE   1u
#define CAL_EVT_LOCKED      2u
#define CAL_EVT_OC          3u

/*=============================================================================
 * Keil Watch 可调变量 / 观测量（定义见 foc_cal.c）
 *=============================================================================*/
extern volatile uint8_t  g_cal_running;   /* 1 = 正在运行 */
extern volatile uint8_t  g_cal_phase;     /* CAL_PHASE_xxx */
extern volatile uint8_t  g_cal_evt;       /* CAL_EVT_xxx */
extern volatile uint16_t g_cal_beta_hw;   /* BETA 结束时 TMRA_1 原始计数 */
extern volatile uint16_t g_cal_alpha_hw;  /* ALPHA 结束时 TMRA_1 原始计数 */
extern volatile int32_t  g_cal_moved;     /* BETA->ALPHA 位移(带符号 counts) */
extern volatile int32_t  g_cal_offset;    /* 锁定的零点 (counts, [0,CPR)) */

/*=============================================================================
 * API
 *=============================================================================*/

/* mode 20 入口：清故障 -> 复用 ALIGN 模式号 -> 零矢量启动 PWM -> BETA 阶段。
 * 主循环上下文调用（dev_comm_runner）。 */
void Foc_Cal_Start(void);

/* 20 kHz ISR 步进：OC 保护 -> 输出固定磁场 -> 计时切相 -> ALPHA 结束锁
 * offset 并关 PWM。由 Foc_Isr 在 g_cal_running 时分发调用。 */
void Foc_Cal_Step(const stc_i_data_t *pData);

/* 停止校准（用户中途切模式时调用）：清运行标志 + 关 PWM（若在输出）。
 * 自带 g_foc_active 清零，防止 ISR 回退到 Foc_Align_Step。 */
void Foc_Cal_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_CAL_H__ */
