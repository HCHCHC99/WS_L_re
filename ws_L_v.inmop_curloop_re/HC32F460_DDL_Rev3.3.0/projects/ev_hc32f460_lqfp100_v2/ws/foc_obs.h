/**
 *******************************************************************************
 * @file  foc_obs.h
 * @brief FOC 观察模块 — 集中存放"看电机转得怎么样"的所有代码。
 *
 *        【这个模块是干什么的】（傻瓜式说明）
 *
 *        mode 30 / 31 / 32 运行时会产生一大堆"给人看"的数据，以前散在
 *        foc_iq_pi.c / foc_lock_iq_pi.c / main.c 三个地方，现在全部搬到
 *        这里集中管理。变量名一个都没改，Keil Watch 照旧添加使用。
 *
 *        它管三件事：
 *
 *        1. 状态历史（行车记录仪）
 *           mode 31 像走路一样一步一步换状态（校准->闭环->饱和->...）。
 *           g_iqpi_step_hist[] 把每次换的步子按顺序记下来（最多 10 条，
 *           新旧的排法：hist[0] 最旧，hist[条数-1] 最新）。电机表现不对
 *           时，回放这段历史就能知道它是在哪一步出的错、之前经历了什么。
 *
 *        2. 转向/运行诊断量（仪表盘）
 *           窗口位移（电机最近 500ms 动了多少）、实测方向 vs 预期方向、
 *           翻转次数、编码器计数镜像……这些数字专门用来回答"它到底
 *           有没有在转、往哪边转、转得正不正常"。
 *
 *        3. 事件快照 + 定期打印（广播员）
 *           ISR 里发生了大事（过流/翻转/锁定成功/失败）时，把现场数据
 *           打包存进 evt_xxx 变量并竖起 flag；主循环里调用一次
 *           Foc_Obs_Task()，它负责：
 *             - 每隔 200ms 打一条运行监视（[ZIZENG_DBG] / [IQPI_MON]）
 *             - 事件发生时打一条快照（[IQPI_FLIP] / [LOCKIQ] / [FOC] /
 *               [ALIGN]），并把 flag 清掉
 *             - 处理 mode 32 锁定成功后的自动交接（启动 mode 31）
 *           ISR 内绝不打印（太慢），打印只发生在主循环，这是本模块存在
 *           的核心原因之一。
 *
 *        【打印标签约定】每条打印自带来源标签（[FOC]/[ALIGN]/
 *        [ZIZENG_DBG]/[IQPI_MON]/[IQPI_FLIP]/[LOCKIQ]，与迁移前完全一致），
 *        因此 OBS_DBG 宏不再额外加前缀。
 *******************************************************************************
 */

#ifndef __FOC_OBS_H__
#define __FOC_OBS_H__

#include <stdint.h>
#include "foc_iq_pi.h"     /* iqpi_step_t（状态历史数组元素类型） */

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * Debug macros（1 = RTT 打印开，0 = 关）
 *=============================================================================*/
#define FOC_OBS_DBG   1
#if FOC_OBS_DBG
    #define OBS_DBG(fmt, ...)      MAIN_D(fmt, ##__VA_ARGS__)
#else
    #define OBS_DBG(fmt, ...)      ((void)0)
#endif

/* mode 31 状态历史容量（随状态机迁移至此，foc_iq_pi.h 不再定义） */
#define IQPI_HISTORY_LEN  10u

/*=============================================================================
 * mode 31 观察量（定义在 foc_obs.c；傻瓜式含义讲解见 foc_iq_pi.h 顶部）
 * ==========================================================================*/
extern volatile iqpi_step_t g_iqpi_step_hist[IQPI_HISTORY_LEN]; /* 状态历史, [0]最旧 */
extern volatile uint8_t g_iqpi_step_hist_cnt;  /* 历史有效条数 0..10 */
extern volatile uint8_t g_iqpi_flip_cnt;       /* 本次运行 180° 框架自动翻转次数 */
extern volatile int32_t  g_iqpi_enc_pos;       /* ISR 累积编码器计数镜像（看转向/速率） */
extern volatile int32_t  g_iqpi_win_moved;     /* 最近完成的 500ms 窗口位移(counts,带符号) */
extern volatile uint32_t g_iqpi_win_evals;     /* 已完成窗口评估次数(0=方向检查从未运行) */
extern volatile int8_t   g_iqpi_cur_dir;       /* 最近窗口实测转向 +1/-1 */
extern volatile int8_t   g_iqpi_expect_dir;    /* 预期转向 +1/-1 */
extern volatile int8_t   g_iqpi_ref_dir;       /* 启动时捕获的拖动方向基准(mode30/32) */

/* --- 翻转事件快照（ISR 置 g_iqpi_evt_flag，Foc_Obs_Task 打印后清零） --- */
extern volatile uint8_t  g_iqpi_evt_flag;
extern volatile uint8_t  g_iqpi_evt_seq;       /* 第几次翻转 (1,2,...) */
extern volatile int32_t  g_iqpi_evt_pos;       /* 翻转时编码器计数 */
extern volatile int32_t  g_iqpi_evt_iq_ma;     /* 翻转时 iq (mA) */
extern volatile int32_t  g_iqpi_evt_vq_mv;     /* 翻转时 vq (mV) */
extern volatile int32_t  g_iqpi_evt_off_deg;   /* 翻转后偏移基线 (deg) */

/*=============================================================================
 * mode 32 观察量（定义在 foc_obs.c；傻瓜式含义讲解见 foc_lock_iq_pi.h 顶部）
 * ==========================================================================*/
extern volatile int32_t g_lockiq_win_moved;     /* 最近完成窗口的平均位置位移 (counts) */
extern volatile uint32_t g_lockiq_win_evals;    /* 已完成窗口评估次数 */
extern volatile int32_t g_lockiq_track_err_cnts; /* VERIFY 跟踪误差 (counts) */

/* --- 锁定/失败事件快照（ISR 置 g_lockiq_evt_flag，Foc_Obs_Task 处理后清零） --- */
extern volatile uint8_t  g_lockiq_evt_flag;
extern volatile uint8_t  g_lockiq_evt_code;     /* LOCKIQ_EVT_xxx（定义见 foc_lock_iq_pi.h） */
extern volatile int32_t  g_lockiq_evt_off_deg; /* 锁定的注入偏移 (deg) */

/*******************************************************************************
 * API
 ******************************************************************************/

/* 清空 mode 31 状态历史（每次成功启动 mode 31 时由 foc_iq_pi 调用，
 * 重新开始记录） */
void Foc_Obs_IqpiHistClear(void);

/* 记录一条 mode 31 状态变化（Iqpi_SetStep 每次换状态时调用；
 * hist 满后自动挤掉最旧的一条，连续相同状态不会重复记录——
 * 去重由调用方保证，本函数只管如实记录） */
void Foc_Obs_IqpiRecordStep(iqpi_step_t s);

/* 观察任务：主循环每圈调用一次。内部按需打印运行监视、处理事件快照、
 * 执行 mode 32 -> mode 31 自动交接。ISR 内禁止调用。 */
void Foc_Obs_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_OBS_H__ */
