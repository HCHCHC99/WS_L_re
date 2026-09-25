/**
 *******************************************************************************
 * @file  foc_32_lockiq.h
 * @brief FOC 模式32 — 自锁偏移 + 自动进入 mode 31 (comm_mode 32)。
 *
 * ============================================================================
 * 【傻瓜式讲解：这个模式是干什么的】
 *
 * 一句话：先把"表"对准，再自动启动 mode 31，是一键到位的启动模式。
 *
 * 它解决什么问题？
 *   mode 31 启动前必须知道"编码器读数"和"磁铁真实角度"差多少（偏移量）。
 *   原来靠 mode 30 拖着转去测，转子可能打滑，测出的偏移可能不准，导致
 *   mode 31 启动方向随机。mode 32 改用"磁铁吸住不动"的笨办法对表：
 *   转子被吸死在固定位置，不可能打滑，测出的偏移一定准。
 *
 * 工作过程（按时间顺序，全程自动）：
 *   1. BETA（约0.5s）：磁场定在 90°电角度，转子"咔"一下被磁力吸到
 *      固定位置（吸力大小 = g_lockiq_align_volt_v）。有响声是正常的。
 *   2. ALPHA：磁场定在 0°，再吸一次，然后等转子完全静止：每 300ms
 *      看一次编码器平均位移，连续 2 次都 <= 4 counts 才算"稳"。
 *      此时记下转子停的位置（这就是"表"的基准点）。
 *   3. VERIFY（防作弊复测）：磁场再定回 90°，转子应该正好挪动
 *      90°电角度（= CPR/(4·极对数) counts）。挪的量和标准答案对不上
 *      （差超过 12 counts）就拒绝启动——防止转子被卡死在别的位置却
 *      "看起来静止"骗过了第 2 步。
 *   4. 锁定：对表完成，偏移量恒定注入 0（为什么是恒定 0 见 .c 文件头
 *      推导：VERIFY 结束时转子被吸停在 90°，而 mode 31 要求
 *      off = 90° − 转子角度 = 0。旧版注入 90° 是加 VERIFY 前的推导，
 *      会造成 90° 框架差 → 堵转/过流），方向基准注入 FOC_ENC_DIR，打印：
 *        "[LOCKIQ] locked off=0 deg dir=±1 -> handoff iqpi"
 *   5. 自动交接：继续保持 90° 磁场吸住转子不放手（保证 mode 31 启动
 *      瞬间转子仍在对齐位，off=0 的前提），观察模块 foc_obs 的
 *      Foc_Obs_Task() 看到事件后自动调用 mode 31 启动，无缝闭环。
 *      整个流程不用人管。
 *
 * 失败怎么办（宁可不起动，不锁坏值）：
 *   等静止超时（3s）/ 复测位移不对 -> 打印 "[LOCKIQ] FAIL code=..."
 *   并自动停机。code=2 等静止超时；code=3 复测位移不对（看
 *   g_lockiq_track_err_cnts 差了多少）。
 *
 * Watch 常用变量：
 *   g_lockiq_step            : 进行到哪一步（含义见下面枚举）
 *   g_lockiq_align_volt_v    : 吸力大小 V，启动前可改
 *   g_lockiq_off_deg         : 已注入 mode 31 的偏移（deg，成功后应为 0）
 *   g_lockiq_track_err_cnts  : 复测位移误差，0 附近=好
 *   窗口诊断量 g_lockiq_win_moved/win_evals、事件快照 g_lockiq_evt_*
 *   已集中迁移到观察模块 foc_obs.h（定义在 foc_obs.c），变量名未变。
 *
 * 使用限制（重要）：
 *   偏移=0 只对"mode 32 -> 自动交接 mode 31"这条链路有效（交接时
 *   转子必须仍停在对齐位）。交接后转子若被人手拧动过，再单独进
 *   mode 31 是无效的，需要重跑 mode 32。
 *
 * ISR 约束：短小、无阻塞、无打印、无 malloc；锁定/失败事件经
 * g_lockiq_evt_flag 由 foc_obs 模块在主循环上下文打印并交接。
 *
 * ============================ 模式速览卡（唯一事实源）========================
 * 模式：32 = 自锁偏移 + 自动交接 mode 31
 *        用**静止吸附法**锁编码器偏移（不受打滑影响），锁定后自动启动 mode 31
 * 入口：comm_mode = 32 ← **闭环启动的推荐入口**（Watch 写 32 即可，不依赖 mode 30）
 * 前置：无（自带对齐 + 复测）
 * 结束：锁定成功后自动交接给 mode 31（转子须仍停在对齐位）
 * ⚠ 交接后若转子被人手拧动过，再单独进 mode 31 无效，需重跑 mode 32
 *
 * 【Watch 可调变量】（名称 = 单位）
 *   g_lockiq_align_volt_v   V   对齐电压幅值（Watch 可调）
 *
 * 【关键观察变量】
 *   g_lockiq_running   1 = 运行中
 *   g_lockiq_step      状态枚举（**Watch 直接显示枚举名**）：
 *        0 IDLE / 1 ALIGN_BETA（磁场90°吸附）/ 2 ALIGN_ALPHA（磁场0°等静止）
 *        3 ALIGN_VERIFY（回90°复测跟踪，位移≈90°电）/ 4 LOCKED_WAIT_HANDOFF（已锁定等交接）
 *        5 LOCK_FAIL（超时/跟踪失败，拒绝启动）/ 6 FAULT_OC
 *   g_lockiq_off_deg   已注入 mode 31 的偏移 (deg)
 *   事件码 FOC32_EVT_*：1 LOCKED / 2 FAIL_TIMEOUT / 3 FAIL_TRACK
 *   （事件经 foc_obs 在主循环打印并执行交接）
 *
 * 【VOFA 通道】**自持布局，固定 16ch**（实现见 foc_32_lockiq.c
 *   Foc_LockIq_VofaFill，单位换算：传"毫单位"，SendScaled 内部 ×0.001）：
 *   ch0~2  三相电流(A)
 *   ch3    静止系电流幅值(A)（吸附期间可见）  ch4 ialpha(A)   ch5 ibeta(A)
 *   ch6    iq 反馈(A)                ch7  id 反馈(A)
 *   ch8    iq 目标(A)（交接后有效）
 *   ch9    vd 输出(V)                ch10 vq 输出(V)
 *   ch11   斜坡后 iq 参考(A)（交接后有效）
 *   ch12   使用的转子电角度(rad)
 *   ch13   U 相占空比(%)
 *   ch14   已注入的偏移(deg)
 *   ch15   **状态 ← 锁定阶段主判据**：
 *          0IDLE / 1ALIGN_BETA / 2ALIGN_ALPHA / 3ALIGN_VERIFY（位移≈90°电）
 *          / 4LOCKED_WAIT_HANDOFF / 5LOCK_FAIL（超时或跟踪失败）/ 6FAULT_OC
 *   ⚠ 共 16ch（与 mode 31 通道一致，便于交接前后连续观察）。
 *   ⚠ **改通道数：改 Foc_LockIq_VofaFill 末尾的 return 值**。
 * ===========================================================================
 */

#ifndef __FOC_32_LOCKIQ_H__
#define __FOC_32_LOCKIQ_H__

#include <stdint.h>
#include "foc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * Debug macros
 *=============================================================================*/
/* 1 = RTT prints on, 0 = off */
#define FOC_LOCKIQ_DBG   1
#if FOC_LOCKIQ_DBG
    #define FOC32_DBG(fmt, ...)   MAIN_D("[LOCKIQ] " fmt, ##__VA_ARGS__)
#else
    #define FOC32_DBG(fmt, ...)   ((void)0)
#endif

/* ============================================================================
 * 参数（编译期宏，Watch 可调项见变量）
 *   BETA 500ms：吸附到 90° 电角度
 *   WIN  300ms：稳定判据窗口，窗口内平均位置位移 <= TOL 视为"静止"
 *   连续 QUIET_NEED 个静止窗口才算稳定（双窗复核，防瞬时噪声骗过）
 *   ALPHA/VERIFY 各有 TIMEOUT 超时：始终不静 -> 失败，拒绝启动
 * ==========================================================================*/
#define FOC32_BETA_MS        500u
#define FOC32_WIN_MS         300u
#define FOC32_TIMEOUT_MS     3000u
#define FOC32_WIN_TOL_CNTS   4        /* 窗口位移容差 (counts)，约3.5°电角度 */
#define FOC32_QUIET_NEED     2u       /* 连续静止窗口数 */
#define FOC32_TRACK_TOL_CNTS 12       /* VERIFY 跟踪误差容差 (counts)，约10°电 */

#define FOC32_BETA_CNT     ((uint32_t)FOC32_BETA_MS * FOC_ISR_HZ / 1000u)
#define FOC32_WIN_CNT      ((uint32_t)FOC32_WIN_MS * FOC_ISR_HZ / 1000u)
#define FOC32_TIMEOUT_CNT  ((uint32_t)FOC32_TIMEOUT_MS * FOC_ISR_HZ / 1000u)

/* ============================================================================
 * mode 32 运行状态机 (g_lockiq_step)
 *   停止(mode 0)后保持最后状态不清除，下次成功启动时复位
 * ==========================================================================*/
typedef enum {
    FOC32_STEP_IDLE                = 0, /* 上电初始/未启动 */
    FOC32_STEP_ALIGN_BETA          = 1, /* 磁场定 90°，转子吸附中 */
    FOC32_STEP_ALIGN_ALPHA         = 2, /* 磁场定 0°，等待转子静止 */
    FOC32_STEP_ALIGN_VERIFY        = 3, /* 磁场定 90°，复测跟踪(位移≈90°电) */
    FOC32_STEP_LOCKED_WAIT_HANDOFF = 4, /* 已锁定+注入完成，等 main.c 交接 */
    FOC32_STEP_LOCK_FAIL           = 5, /* 超时/跟踪失败，拒绝启动 */
    FOC32_STEP_FAULT_OC            = 6, /* 过流保护停机 */
} lockiq_step_t;

/* 事件码 (g_lockiq_evt_code)，由观察模块 foc_obs (Foc_Obs_Task) 处理 */
#define FOC32_EVT_LOCKED        1u   /* 锁定成功，应调 Foc_IqPi_Start() 交接 */
#define FOC32_EVT_FAIL_TIMEOUT  2u   /* 等静止超时，应 Stop */
#define FOC32_EVT_FAIL_TRACK    3u   /* 跟踪复测失败，应 Stop */

/* ============================================================================
 * Keil Watch 可调变量 / 观测量（定义见 foc_32_lockiq.c）
 *   窗口诊断量 g_lockiq_win_moved/win_evals/track_err_cnts 与锁定/失败
 *   事件快照 g_lockiq_evt_* 已集中迁移到观察模块 foc_obs.h（定义在
 *   foc_obs.c），变量名未变，Keil Watch 用法不变。
 * ==========================================================================*/
extern volatile uint8_t g_lockiq_running;       /* 1 = 模式32 运行中 */
extern volatile lockiq_step_t g_lockiq_step;    /* 当前/最后状态 (Watch 看枚举名) */
extern volatile float   g_lockiq_align_volt_v;  /* 对齐电压幅值 (V)，Watch 可调
                                                   (0.4V -> ≈4A 直流，量程内) */
extern volatile int32_t g_lockiq_off_deg;       /* 已注入 mode 31 的偏移 (deg) */

/*******************************************************************************
 * API
 ******************************************************************************/

/* 启动模式32：清故障 -> PWM 启动 -> 直流对齐锁定偏移 -> 置事件等 foc_obs
 * 交接 Foc_IqPi_Start()。不依赖 mode 30（会覆盖其锁定值与方向基准）。 */
void Foc_LockIqPi_Start(void);

/* 停止模式32（含对齐失败后的停机；不影响已交接运行的 mode 31） */
void Foc_LockIqPi_Stop(void);

/* 模式32 单步运算（20 kHz ISR 中由 Foc_Isr 分发调用） */
void Foc_LockIqPi_Step(const stc_i_data_t *pData);

/* 模式自持 VOFA（16ch，通道与 mode 31 一致便于交接前后连续观察） */
int  Foc_LockIq_VofaFill(int32_t *cur);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_32_LOCKIQ_H__ */
