/**
 *******************************************************************************
 * @file  foc_20_cal.h
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
 * 和 mode 23（foc_23_align）的区别：
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
 * ISR 约束：短小、无阻塞、无打印、无 malloc。打印全部由 foc_obs 在
 * 主循环完成（事件快照模式，与 mode 23/31/32 一致）。
 *
 * ==============================================================================
 *   模式速览卡   MODE 20   编码器零点校准（BETA 2s + ALPHA 2s 自动锁零点）
 *   本卡是本模式唯一事实源；代码内注释与本卡冲突时，以本卡为准
 * ==============================================================================
 *   入口   comm_mode = 20
 *   前置   无；进模式即清故障 + 零矢量起 PWM，转子需能自由转动
 *   结束   约 4s 自动：锁零点 + 关 PWM + 自动回 mode 0。LOCKED 和过流都自动回 0
 *   标记   [!] 易错点   (*) 主判据   [可调] Watch 可写   [只读] 仅观察
 * ------------------------------------------------------------------------------
 *   [可调] Watch 变量 -- 改值即时生效，复位后回宏默认
 * ------------------------------------------------------------------------------
 *       变量                        当前值  单位     含义
 *       --------------------------- ------- -------- ---------------------------
 *       g_lockiq_align_volt_v       0.4     V        吸附电压（复用 mode 32）
 *       g_foc_enc_dir               -1      -        编码器方向，参与零点计算
 *   (*) 0.4V 实测吸持约 4A 直流（Rs 约 0.1 欧），在 9A
 *       过流阈值内。本模式没有自己的电压变量，改的就是 mode 32 的
 *       g_lockiq_align_volt_v（宏默认 FOC_ALIGN_VOLT_V）。
 *   [!] 电压改成 0 等于没打磁场，但事件照样报 LOCKED，锁出废零点（g_cal_moved 约
 *       0）；抬到约 0.9V 以上会撞 9A 过流，g_cal_evt = 3 并自动回 mode 0。
 * ------------------------------------------------------------------------------
 *   [只读] 关键观察变量
 * ------------------------------------------------------------------------------
 *       变量                           含义
 *       ------------------------------ -----------------------------------------
 *       g_cal_running                  1 = 校准运行中；锁定或过流后自动回 0
 *       g_cal_phase                    0 BETA / 1 ALPHA / 2 DONE / 3 FAULT_OC
 *       g_cal_evt                      1 BETA_DONE / 2 LOCKED / 3 OC
 *       g_cal_beta_hw                  BETA 结束时 TMRA_1 原始计数
 *       g_cal_alpha_hw                 ALPHA 结束时 TMRA_1 原始计数
 *   (*) g_cal_moved                    BETA->ALPHA 位移 counts（应约 +/-102）
 *   (*) g_cal_offset                   锁定的零点 counts，已写入共享对齐零点
 *       g_foc_elec_deg                 mode 0 实时电角度 deg（范围 0-3600）
 *       g_foc_fault                    非 0 = 故障停机（过流）
 *       g_foc_fault_i_ma               过流时刻最大相电流 mA
 *   [!] 纯定时、无吸稳校验：BETA 到 2s 就无条件记 g_cal_beta_hw 并切
 *       ALPHA。转子初始还在转、被齿槽卡住或相序不对时，事件仍然报
 *       LOCKED，零点已经错了。
 *   [!] 中途改 comm_mode 走 Foc_Cal_Stop()：只清运行标志 + 关 PWM，不写
 *       offset，也不清 g_cal_phase / g_cal_evt，看到停在 0 或 1 属正常。
 * ------------------------------------------------------------------------------
 *   VOFA 通道 -- 14ch，填充见 Foc_Cal_VofaFill
 * ------------------------------------------------------------------------------
 *       通道  含义                   单位     备注
 *       ----- ---------------------- -------- ----------------------------------
 *       ch0   U 相电流               A        BETA/ALPHA 吸附电流包络
 *       ch1   V 相电流               A
 *       ch2   W 相电流               A
 *   (*) ch3   id 反馈                A        磁场轴上的吸持电流
 *       ch4   iq 反馈                A        吸稳后应接近 0
 *       ch5   阶段                   -        0 BETA / 1 ALPHA / 2 DONE / 3 OC
 *   (*) ch6   事件                   -        1 BETA_DONE / 2 LOCKED / 3 OC
 *       ch7   BETA 结束计数          counts   TMRA_1 原始值
 *       ch8   ALPHA 结束计数         counts   TMRA_1 原始值
 *   (*) ch9   BETA 到 ALPHA 位移     counts   应约 +/-102 = CPR/(4 x 极对数)
 *   (*) ch10  锁定的零点             counts   本次校准产物
 *       ch11  已生效零点             counts   = g_foc_align_offset，应与 ch10
 *                                             相同
 *       ch12  转子电角度             deg      锁完停在 0 附近
 *       ch13  故障标志               0/1
 * ------------------------------------------------------------------------------
 *   判据与坑
 * ------------------------------------------------------------------------------
 *   (*) 判成功：RTT 打出 [CAL] ALPHA done（g_cal_evt =
 *       2，CAL_EVT_LOCKED）后自动回 mode 0；此时 g_cal_phase =
 *       2（CAL_PHASE_DONE）、g_cal_running = 0、g_cal_offset 落在 [0,4096) 且与
 *       g_foc_align_offset（VOFA ch11）同值。
 *   (*) 判准确：|g_cal_moved| 应约等于 CPR/(4*极对数) = 4096/40 = 102
 *       counts（ENCODER_CPR = 4096，FOC_POLE_PAIRS = 10），即 BETA 90 度到 ALPHA
 *       0 度的 90 度电角度位移。明显偏离就是没吸到位，零点不可信。
 *   [!] 本模式是纯定时（CAL_BETA_MS 2000 + CAL_ALPHA_MS 2000，共
 *       4s），没有静止/吸稳判据。重跑前先手动把转子拨到大致位置、确认轴能自由转
 *       动，能显著提高一次成功率。
 *   [!] 吸附电压复用 mode 32 的 g_lockiq_align_volt_v：0.4V 约 4A 吸持；改 0
 *       会静默锁废零点；抬太高会先撞 9A 过流（g_cal_evt = 3，CAL_EVT_OC）。
 *   [!] VOFA+ 通道数必须同步配成 14（本模式自持布局），否则整帧错位。本布局只在
 *       g_cal_running = 1 期间出现：锁定后 g_cal_running 清 0，帧立刻切回通用
 *       20ch（那时看 ch15 对齐零点）。
 * ==============================================================================
 */

#ifndef __FOC_20_CAL_H__
#define __FOC_20_CAL_H__

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
 * Keil Watch 可调变量 / 观测量（定义见 foc_20_cal.c）
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

/* 模式自持 VOFA：固定 14ch，返回通道数；通道含义见本文件顶部速览卡。 */
int  Foc_Cal_VofaFill(int32_t *cur);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_20_CAL_H__ */
