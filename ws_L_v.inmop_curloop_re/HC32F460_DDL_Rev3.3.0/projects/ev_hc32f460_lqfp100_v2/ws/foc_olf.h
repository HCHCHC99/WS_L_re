/**
 *******************************************************************************
 * @file  foc_olf.h
 * @brief FOC 模式26 — 开环 VF 负载角实验 (comm_mode 26, Open Loop Field-angle)。
 *
 * ============================================================================
 * 【傻瓜式讲解：这个模式是干什么的】
 *
 * 一句话：先校准零点（和 mode 25 相同的吸附校准），然后让磁场角度
 *         自动匀速前进（和 mode 30 相同的拖动），你可以随时调快磁场
 *         转速，观察"磁场角 - 转子角"的差值（负载角 delta）怎么变化
 *         ——这是一个测同步电机"功角特性"的教学实验。
 *
 * 实验原理（为什么差值会变大）：
 *   磁场拖着转子转，磁场必须领先转子一个负载角 delta，才能产生
 *   转矩去克服摩擦。delta 越大转矩越大（T ∝ sin(delta)）：
 *     - 频率低：只需克服摩擦 -> delta 稳在 5~15°（静摩擦+齿槽决定，
 *       不会是 0，那是"摩擦锥"角度）
 *     - 频率升高：反电动势变大偷走电压 -> 电流变小 -> 要同样的转矩
 *       必须更大的 delta -> delta 缓慢上升
 *     - delta 逼近 90°：sin(delta) 达到峰值，这是稳定边界（悬崖边）
 *     - 越过 90°：转矩反而下降 -> 转子落伍 -> delta 更大 -> 失步！
 *       表现为 diff 剧烈振荡（磁场把转子拉回来又甩出去，锯齿状崩塌）
 *
 * id/iq 的读法（本实验的核心观测量，真实转子系！）：
 *   本模式已通过校准锁定零点，所以 id/iq 用"编码器实测的转子真实
 *   角度"做 Park 变换（mode 30 只能用磁场角近似）：
 *     iq = I·sin(delta)  力矩电流，随 delta 增大而增大，失步后振荡
 *     id = I·cos(delta)  磁链分量，低频 ≈0；频率高后转负（反电动势
 *                        把电流矢量拉向去磁侧 = 电压饥饿），这正是
 *                        真实 V/f 变频器要按 V/f 比例升压的原因
 *
 * 工作过程（按时间顺序）：
 *   1. 进入 mode 26：自动跑 mode 25 式校准（BETA 90° 吸 2s -> ALPHA
 *      0° 吸 2s -> 锁零点 offset）。
 *   2. 校准完成立即进入拖动：theta 以 g_olf_freq_hz（默认
 *      0.5Hz，低速起步）匀速自增（磁场角 = theta+90° 随之同速旋转），
 *      电压 g_olf_volt_v（默认
 *      0.6V，与 mode 30 相同）。输出约定 q 轴电压（与 mode 30 一致）：
 *      theta 为控制系 d 轴角，电压矢量在 theta+90°，磁场角 = theta+90°，
 *      与转子 N 极对齐时 delta=0；锁定时 g_foc_id_ma≈0, g_foc_iq_ma≈I
 *      （真实转子系 g_olf_id/iq_ma 仍按物理分布 id≈I, iq 随 delta 增大）。
 *   3. 自增三要素（全部 Keil Watch 可调）：
 *      a) g_olf_freq_hz 磁场转速 (Hz)：SW1 每按 +0.5（手动斜坡防失步），
 *         Watch 直改阶跃生效，改动大会失步；重新进入恢复默认 0.5Hz
 *      b) g_olf_step_010 自增步长 (×0.1°/步)：磁场角攒够一个步长跳一步，
 *         1 = 0.1°/步 ≈ 连续旋转（默认）；调大变"大步跳跃"实验
 *         （等效自增节拍 = 360×freq/step 次/秒）
 *      c) g_olf_dir 方向：+1 = 角度递加（默认），-1 = 角度递减
 *   4. foc_obs 每 200ms 打印一行实验数据（见下），RTT Viewer 里
 *      直接看 delta 随频率爬升 -> 90° 崩塌的全过程。
 *
 * 定时打印格式（200ms 一行，全整型）：
 *   [OLF] f=%d mHz fld=%d deg rot=%d deg diff=%d deg id=%d iq=%d mA
 *         磁场转速   磁场角    转子角    负载角delta   真实转子系电流
 *
 * 实验预期数据曲线（低频段，0.6V）：
 *   f:  0.5Hz -> SW1 每按 +0.5Hz x N
 *   diff: 5~15° 稳定 -> 缓升 -> 快速冲向 90° -> 锯齿崩塌振荡（失步）
 *   iq:   小 -> 随 diff 上升 -> 失步后剧烈振荡
 *   id:   ≈0 -> 转负（BEMF 去磁）-> 失步后剧烈振荡
 *
 * Watch 常用变量：
 *   g_olf_freq_hz  : 磁场角自增频率 (Hz)，Start 时复位为 0.5，SW1 每按 +0.5
 *   g_olf_volt_v   : 拖动电压 (V)，Start 时复位为 0.6（≈5A，量程内）
 *   g_olf_state    : 0=空闲 1=校准BETA 2=校准ALPHA 3=拖动 4=过流
 *   g_olf_diff_deg : 负载角 delta (deg, -180~180) —— 实验主指标
 *   g_olf_field_deg / g_olf_rotor_deg : 磁场角/转子角 (deg, 0~359)
 *   g_olf_id_ma / g_olf_iq_ma : 真实转子系电流 (mA)
 *   g_olf_theta_rad : 控制系 d 轴角 (rad, 磁场角 = theta+90°)
 *   g_olf_du/dv/dw : 三相占空比 (%)
 *
 * 与 mode 30 的区别：mode 30 目的是锁偏移（编码器只旁观）；本模式
 * 零点校准后精确已知，目的是观察负载角与失步（编码器是主角）。
 *
 * ISR 约束：短小、无阻塞、无打印、无 malloc。打印全部由 foc_obs 在
 * 主循环完成（事件快照 + 200ms 周期）。
 *******************************************************************************
 */

#ifndef __FOC_OLF_H__
#define __FOC_OLF_H__

#include <stdint.h>
#include "foc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * Debug macros（RTT打印规范：0/1 赋值式开关，定义在 .h，.c 只调封装宏）
 *=============================================================================*/
/* 1 = RTT prints on, 0 = off */
#define FOC_OLF_DBG   1
#if FOC_OLF_DBG
    #define OLF_DBG(fmt, ...)   MAIN_D("[OLF] " fmt, ##__VA_ARGS__)
#else
    #define OLF_DBG(fmt, ...)   ((void)0)
#endif

/*=============================================================================
 * 时长（编译期常量，FOC_ISR_HZ tick 换算在 .c 内完成）
 *=============================================================================*/
#define OLF_BETA_MS     2000u  /* 校准 BETA 吸附时长 */
#define OLF_ALPHA_MS    2000u  /* 校准 ALPHA 吸附时长 */

/*=============================================================================
 * 状态机（g_olf_state）
 *=============================================================================*/
#define OLF_STEP_IDLE       0u  /* 未运行 */
#define OLF_STEP_CAL_BETA   1u  /* 校准：磁场 90°，2s */
#define OLF_STEP_CAL_ALPHA  2u  /* 校准：磁场 0°，2s，结束锁零点 */
#define OLF_STEP_DRAG       3u  /* 磁场角自增拖动（实验主体） */
#define OLF_STEP_FAULT_OC   4u  /* 过流停机 */

/*=============================================================================
 * 事件码（g_olf_evt，ISR 置位，Foc_Obs_Task 打印后清零）
 *=============================================================================*/
#define OLF_EVT_BETA_DONE  1u
#define OLF_EVT_LOCKED     2u
#define OLF_EVT_OC         3u

/*=============================================================================
 * Keil Watch 可调变量 / 观测量（定义见 foc_olf.c）
 *=============================================================================*/
extern volatile float    g_olf_freq_hz;    /* 磁场角自增频率 (Hz, 默认 0.5, Start 复位) */
extern volatile int32_t  g_olf_step_010;   /* 自增步长 (×0.1°/步, 1≈连续旋转, 建议 1~3600) */
extern volatile int32_t  g_olf_dir;        /* 自增方向 (+1=角度加 / -1=角度减, Start 复位 +1) */
extern volatile float    g_olf_volt_v;     /* 拖动电压幅值 (V, 默认 0.6 与 mode30 同) */
extern volatile uint8_t  g_olf_running;    /* 1 = 正在运行 */
extern volatile uint8_t  g_olf_state;      /* OLF_STEP_xxx */
extern volatile uint8_t  g_olf_evt;        /* OLF_EVT_xxx */
extern volatile int32_t  g_olf_offset;     /* 校准锁定的零点 (counts) */
extern volatile int32_t  g_olf_field_deg;  /* 磁场电角度 (deg, 0~359) */
extern volatile int32_t  g_olf_rotor_deg;  /* 转子电角度 (deg, 0~359, 已扣零点) */
extern volatile int32_t  g_olf_diff_deg;   /* 负载角 delta = field - rotor (deg, -180~180) */
extern volatile float    g_olf_id_ma;      /* 真实转子系 id (mA, 磁链分量) */
extern volatile float    g_olf_iq_ma;      /* 真实转子系 iq (mA, 力矩分量) */
extern volatile float    g_olf_theta_rad;  /* 控制系 d 轴角 (rad, 磁场角=theta+90°) */
extern volatile float    g_olf_du;         /* 三相占空比观测 (%) */
extern volatile float    g_olf_dv;
extern volatile float    g_olf_dw;

/*=============================================================================
 * API
 *=============================================================================*/

/* mode 26 入口：清故障 -> 自动校准（BETA 2s + ALPHA 2s 锁零点）-> 拖动。
 * 主循环上下文调用（dev_comm_runner）。 */
void Foc_Olf_Start(void);

/* 20 kHz ISR 步进：OC 保护 -> 校准/拖动输出 -> 角度与电流观测。
 * 由 Foc_Isr 在 g_olf_running 时分发调用。 */
void Foc_Olf_Step(const stc_i_data_t *pData);

/* 停止（用户中途切模式时调用）：清运行标志 + 关 PWM（若在输出）。
 * 自带 g_foc_active 清零，防止 ISR 回退。 */
void Foc_Olf_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_OLF_H__ */
