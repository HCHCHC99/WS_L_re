/**
 *******************************************************************************
 * @file  foc_dcl.h
 * @brief FOC 模式27 — 功角闭环拖动 (comm_mode 27, Delta Closed-Loop)。
 *
 * ============================================================================
 * 【傻瓜式讲解：这个模式是干什么的】
 *
 * 一句话：校准完零点后，磁场角度不再"按时间自转"（那是 mode 26），
 *         而是实时问编码器"转子在哪"，然后把磁场打在转子前方固定的
 *         功角 delta 处：磁场角 = 转子角 + delta。delta 本身从一个
 *         初始值爬坡到目标值（可设过渡时间），功角因此被"钉死"。
 *
 * 和 mode 26 的本质区别：
 *   mode 26 时间开环：磁场自己匀速转，转子爱跟不跟，功角 delta 是
 *                     力矩平衡"漂"出来的（会猎振、会失步）。
 *   mode 27 转子锁相：磁场跟着转子走，delta 永远 = 设定值（±0.1°级），
 *                     理论上不可能失步（磁场天涯海角跟着转子）。
 *   代价/变化：转速不再由指令决定，而是像直流电机一样"电压定了转速"：
 *                     转速↑ -> 反电动势↑ -> 电流↓ -> 力矩↓ -> 减速，
 *                     稳态转速 = 电压/功角/摩擦共同决定的平衡点。
 *                     （所以本模式没有 freq 变量，调速靠改电压 g_dcl_volt_v）
 *
 * 工作过程（按时间顺序）：
 *   1. 进入 mode 27：自动跑 mode 26/25 式校准（BETA 90° 吸 2s ->
 *      ALPHA 0° 吸 2s -> 锁零点 offset）。锁零点 = 记下这一刻编码器
 *      相对计数 s_enc_pos 作为 s_off_rel（此后转子电角 =
 *      mod((s_enc_pos - s_off_rel) × 方向, CPR)）。
 *   2. 进入 RUN：delta 从 g_dcl_dlt_init_deg（默认 5°，软吸附起步）
 *      线性爬坡到 g_dcl_dlt_targ_deg（默认 45°），历时
 *      g_dcl_dlt_tr_ms（默认 2000ms）。每 50µs 一拍：
 *        磁场角 = 转子实测电角 + delta_now
 *      到点后置 DCL_EVT_RAMP_DONE 事件一次。
 *   3. 电压幅值 g_dcl_volt_v（默认 0.6V）：调它 = 调转速（直流电机式）。
 *
 * 输出约定（q 轴电压，与 mode 26/30 一致）：
 *   磁场角 fa 直接作为控制变量输出：valpha=V·cos(fa), vbeta=V·sin(fa)。
 *   等效 d 轴角 theta = fa - 90°（g_foc_theta_rad 语义与 mode 26 统一），
 *   锁相稳态时控制系电流 g_foc_id_ma≈0, g_foc_iq_ma≈I。
 *
 * 编码器路径（本模式编码器从"观察者"升级为"控制者"）：
 *   - 增量累积式读法（与 mode 31 同思路）：每拍 wrap-safe 差分后
 *     ±DCL_ENC_DELTA_MAX(32) counts 限幅再累加。物理依据：7800rpm
 *     极限下单拍真实增量最多 26.6 counts，超过必是毛刺。
 *   - 限幅把单次毛刺对功角框架的永久污染封顶在 ±32 counts ≈ ±2.8° 电角
 *     （1 count = 360°/4096 ≈ 0.088° 机械 = 0.879° 电角）。
 *   - 无跟踪观测器（PLL）：量化噪声 ±0.44° 电角远小于实验精度需求，
 *     PLL 留待实测出现症状再加（v2，纯加法无侵入）。
 *
 * Watch 常用变量：
 *   g_dcl_dlt_init_deg / g_dcl_dlt_targ_deg / g_dcl_dlt_tr_ms
 *                     : 功角爬坡三参数（Start 不复位，预设后启动）
 *   g_dcl_volt_v      : 电压（=调速旋钮），Start 复位 0.6
 *   g_dcl_dlt_now_deg : 当前爬坡中的 delta 指令（实时）
 *   g_dcl_speed_hz    : 实测电频率（200ms 窗口，带符号）
 *   g_dcl_diff_deg    : 功角实测 = 磁场角-转子角（应≈dlt_now，验证用）
 *   g_dcl_id_ma / g_dcl_iq_ma : 真实转子系电流（id≈I·cosδ, iq≈I·sinδ）
 *   g_dcl_state       : 0 空闲 1 校准BETA 2 校准ALPHA 3 运行 4 过流
 *   g_dcl_enc_pos     : 编码器相对计数镜像（毛刺/限幅排查用）
 *
 * 实验提示：
 *   - 扫 g_dcl_dlt_targ_deg（5->45->89）看 g_dcl_speed_hz：90° 附近
 *     转速最高（MTPA，id 最小）——功角特性的闭环版验证
 *   - 长时间运行注意电机温升：delta 小角度时 id 大（铜损 ~2W 量级）
 *
 * ISR 约束：短小、无阻塞、无打印、无 malloc。打印全部由 foc_obs 在
 * 主循环完成（事件快照 + 200ms 周期）。
 *******************************************************************************
 */

#ifndef __FOC_DCL_H__
#define __FOC_DCL_H__

#include <stdint.h>
#include "foc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * Debug macros（RTT打印规范：0/1 赋值式开关，定义在 .h，.c 只调封装宏）
 *=============================================================================*/
/* 1 = RTT prints on, 0 = off */
#define FOC_DCL_DBG   1
#if FOC_DCL_DBG
    #define DCL_DBG(fmt, ...)   MAIN_D("[DCL] " fmt, ##__VA_ARGS__)
#else
    #define DCL_DBG(fmt, ...)   ((void)0)
#endif

/*=============================================================================
 * 时长/窗口（编译期常量，FOC_ISR_HZ tick 换算在 .c 内完成）
 *=============================================================================*/
#define DCL_BETA_MS       2000u  /* 校准 BETA 吸附时长 */
#define DCL_ALPHA_MS      2000u  /* 校准 ALPHA 吸附时长 */
#define DCL_SPEED_WIN_MS  200u   /* 转速测量窗口 */

/*=============================================================================
 * 编码器增量限幅（物理极限 7800rpm -> 单拍真实增量上限 ≈26.6 counts，
 * 超过判定为毛刺，钳位防功角框架永久污染）
 *=============================================================================*/
#define DCL_ENC_DELTA_MAX 32

/*=============================================================================
 * 状态机（g_dcl_state）
 *=============================================================================*/
#define DCL_STEP_IDLE       0u  /* 未运行 */
#define DCL_STEP_CAL_BETA   1u  /* 校准：磁场 90°，2s */
#define DCL_STEP_CAL_ALPHA  2u  /* 校准：磁场 0°，2s，结束锁零点 */
#define DCL_STEP_RUN        3u  /* 功角闭环拖动（磁场 = 转子 + delta） */
#define DCL_STEP_FAULT_OC   4u  /* 过流停机 */

/*=============================================================================
 * 事件码（g_dcl_evt，ISR 置位，Foc_Obs_Task 打印后清零）
 *=============================================================================*/
#define DCL_EVT_BETA_DONE  1u
#define DCL_EVT_LOCKED     2u
#define DCL_EVT_RAMP_DONE  3u
#define DCL_EVT_OC         4u

/*=============================================================================
 * Keil Watch 可调变量 / 观测量（定义见 foc_dcl.c）
 *=============================================================================*/
extern volatile float    g_dcl_dlt_init_deg;  /* 功角爬坡起点 (deg, 默认5, Start不复位) */
extern volatile float    g_dcl_dlt_targ_deg;  /* 功角爬坡终点 (deg, 默认45, Start不复位) */
extern volatile uint32_t g_dcl_dlt_tr_ms;     /* 功角爬坡过渡时间 (ms, 0=立即, Start不复位) */
extern volatile float    g_dcl_volt_v;        /* 电压幅值 (V, 默认0.6, Start复位; =调速旋钮) */
extern volatile uint8_t  g_dcl_running;       /* 1 = 正在运行 */
extern volatile uint8_t  g_dcl_state;         /* DCL_STEP_xxx */
extern volatile uint8_t  g_dcl_evt;           /* DCL_EVT_xxx */
extern volatile int32_t  g_dcl_offset;        /* 校准锁零点 (hw绝对帧counts, 显示/对比用) */
extern volatile float    g_dcl_dlt_now_deg;   /* 当前 delta 指令 (deg, 爬坡中实时) */
extern volatile float    g_dcl_speed_hz;      /* 实测电频率 (Hz, 200ms窗口, 带符号) */
extern volatile int32_t  g_dcl_field_deg;     /* 磁场电角度 (deg, 0~359) */
extern volatile int32_t  g_dcl_rotor_deg;     /* 转子电角度 (deg, 0~359, 已扣零点) */
extern volatile int32_t  g_dcl_diff_deg;      /* 功角实测 = field-rotor (deg, -180~180) */
extern volatile float    g_dcl_id_ma;         /* 真实转子系 id (mA, 磁链分量) */
extern volatile float    g_dcl_iq_ma;         /* 真实转子系 iq (mA, 力矩分量) */
extern volatile int32_t  g_dcl_enc_pos;       /* 编码器相对计数镜像 (毛刺排查用) */
extern volatile float    g_dcl_du;            /* 三相占空比观测 (%) */
extern volatile float    g_dcl_dv;
extern volatile float    g_dcl_dw;

/*=============================================================================
 * API
 *=============================================================================*/

/* mode 27 入口：清故障 -> 自动校准（BETA 2s + ALPHA 2s 锁零点）-> 功角闭环。
 * 主循环上下文调用（dev_comm_runner）。g_dcl_volt_v 复位 0.6V，
 * 功角三参数不复位（便于预设后启动）。 */
void Foc_Dcl_Start(void);

/* 20 kHz ISR 步进：OC 保护 -> 校准/锁相输出 -> 转速与电流观测。
 * 由 Foc_Isr 在 g_dcl_running 时分发调用。 */
void Foc_Dcl_Step(const stc_i_data_t *pData);

/* 停止（用户中途切模式时调用）：清运行标志 + 关 PWM（若在输出）。
 * 自带 g_foc_active 清零，防止 ISR 回退。 */
void Foc_Dcl_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_DCL_H__ */
