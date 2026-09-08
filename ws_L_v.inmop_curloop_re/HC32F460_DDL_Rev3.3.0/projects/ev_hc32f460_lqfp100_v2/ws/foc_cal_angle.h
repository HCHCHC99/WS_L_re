/**
 *******************************************************************************
 * @file  foc_cal_angle.h
 * @brief FOC 模式25 — 手动角度吸附 (comm_mode 25)。
 *
 * ============================================================================
 * 【傻瓜式讲解：这个模式是干什么的】
 *
 * 一句话：你在 Watch 里写一个角度（0~360 电角度），磁场就指向那个方向，
 *         把转子"吸"过去，吸 2 秒后用编码器检查有没有吸稳，然后回到刹车
 *         （50/50/50 零矢量），等你写下一个角度。
 *
 * 它是干什么用的？
 *   帮你直观理解 SVPWM 控制电机的完整链路：
 *     你写的角度 θ(电角度)
 *       -> valpha = V·cos(θ), vbeta = V·sin(θ)   （αβ 静止系电压指令）
 *       -> Foc_Svpwm 算出三相占空比 du/dv/dw     （看 g_calang_du/dv/dw！）
 *       -> 三相绕组合成一个指向 θ 的磁场
 *       -> 转子 N 极被吸到 θ 方向                 （编码器实测验证）
 *
 * 工作过程（按时间顺序）：
 *   1. 进入 mode 25：自动先跑一遍 mode 20 式校准（BETA 90° 吸 2s ->
 *      ALPHA 0° 吸 2s -> 锁零点 offset），保证"0°"有绝对意义。
 *      注意：校准期间预设的 g_foc_angle_input 会在校准完成后立即执行。
 *   2. 校准完成 -> 刹车状态（50/50/50 零矢量，不出力），模式停在 25。
 *   3. 检测到 g_foc_angle_input 改值（x -> y），吸附 2 秒分三段：
 *      引导点 300ms（触发瞬间实测转子位置判来向，先吸到"转子一侧
 *      15°"，让最后一段拉程短、到站动量小；转子已在目标 ±5° 内则
 *      跳过）-> 目标角 ±3° 抖动 400ms（50Hz，拔齿槽/破静摩擦）
 *      -> 纯目标角 1.3s（落座）。
 *   4. 500ms 校验窗（磁场仍吸在 y）：窗口内编码器位移 max-min <=
 *      g_calang_stable_cnts（默认 8 counts）-> 成功（吸稳了）；
 *      还在动 -> 失败。
 *   5. 无论成败都回刹车 50/50/50，停在 mode 25 等下一次输入。
 *      只有过流会自动退回 mode 0。
 *
 * 精度预期（开环静态吸附的物理极限）：
 *   编码器量化底噪 ±0.88° 电角度/count；静摩擦死区（转子停在
 *   吸引转矩=摩擦转矩处）与齿槽转矩是误差主项。引导点+抖动后
 *   典型 ±4~5° 电角度（±0.5° 机械角）。想再准就得闭环，那就不是
 *   开环 SVPWM 教学了。
 *
 * g_foc_angle_input 的语义与陷阱（重要）：
 *   - 单位是电角度（0~360），静止系；校准后 0° = 转子 N 极零位
 *   - 渐进扫描（每步 <=90°）：90 -> mech 9°、180 -> 18°、270 -> 27°、
 *     0/360 -> 36°，每步 +9° 机械（360°/10 对极），四步一个电周期
 *   - 陷阱 1：跳变 >180° 电角度时转子走"短路径"（0° 直接设 270° 会
 *     倒退到 mech -9°=351°，因为磁场在静止系没有"圈数"概念）
 *   - 陷阱 2：恰好 180° 跳变转矩为零（sin180°=0），死点，避开
 *   - 0° 与 360° 等价（代码内 wrap）
 *
 * Watch 常用变量：
 *   g_foc_angle_input : 你要写的目标电角度（deg，改值即触发）
 *   g_calang_volt_v   : 吸附电压（默认 0.4V ≈ 4A，运行中可调）
 *   g_calang_state    : 0=空闲 1=校准BETA 2=校准ALPHA 3=刹车等待
 *                       4=吸附2s 5=校验500ms 6=过流停机
 *   g_calang_du/dv/dw : 三相占空比（%，看 SVPWM 波形的重点变量）
 *   g_calang_target_deg / g_calang_meas_deg / g_calang_err_deg :
 *                       最近一次吸附的目标角/实测角/误差（deg 快照）
 *   g_calang_win_moved: 校验窗内位移（counts，> g_calang_stable_cnts 即失败）
 *
 * ISR 约束：短小、无阻塞、无打印、无 malloc。打印全部由 foc_obs 在
 * 主循环完成（事件快照模式，与 mode 20/23/31/32 一致）。
 *******************************************************************************
 */

#ifndef __FOC_CAL_ANGLE_H__
#define __FOC_CAL_ANGLE_H__

#include <stdint.h>
#include "foc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * Debug macros（RTT打印规范：0/1 赋值式开关，定义在 .h，.c 只调封装宏）
 *=============================================================================*/
/* 1 = RTT prints on, 0 = off */
#define FOC_CAL_ANGLE_DBG   1
#if FOC_CAL_ANGLE_DBG
    #define CALANG_DBG(fmt, ...)   MAIN_D("[CALANG] " fmt, ##__VA_ARGS__)
#else
    #define CALANG_DBG(fmt, ...)   ((void)0)
#endif

/*=============================================================================
 * 时长（编译期常量，FOC_ISR_HZ tick 换算在 .c 内完成）
 *=============================================================================*/
#define CALANG_BETA_MS     2000u  /* 校准 BETA 吸附时长 */
#define CALANG_ALPHA_MS    2000u  /* 校准 ALPHA 吸附时长 */
#define CALANG_HOLD_MS     2000u  /* 目标角吸附时长 */
#define CALANG_VERIFY_MS    500u  /* 吸稳校验窗时长 */

/*=============================================================================
 * 状态机（g_calang_state）
 *=============================================================================*/
#define CALANG_STEP_IDLE         0u  /* 未运行 */
#define CALANG_STEP_CAL_BETA     1u  /* 校准：磁场 90°，2s */
#define CALANG_STEP_CAL_ALPHA    2u  /* 校准：磁场 0°，2s，结束锁零点 */
#define CALANG_STEP_BRAKE_WAIT   3u  /* 刹车 50/50/50，等输入改值 */
#define CALANG_STEP_HOLD_ATTRACT 4u  /* 磁场指向目标角，吸附 2s */
#define CALANG_STEP_HOLD_VERIFY  5u  /* 磁场保持，500ms 校验窗判吸稳 */
#define CALANG_STEP_FAULT_OC     6u  /* 过流停机 */

/*=============================================================================
 * 事件码（g_calang_evt，ISR 置位，Foc_Obs_Task 打印后清零）
 *=============================================================================*/
#define CALANG_EVT_BETA_DONE  1u
#define CALANG_EVT_LOCKED     2u
#define CALANG_EVT_DONE_OK    3u
#define CALANG_EVT_DONE_FAIL  4u
#define CALANG_EVT_OC         5u

/*=============================================================================
 * Keil Watch 可调变量 / 观测量（定义见 foc_cal_angle.c）
 *=============================================================================*/
extern volatile int32_t  g_foc_angle_input;  /* 用户输入目标电角度 (deg, 改值即触发) */
extern volatile float    g_calang_volt_v;    /* 吸附电压 (V, 默认 FOC_ALIGN_VOLT_V) */
extern volatile uint8_t  g_calang_running;   /* 1 = 正在运行 */
extern volatile uint8_t  g_calang_state;     /* CALANG_STEP_xxx */
extern volatile uint8_t  g_calang_evt;       /* CALANG_EVT_xxx */
extern volatile int32_t  g_calang_stable_cnts; /* 校验窗静止判据 (counts, 默认 8) */
extern volatile int32_t  g_calang_target_deg;  /* 快照：锁存的目标角 (deg) */
extern volatile int32_t  g_calang_meas_deg;    /* 快照：校验结束实测电角度 (deg, 0-360) */
extern volatile int32_t  g_calang_err_deg;     /* 快照：meas-target 折叠 (-180,180] */
extern volatile int32_t  g_calang_win_moved;   /* 快照：校验窗位移 (counts, max-min) */
extern volatile int32_t  g_calang_offset;      /* 快照：校准锁定的零点 (counts) */
extern volatile float    g_calang_du;          /* 三相占空比观测 (%) — SVPWM */
extern volatile float    g_calang_dv;
extern volatile float    g_calang_dw;

/*=============================================================================
 * API
 *=============================================================================*/

/* mode 25 入口：清故障 -> 自动校准（BETA 2s + ALPHA 2s）-> 刹车等待输入。
 * 主循环上下文调用（dev_comm_runner）。 */
void Foc_CalAngle_Start(void);

/* 20 kHz ISR 步进：OC 保护 -> 按状态输出磁场/刹车 -> 计时/判据跳转。
 * 由 Foc_Isr 在 g_calang_running 时分发调用。 */
void Foc_CalAngle_Step(const stc_i_data_t *pData);

/* 停止（用户中途切模式时调用）：清运行标志 + 关 PWM（若在输出）。
 * 自带 g_foc_active 清零，防止 ISR 回退。 */
void Foc_CalAngle_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_CAL_ANGLE_H__ */
