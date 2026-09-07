/**
 *******************************************************************************
 * @file  foc_iq_pi.h
 * @brief FOC 模式31 — 编码器转子角度 PI 电流环 (comm_mode 31)。
 *
 * ============================================================================
 * 【傻瓜式讲解：这个模式是干什么的】
 *
 * 一句话：真正"看着转"的闭环模式。每 50µs 读一次编码器知道转子在哪，
 *         用两个 PI 调节器精确控制电流，想要多大力矩就出多大力矩。
 *
 * 和 mode 30 的区别（重要）：
 *   mode 30 是"闭着眼施力"：磁场自己匀速转，不管转子在哪，靠磁力拖着走。
 *   mode 31 是"睁着眼施力"：先问编码器"转子现在在哪"，再算"要出这个力，
 *   电压该往哪个方向打、打多大"，每一拍都重新算。
 *
 * 前置条件：
 *   必须先跑过 mode 30 或 mode 32，拿到"偏移量"（编码器读数换算成磁铁
 *   真实角度要扣的修正值）。没锁定过偏移就启动会被拒绝（step=1）。
 *
 * 工作过程（按时间顺序）：
 *   1. 启动检查：偏移没锁定 -> 拒绝启动，step=1 (ERR_NO_OFFSET)。
 *   2. 校准窗口（step=2，约 210ms）：三相 50/50/50 不出力，测电流传感器
 *      零点误差。这是每次启动的必经步骤，不是卡住了。
 *   3. 闭环运行（step=3）：目标电流 iq_ref 从 0 按斜坡慢慢升到
 *      g_iqpi_iq_ref_ma（软启动，防电流冲击）。两个 PI 分别干活：
 *        - id PI：把"无用功电流" id 压到 0
 *        - iq PI：把"力矩电流" iq 压到目标值（力矩大小和 iq 成正比）
 *   4. 每 500ms 做一次"体检"（窗口检查）：
 *      （窗口从闭环真正启动那一刻才起算：校准窗口里转子的自由漂移
 *        不计入第一窗，防止把漂移误判成"转反了"而错误翻转框架）
 *        - 这 500ms 转子基本没动     -> step=5，疑似堵转
 *        - 转了，但方向和预期相反    -> 说明角度框架差了 180°（这种错误
 *          在电流读数上看不出来，只有转向能暴露），自动把角度基线翻转
 *          180° 修正，step=7，打印 [IQPI_FLIP]
 *        - vq/vd 顶到限幅           -> step=4，电压饱和（电压不够用，
 *          常见于转速太高或目标电流太大）
 *   5. 任何时刻过流 -> step=6，立即停机保护。
 *
 * RTT 怎么看它运行得好不好（每 200ms 一条 [IQPI_MON]，由 foc_obs 打印）：
 *   st    : 当前状态码（含义见下面枚举）
 *   iq/id : 实际电流 mA，正常应 iq≈rr、id≈0
 *   vq/vd : 电压指令 mV，±3500 = 顶满限幅（饱和）
 *   rr    : 斜坡后的目标电流 mA（启动时应从 0 慢慢涨上来）
 *   win   : 最近 500ms 编码器位移 counts（正负=方向，0=没动）
 *   ev    : 体检已做次数（0=一次都没做，方向检查没运行过）
 *   cd/ed : 实际方向 / 预期方向，正常应相等
 *   rd    : 启动时捕获的拖动方向基准
 *   flip  : 本次运行翻转次数（>0 说明发生过 180° 修正）
 *   pos   : 编码器累积计数（看转速和方向最直观）
 *
 * Watch 可调参数：
 *   g_iqpi_iq_ref_ma    : 目标力矩电流 mA，正负号决定转向，运行中可改
 *   g_iqpi_iq_ramp_ma_s : 电流斜坡斜率 mA/s
 *   g_iqpi_pid_iq_cfg / g_iqpi_pid_id_cfg : PI 参数（kp/ki 已按 1kHz
 *       带宽整定；带宽不得超过 PWM 频率的 1/10，不要随意加大）
 *
 * 状态历史：g_iqpi_step_hist[]（在 foc_obs 观察模块）像行车记录仪一样
 *   记录最近 10 次状态变化，出问题回放即可知道它在哪一步出的错。
 *
 * ISR 约束：短小、无阻塞、无打印、无 malloc。
 *******************************************************************************
 */

#ifndef __FOC_IQ_PI_H__
#define __FOC_IQ_PI_H__

#include <stdint.h>
#include "foc_core.h"
#include "../Utils/dev_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * Debug macros
 *=============================================================================*/
/* 1 = RTT prints on, 0 = off */
#define FOC_IQPI_DBG   1
#if FOC_IQPI_DBG
    #define IQPI_DBG(fmt, ...)     MAIN_D("[IQPI] " fmt, ##__VA_ARGS__)
#else
    #define IQPI_DBG(fmt, ...)     ((void)0)
#endif

/*=============================================================================
 * PI 初始参数（编译期宏，Keil Watch 中可运行时实时覆盖）
 *   带宽设计：fc ≈ kp/(2π·Ls)，Ls=42.3µH, Rs=0.1Ω
 *   kp=0.27 -> fc≈1kHz = PWM(10kHz)/10，规则上限；
 *   零点对消整定 ki = kp·Rs/Ls ≈ 630
 *   如振荡可 Watch 运行时下调 kp/ki（0.10/240 与 mode22 一致，已验证）
 * ==========================================================================*/
#define IQPI_PI_KP           0.27f     /* d/q 轴 PI 比例增益 */
#define IQPI_PI_KI           630.0f    /* d/q 轴 PI 积分增益（零极点对消） */
#define IQPI_PI_UMAX_V       3.5f      /* PI 输出限幅 (V)，|vd/vq| ≤ UMAX
                                        (SVPWM 线性区上限 12/√3≈6.9V，留足余量) */
#define IQPI_INTEGRAL_MAX    3.0f      /* 积分项限幅：必须 > 运行时bemf(否则高速段
                                        积分扛不住反电动势，稳态droop回来) */

/* iq 目标初值 (mA) 与软启动斜率 (mA/s)，Watch 运行时可改 g_iqpi_iq_ref_ma */
#define IQPI_IQ_REF_MA       1500.0f
#define IQPI_IQ_RAMP_MA_S    1000.0f

/* 堵转检测参数（g_iqpi_step = IQPI_STEP_RUNNING_STALL 的判定条件）：
 * 斜坡参考和实测 iq 都超过 IQPI_STALL_IQ_MIN_MA 且持续一个窗口期后，
 * 窗口内机械计数位移 < IQPI_STALL_MIN_CNTS 判为疑似堵转 */
#define IQPI_STALL_WIN_MS      500u            /* 检测窗口 (ms) */
#define IQPI_STALL_MIN_CNTS    (ENCODER_CPR/20u) /* 窗口内位移低于此值算"没动" */
#define IQPI_STALL_IQ_MIN_MA   300.0f          /* 电流低于此值不评估(斜坡初期不误报) */

/* ============================================================================
 * mode 31 运行状态机 (g_iqpi_step) — 启动失败/转动异常时看此值定位原因
 *   Keil Watch 中按枚举名显示；停止(mode 0)后保持最后状态不清除，
 *   下次成功启动时复位
 * ==========================================================================*/
typedef enum {
    IQPI_STEP_IDLE            = 0,  /* 上电初始/未启动 */
    IQPI_STEP_ERR_NO_OFFSET   = 1,  /* 启动被拒: mode30 偏移未锁定 -> 先跑 mode 30 */
    IQPI_STEP_PWM_ZERO_VECTOR = 2,  /* 已启动: 零矢量 + 零偏校准窗口(~210ms)，正常必经 */
    IQPI_STEP_CLOSED_LOOP     = 3,  /* 闭环正常运行 */
    IQPI_STEP_RUNNING_VQ_SAT  = 4,  /* 闭环但 vq/vd 顶满限幅: 电压饱和
                                       (bemf 过高 / iq_ref 过大 / UMAX 太小) */
    IQPI_STEP_RUNNING_STALL   = 5,  /* 闭环但窗口内转子几乎未动: 疑似堵转
                                       (静摩擦不足 / 框架角度错误 / 电压饱和连带) */
    IQPI_STEP_FAULT_OC        = 6,  /* 过流保护停机 */
    IQPI_STEP_DIR_FLIPPED     = 7,  /* 检测到转向与 mode 30 相反(180°框架误差),
                                       已自动翻转框架修正, 数秒内应回到 3/4 */
} iqpi_step_t;

/* ============================================================================
 * Keil Watch 可调变量 / 观测量（定义见 foc_iq_pi.c）
 *   状态历史 g_iqpi_step_hist、转向诊断量 g_iqpi_win_*、g_iqpi_cur_dir 等、
 *   翻转事件快照 g_iqpi_evt_* 已集中迁移到观察模块 foc_obs.h（定义在
 *   foc_obs.c），变量名未变，Keil Watch 用法不变。
 * ==========================================================================*/
extern volatile uint8_t g_iqpi_running;        /* 1 = 模式31 运行中 */
extern volatile iqpi_step_t g_iqpi_step;       /* 模式31 当前/最后状态 (Watch 看枚举名) */
extern volatile float   g_iqpi_iq_ref_ma;      /* iq 目标 (mA)，Watch 可实时修改 */
extern volatile float   g_iqpi_iq_ref_ramp_ma; /* 斜坡后的实际 iq 参考 (mA) */
extern volatile float   g_iqpi_iq_ramp_ma_s;   /* iq 斜率 (mA/s)，Watch 可调 */
extern volatile float   g_iqpi_theta_rad;      /* 当前使用的转子电角度 (rad) */

/* d/q 轴 PI 配置（字段 volatile，Watch 可实时改 kp/ki/限幅） */
extern pid_config_t g_iqpi_pid_id_cfg;
extern pid_config_t g_iqpi_pid_iq_cfg;

/*******************************************************************************
 * API
 ******************************************************************************/

/* PI 实例绑定配置（Foc_Init 调用一次） */
void Foc_IqPi_InitPids(void);

/* 启动模式31：需先跑过 mode 30 锁定 ZIZENG 偏移，否则拒绝启动。
 * 内部：清故障 -> PWM 启动（零矢量）-> foc_calib 校准窗口 -> 闭环。 */
void Foc_StartIqPi(void);

/* 停止模式31 */
void Foc_StopIqPi(void);

/* 模式31 单步运算（20 kHz ISR 中由 Foc_Isr 分发调用） */
void Foc_IqPi_Step(const stc_i_data_t *pData);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_IQ_PI_H__ */
