/**
 *******************************************************************************
 * @file  foc_iq_pi.h
 * @brief FOC 模式31 — 编码器转子角度 PI 电流环 (comm_mode 31)。
 *
 *        前置条件：先运行 mode 30 (ZIZENG) 完成偏移锁定，使编码器电角度
 *        绝对化（扣除 ZIZENG 锁定的基线偏移），本模式即可直接闭环。
 *
 *        控制流程（20 kHz ISR）：
 *          TIMERA_1 硬件计数 -> 转子电角度（扣 ZIZENG 偏移） -> Park
 *          -> id PI(id_ref=0) / iq PI(iq_ref) -> InvPark -> SVPWM
 *
 *        启动时复用 foc_calib 零偏自校准窗口（~210ms 零矢量），
 *        锁定后 iq_ref 从 0 软启动斜坡至目标值。
 *
 *        ISR 约束：短小、无阻塞、无打印、无 malloc。
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

/* 状态历史（每次状态变化记录一条，hist[0] 最旧，hist[cnt-1] 最新；
 * 成功启动 mode 31 时清空重记。Watch 直接看数组各元素的枚举名） */
#define IQPI_HISTORY_LEN  10u

/* ============================================================================
 * Keil Watch 可调变量 / 观测量（定义见 foc_iq_pi.c）
 * ==========================================================================*/
extern volatile uint8_t g_iqpi_running;        /* 1 = 模式31 运行中 */
extern volatile iqpi_step_t g_iqpi_step;       /* 模式31 当前/最后状态 (Watch 看枚举名) */
extern volatile iqpi_step_t g_iqpi_step_hist[IQPI_HISTORY_LEN]; /* 状态历史, [0]最旧 */
extern volatile uint8_t g_iqpi_step_hist_cnt;  /* 历史有效条数 0..10 */
extern volatile uint8_t g_iqpi_flip_cnt;       /* 本次运行中框架180°自动翻转次数 */
extern volatile float   g_iqpi_iq_ref_ma;      /* iq 目标 (mA)，Watch 可实时修改 */
extern volatile float   g_iqpi_iq_ref_ramp_ma; /* 斜坡后的实际 iq 参考 (mA) */
extern volatile float   g_iqpi_iq_ramp_ma_s;   /* iq 斜率 (mA/s)，Watch 可调 */
extern volatile float   g_iqpi_theta_rad;      /* 当前使用的转子电角度 (rad) */

/* --- 转向诊断观测量（ISR 更新；main.c 周期打印 [IQPI_MON]） --- */
extern volatile int32_t  g_iqpi_enc_pos;       /* ISR 累积编码器计数镜像 */
extern volatile int32_t  g_iqpi_win_moved;     /* 最近完成的500ms窗口位移(counts,带符号) */
extern volatile uint32_t g_iqpi_win_evals;     /* 已完成窗口评估次数(0=方向检查从未运行) */
extern volatile int8_t   g_iqpi_cur_dir;       /* 最近窗口实测方向 +1/-1 */
extern volatile int8_t   g_iqpi_expect_dir;    /* 预期方向 +1/-1 */
extern volatile int8_t   g_iqpi_ref_dir;       /* 启动时捕获的 mode30 拖动方向 */

/* --- 翻转事件快照（ISR 置 g_iqpi_evt_flag，main.c 打印后清零） --- */
extern volatile uint8_t  g_iqpi_evt_flag;
extern volatile uint8_t  g_iqpi_evt_seq;       /* 第几次翻转 (1,2,...) */
extern volatile int32_t  g_iqpi_evt_pos;       /* 翻转时编码器计数 */
extern volatile int32_t  g_iqpi_evt_iq_ma;     /* 翻转时 iq (mA) */
extern volatile int32_t  g_iqpi_evt_vq_mv;     /* 翻转时 vq (mV) */
extern volatile int32_t  g_iqpi_evt_off_mrad;  /* 翻转后偏移基线 (mrad) */

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
