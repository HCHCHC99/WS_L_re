/**
 *******************************************************************************
 * @file  foc_speed40.h
 * @brief FOC mode 40 - encoder FOC with cascade speed/current loops.
 *
 * The speed PI runs on a fixed 5 ms tick and generates the q-axis current
 * reference.  The id reference is zero.  The independent id/iq current PIs
 * run every current-sample ISR.  Mode 40 requires a valid mode 24 calibration
 * before start.
 *
 * ============================ 模式速览卡（唯一事实源）========================
 * 模式：40 = 编码器 FOC 速度/电流双闭环（无自动爬坡，目标纯手动设置，
 *       内部经斜坡限幅整形）
 *
 * 【Watch 可调变量】（名称 = 默认值 单位）
 *   g_speed40_speed_target_rpm = 0     rpm       目标转速
 *   g_speed40_iq_filt_alpha    = 0.10  -         iq 反馈滤波系数
 *   g_speed40_pid_speed_cfg.kp = 2.0   mA/rpm    速度环 P（Watch 改立即生效）
 *   g_speed40_pid_speed_cfg.ki = 0.45  mA/rpm/s  速度环 I（同上）
 *        ↑ 2026-09-23：kp 由 1.8 调到 2.0，ki 0.2→0.45
 *          ⚠ 速度环 P 是唯一的带宽旋钮（ωc≈72·kp）；ki 只管稳态精度
 *   g_speed40_pid_id_cfg / iq_cfg .kp = 0.1  V/A  电流环 P（带 6.2V 输出限幅）
 *   g_speed40_pid_id_cfg / iq_cfg .ki = 300  V/(A·s)
 *        ↑ 2026-09-23 实测：kp 0.5→0.1 大幅降低静止噪声（0.1 与 0.05 无差别，
 *          已到底噪）。⚠ 0.1 是串级结构下限，勿再降；完整机理见下方宏定义处。
 *
 * 【关键观察变量】
 *   g_speed40_speed_filt_rpm   PI 反馈真实转速（调参判超调/振铃看这个；
 *                              VOFA 已引出到 ch17，CH15 显示滤波值 α=0.05
 *                              滞后大只看趋势）
 *   g_speed40_vd / g_speed40_vq 电流环输出电压；vq 顶到 ~6.2V = 电压墙
 *   g_speed40_vsat             电流环饱和标志（1=输出贴限幅）
 *   g_speed40_state / g_speed40_evt  状态机与事件（2=过流故障停机）
 *
 * 【VOFA 通道】固定 18ch（mode40 专属语义，填充见 Foc_Speed40_VofaFill）：
 *   ch0~2 三相电流(A)  ch3 静止系ialpha(A)  ch4 静止系ibeta(A)
 *   ch5 iq(A)  ch6 id参考(A)  ch7 iq参考(A)
 *   ch8 vd(V)  ch9 vq(V)             ← 验电压墙看 ch9 是否贴 6.2V
 *   ch10 静止系电流幅值(A)  ch11~12 预留 0
 *   ch13 斜坡输出转速(rpm)  ← CH15 追 CH14 慢时：ch13 慢=斜坡限幅，
 *                              ch13 快 ch15 慢=显示滤波/环路滞后
 *   ch14 目标转速(rpm)  ch15 实际转速-显示强滤波(rpm)  ch16 iq滤波(A)
 *   ch17 实际转速-PI 反馈(rpm)  ← 速度环真正吃的反馈（α=0.25 @5ms，τ≈20ms）
 *        调外环只认 ch17：它和 ch13（斜坡给定）之差决定 P 项大小。
 *        ch15 是 α=0.05 显示强滤波（τ≈100ms），**刻意更滞后**，只用来看趋势；
 *        两者稳态重合，动态过程中 ch15 必然落后，属预期不是故障。
 *        ⚠ VOFA+ 需同步把通道数配成 18。
 * ===========================================================================
 */

#ifndef __FOC_SPEED40_H__
#define __FOC_SPEED40_H__

#include <stdint.h>
#include "foc_core.h"
#include "../Utils/dev_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPEED40_DBG   1
#if SPEED40_DBG
#define SPEED40_LOG(fmt, ...)  MAIN_D("[SPEED40] " fmt, ##__VA_ARGS__)
#else
#define SPEED40_LOG(fmt, ...)  ((void)0)
#endif

/* Inner current PI.
 * ⚠ 2026-09-23 实测调参：kp 0.5 → 0.1（= mode 40 静止噪声的处置，已在用）
 *
 * 现象：目标 0、静止时电机有嘈杂噪声，且电流反馈真的在振荡
 *       （VOFA ch5 iq 反馈 +1.21/−0.8A，ch8 vd ±0.3V、ch9 vq ±0.5V 不规则抖动）。
 * 机理：开关纹波 → 无抗混叠的电流反馈原样采入 → PI 的 P 项（kp × 纹波）放大
 *       → 占空比被推偏 → 纹波更大 = 正反馈极限环。kp 直接决定这条路的增益。
 * 实测：kp=0.5 → 明显噪声；kp=0.1 → 大幅下降；kp=0.05 → 与 0.1 无差别
 *       ⇒ 0.1 以下噪声已到底噪（PWM 开关本身），再降无收益。
 *
 * ⚠⚠ 0.1 是**结构下限，不要再降**：
 *   电流环带宽 ωc = kp/L = 0.1/42.3µH = 376 rad/s ≈ 60 Hz，
 *   仅比速度环（kp=2 → 约 36 Hz）快 1.7 倍，**已低于串级经验要求 5~10 倍**。
 *   再降内环会慢过外环，串级结构失效。
 *   要恢复带宽必须从硬件侧切断纹波（传感器输出 1kΩ+100nF → fc≈1.6kHz，
 *   10kHz 处 −16dB，且在环路之外不消耗相位裕度），之后 kp 可回到 0.3~0.5。
 *
 * 副产物：kp=0.1 时电流环 ζ = (R+kp)/(2√(ki·L)) = 0.2/(2×0.1127) ≈ 0.89，
 *   接近临界阻尼（ζ=1 对应 kp≈0.108），比原来的过阻尼(2.66)更"刚好"。
 *
 * 注：运行时可用 Watch 改 g_speed40_pid_id_cfg / iq_cfg .kp，此处为上电默认值。 */
#define SPEED40_PI_KP             0.1f
#define SPEED40_PI_KI             300.0f
/* ⚠ UMAX 3.5→6.2 / ITERM 3.2→6.0（与 mode 45 SMO45 同步，2026-09-23）：
 *   12V 母线 SVPWM 线性区相电压峰值 6.93V，原 3.5V 是高速电压墙（卡 ~4800rpm） */
#define SPEED40_PI_UMAX_V         6.2f
#define SPEED40_ITERM_MAX_V       6.0f
#define SPEED40_IQ_FILT_ALPHA     0.10f

/* Outer speed PI output is a signed q-axis current reference in mA.
 * 2026-09-23 调参：kp 1.8 → **2.0**，ki 0.2 → 0.45。
 *   2.0 是折中：比原 1.8 略快，但不逼近 200Hz 采样率的带宽极限
 *   （kp=4 时 ωc≈46Hz，约为速度环采样率 200Hz 的 1/4.3，偏激进）。
 *
 * 理论要点（详见 md_record/mode40控制系统_纯理论计算手册.md）：
 *   ωc ≈ 72·kp（在机械阻尼 kfb=2 下）→ kp=2.0 约 36 Hz
 *   相位裕度随 kp 上升而下降；随 ki 上升而**上升**（PI 零点抬高提供相位超前）
 *   ⚠ 速度环 P 是唯一的带宽旋钮——改 ki 改不了响应速度（PI 零点比穿越频率低 3 个数量级）
 *
 * 注：运行时可用 Watch 改 g_speed40_pid_speed_cfg.kp/.ki，此处仅为上电默认值。 */
#define SPEED40_SPD_KP_MA_PER_RPM       2.0f
#define SPEED40_SPD_KI_MA_PER_RPM_S     0.45f
/* 速度环输出限幅 = 最大电流 × 20%（17A × 0.2 = 3400mA）。
 * 取 20% 而非更高：17A 是峰值能力（厂商规格书原文「最大电流」），
 * 按其 20% 折算已相当于一个合理的连续工作点。 */
#define SPEED40_SPD_IQ_LIMIT_MA         ((float)FOC_MOTOR_MAX_CURRENT_A \
                                        * 1000.0f * 0.20f)

/* Safety envelope derived from motor_config.h. */
#define SPEED40_SPEED_REF_LIMIT_RPM     ((float)FOC_MOTOR_MAX_SPEED_RPM)
#define SPEED40_ACCEL_LIMIT_RPM_S       ((float)FOC_MOTOR_MAX_SPEED_RPM \
                                        * 0.25f)

#define SPEED40_SPD_WIN_MS              5u
#define SPEED40_SPD_WIN_US              (SPEED40_SPD_WIN_MS * 1000u)
#define SPEED40_SPD_FILT_ALPHA          0.25f
/* 显示专用滤波（VOFA 曲线平滑用；α 越小越平滑越滞后，不进 PI 反馈） */
#define SPEED40_SPD_DISP_ALPHA          0.05f
/* 编码器每拍增量限幅 = 单拍物理极限 × 1.35 裕度。
 * 物理极限 = 7800rpm 折算到"每个 ISR 拍"的 counts 数。
 *   10 kHz（100µs/拍）：53 counts → 限幅 72
 *   20 kHz（ 50µs/拍）：26.5 counts → 限幅 36   ← 2026-09-23 随 PWM 频率提升同步改
 * 历史教训：原值 32 只支持 4687rpm（10kHz 时），超限导致测速削顶 +
 *   角度积分丢拍（高速下 Park 角落后）。限幅必须始终 ≥ 物理极限，
 *   否则高速时会削顶；但也不能过大，否则编码器毛刺被当真转速放行。
 * ⚠ 若再改 MOTOR_PWM_FREQ_HZ，此值须按 1/f 同步缩放 */
#define SPEED40_ENC_DELTA_MAX           36

#define SPEED40_STEP_IDLE               0u
#define SPEED40_STEP_RUN                1u
#define SPEED40_STEP_FAULT_OC           2u

#define SPEED40_EVT_OC                  1u

extern volatile uint8_t  g_speed40_running;
extern volatile uint8_t  g_speed40_state;
extern volatile uint8_t  g_speed40_evt;
extern volatile float    g_speed40_speed_target_rpm;
extern volatile float    g_speed40_speed_ramp_rpm;
extern volatile float    g_speed40_speed_meas_rpm;
extern volatile float    g_speed40_speed_filt_rpm;
extern volatile float    g_speed40_speed_disp_rpm;  /* 显示专用强滤波转速（α=0.05，
                                              * 仅 VOFA/Watch 看趋势；PI 反馈仍用
                                              * speed_filt_rpm，勿混用防环路滞后） */
extern volatile float    g_speed40_speed_err_rpm;
extern volatile float    g_speed40_speed_out_ma;
extern volatile int32_t  g_speed40_rotor_count;
extern volatile int32_t  g_speed40_rotor_deg;
extern volatile float    g_speed40_id_ref_ma;
extern volatile float    g_speed40_iq_ref_ma;
extern volatile float    g_speed40_id_ma;
extern volatile float    g_speed40_iq_ma;
extern volatile float    g_speed40_iq_filt_ma;
extern volatile float    g_speed40_iq_filt_alpha;
extern volatile float    g_speed40_vd;
extern volatile float    g_speed40_vq;
extern volatile uint8_t  g_speed40_vsat;
extern volatile float    g_speed40_du;
extern volatile float    g_speed40_dv;
extern volatile float    g_speed40_dw;

extern pid_config_t g_speed40_pid_speed_cfg;
extern pid_config_t g_speed40_pid_id_cfg;
extern pid_config_t g_speed40_pid_iq_cfg;

void Foc_Speed40_InitPids(void);
void Foc_Speed40_SetTargetRPM(float target_rpm);
void Foc_Speed40_Start(void);
void Foc_Speed40_Stop(void);
void Foc_Speed40_Step(const stc_i_data_t *pData);
int  Foc_Speed40_VofaFill(int32_t *cur);  /* 模式自持 VOFA，见顶部速览卡 */

#ifdef __cplusplus
}
#endif

#endif /* __FOC_SPEED40_H__ */
