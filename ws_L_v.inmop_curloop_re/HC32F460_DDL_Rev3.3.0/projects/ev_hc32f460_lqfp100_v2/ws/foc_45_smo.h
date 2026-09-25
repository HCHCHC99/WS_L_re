/**
 *******************************************************************************
 * @file  foc_45_smo.h
 * @brief FOC mode 45 - SMO+PLL 无感速度/电流双闭环（分步开发中）。
 *
 * 第 1 步（已完成）：完全复刻 mode 40（编码器 FOC 速度/电流双闭环）
 * + 启动转速自动爬坡 profile（0→200→500→1000→1500→2000 rpm）+
 * SMO 纯旁观（foc_smo.c，编码器闭环不受影响）。
 * 第 2 步（已完成）：PLL 旁观（foc_pll.c）从 e_hat 提取 theta_hat/omega_hat，
 * 与编码器对比。
 * 第 3 步（当前）：g_smo45_sensorless=1 切无感（θ_park 取 PLL 补偿外推角），
 * 编码器角保留作裁判。
 *
 * 结构（与 mode 40 相同）：
 *   速度 PI（5ms 节拍）输出 q 轴电流参考 (mA)，id 参考 = 0；
 *   独立 id/iq 电流 PI 在每个电流采样 ISR（= FOC_ISR_HZ，当前 20kHz）执行。
 *   启动前置：有效的 mode 24 校准（foc_24_dcal 快照）。
 *
 * ============================ 模式速览卡（唯一事实源）========================
 * 模式：45 = SMO + PLL 无感 FOC（滑模观测器 + 锁相环，速度/电流双闭环）
 *        第 1 步 = 复刻 mode 40 + 自动转速 profile；编码器角保留作裁判
 *        无感切换后转子角由 SMO/PLL 估计给出（g_smo45_sensorless=1 申请）
 * 入口：comm_mode = 45
 * 前置：**必须先跑 mode 24**（读 foc_24_dcal 快照）
 * 结束：持续运行；OC 自动停
 * 流程：mode 24 校准 → mode 45 → 自动爬坡到 1000rpm → Watch 接管调目标
 *
 * 【Watch 可调变量】（名称 = 默认值 单位）
 *   g_smo45_speed_target_rpm = 0     rpm       目标转速（爬坡到顶后接管）
 *   g_smo45_auto_ramp        = 1     -         启动自动爬坡开关（0=纯手动）
 *   g_smo45_sensorless       = 0     -         无感切换开关（1=申请切换，需
 *                                              g_smo_emf_ok=1；回 0 退出）
 *   g_smo45_ang_lead_ticks   = 1.5   拍        有感角度超前补偿（0=关，
 *                                              高速 4500rpm+ 必需）
 *   g_smo45_iq_filt_alpha    = 0.10  -         iq 反馈滤波系数
 *   g_smo45_wave_mode        = 0     -         VOFA 布局选择（见下）
 *   g_smo45_pid_speed_cfg.kp = 1.8   mA/rpm    速度环 P（Watch 改立即生效，
 *                                              稳态中小步 ±20% 调）
 *   g_smo45_pid_speed_cfg.ki = 0.2   mA/rpm/s  速度环 I（同上）
 *   g_pll_comp_bias_deg      = 0     deg       PLL 恒定偏置校准（无感稳态
 *                                              角度误差均值补偿，见 foc_pll.h）
 *
 * 【关键观察变量】
 *   g_smo45_speed_filt_rpm   PI 反馈真实转速（调参判抖动看这个，勿用
 *                            CH14 显示滤波值 α=0.05，其滞后大只看趋势）
 *   g_smo45_vd / g_smo45_vq  电流环输出电压；vq 顶到 ~6.2V = 电压墙
 *   g_smo45_vsat             电流环饱和标志（1=输出贴限幅）
 *   g_smo45_sl_active        无感锁存状态（1=Park 角已用 PLL 无感量）
 *   g_smo_emf_ok             SMO 反电动势自动判定通过（foc_smo.h 有判据）
 *   g_smo_jdg_fail           判定失败掩码 bit0 转速/bit1 幅值/bit2 相位
 *   g_smo45_state / g_smo45_evt  状态机与事件（2=过流故障停机）
 *
 * 【VOFA 通道】**自持布局，通道数随 g_smo45_wave_mode 变**（实现见
 *   foc_45_smo.c Foc_Smo45_VofaFill；单位换算：传"毫单位"，×0.001）：
 *   ⚠ 本模式是**变长帧（8 或 16ch）**，与本工程其它模式不同 ——
 *     切 wave_mode 时 VOFA+ 通道数必须同步跟着改。
 *   wave_mode=0：**16ch** SMO 验收面板（直流观测量为主）
 *     ch0 iq(A) ch1 iq滤波(A) ch2 iq参考(A)
 *     ch3 理论eα(V) ch4 理论eβ(V) ch5 原始zα(V) ch6 原始zβ(V)
 *     ch7 滤波eα̂(V) ch8 滤波eβ̂(V) ch9 相位误差(deg)
 *     ch10 e_on_q(V) ch11 e_on_d(V) ch12 误差范数(V) ch13 理论幅值ωψf(V)
 *     ch14 实际转速-滤波(rpm) ch15 相位误差滤波(deg)
 *     ⚠ ch3~8 为电频率正弦，VOFA 帧率必混叠只查存在性；验收看 ch9~13
 *   wave_mode=1：**8ch** 波形窄帧（~2.9kHz 帧率，看正弦细节降到 1500rpm）
 *     ch0 zα原始(V) ch1 zβ原始(V) ch2 eα̂滤波(V) ch3 eβ̂滤波(V)
 *     ch4 理论eα(V) ch5 理论eβ(V) ch6 相位差(deg) ch7 相位差滤波(deg)
 *   wave_mode=2：**16ch** PLL/无感验收面板
 *     ch0 enc原始转速 ch1 enc滤波转速 ch2 ω̂原始 ch3 ω̂低通(无感反馈) (rpm)
 *     ch4 转速差原始 ch5 转速差滤波 (rpm)
 *     ch6 角差原始(deg) ch7 角差滤波(deg) ch8 iq(A) ch9 补偿角δ̂(deg)
 *     ch10 无感锁存(0/1) ch11 |e_hat|(V) ch12 sE同拍角差(deg)
 *     ch13 目标转速(rpm) ch14 显示滤波转速(rpm) ch15 斜坡输出转速(rpm)
 * ===========================================================================
 */

#ifndef __FOC_45_SMO_H__
#define __FOC_45_SMO_H__

#include <stdint.h>
#include "foc_core.h"
#include "../Utils/dev_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SMO45_DBG   1
#if SMO45_DBG
#define SMO45_LOG(fmt, ...)  MAIN_D("[SMO45] " fmt, ##__VA_ARGS__)
#else
#define SMO45_LOG(fmt, ...)  ((void)0)
#endif

/* Inner current PI: start from the verified mode 29 values.
 * ⚠ UMAX 3.5→6.2（Step4 高速）：4800rpm 反电动势已 ≈3.4V，3.5V 限幅是
 *   高速第一电压墙。上限约束：12V 母线 SVPWM 线性区相电压峰值 = 12/√3
 *   = 6.93V，留 ~10% 余量防过调制。ITERM 同步放大防积分 early 饱和。 */
#define SMO45_PI_KP             0.5f
#define SMO45_PI_KI             300.0f
#define SMO45_PI_UMAX_V         6.2f
#define SMO45_ITERM_MAX_V       6.0f
#define SMO45_IQ_FILT_ALPHA     0.10f

/* Outer speed PI output is a signed q-axis current reference in mA. */
#define SMO45_SPD_KP_MA_PER_RPM       1.8f
#define SMO45_SPD_KI_MA_PER_RPM_S     0.2f
#define SMO45_SPD_IQ_LIMIT_MA         ((float)FOC_MOTOR_MAX_CURRENT_A \
                                      * 1000.0f * 0.20f)

/* Safety envelope derived from motor_config.h. */
#define SMO45_SPEED_REF_LIMIT_RPM     ((float)FOC_MOTOR_MAX_SPEED_RPM)
#define SMO45_ACCEL_LIMIT_RPM_S       ((float)FOC_MOTOR_MAX_SPEED_RPM \
                                      * 0.25f)

#define SMO45_SPD_WIN_MS              5u
#define SMO45_SPD_WIN_US              (SMO45_SPD_WIN_MS * 1000u)
#define SMO45_SPD_FILT_ALPHA          0.25f
/* 显示专用滤波（VOFA 曲线平滑用；α 越小越平滑越滞后，不进 PI 反馈） */
#define SMO45_SPD_DISP_ALPHA          0.05f
/* 编码器每拍增量限幅 = 单拍物理极限 × 1.35 裕度。物理极限 = 7800rpm 折算到每拍：
 *   10 kHz（100µs/拍）：7800/60×4096×100µs = 53 counts → 限幅 72
 *   20 kHz（ 50µs/拍）：26.5 counts               → 限幅 36  ← 2026-09-23 同步改
 * ⚠ 原 32 只支持到 4687rpm（10kHz 时）：超限后测速削顶 + s_rotor_count 角度积分丢拍
 * （Park 角持续落后，高速转矩错位）——高速上不去的第一堵墙。
 * ⚠ 若再改 MOTOR_PWM_FREQ_HZ，此值须按 1/f 同步缩放 */
#define SMO45_ENC_DELTA_MAX           36

/* 启动自动转速 profile（g_smo45_auto_ramp=1 时生效，到顶后停止写入）：
 * 秒 0/1/2/3 -> 0/200/500/1000 rpm，之后保持 1000（Watch 可接管）
 * （Step 4 无感首切定在 1000rpm） */
#define SMO45_AUTO_RAMP_DEFAULT       1u
#define SMO45_AUTO_RAMP_SECS          3u

#define SMO45_STEP_IDLE               0u
#define SMO45_STEP_RUN                1u
#define SMO45_STEP_FAULT_OC           2u

#define SMO45_EVT_OC                  1u

extern volatile uint8_t  g_smo45_running;
extern volatile uint8_t  g_smo45_state;
extern volatile uint8_t  g_smo45_evt;
extern volatile uint8_t  g_smo45_auto_ramp;
extern volatile uint8_t  g_smo45_wave_mode;   /* VOFA 帧布局：0=16ch SMO 验收面板，
                                               * 1=8ch 波形窄帧（32B @921600 → ~2.9kHz
                                               * 帧率），2=16ch PLL/无感验收面板 */
extern volatile float    g_smo45_speed_target_rpm;
extern volatile float    g_smo45_speed_ramp_rpm;
extern volatile float    g_smo45_speed_meas_rpm;
extern volatile float    g_smo45_speed_filt_rpm;
extern volatile float    g_smo45_speed_disp_rpm;    /* 显示专用强滤波转速（α=0.05，
                                              * 仅 VOFA/Watch 看趋势；PI 反馈仍用
                                              * speed_filt_rpm，勿混用防环路滞后） */
extern volatile float    g_smo45_speed_err_rpm;
extern volatile float    g_smo45_speed_out_ma;
extern volatile int32_t  g_smo45_rotor_count;
extern volatile int32_t  g_smo45_rotor_deg;
extern volatile float    g_smo45_id_ref_ma;
extern volatile float    g_smo45_iq_ref_ma;
extern volatile float    g_smo45_id_ma;
extern volatile float    g_smo45_iq_ma;
extern volatile float    g_smo45_iq_filt_ma;
extern volatile float    g_smo45_iq_filt_alpha;
extern volatile float    g_smo45_vd;
extern volatile float    g_smo45_vq;
extern volatile uint8_t  g_smo45_vsat;
extern volatile uint8_t  g_smo45_sensorless;  /* 无感切换开关（Step 4）：0=编码器角有感
                                               * 闭环（现状），1=θ_park(PLL 补偿角)+
                                               * ω̂_lpf 无感闭环。置 1 需 SMO 判定
                                               * g_smo_emf_ok=1（ISR 内锁存，回 0 退出） */
extern volatile float    g_smo45_ang_lead_ticks; /* 有感角度超前补偿拍数（默认 1.5，0=关）：
                                               * 采样→PWM 作用中心的延迟补偿，高速必需 */
extern volatile uint8_t  g_smo45_sl_active;   /* 无感锁存状态（只读观察）：1=当前 Park/速度
                                               * 反馈已用 PLL 无感量，0=编码器 */
extern volatile float    g_smo45_du;
extern volatile float    g_smo45_dv;
extern volatile float    g_smo45_dw;

extern pid_config_t g_smo45_pid_speed_cfg;
extern pid_config_t g_smo45_pid_id_cfg;
extern pid_config_t g_smo45_pid_iq_cfg;

void Foc_Smo45_InitPids(void);
void Foc_Smo45_SetTargetRPM(float target_rpm);
void Foc_Smo45_Start(void);
void Foc_Smo45_Stop(void);
void Foc_Smo45_Step(const stc_i_data_t *pData);
int  Foc_Smo45_VofaFill(int32_t *cur);   /* 模式自持 VOFA，见顶部速览卡 */

#ifdef __cplusplus
}
#endif

#endif /* __FOC_45_SMO_H__ */
