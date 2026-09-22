/**
 *******************************************************************************
 * @file  foc_smo45.h
 * @brief FOC mode 45 - SMO+PLL 无感速度/电流双闭环（分步开发中）。
 *
 * 第 1 步（当前，已完成）：完全复刻 mode 40（编码器 FOC 速度/电流双闭环）
 * + 启动转速自动爬坡 profile（0→200→500→1000→1500→2000 rpm）+
 * SMO 纯旁观（foc_smo.c，编码器闭环不受影响）：
 *   输入 = 上一拍指令 valpha/vbeta + 同源 Clarke 电流；
 *   输出 = e_alpha_hat/e_beta_hat（VOFA CH11/12），
 *   诊断 = |e_hat| vs ω·ψf、atan2 角 vs 编码器角（CH13）。
 *   判据：匀速下 e_hat 幅值稳定/互差 90°/接近正弦，
 *   幅值 ≈ ω·ψf×(1−R/(k/φ))（k=4.6V 固定，按 2600rpm 整定）。
 * 第 2 步（待做）：PLL 从 e_hat 提取 theta_hat/omega_hat，仍旁观。
 * 第 3 步（待做）：脱编码器无感闭环。
 *
 * 结构（与 mode 40 相同）：
 *   速度 PI（5ms 节拍）输出 q 轴电流参考 (mA)，id 参考 = 0；
 *   独立 id/iq 电流 PI 在每个电流采样 ISR（20kHz）执行。
 *   启动前置：有效的 mode 24 校准（foc_dcal24 快照）。
 *******************************************************************************
 */

#ifndef __FOC_SMO45_H__
#define __FOC_SMO45_H__

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

/* Inner current PI: start from the verified mode 29 values. */
#define SMO45_PI_KP             0.5f
#define SMO45_PI_KI             300.0f
#define SMO45_PI_UMAX_V         3.5f
#define SMO45_ITERM_MAX_V       3.2f
#define SMO45_IQ_FILT_ALPHA     0.10f

/* Outer speed PI output is a signed q-axis current reference in mA. */
#define SMO45_SPD_KP_MA_PER_RPM       1.8f
#define SMO45_SPD_KI_MA_PER_RPM_S     0.2f
#define SMO45_SPD_IQ_LIMIT_MA         ((float)FOC_MOTOR_RATED_CURRENT_A \
                                      * 1000.0f * 0.20f)

/* Safety envelope derived from motor_config.h. */
#define SMO45_SPEED_REF_LIMIT_RPM     ((float)FOC_MOTOR_MAX_SPEED_RPM)
#define SMO45_ACCEL_LIMIT_RPM_S       ((float)FOC_MOTOR_MAX_SPEED_RPM \
                                      * 0.25f)

#define SMO45_SPD_WIN_MS              5u
#define SMO45_SPD_WIN_US              (SMO45_SPD_WIN_MS * 1000u)
#define SMO45_SPD_FILT_ALPHA          0.25f
#define SMO45_ENC_DELTA_MAX           32

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
extern volatile uint8_t  g_smo45_wave_mode;   /* VOFA 帧布局：0=15ch 验收面板，1=7ch 波形窄帧
                                               * （32B @921600 → ~2.9kHz 帧率，2600rpm 6.6 点/周期） */
extern volatile float    g_smo45_speed_target_rpm;
extern volatile float    g_smo45_speed_ramp_rpm;
extern volatile float    g_smo45_speed_meas_rpm;
extern volatile float    g_smo45_speed_filt_rpm;
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

#ifdef __cplusplus
}
#endif

#endif /* __FOC_SMO45_H__ */
