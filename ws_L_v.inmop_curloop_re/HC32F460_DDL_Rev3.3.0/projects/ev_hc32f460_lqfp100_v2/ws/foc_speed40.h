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
 *   g_speed40_pid_speed_cfg.kp = 1.8   mA/rpm    速度环 P（Watch 改立即生效，
 *                                                稳态中小步 ±20% 调）
 *   g_speed40_pid_speed_cfg.ki = 0.2   mA/rpm/s  速度环 I（同上）
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

/* Inner current PI: start from the verified mode 29 values. */
#define SPEED40_PI_KP             0.5f
#define SPEED40_PI_KI             300.0f
/* ⚠ UMAX 3.5→6.2 / ITERM 3.2→6.0（与 mode 45 SMO45 同步，2026-09-23）：
 *   12V 母线 SVPWM 线性区相电压峰值 6.93V，原 3.5V 是高速电压墙（卡 ~4800rpm） */
#define SPEED40_PI_UMAX_V         6.2f
#define SPEED40_ITERM_MAX_V       6.0f
#define SPEED40_IQ_FILT_ALPHA     0.10f

/* Outer speed PI output is a signed q-axis current reference in mA. */
#define SPEED40_SPD_KP_MA_PER_RPM       1.8f
#define SPEED40_SPD_KI_MA_PER_RPM_S     0.2f
#define SPEED40_SPD_IQ_LIMIT_MA         ((float)FOC_MOTOR_RATED_CURRENT_A \
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
/* 编码器每拍增量限幅：与 mode 45 同步 32→72（7800rpm 需 53 counts，×1.35 裕度）。
 * ⚠ 原 32 只支持 4687rpm：超限测速削顶 + 角度积分丢拍（高速 Park 角落后） */
#define SPEED40_ENC_DELTA_MAX           72

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
