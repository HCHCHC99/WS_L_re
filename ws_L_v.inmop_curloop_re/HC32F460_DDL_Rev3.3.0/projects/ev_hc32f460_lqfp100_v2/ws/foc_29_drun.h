/**
 *******************************************************************************
 * @file  foc_29_drun.h
 * @brief FOC mode 29 - independent run-only current loop.
 *
 * Mode 29 copies the calibration result produced by mode 24, samples the live
 * rotor angle when it starts, and then owns its entire 20 kHz current loop.
 *
 * ============================ 模式速览卡（唯一事实源）========================
 * 模式：29 = 纯电流环（复用 mode 24 已锁的零偏 + 编码器 offset，直接进 RUN）
 *        用于捏死转子测 R+L 对象、调 P/pi；也可做功角 δ 实验。
 * 入口：comm_mode = 29
 * 前置：**必须先跑 mode 24**（否则 Start 检查不通过直接拒绝，RTT 报
 *       "calib/offset not locked, run mode 24 first"）
 * 结构：只有电流环，**没有速度外环**（对比 mode 40 是双闭环）
 *
 * 【Watch 可调变量】（名称 = 默认值 单位）
 *   g_drun29_i_ref_ma      = 500    mA        电流矢量**幅值**参考
 *   g_drun29_i_ramp_ma_s   = 0      mA/s      软启动斜率；**<=0 = 直通(阶跃)**
 *   g_drun29_dlt_init_deg / _targ_deg / _tr_ms 功角 δ 三参数（爬坡对象）
 *   g_drun29_iq_filt_alpha = 0.10   -         iq 显示滤波系数（不进 PI）
 *   g_drun29_pid_id_cfg / iq_cfg 的 .kp/.ki/.i_valid
 *        默认 kp=0.5 V/A、ki=300 V/(A·s)、i_valid=1
 *        ⚠ i_valid=0 → 纯 P，稳态跟踪率降为 kp/(kp+R)（mode 28 调参用）
 *
 * 【电流参考的生成】（与 mode 40 的关键差异）
 *   id_ref = I_ref·cos(δ)      iq_ref = I_ref·sin(δ)
 *   ⇒ δ=90° 时 id_ref=0、iq_ref=I_ref（经典 id=0 FOC）
 *   ⚠ 所以 ch6（id 参考）**不恒为 0**，与 mode 40 不同！
 *
 * 【关键观察变量】
 *   g_drun29_iq_ma / g_drun29_id_ma        控制系电流反馈（零偏校正后 = 环反馈）
 *   g_drun29_iq_ref_ma / _id_ref_ma        电流参考实时值
 *   g_drun29_eq_mean_ma / _pp_ma           q 轴误差均值 / 峰峰值（P 调参主判据）
 *   g_drun29_ed_mean_ma / _pp_ma           d 轴同上
 *   g_drun29_iq_mean_ma / g_drun29_id_mean_ma   3s 均值
 *   g_drun29_eq_pp_ma / g_drun29_ed_pp_ma       5s 误差峰峰值（振铃判据）
 *   g_drun29_vsat                          电压饱和标志（1=贴 UMAX，数据作废）
 *   g_drun29_speed_hz                      实测电频率（200ms 窗，带符号）
 *   g_drun29_state                         0 空闲 1 运行 2 过流
 *   g_drun29_rotor_deg / g_drun29_diff_deg 转子电角/功角
 *   g_drun29_time_us                       运行时长（µs）
 *   g_drun29_iq_step_state / g_drun29_iq_step_t90_us
 *        **自带阶跃响应测量**：进 mode 29 第一拍武装（之后改参考不重启），
 *        用参考值当目标，测电流到达其 90% 的时间(µs)。
 *        state: 0=IDLE 1=WAIT 2=DONE 3=TIMEOUT；超时值 = 0xFFFFFFFF
 *   g_drun29_step_frac_pct                 判据百分比（每次 Start 复位为 75）
 *
 * 【VOFA 通道】**自持布局，固定 19ch**（实现见 foc_29_drun.c
 *   Foc_Drun_VofaFill，单位换算：传"毫单位"，SendScaled 内部 ×0.001）：
 *   ch0~2  三相电流(A)              ch3 静止系 ialpha(A)
 *   ch4    静止系 ibeta(A)
 *   ch5    **iq 反馈(A)**           ch6  **id 反馈(A)**        ← 环反馈
 *   ch7    **iq 参考(A)**           ch8  **id 参考(A)** ← **不恒为 0**（= I_ref·cosδ）
 *   ch9    **vq 输出(V)**           ch10 **vd 输出(V)**
 *   ch11   **q 轴误差均值(A)** ← P 调参主判据
 *   ch12   d 轴误差均值(A)
 *   ch13   iq 3s 均值(A)            ch14 id 3s 均值(A)
 *   ch15   iq 误差峰峰值(A)（振铃判据）
 *   ch16   显示用滤波 iq(A)（EMA；PI 仍用 ch5 原始 iq）
 *   ch17   功角 delta(deg)          ch18 实测电频率(Hz)
 *   ⚠ 共 19ch（原通用布局为 17ch）——切到本模式时 VOFA+ 通道数需同步改为 19。
 *   ⚠ **改通道数：改 Foc_Drun_VofaFill 末尾的 return 值**。
 * ===========================================================================
 */

#ifndef __FOC_29_DRUN_H__
#define __FOC_29_DRUN_H__

#include <stdint.h>
#include "foc_core.h"
#include "../Utils/dev_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FOC29_DBG   1
#if FOC29_DBG
#define FOC29_LOG(fmt, ...)  MAIN_D("[DRUN29] " fmt, ##__VA_ARGS__)
#else
#define FOC29_LOG(fmt, ...)  ((void)0)
#endif

#define FOC29_PI_KP             0.5f
#define FOC29_PI_KI             300.0f
#define FOC29_PI_UMAX_V         3.5f
#define FOC29_ITERM_MAX_V       3.2f
#define FOC29_IQ_FILT_ALPHA     0.10f
#define FOC29_I_REF_MA          500.0f
#define FOC29_I_RAMP_MA_S       0.0f
#define FOC29_ENC_DELTA_MAX     32

#define FOC29_SPEED_WIN_MS      200u
#define FOC29_PP_WIN_MS         5000u
#define FOC29_MEAN_WIN_MS       3000u
#define FOC29_ERR_WIN_MS        5000u

#define FOC29_STEP_IDLE         0u
#define FOC29_STEP_RUN          1u
#define FOC29_STEP_FAULT_OC     2u

#define FOC29_EVT_RAMP_DONE     1u
#define FOC29_EVT_OC            2u

#define FOC29_STEP_DEADBAND_MA  50.0f
#define FOC29_STEP_MIN_MA       100.0f
#define FOC29_STEP_TIMEOUT_US   200000u
#define FOC29_STEP_CONFIRM_TICK 3u
#define FOC29_STEP_TIME_TIMEOUT 0xFFFFFFFFu

#define FOC29_STEP_ST_IDLE      0u
#define FOC29_STEP_ST_WAIT      1u
#define FOC29_STEP_ST_DONE      2u
#define FOC29_STEP_ST_TIMEOUT   3u

extern volatile uint8_t  g_drun29_running;
extern volatile uint8_t  g_drun29_state;
extern volatile uint8_t  g_drun29_evt;
extern volatile float    g_drun29_dlt_init_deg;
extern volatile float    g_drun29_dlt_targ_deg;
extern volatile uint32_t g_drun29_dlt_tr_ms;
extern volatile float    g_drun29_dlt_now_deg;
extern volatile float    g_drun29_speed_hz;
extern volatile int32_t  g_drun29_field_deg;
extern volatile int32_t  g_drun29_rotor_deg;
extern volatile int32_t  g_drun29_diff_deg;
extern volatile int32_t  g_drun29_rotor_count;
extern volatile float    g_drun29_id_ma;
extern volatile float    g_drun29_iq_ma;
extern volatile float    g_drun29_iq_filt_ma;
extern volatile float    g_drun29_iq_filt_alpha;
extern volatile float    g_drun29_id_pp_ma;
extern volatile float    g_drun29_iq_pp_ma;
extern volatile float    g_drun29_id_mean_ma;
extern volatile float    g_drun29_iq_mean_ma;
extern volatile float    g_drun29_i_ref_ma;
extern volatile float    g_drun29_id_ref_ma;
extern volatile float    g_drun29_iq_ref_ma;
extern volatile float    g_drun29_i_ramp_ma_s;
extern volatile uint8_t  g_drun29_vsat;
extern volatile float    g_drun29_ed_mean_ma;
extern volatile float    g_drun29_eq_mean_ma;
extern volatile float    g_drun29_ed_pp_ma;
extern volatile float    g_drun29_eq_pp_ma;
extern volatile float    g_drun29_du;
extern volatile float    g_drun29_dv;
extern volatile float    g_drun29_dw;

/* Step-response instrumentation.  Times are measured from the first ISR where
 * the selected axis reference differs from its previous value by more than
 * FOC29_STEP_DEADBAND_MA.  A crossing must remain present for
 * FOC29_STEP_CONFIRM_TICK samples before it is accepted. */
extern volatile uint32_t g_drun29_time_us;
extern volatile float    g_drun29_step_frac_pct;
extern volatile uint8_t  g_drun29_id_step_state;
extern volatile uint8_t  g_drun29_iq_step_state;
extern volatile float    g_drun29_id_step_target_ma;
extern volatile float    g_drun29_iq_step_target_ma;
extern volatile uint32_t g_drun29_id_step_t90_us;
extern volatile uint32_t g_drun29_iq_step_t90_us;

extern pid_config_t g_drun29_pid_id_cfg;
extern pid_config_t g_drun29_pid_iq_cfg;

void Foc_Drun_InitPids(void);
void Foc_Drun_Start(void);
void Foc_Drun_Stop(void);
void Foc_Drun_Step(const stc_i_data_t *pData);
int  Foc_Drun_VofaFill(int32_t *cur);   /* 模式自持 VOFA，19ch，见顶部速览卡 */

#ifdef __cplusplus
}
#endif

#endif /* __FOC_29_DRUN_H__ */
