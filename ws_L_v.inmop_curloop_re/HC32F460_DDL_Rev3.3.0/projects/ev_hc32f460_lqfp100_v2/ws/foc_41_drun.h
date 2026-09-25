/**
 *******************************************************************************
 * @file  foc_41_drun.h
 * @brief FOC mode 41 - current loop with dq feed-forward experiment.
 *
 * Mode 41 copies mode 29 and adds runtime-switchable BEMF/cross-coupling
 * feed-forward.  Disable the feed-forward flags for a mode29-equivalent A/B
 * baseline.  It reuses the mode 24 calibration result.
 *
 * ============================ 模式速览卡（唯一事实源）========================
 * 模式：41 = 纯电流环 + 可选 dq 前馈实验（= mode 29 + 解耦前馈）
 *        前馈三路：反电动势 ω_e·ψf、交叉耦合 ω_e·L·iq / ω_e·L·id、电阻项 R·iq_ref
 * 入口：comm_mode = 41
 * 前置：**必须先跑 mode 24**（复用其校准快照）
 * 结束：持续运行；仅 OC 自动停
 * A/B 用法：把三个 ff_gain 置 0 即得到 mode 29 等价基线，便于对比前馈效果
 *
 * 【Watch 可调变量】（名称 = 默认值 单位）
 *   g_drun41_ff_enable      0/1    前馈总开关（0 = mode29 等价基线）
 *   g_drun41_ff_bemf_gain   1.0    BEMF 前馈增益 → vq += gain·ω_e·ψf
 *   g_drun41_ff_cross_gain  1.0    交叉耦合前馈增益 → vd += −gain·ω_e·L·iq
 *                                                  vq += +gain·ω_e·L·id
 *   g_drun41_ff_res_gain    1.0    电阻前馈增益 → vq += gain·R·iq_ref
 *   g_drun41_vmax_v         0       dq 合成电压限幅（0 = 不限）
 *   g_drun41_i_ref_ma       mA      电流矢量幅值参考
 *   g_drun41_i_ramp_ma_s    mA/s    软启动斜率（<=0 直通/阶跃）
 *   g_drun41_dlt_init_deg / _targ_deg / _tr_ms   δ 爬坡三参数
 *   g_drun41_iq_filt_alpha  0.10    iq 显示滤波系数（不进 PI）
 *   g_drun41_pid_id_cfg / iq_cfg 的 .kp/.ki/.i_valid
 *
 * 【电流参考生成】id_ref = I_ref·cosδ ； iq_ref = I_ref·sinδ（同 mode 28/29）
 *
 * 【关键观察变量】
 *   g_drun41_running / g_drun41_state   0 IDLE / 1 RUN / 2 FAULT_OC
 *   g_drun41_evt        1 RAMP_DONE / 2 OC
 *   **g_drun41_vd_ff_v / g_drun41_vq_ff_v  前馈电压分量(V) ← 前馈实验核心**
 *   g_drun41_vd_pi_v / g_drun41_vq_pi_v    PI 本体输出(V)（不含前馈）
 *       ⇒ 对比 vd/vq 总输出 与 PI 输出，可看出前馈承担了多少
 *   g_drun41_omega_e_rad_s  电角速度(rad/s)
 *   g_drun41_i_ref_ma / id_ref_ma / iq_ref_ma  电流参考
 *   g_drun41_id_ma / iq_ma / iq_filt_ma  电流反馈
 *   g_drun41_eq_mean_ma / eq_pp_ma / ed_mean_ma / ed_pp_ma  误差统计（主判据）
 *   g_drun41_id_mean_ma / iq_mean_ma     3s 均值
 *   g_drun41_vsat       电压饱和标志
 *   g_drun41_dlt_now_deg / speed_hz / field_deg / rotor_deg / diff_deg   角度与转速
 *   g_drun41_time_us    运行时长(µs)
 *   g_drun41_iq_step_state / iq_step_t90_us  阶跃响应测量（同 mode 29 机制）：
 *       进模式第一拍武装；state 0 IDLE / 1 WAIT / 2 DONE / 3 TIMEOUT
 *
 * 【VOFA 通道】**自持布局，固定 21ch**（实现见 foc_41_drun.c
 *   Foc_Drun41_VofaFill，单位换算：传"毫单位"，SendScaled 内部 ×0.001）：
 *   ch0~18 **与 mode 29 完全一致**（便于 A/B 对比前馈效果）：
 *     ch0~2 三相电流(A)   ch3 ialpha(A)   ch4 ibeta(A)
 *     ch5 iq 反馈(A)   ch6 id 反馈(A)
 *     ch7 iq 参考(A)   ch8 id 参考(A)（不恒为 0）
 *     ch9 vq 总输出(V)   ch10 vd 总输出(V)
 *     ch11 q 误差均值(A)   ch12 d 误差均值(A)
 *     ch13 iq 3s 均值(A)   ch14 id 3s 均值(A)
 *     ch15 q 误差峰峰值(A)   ch16 滤波 iq(A)
 *     ch17 功角 delta(deg)   ch18 实测电频率(Hz)
 *   ch19 **d 轴前馈电压(V)**  ← 本模式专属
 *   ch20 **q 轴前馈电压(V)**  ← 本模式专属
 *   ⚠ 共 21ch —— 切到本模式时 VOFA+ 通道数需同步改为 21。
 *   ⚠ ch9/ch10 是**总输出（含前馈）**，ch19/ch20 是前馈分量；
 *     两者相减可得 PI 本体输出（也可用 Watch 读 g_drun41_vd_pi_v / vq_pi_v）。
 *   ⚠ **改通道数：改 Foc_Drun41_VofaFill 末尾的 return 值**。
 * ===========================================================================
 */

#ifndef __FOC_41_DRUN_H__
#define __FOC_41_DRUN_H__

#include <stdint.h>
#include "foc_core.h"
#include "../Utils/dev_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DRUN41_DBG   1
#if DRUN41_DBG
#define DRUN41_LOG(fmt, ...)  MAIN_D("[DRUN41] " fmt, ##__VA_ARGS__)
#else
#define DRUN41_LOG(fmt, ...)  ((void)0)
#endif

#define DRUN41_PI_KP             0.5f
#define DRUN41_PI_KI             300.0f
#define DRUN41_PI_UMAX_V         3.5f
#define DRUN41_ITERM_MAX_V       3.2f
#define DRUN41_VMAX_DEFAULT_V    6.0f
#define DRUN41_IQ_FILT_ALPHA     0.10f
#define DRUN41_I_REF_MA          500.0f
#define DRUN41_I_RAMP_MA_S       0.0f
#define DRUN41_ENC_DELTA_MAX     32

#define DRUN41_SPEED_WIN_MS      200u
#define DRUN41_PP_WIN_MS         5000u
#define DRUN41_MEAN_WIN_MS       3000u
#define DRUN41_ERR_WIN_MS        5000u

#define DRUN41_STEP_IDLE         0u
#define DRUN41_STEP_RUN          1u
#define DRUN41_STEP_FAULT_OC     2u

#define DRUN41_EVT_RAMP_DONE     1u
#define DRUN41_EVT_OC            2u

#define DRUN41_STEP_DEADBAND_MA  50.0f
#define DRUN41_STEP_MIN_MA       100.0f
#define DRUN41_STEP_TIMEOUT_US   200000u
#define DRUN41_STEP_CONFIRM_TICK 3u
#define DRUN41_STEP_TIME_TIMEOUT 0xFFFFFFFFu

#define DRUN41_STEP_ST_IDLE      0u
#define DRUN41_STEP_ST_WAIT      1u
#define DRUN41_STEP_ST_DONE      2u
#define DRUN41_STEP_ST_TIMEOUT   3u

extern volatile uint8_t  g_drun41_running;
extern volatile uint8_t  g_drun41_state;
extern volatile uint8_t  g_drun41_evt;
extern volatile float    g_drun41_dlt_init_deg;
extern volatile float    g_drun41_dlt_targ_deg;
extern volatile uint32_t g_drun41_dlt_tr_ms;
extern volatile float    g_drun41_dlt_now_deg;
extern volatile float    g_drun41_speed_hz;
extern volatile int32_t  g_drun41_field_deg;
extern volatile int32_t  g_drun41_rotor_deg;
extern volatile int32_t  g_drun41_diff_deg;
extern volatile int32_t  g_drun41_rotor_count;
extern volatile float    g_drun41_id_ma;
extern volatile float    g_drun41_iq_ma;
extern volatile float    g_drun41_iq_filt_ma;
extern volatile float    g_drun41_iq_filt_alpha;
extern volatile float    g_drun41_id_pp_ma;
extern volatile float    g_drun41_iq_pp_ma;
extern volatile float    g_drun41_id_mean_ma;
extern volatile float    g_drun41_iq_mean_ma;
extern volatile float    g_drun41_i_ref_ma;
extern volatile float    g_drun41_id_ref_ma;
extern volatile float    g_drun41_iq_ref_ma;
extern volatile float    g_drun41_i_ramp_ma_s;
extern volatile uint8_t  g_drun41_vsat;
extern volatile float    g_drun41_ed_mean_ma;
extern volatile float    g_drun41_eq_mean_ma;
extern volatile float    g_drun41_ed_pp_ma;
extern volatile float    g_drun41_eq_pp_ma;
extern volatile float    g_drun41_du;
extern volatile float    g_drun41_dv;
extern volatile float    g_drun41_dw;
extern volatile uint8_t  g_drun41_ff_enable;
extern volatile float    g_drun41_ff_bemf_gain;
extern volatile float    g_drun41_ff_cross_gain;
extern volatile float    g_drun41_ff_res_gain;
extern volatile float    g_drun41_omega_e_rad_s;
extern volatile float    g_drun41_vd_pi_v;
extern volatile float    g_drun41_vq_pi_v;
extern volatile float    g_drun41_vd_ff_v;
extern volatile float    g_drun41_vq_ff_v;
extern volatile float    g_drun41_vmax_v;

/* Step-response instrumentation.  Times are measured from the first ISR where
 * the selected axis reference differs from its previous value by more than
 * DRUN41_STEP_DEADBAND_MA.  A crossing must remain present for
 * DRUN41_STEP_CONFIRM_TICK samples before it is accepted. */
extern volatile uint32_t g_drun41_time_us;
extern volatile float    g_drun41_step_frac_pct;
extern volatile uint8_t  g_drun41_id_step_state;
extern volatile uint8_t  g_drun41_iq_step_state;
extern volatile float    g_drun41_id_step_target_ma;
extern volatile float    g_drun41_iq_step_target_ma;
extern volatile uint32_t g_drun41_id_step_t90_us;
extern volatile uint32_t g_drun41_iq_step_t90_us;

extern pid_config_t g_drun41_pid_id_cfg;
extern pid_config_t g_drun41_pid_iq_cfg;

void Foc_Drun41_InitPids(void);
void Foc_Drun41_Start(void);
void Foc_Drun41_Stop(void);
void Foc_Drun41_Step(const stc_i_data_t *pData);
int  Foc_Drun41_VofaFill(int32_t *cur);   /* 模式自持 VOFA，21ch，见顶部速览卡 */

#ifdef __cplusplus
}
#endif

#endif /* __FOC_41_DRUN_H__ */
