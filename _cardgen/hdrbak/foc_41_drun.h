/**
 *******************************************************************************
 * @file  foc_41_drun.h
 * @brief FOC mode 41 - current loop with dq feed-forward experiment.
 *
 * Mode 41 copies mode 29 and adds runtime-switchable BEMF/cross-coupling
 * feed-forward.  Disable the feed-forward flags for a mode29-equivalent A/B
 * baseline.  It reuses the mode 24 calibration result.
 *
 * ==============================================================================
 *   模式速览卡   MODE 41   纯电流环 + 可选 dq 前馈实验（mode 29 加解耦前馈）
 *   本卡是本模式唯一事实源；代码内注释与本卡冲突时，以本卡为准
 * ==============================================================================
 *   入口   comm_mode = 41
 *   前置   必须先跑 mode 24（复用其零偏与偏移快照），否则 Start 拒绝启动
 *   结束   持续运行；仅过流自动停（state=2、evt=2），手动停写 comm_mode = 0
 *   标记   [!] 易错点   (*) 主判据   [可调] Watch 可写   [只读] 仅观察
 * ------------------------------------------------------------------------------
 *   [可调] Watch 变量 -- 改值即时生效，复位后回宏默认
 * ------------------------------------------------------------------------------
 *       变量                        当前值  单位     含义
 *       --------------------------- ------- -------- ---------------------------
 *       g_drun41_ff_enable          0       -        前馈总开关；0=mode29 基线
 *       g_drun41_ff_bemf_gain       1.0     -        反电动势前馈增益
 *       g_drun41_ff_cross_gain      1.0     -        交叉耦合增益；噪声最敏感
 *       g_drun41_ff_res_gain        1.0     -        电阻项前馈增益
 *       g_drun41_vmax_v             6.0     V        dq 合成电压限幅；0 = 不限
 *   (*) 前馈公式（ff_enable=1 时才生效）：vd_ff = -cross*omega*Ls*iq；vq_ff =
 *       cross*omega*Ls*id + bemf*omega*psi_f + res*Rs*iq_ref。系数取
 *       FOC_MOTOR_LS_UH / FOC_MOTOR_FLUX_VS / FOC_MOTOR_RS_OHM。
 *   [!] 交叉项用实测 id/iq、BEMF 项用 200ms 窗速度、电阻项用 iq_ref：cross_gain
 *       最容易把采样噪声灌进输出。
 *       g_drun41_i_ref_ma           500     mA       电流矢量幅值参考
 *       g_drun41_i_ramp_ma_s        0       mA/s     软启动斜率；<=0 直通
 *       g_drun41_dlt_init_deg       90      deg      功角 delta 起点
 *       g_drun41_dlt_targ_deg       90      deg      功角 delta 终点
 *       g_drun41_dlt_tr_ms          0       ms       delta 爬坡时长；0 立即到位
 *   (*) I_ref=500mA、delta=90 度时 id_ref=0、iq_ref=500mA（纯 q 轴）；改 delta
 *       会同时改两个轴的参考。
 *       g_drun41_iq_filt_alpha      0.10    -        iq 显示滤波；不进 PI
 *       g_drun41_pid_id_cfg.kp      0.5     V/A      d 轴电流环 P
 *       g_drun41_pid_iq_cfg.kp      0.5     V/A      q 轴电流环 P
 *       g_drun41_pid_id_cfg.ki      300     V/A/s    d 轴电流环 I
 *       g_drun41_pid_iq_cfg.ki      300     V/A/s    q 轴电流环 I
 *   [!] 前馈承担模型电压后可适当降 kp（0.15~0.25）、升 ki（300~400）：0.5 的 P
 *       会把电流采样噪声较快映射到输出电压。kp/ki 每拍从 cfg 实时读取，Watch
 *       改值立即生效（不用重进模式），但改 ki 不清积分，切增益后先看一段再判。
 * ------------------------------------------------------------------------------
 *   [只读] 关键观察变量
 * ------------------------------------------------------------------------------
 *       变量                           含义
 *       ------------------------------ -----------------------------------------
 *   (*) g_drun41_eq_mean_ma            q 轴误差均值；判前馈好坏先看它
 *   (*) g_drun41_eq_pp_ma              q 误差峰峰值；振铃/抖动判据（5s 窗）
 *   (*) g_drun41_vd_ff_v               d 轴前馈电压；ch19，前馈核心
 *   (*) g_drun41_vq_ff_v               q 轴前馈电压；ch20，前馈核心
 *       g_drun41_vd_pi_v               d 轴 PI 本体输出（不含前馈）
 *       g_drun41_vq_pi_v               q 轴 PI 本体输出；前馈对时它变小
 *   [!] g_drun41_vsat                  =1 时电压贴限幅或撞 vmax，数据作废
 *       g_drun41_omega_e_rad_s         电角速度；前馈量的来源（200ms 窗）
 *       g_drun41_iq_ma                 q 轴反馈；PI 用的原始值（会抖）
 *       g_drun41_iq_filt_ma            显示用滤波 iq；不进 PI
 *       g_drun41_id_ref_ma             d 参考 = I_ref cos(delta)
 *       g_drun41_iq_ref_ma             q 参考 = I_ref sin(delta)
 *       g_drun41_diff_deg              功角 delta(deg)；默认 90 度
 *       g_drun41_speed_hz              实测电频率 Hz；omega 由它算
 *       g_drun41_running               1 = mode41 运行中
 *       g_drun41_state                 0 IDLE / 1 RUN / 2 FAULT_OC
 *       g_drun41_evt                   事件：1 RAMP_DONE / 2 OC
 *       g_drun41_iq_step_state         阶跃测量：0 IDLE 1 WAIT 2 DONE 3 TIMEOUT
 *   [!] g_drun41_iq_step_t90_us        iq 阶跃到阈值的时间 us
 *   [!] g_drun41_iq_step_t90_us 的阈值实际是 75% 不是
 *       90%：g_drun41_step_frac_pct 声明初值 90，但进模式时被清零写成
 *       75（名字与行为不符）。计时在进模式后第一次 |参考| >= 100mA
 *       的那一拍武装，之后再改参考不会重新起测。
 * ------------------------------------------------------------------------------
 *   VOFA 通道 -- 21ch，填充见 Foc_Drun41_VofaFill
 * ------------------------------------------------------------------------------
 *       通道  含义                   单位     备注
 *       ----- ---------------------- -------- ----------------------------------
 *       ch0   U 相电流               A
 *       ch1   V 相电流               A
 *       ch2   W 相电流               A
 *       ch3   静止系 ialpha          A
 *       ch4   静止系 ibeta           A
 *       ch5   iq 反馈                A        PI 用的原始值
 *       ch6   id 反馈                A
 *       ch7   iq 参考                A        = I_ref sin(delta)
 *       ch8   id 参考                A        不恒为 0
 *       ch9   vq 总输出              V        含前馈
 *       ch10  vd 总输出              V        含前馈
 *   (*) ch11  q 轴误差均值           A        前馈好坏首要判据
 *       ch12  d 轴误差均值           A
 *       ch13  iq 3s 均值             A
 *       ch14  id 3s 均值             A
 *       ch15  q 轴误差峰峰值         A        振铃判据
 *       ch16  显示滤波 iq            A        不进 PI
 *       ch17  功角 delta             deg
 *       ch18  实测电频率             Hz
 *   (*) ch19  d 轴前馈电压           V        本模式专属
 *   (*) ch20  q 轴前馈电压           V        本模式专属
 * ------------------------------------------------------------------------------
 *   判据与坑
 * ------------------------------------------------------------------------------
 *   (*) 判前馈好坏：eq_mean_ma 下降、eq_pp_ma 不恶化或下降、vd_pi/vq_pi
 *       分量变小、vsat 不更易触发 = 好前馈；iq 抖动变大、id/iq 互串加重、vsat
 *       更易触发 = 坏前馈。
 *   (*) A/B 基线：ff_enable=0 就是 mode29 等价（三个 gain 默认 1.0
 *       但不生效），ch0~18 与 mode29 通道逐一对应，可直接叠波形对比。
 *   [!] 2026-09-19 实测：打开前馈后 iq_ref 200mA 尚可跟随，>=300mA
 *       抖动随目标电流增大。最可疑的是 cross_gain 用实测 id/iq 算 vd_ff
 *       把采样噪声注入输出。建议按 BEMF -> 电阻 -> 交叉 分项隔离，交叉档从 0 ->
 *       0.2 -> 0.5 -> 1.0 逐档加，每档看 eq_pp_ma 与 ch16。
 *   [!] ch9/ch10 是含前馈的总输出，ch19/ch20 才是前馈分量；相减即 PI
 *       本体输出（也可直接 Watch g_drun41_vd_pi_v / g_drun41_vq_pi_v）。别拿
 *       ch9/ch10 当 PI 输出判参数。
 *   [!] VOFA+ 通道数必须同步配成 21，否则整帧错位。vmax_v 只在 ff_enable=1
 *       时起作用：vmax_v>0 时前馈加入后对 dq 合成矢量整体缩放（此时 vsat=1
 *       表示撞 vmax），vmax_v=0 则不限幅、vsat 只反映 PI 是否贴 3.5V。
 * ==============================================================================
 */

#ifndef __FOC_41_DRUN_H__
#define __FOC_41_DRUN_H__

#include <stdint.h>
#include "foc_core.h"
#include "../Utils/dev_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FOC41_DBG   1
#if FOC41_DBG
#define FOC41_LOG(fmt, ...)  MAIN_D("[DRUN41] " fmt, ##__VA_ARGS__)
#else
#define FOC41_LOG(fmt, ...)  ((void)0)
#endif

#define FOC41_PI_KP             0.5f
#define FOC41_PI_KI             300.0f
#define FOC41_PI_UMAX_V         3.5f
#define FOC41_ITERM_MAX_V       3.2f
#define FOC41_VMAX_DEFAULT_V    6.0f
#define FOC41_IQ_FILT_ALPHA     0.10f
#define FOC41_I_REF_MA          500.0f
#define FOC41_I_RAMP_MA_S       0.0f
#define FOC41_ENC_DELTA_MAX     32

#define FOC41_SPEED_WIN_MS      200u
#define FOC41_PP_WIN_MS         5000u
#define FOC41_MEAN_WIN_MS       3000u
#define FOC41_ERR_WIN_MS        5000u

#define FOC41_STEP_IDLE         0u
#define FOC41_STEP_RUN          1u
#define FOC41_STEP_FAULT_OC     2u

#define FOC41_EVT_RAMP_DONE     1u
#define FOC41_EVT_OC            2u

/* 阶跃测量武装条件：进模式后第一次 |ref| >= FOC41_STEP_MIN_MA 的那一拍。
 * （2026-09-26 删除死宏 FOC41_STEP_DEADBAND_MA：它属早期"与上一拍参考之差"
 *   算法，现行代码只看 STEP_MIN_MA，定义后从未被引用。） */
#define FOC41_STEP_MIN_MA       100.0f
#define FOC41_STEP_TIMEOUT_US   200000u
#define FOC41_STEP_CONFIRM_TICK 3u
#define FOC41_STEP_TIME_TIMEOUT 0xFFFFFFFFu

#define FOC41_STEP_ST_IDLE      0u
#define FOC41_STEP_ST_WAIT      1u
#define FOC41_STEP_ST_DONE      2u
#define FOC41_STEP_ST_TIMEOUT   3u

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

/* Step-response instrumentation.  Armed once per run: the first ISR after
 * entering the mode where |ref| >= FOC41_STEP_MIN_MA starts the timer.
 * The reported time is the first sample where |actual| reaches
 * g_drun41_step_frac_pct percent of that target; the crossing must persist
 * for FOC41_STEP_CONFIRM_TICK samples before it is accepted.  (字段名沿用
 * t90，但阈值由 step_frac_pct 决定，默认被 ClearObservables 写成 75。） */
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
