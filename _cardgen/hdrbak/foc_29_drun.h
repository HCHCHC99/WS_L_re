/**
 *******************************************************************************
 * @file  foc_29_drun.h
 * @brief FOC mode 29 - independent run-only current loop.
 *
 * Mode 29 copies the calibration result produced by mode 24, samples the live
 * rotor angle when it starts, and then owns its entire 20 kHz current loop.
 *
 * ==============================================================================
 *   模式速览卡   MODE 29   纯电流环（复用 mode 24 校准，直接进 RUN）
 *   本卡是本模式唯一事实源；代码内注释与本卡冲突时，以本卡为准
 * ==============================================================================
 *   入口   comm_mode = 29
 *   前置   必须先跑 mode 24 校准；无校准 Start 直接拒绝启动
 *   结束   持续运行；仅过流自动回 mode 0
 *   标记   [!] 易错点   (*) 主判据   [可调] Watch 可写   [只读] 仅观察
 * ------------------------------------------------------------------------------
 *   [可调] Watch 变量 -- 改值即时生效，复位后回宏默认
 * ------------------------------------------------------------------------------
 *       变量                        当前值  单位     含义
 *       --------------------------- ------- -------- ---------------------------
 *       g_drun29_i_ref_ma           500     mA       电流矢量幅值参考=力矩旋钮
 *       g_drun29_i_ramp_ma_s        0       mA/s     软启动斜率；<=0 直通阶跃
 *       g_drun29_dlt_init_deg       90      deg      爬坡起点（Start 不复位）
 *       g_drun29_dlt_targ_deg       90      deg      爬坡终点（90 = id_ref 0）
 *       g_drun29_dlt_tr_ms          0       ms       过渡时间；0 = 立即到位
 *   (*) 上三行默认 90/90/0 = 阶跃激励（当前已配好）：进 RUN 第一拍 iq_ref
 *       即满值。
 *       g_drun29_iq_filt_alpha      0.10    -        ch16 显示滤波；PI 不用它
 *       g_drun29_step_frac_pct      75      %        阶跃判据百分比（Start
 *                                                    复位）
 *       g_drun29_pid_iq_cfg.kp      0.5     V/A      q 轴 P（调参主旋钮）
 *       g_drun29_pid_id_cfg.kp      0.5     V/A      d 轴 P；两轴必须同步改
 *   (*) 捏死转子（BEMF=0 纯 R+L 对象）下调 P：快速爬到平台且平台上方不振铃，kp
 *       扫 0.5 -> 1.0 -> 2.0。
 *       g_drun29_pid_iq_cfg.ki      300     V/A/s    q 轴 I（约 kp x R/L）
 *       g_drun29_pid_id_cfg.ki      300     V/A/s    d 轴 I；两轴一起改
 *       g_drun29_pid_iq_cfg.i_valid 1       -        1=开 I；置 0 = 纯 P 重调
 *       g_drun29_pid_id_cfg.i_valid 1       -        同上；两轴一起改
 *   [!] kp 过小实测（0.05）电机纹丝不动：P 环输出 kp x 误差
 *       克服不了铜阻压降；P-only 门槛约 (3~7) x R = 0.5~1 V/A。
 * ------------------------------------------------------------------------------
 *   [只读] 关键观察变量
 * ------------------------------------------------------------------------------
 *       变量                           含义
 *       ------------------------------ -----------------------------------------
 *   (*) g_drun29_eq_mean_ma            q 误差均值 = 稳态落差（ch11，5s 窗口）
 *   (*) g_drun29_iq_pp_ma              iq 反馈 5s 峰峰值（ch15）；振铃判据
 *       g_drun29_eq_pp_ma              q 误差峰峰值（5s 窗口，不在 VOFA 里）
 *   (*) g_drun29_iq_step_t90_us        阶跃到 90% 的耗时 (us)；超时 = 0xFFFFFFFF
 *       g_drun29_iq_step_state         阶跃测量状态 0 IDLE 1 WAIT 2 DONE 3
 *                                      TIMEOUT
 *       g_drun29_iq_step_target_ma     阶跃测量锁定的目标参考 (mA)
 *       g_drun29_ed_mean_ma            d 轴误差均值（5s 窗口）
 *       g_drun29_ed_pp_ma              d 轴误差峰峰值
 *       g_drun29_iq_ma                 q 轴反馈（零偏校正后 = 环反馈）
 *       g_drun29_id_ma                 d 轴反馈（零偏校正后）
 *       g_drun29_iq_ref_ma             q 轴参考实时值（= I_ref x sin delta）
 *       g_drun29_id_ref_ma             d 轴参考实时值（不恒为 0）
 *       g_drun29_iq_mean_ma            iq 3s 均值（慢速水平线）
 *       g_drun29_id_mean_ma            id 3s 均值
 *   [!] g_drun29_vsat                  =1 时输出顶到 UMAX 3.5V，处理数据作废
 *       g_drun29_iq_filt_ma            显示用滤波 iq；PI 吃 ch5 原始 iq
 *       g_drun29_speed_hz              实测电频率（200ms 窗，带符号）
 *       g_drun29_diff_deg              参考功角 = delta 指令（非独立测量）
 *       g_drun29_rotor_count           编码器累计相对计数（增量）
 *       g_drun29_time_us               进模式后运行时长 (us)
 *       g_drun29_state                 0 空闲 / 1 运行 / 2 过流
 *       g_drun29_evt                   1 爬坡完成 / 2 过流
 *       g_drun29_running               运行标志（1 = 正在运行）
 *       g_foc_elec_deg                 mode 0 下实时转子电角，捏转子对位看它
 * ------------------------------------------------------------------------------
 *   VOFA 通道 -- 19ch，填充见 Foc_Drun_VofaFill
 * ------------------------------------------------------------------------------
 *       通道  含义                   单位     备注
 *       ----- ---------------------- -------- ----------------------------------
 *       ch0   U 相电流               A
 *       ch1   V 相电流               A
 *       ch2   W 相电流               A
 *       ch3   静止系 ialpha          A
 *       ch4   静止系 ibeta           A
 *   (*) ch5   iq 反馈                A        环反馈（PI 用的原始 iq）
 *       ch6   id 反馈                A        环反馈
 *   (*) ch7   iq 参考                A        = I_ref x sin delta
 *       ch8   id 参考                A        不恒为 0（= I_ref x cos delta）
 *       ch9   vq 输出                V        贴 3.5V = 撞电压墙
 *       ch10  vd 输出                V
 *   (*) ch11  q 误差均值             A        P 调参主判据（5s 窗口）
 *       ch12  d 误差均值             A        5s 窗口刷新
 *       ch13  iq 3s 均值             A
 *       ch14  id 3s 均值             A
 *   (*) ch15  iq 反馈峰峰值          A        5s 窗口；iq 反馈 pp，非误差 pp
 *       ch16  滤波 iq                A        显示用；PI 不吃它
 *       ch17  delta 功角             deg      = 指令值（非独立测量）
 *       ch18  实测电频率             Hz
 * ------------------------------------------------------------------------------
 *   判据与坑
 * ------------------------------------------------------------------------------
 *   (*) 标准流程：comm_mode=24 等 RTT 打印 cal-only done -> mode 0
 *       捏死转子（BEMF=0，纯 R+L 对象）-> comm_mode=29 直接进电流环。
 *   (*) Start 会重锚定编码器帧：读 TMRA_1
 *       当前计数重建相对帧，校准后转子被捏动多少都被吸收，第一拍磁场方向即对准当
 *       前转子位置。
 *   [!] 本模式默认已是 PI（i_valid=1、ki=300）；要纯 P 判据需把 i_valid 置
 *       0。P-only 稳态落差 = kp/(kp+R) x i_ref（kp=0.5 只到约
 *       77%）属理论必然，不是故障。
 *   (*) 自带阶跃测量：进模式后首次 |参考| >= 100mA
 *       时武装，之后改参考不重启；t90_us 超时值 = 0xFFFFFFFF。
 *   [!] VOFA 帧率约 90fps 远慢于电气时间常数，上升沿看不见：看平台高度 +
 *       平台上方振铃 + ch11/ch15 统计；VOFA+ 通道数必须同步改成 19。
 *   [!] ch8（id 参考）不恒为 0，只有 delta=90 deg 才为 0，与 mode 40 不同；ch16
 *       只是显示滤波；g_drun29_vsat=1（顶到 UMAX 3.5V）时数据作废。
 * ==============================================================================
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

/* 阶跃测量武装条件：进模式后第一次 |ref| >= FOC29_STEP_MIN_MA 的那一拍。
 * （2026-09-26 删除死宏 FOC29_STEP_DEADBAND_MA：它属早期"与上一拍参考之差"
 *   算法，现行代码只看 STEP_MIN_MA，定义后从未被引用。） */
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

/* Step-response instrumentation.  Armed once per run: the first ISR after
 * entering the mode where |ref| >= FOC29_STEP_MIN_MA starts the timer.
 * The reported time is the first sample where |actual| reaches
 * g_drun29_step_frac_pct percent of that target; the crossing must persist
 * for FOC29_STEP_CONFIRM_TICK samples before it is accepted.  (字段名沿用
 * t90，但阈值由 step_frac_pct 决定，默认被 ClearObservables 写成 75。） */
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
