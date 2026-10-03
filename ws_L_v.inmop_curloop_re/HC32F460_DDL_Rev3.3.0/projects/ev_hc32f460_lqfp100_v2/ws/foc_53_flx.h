/**
 *******************************************************************************
 * @file  foc_53_flx.h
 * @brief FOC mode 53 - 永磁磁链 psi_f 辨识（编码器速度环 + 变转速 vq/omega_e 最小二乘）。
 *
 * 目的：**验证参数辨识的思路与方法**（不是生产标定）
 *   - 结果只放 Watch / VOFA，不写 Flash、不改 motor_config.h；
 *   - 同时给出 psi_f / 手册值比值，与 mode 45 的 SMO 反电动势数据交叉对比。
 *
 * 原理：稳态、id 约 0、diq/dt 约 0 时
 *     vq = R_eff·iq + omega_e·psi_f + V_dt
 *   把 R 固定取 mode 51 实测值（R 与 psi_f 近似共线、不能同时辨识），对
 *     y = vq - R_eff·iq      做二参数最小二乘 y = psi_f·omega_e + V_dt
 *   斜率即 psi_f（主结果）、截距即 V_dt。**斜率对常数偏置免疫**：R·iq 偏置、
 *   电流零偏、Vcc 增益误差里凡不随转速变化的部分都被截距吸收。
 *   电机由自带速度环自己爬到各转速点，**不需要任何外力**。
 *
 * 前置：必须先跑 mode 24（编码器零点），否则 Start 拒绝启动。
 * 结束：自动（减速关 PWM，state=DONE）；不写 Flash、不改 motor_config.h。
 * ==============================================================================
 *
 * ==============================================================================
 *   模式速览卡   MODE 53   永磁磁链 psi_f 辨识（编码器速度环 + 变转速 vq/omega_e
 *                          最小二乘）
 *   本卡是本模式唯一事实源；代码内注释与本卡冲突时，以本卡为准
 * ==============================================================================
 *   入口   comm_mode = 53（Watch 直接写）
 *   前置   必须先跑 mode 24（编码器零点）；无零点时 Start 拒绝启动并置
 *          evt=6。电机需能自由空载自转（自带速度环拖动）
 *   结束   自动：拟合 -> 减速 -> 关 PWM，state=DONE；不写 Flash、不改
 *          motor_config.h
 *   标记   [!] 易错点   (*) 主判据   [可调] Watch 可写   [只读] 仅观察
 * ------------------------------------------------------------------------------
 *   [可调] Watch 变量 -- 改值即时生效，复位后回宏默认
 * ------------------------------------------------------------------------------
 *       变量                        当前值  单位     含义
 *       --------------------------- ------- -------- ---------------------------
 *   (*) g_flx53_rpm                 800/    rpm      转速序列；默认 800 1600
 *                                                    2400 3200，取前 points
 *                                                    个。低于 100 无信噪比
 *   (*) g_flx53_points              4       -        取点数 2~6（斜率至少需 2
 *                                                    点，建议 4 点覆盖大转速差）
 *   (*) g_flx53_r_used_ohm          0.098   ohm      算 psi_f 用的 R；填 mode 51
 *                                                    实测 R_eff。R 与 psi_f
 *                                                    共线，必须固定不可同时辨识
 *       g_flx53_vdead_v             0.160   V        死区先验 V_dt；fit_vdt=0
 *                                                    时用它，fit_vdt=1
 *                                                    时只作对照
 *   (*) g_flx53_fit_vdt             1       -        1 = V_dt 与 psi_f
 *                                                    一起拟合（取截距）；0 =
 *                                                    用固定先验。斜率两法相同
 *   (*) g_flx53_avg_ms              300     ms       每点平均窗口
 *                                                    100~1000；越长越抗噪，但转
 *                                                    子拖动时间越长
 *       g_flx53_settle_ms           300     ms       稳态持续判据：
 *                                                    |ramp-filt|<tol 且
 *                                                    |diq|<20mA 连续这么久才开窗
 *       g_flx53_rpm_tol             15      rpm      稳态转速容差（爬速到位与稳
 *                                                    态判据共用）
 *   (*) g_flx53_iq_filt_alpha       0.05    -        测量侧 iq/vq EMA（tau 约
 *                                                    1ms）；只滤测量副本，不进电
 *                                                    流 PI 反馈链
 *       g_flx53_reject_en           0       -        3sigma
 *                                                    离群剔除（默认关）；采样噪
 *                                                    声大时再开
 *   [!] g_flx53_rpm_max             4000    rpm      超速保护阈值；滤波转速超它
 *                                                    立即停机置 evt=5。须低于
 *                                                    12V 极对数的电压墙转速
 *   [!] g_flx53_vbus_scale          1.000   -        残差微调系数，正常固定 1.000
 *                                                    （Vbus 已改读 PA4 实测值，
 *                                                    勿填 实测Vbus/12.0）
 *   (*) 上十二行为本轮可调项。全程约 4~6s（4 点 x 爬速约 1s + 稳态 0.3s + 窗
 *       0.3s，加收尾）。
 *   [!] g_flx53_r_used_ohm 是系统性输入：错 10% 约使 psi 差 1~2%，且不随转速
 *       变，故只进截距、不被斜率吸收。g_flx53_vbus_scale 同理，但已降级为
 *       残差微调。
 *   [!] 转速序列必须单调、点间拉开（建议间隔 >=
 *       800rpm）。点太近则斜率被噪声放大；最慢点决定信噪比下限。
 * ------------------------------------------------------------------------------
 *   [只读] 关键观察变量
 * ------------------------------------------------------------------------------
 *       变量                           含义
 *       ------------------------------ -----------------------------------------
 *   (*) g_flx53_psi_mwb                psi_f 主结果（拟合斜率）---- mWb，与手册
 *                                      0.84 对比
 *   (*) g_flx53_ratio                  psi_f / FOC_MOTOR_FLUX_VS ----
 *                                      与商家参数直接对比
 *   (*) g_flx53_psi_diff_mwb           端点差分校验值（首末两点斜率）----
 *                                      与主结果应接近
 *   (*) g_flx53_vdt_fit_mv             拟合截距 V_dt (mV) ---- 应与 mode 51 的
 *                                      160mV 互证
 *   (*) g_flx53_r2                     二参数拟合优度；>= 0.99
 *                                      才说明线性模型成立
 *       g_flx53_psi_spread_mwb         各点 psi_f 离散度 (max-min, mWb)；< 10%
 *                                      psi 为合格
 *       g_flx53_psi_alt_mwb            用 FOC_MOTOR_RS_OHM(0.1) 重算的 psi_f
 *                                      ---- 看对 R 的敏感度
 *       g_flx53_win_split_pct          窗内前/后半均值相对差
 *                                      (%)；大则说明窗内有漂移或噪声
 *       g_flx53_psi_live_mwb           当前点实时 psi_f 估计 (mWb)
 *       g_flx53_target_rpm             当前转速给定 (rpm)
 *       g_flx53_rpm_meas               PI 真实反馈转速 (rpm)
 *       g_flx53_omega_e                当前点电角频率 (rad/s)；窗内编码器计数 +
 *                                      Timer6 us 时基直算
 *       g_flx53_state                  0 IDLE / 1 爬速 / 2 稳态 / 3 平均窗 / 4
 *                                      推进 / 5 拟合 / 6 减速 / 7 DONE / 8 OC /
 *                                      9 超速
 *       g_flx53_evt                    1 OC / 2 完成OK / 3 完成离散度大 / 4
 *                                      超时跳过 / 5 超速 / 6 无 mode24 校准
 *       g_flx53_pts_done               已完成点数（RTT 进度打印靠它）
 *   [!] g_flx53_vsat                   1 = vd/vq 贴限幅（撞电压墙）；此时该点 vq
 *                                      被削平，数据不可用
 *       g_flx53_psi_pts                各点 psi_f 数组 (mWb)
 *       g_flx53_pts_vq                 各点 vq 数组 (V)；画 vq-R*iq 对 omega_e
 *                                      的直线用
 *       g_flx53_pts_iq                 各点 iq 数组 (A)
 *       g_flx53_pts_we                 各点 omega_e 数组 (rad/s)
 * ------------------------------------------------------------------------------
 *   VOFA 通道 -- 16ch，填充见 Foc_FlxId_VofaFill
 * ------------------------------------------------------------------------------
 *       通道  含义                   单位     备注
 *       ----- ---------------------- -------- ----------------------------------
 *       ch0   状态码                 -        见状态机表
 *       ch1   目标转速               rpm      当前点给定
 *   (*) ch2   实际转速               rpm      PI 反馈滤波值
 *   (*) ch3   vq                     V        真实 q 轴电压（实测 Vbus 折算）
 *   (*) ch4   iq                     A        测量侧 EMA 后
 *   (*) ch5   psi 实时               mWb      当前点估计，逐点阶跃
 *   (*) ch6   psi 最终               mWb      拟合完成后为定值
 *       ch7   离散度                 mWb      各点 max-min
 *       ch8   完成点数               -        0..points
 *       ch9   r2                     -        >= 0.99 合格
 *       ch10  窗内前后半差           %        > 1 说明窗内有漂移
 *       ch11  Vdead 用值             V        拟合值或先验
 *       ch12  psi 差分校验           mWb      首末两点斜率
 *       ch13  psi / 手册             -        接近 1 说明吻合
 *   [!] ch14  Vsat                   -        1 = 撞电压墙
 *       ch15  累计时间               ms       全程约 4000~6000
 * ------------------------------------------------------------------------------
 *   判据与坑
 * ------------------------------------------------------------------------------
 *   (*) 原理：稳态、id 约 0、diq/dt 约 0 时 vq = R_eff*iq + omega_e*psi_f +
 *       V_dt。把 R 固定取 mode 51 实测值，对 y = vq - R_eff*iq 做二参数最小二乘
 *       y = psi_f*omega_e + V_dt，斜率即 psi_f（主结果）、截距即 V_dt。
 *   (*) 为什么取斜率：R*iq 的偏差、电流零偏、Vcc 增益误差、死区 V_dt
 *       里凡不随转速变化的部分全被截距吸收，只有随 omega_e
 *       增长的项才进斜率。故电流采样偏置不污染主结果。
 *   (*) 为什么必须变转速：单点测量无法分离 R 与
 *       psi_f（都乘在电流/转速上），至少两个转速点才能定出斜率。转速点要拉开，斜
 *       率才不被噪声放大。
 *   [!] Vbus 已改读 PA4 实测值（20k/3k 分压，Foc_Vbus_GetV），"12.0V 假设"
 *       这一最大系统性误差源已消除；残余的 VREF/分压容差用 g_flx53_vbus_scale
 *       微调（正常 1.000）。电流采样噪声（Vcc 3.3V 抖动）经 EMA+长窗后仅
 *       <1%，不是主导。
 *   [!] 抗噪组合：测量侧 iq/vq EMA（alpha=0.05，tau 约 1ms，绝不进 PI 反馈链）+
 *       长窗平均（300ms）+ 窗内前 20ms 丢弃 + 斜率对常数偏置免疫。可选 3sigma
 *       离群剔除。
 *   (*) 流程：校验 mode24 -> 逐点：速度环爬速（斜坡限幅）->
 *       判稳态（|ramp-filt|<tol 且 |diq|<20mA 连续 300ms）-> 300ms
 *       平均窗（Timer6 us 时基 + 窗内编码器计数直算 omega_e）->
 *       全部点完后二参数最小二乘 -> 减速关 PWM。
 *   [!] 判读：r2 >= 0.99 且离散度 < 10% psi 才算合格；psi_diff
 *       与主结果应接近；Vdt 应与 mode 51 的 160mV 接近；ratio 接近 1
 *       说明与手册吻合。任一不符先查 r_used / 是否 Vsat（vbus_scale 应为 1.0）。
 *   [!] 本模式只输出观测量：不写 Flash、不改
 *       motor_config.h。用途是验证辨识思路，并与 mode 45 的 SMO
 *       反电动势数据交叉对比。
 *   (*) RTT 输出（开关 FOC_FLX53_DBG，定义在 foc_53_flx.h；打印点在 foc_obs
 *       事件段，ISR 内不打印，符合 RTT
 *       打印规范）：进入时打一行配置；每完成一点打 rpm/vq/iq/we/psi；结束时打
 *       psi/R2/ratio/spread/win_split/Vdt 与全部原始点，并对离散度大或 Vsat 追行
 *       warn；超时/OC/超速各一行。
 * ==============================================================================
 *******************************************************************************
 */

#ifndef __FOC_53_FLX_H__
#define __FOC_53_FLX_H__

#include <stdint.h>
#include "foc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * Debug macros（RTT打印规范：0/1 赋值式开关，定义在 .h，.c 只调封装宏；
 * 打印点落在 foc_obs 事件段 / Start（主循环上下文）；ISR 内不打印；
 * 禁止 %f 与中文，物理量一律缩放为整型 mV/mA/mWb）
 *=============================================================================*/
/* Debug switch: 1 = RTT prints on, 0 = off */
#define FOC_FLX53_DBG   1
#if FOC_FLX53_DBG
    #define FLX53_DBG(fmt, ...)   MAIN_D("[FLX53] " fmt, ##__VA_ARGS__)
#else
    #define FLX53_DBG(fmt, ...)   ((void)0)
#endif

/*=============================================================================
 * 编译期常量
 *=============================================================================*/
#define FOC53_MAX_POINTS       6u       /* 转速序列容量（VOFA/数组上限） */
#define FOC53_AVG_MIN_MS       100u     /* 平均窗口下限 (ms) */
#define FOC53_AVG_MAX_MS       1000u    /* 平均窗口上限 (ms) */
#define FOC53_AVG_SKIP_MS      20u      /* 平均窗前段丢弃（切入新转速点、环路未稳态） */
#define FOC53_SETTLE_MIN_MS    50u      /* 稳态持续判据下限 (ms) */
#define FOC53_SETTLE_MAX_MS    2000u    /* 稳态持续判据上限 (ms) */
#define FOC53_SPINUP_TIMEOUT_MS  5000u  /* 单点爬速超时 (ms)：超时跳过该点 */
#define FOC53_SETTLE_TIMEOUT_MS  5000u  /* 单点稳态超时 (ms)：超时跳过该点 */
#define FOC53_RAMPDOWN_TIMEOUT_MS 3000u /* 收尾减速超时 (ms)：超时强关 */
#define FOC53_RPM_MIN          100.0f   /* 转速点下限 (rpm)：低于它无信噪比 */
#define FOC53_RPM_STOP         30.0f    /* 收尾判"已停"阈值 (rpm) */
#define FOC53_IQ_STABLE_A      0.020f   /* 稳态判据 |Δiq| 阈值 (A, 20mA) */
#define FOC53_IQ_SLOW_ALPHA    0.01f    /* 稳态判据慢 EMA (τ 约 5ms)，只判稳不测量 */
#define FOC53_SPREAD_LIMIT_PCT 10.0f    /* 各点 psi_f 离散度合格线 (%) */
#define FOC53_RPM_MAX_HARD     7800.0f  /* 超速阈值硬上限 (rpm) = FOC_MOTOR_MAX_SPEED_RPM */
#define FOC53_VBUS_SCALE_MIN   0.5f     /* 残差微调系数护栏（正常 1.0） */
#define FOC53_VBUS_SCALE_MAX   1.5f

/* 环路参数（镜像 mode 40 已验证值，同电机同硬件；模块自持便于单独调） */
#define FOC53_PI_KP              0.1f   /* 电流环 P (V/A)：kp=0.1 是压静止噪声的串级下限 */
#define FOC53_PI_KI              300.0f /* 电流环 I (V/A/s) */
#define FOC53_PI_UMAX_V          6.2f   /* 电流环输出限幅 (V) */
#define FOC53_ITERM_MAX_V        6.0f   /* 电流环积分限幅 (V) */
#define FOC53_SPD_KP_MA_PER_RPM  0.8f   /* 速度环 P (mA/rpm) */
#define FOC53_SPD_KI_MA_PER_RPM_S 0.03f /* 速度环 I (mA/rpm/s) */
#define FOC53_SPD_IQ_LIMIT_MA    ((float)FOC_MOTOR_MAX_CURRENT_A * 1000.0f * 0.20f)
#define FOC53_SPEED_REF_LIMIT_RPM ((float)FOC_MOTOR_MAX_SPEED_RPM)
#define FOC53_ACCEL_LIMIT_RPM_S  ((float)FOC_MOTOR_MAX_SPEED_RPM * 0.25f)
#define FOC53_SPD_WIN_MS         5u     /* 速度环抽样窗 (ms) */
#define FOC53_SPD_WIN_US         (FOC53_SPD_WIN_MS * 1000u)
#define FOC53_SPD_FILT_ALPHA     0.25f  /* 速度反馈 EMA（进 PI） */
#define FOC53_SPD_DISP_ALPHA     0.05f  /* 速度显示专用强滤波（不进 PI） */
#define FOC53_ENC_DELTA_MAX      36     /* 编码器每拍增量限幅 (counts，20kHz 下 7800rpm 的 1.35 倍) */

/*=============================================================================
 * 状态机（g_flx53_state）
 *=============================================================================*/
#define FOC53_STEP_IDLE        0u  /* 未运行 */
#define FOC53_STEP_SPINUP      1u  /* 速度环爬向当前转速点（斜坡限幅），同时进稳态计时 */
#define FOC53_STEP_SETTLE      2u  /* 判稳态：|ramp-filt|<tol 且 |Δiq|<20mA 连续 settle_ms */
#define FOC53_STEP_AVG         3u  /* avg_ms 窗：前 20ms 丢弃；EMA 后累加 iq/vq；Timer6+编码器算 omega_e */
#define FOC53_STEP_NEXT        4u  /* 推进到下一个转速点，或转 FIT */
#define FOC53_STEP_FIT         5u  /* 二参数最小二乘（斜率=psi_f）+ 逐点/离散度/r2/差分校验 */
#define FOC53_STEP_RAMPDOWN    6u  /* 目标置 0，等 |filt rpm| < 30 后关 PWM */
#define FOC53_STEP_DONE        7u  /* 完成（PWM 已关）；running 保持 1 供回看 */
#define FOC53_STEP_FAULT_OC    8u  /* 过流停机 */
#define FOC53_STEP_FAULT_OVRPM 9u  /* 超速停机 */

/*=============================================================================
 * 事件码（g_flx53_evt，ISR 置位 + g_flx53_evt_seq 递增，主循环打印去重）
 *=============================================================================*/
#define FOC53_EVT_OC           1u  /* 过流停机 */
#define FOC53_EVT_DONE_OK      2u  /* 完成且离散度合格 */
#define FOC53_EVT_DONE_POOR    3u  /* 完成但离散度大（数据可疑） */
#define FOC53_EVT_TIMEOUT      4u  /* 某点 5s 未稳 -> 跳过 */
#define FOC53_EVT_OVRPM        5u  /* 超速停机 */
#define FOC53_EVT_NO_CAL       6u  /* 无 mode 24 校准，拒绝启动 */

/*=============================================================================
 * Keil Watch 可调参数（定义见 foc_53_flx.c）
 *=============================================================================*/
extern volatile float    g_flx53_rpm[FOC53_MAX_POINTS]; /* 转速序列 (rpm, 前 points 个有效, 默认 800/1600/2400/3200/0/0) */
extern volatile uint32_t g_flx53_points;      /* 取点数 (2~6, 默认 4) */
extern volatile float    g_flx53_r_used_ohm;  /* 算 psi_f 用的 R (Ω, 默认 0.098, 填 mode 51 实测值) */
extern volatile float    g_flx53_vdead_v;     /* 死区先验 V_dt (V, 默认 0.160, fit_vdt=0 时使用) */
extern volatile uint32_t g_flx53_fit_vdt;     /* 1=把 V_dt 与 psi_f 一起拟合; 0=用固定先验 */
extern volatile uint32_t g_flx53_avg_ms;      /* 每点平均窗口 (ms, 默认 300, 100~1000) */
extern volatile uint32_t g_flx53_settle_ms;   /* 稳态持续判据 (ms, 默认 300) */
extern volatile float    g_flx53_rpm_tol;     /* 稳态转速容差 (rpm, 默认 15) */
extern volatile float    g_flx53_iq_filt_alpha; /* 测量侧 iq EMA (默认 0.05, τ 约 1ms, 不进环路) */
extern volatile uint32_t g_flx53_reject_en;   /* 3σ 离群剔除开关 (默认 0 关) */
extern volatile float    g_flx53_rpm_max;     /* 超速保护阈值 (rpm, 默认 4000) */
extern volatile float    g_flx53_vbus_scale;  /* 残差微调系数 (默认 1.000；Vbus 已改实测，勿填 实测Vbus/12.0) */

/*=============================================================================
 * 观测量（Watch / VOFA）
 *=============================================================================*/
extern volatile uint8_t  g_flx53_running;
extern volatile uint8_t  g_flx53_state;
extern volatile uint8_t  g_flx53_evt;
extern volatile uint8_t  g_flx53_evt_seq;     /* 每置一次 evt 递增（foc_obs 据此去重打印） */
extern volatile float    g_flx53_psi_mwb;     /* psi_f 主结果（斜率，mWb） */
extern volatile float    g_flx53_psi_diff_mwb;/* psi_f 端点差分校验值 (mWb) */
extern volatile float    g_flx53_vdt_fit_mv;  /* 拟合截距 V_dt (mV) —— 与 mode 51 的 160mV 互证 */
extern volatile float    g_flx53_vdt_used_mv; /* 本轮折算用的 V_dt (mV)：fit_vdt=1 用拟合值，否则用先验 */
extern volatile float    g_flx53_ratio;       /* psi_f / FOC_MOTOR_FLUX_VS */
extern volatile float    g_flx53_psi_alt_mwb; /* 用 FOC_MOTOR_RS_OHM(0.1) 重算的 psi_f (mWb)：R 敏感度 */
extern volatile float    g_flx53_psi_spread_mwb; /* 各点 psi_f 离散度 max-min (mWb) */
extern volatile float    g_flx53_r2;          /* 二参数拟合优度 */
extern volatile float    g_flx53_win_split_pct; /* 窗内前/后半均值相对差 (%)：诊断噪声/漂移 */
extern volatile float    g_flx53_psi_live_mwb;/* 当前点实时 psi_f (mWb) */
extern volatile float    g_flx53_target_rpm;  /* 当前转速给定 (rpm) */
extern volatile float    g_flx53_rpm_meas;    /* PI 真实反馈转速 (rpm) */
extern volatile float    g_flx53_rpm_disp;    /* 显示强滤波转速 (rpm, α=0.05, 只看趋势) */
extern volatile float    g_flx53_vq;          /* 真实 q 轴电压（实测 Vbus 折算 × 残差刻度）(V) */
extern volatile float    g_flx53_iq;          /* q 轴电流反馈原始值 (mA) */
extern volatile float    g_flx53_iq_filt;     /* q 轴电流测量侧 EMA (mA, 不进环路) */
extern volatile uint8_t  g_flx53_vsat;        /* 1 = vd/vq 贴限幅（撞电压墙） */
extern volatile float    g_flx53_omega_e;     /* 当前点电角频率 (rad/s) */
extern volatile uint32_t g_flx53_pts_done;    /* 已完成点数（foc_obs 靠它打印进度） */
extern volatile uint32_t g_flx53_elapsed_ms;  /* 累计耗时 (ms) */
extern volatile float    g_flx53_psi_pts[FOC53_MAX_POINTS];   /* 各点 psi_f (mWb) */
extern volatile float    g_flx53_pts_rpm[FOC53_MAX_POINTS];   /* 各点转速 (rpm) */
extern volatile float    g_flx53_pts_vq[FOC53_MAX_POINTS];    /* 各点 vq (V) */
extern volatile float    g_flx53_pts_iq[FOC53_MAX_POINTS];    /* 各点 iq (A) */
extern volatile float    g_flx53_pts_we[FOC53_MAX_POINTS];    /* 各点 omega_e (rad/s) */

/*=============================================================================
 * API
 *=============================================================================*/

/* mode 53 入口：校验 mode 24 -> 参数护栏 -> 清结果 -> 起速度环 -> 逐点爬速/稳态/平均 -> 拟合。
 * 主循环上下文调用（dev_comm_runner）。 */
void Foc_FlxId_Start(void);

/* 20 kHz ISR 步进：OC 保护 -> 速度/电流双闭环 -> 按状态机测量/滤波/拟合。
 * 由 Foc_Isr 在 g_flx53_running 时分发调用。 */
void Foc_FlxId_Step(const stc_i_data_t *pData);

/* 停止（用户中途切模式时调用）：清运行标志 + 关 PWM。 */
void Foc_FlxId_Stop(void);

/* 模式自持 VOFA：固定 16ch，返回通道数；通道含义见本文件顶部速览卡。 */
int  Foc_FlxId_VofaFill(int32_t *cur);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_53_FLX_H__ */
