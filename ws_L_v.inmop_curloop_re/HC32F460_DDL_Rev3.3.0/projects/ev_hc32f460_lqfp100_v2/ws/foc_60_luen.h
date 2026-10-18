/**
 *******************************************************************************
 * @file  foc_60_luen.h
 * @brief FOC mode 60 - 龙伯格观测器无感速度/电流双闭环（编码器角作裁判）。
 *
 * 定位：与 mode 45（SMO+PLL 无感）**同源**、只把观测器内核从滑模换成龙伯格，
 *   便于直接 A/B 对比两种反电动势观测器的收敛、噪声与相位滞后。
 *
 * 结构（与 mode 40 / mode 45 相同）：
 *   速度 PI（5ms 节拍）输出 q 轴电流参考 (mA)，id 参考 = 0；
 *   独立 id/iq 电流 PI 在每个电流采样 ISR（= FOC_ISR_HZ，当前 20kHz）执行。
 *   有感段用编码器转子角做 Park/电流环；龙伯格观测器（foc_luen.c）纯旁观，
 *   输入 = 前后两拍指令电压平均 + 同源 Clarke 电流 + PLL 回灌的 ω̂。
 *   启动自动转速 profile（0/200/500/1000 rpm，每秒一档）。
 *   切换无感：Watch 置 g_luen60_sensorless=1 且判定 g_luen_emf_ok=1 时锁存，
 *   Park 角换 PLL 补偿外推角、速度反馈换 PLL 低通转速估计；编码器角保留作裁判。
 *
 * 观测器（foc_luen.c，独立共享模块）：
 *   静止 αβ 扩展反电动势模型：dê/dt = ω̂·J·ê − l2·(i − î)（修正项必须取负，
 *   取正号会在 0~Ωo 区间发散；推导与稳定域见 foc_luen.h）：
 *     l1 = 2·Ωo − R/L，l2 = L·Ωo²（恒正，全转速段极点都在左半平面）
 *   ê 喂共享 foc_pll 出 θ̂/ω̂（与 mode 45 的 SMO 输出同口径）。
 *   模型参数 R/L/ψf 与带宽 Ωo 全部 Watch 可调。
 *
 * 前置：有效的 mode 24 校准（foc_24_dcal 快照）。
 * ==============================================================================
 *
 * ==============================================================================
 *   模式速览卡   MODE 60   龙伯格观测器无感 FOC
 *                          速度/电流双闭环（编码器角作裁判）
 *   本卡是本模式唯一事实源；代码内注释与本卡冲突时，以本卡为准
 * ==============================================================================
 *   入口   comm_mode = 60
 *   前置   必须先跑 mode 24 校准；否则 Foc_Luen60_Start 直接拒绝（RTT 打
 *          ERROR），g_luen60_running 保持 0
 *   结束   持续运行不自动停；过流自动停（g_luen60_state=2 / evt=1）；手动停
 *          comm_mode = 0
 *   标记   [!] 易错点   (*) 主判据   [可调] Watch 可写   [只读] 仅观察
 * ------------------------------------------------------------------------------
 *   [可调] Watch 变量 -- 改值即时生效，复位后回宏默认
 * ------------------------------------------------------------------------------
 *       变量                        当前值  单位     含义
 *       --------------------------- ------- -------- ---------------------------
 *       g_luen60_speed_target_rpm   0       rpm      目标转速；自动爬坡期间被覆
 *                                                    盖
 *       g_luen60_auto_ramp          1       -        启动自动爬坡；0=纯手动给目
 *                                                    标
 *   (*) 自动爬坡 profile：秒 0/1/2/3 -> 0/200/500/1000 rpm（表 s_auto_rpm）。t <
 *       4s 期间固件每拍写目标，Watch 改值会被立刻覆盖；t >= 4s 起停止写入，交还
 *       Watch。要全程手动就在进模式前把 g_luen60_auto_ramp 置 0。
 *       g_luen60_sensorless         0       -        1=切无感（需
 *                                                    g_luen_emf_ok=1）
 *       g_luen60_ang_lead_ticks     1.5     tick     有感角度超前补偿拍数，0=关
 *       g_luen60_iq_filt_alpha      0.10    -        iq 反馈滤波系数（显示用）
 *       g_luen60_wave_mode          0       -        VOFA 布局选择（见通道表）
 *       g_luen60_pid_speed_cfg.kp   0.8     mA/rpm   速度环 P（Watch
 *                                                    改立即生效）
 *       g_luen60_pid_speed_cfg.ki   0.03    mA/rpm/s 速度环 I
 *       g_luen60_pid_id_cfg.kp      0.1     V/A      电流环 P（d 轴）
 *       g_luen60_pid_iq_cfg.kp      0.1     V/A      电流环 P（q 轴）
 *       g_luen60_pid_id_cfg.ki      300     V/A/s    电流环 I（d 轴）
 *       g_luen60_pid_iq_cfg.ki      300     V/A/s    电流环 I（q 轴）
 *   (*) 上 6 行与 mode 40 / mode 45 同源定稿值（电流 kp 0.1 / ki 300、速度 kp
 *       0.8 / ki 0.03），便于两种观测器直接 A/B 对比。id / iq 两个 kp
 *       必须同步改。
 *       g_luen_model_r_ohm          0.1     ohm      模型定子电阻
 *                                                    R；错则幅值/相位偏
 *       g_luen_model_l_h            42.3e-6 H        模型电感 L（表贴式 Ld=Lq）
 *       g_luen_model_psi_wb         0.00084 Wb       模型磁链
 *                                                    psi_f；理论幅值用它
 *   (*) g_luen_omega_o              4712    rad/s    观测器带宽 Omega_o（极点
 *                                                    -Omega_o）
 *   (*) Omega_o 是龙伯格唯一的核心旋钮：建议取电流环带宽 kp/L 约 2364 rad/s 的
 *       2~5 倍，默认 4712（= 2pi x
 *       750）。连续极点全转速都在左半平面，无硬上限；固件 e_hat
 *       旋转项已改用精确旋转 R(omega_hat*Ts)（模恒为 1），离散极点由 -l2*err
 *       修正项压进单位圆内，7800rpm 下谱半径约 0.985<1，全转速段稳定，详见
 *       foc_luen.h。过大 = 高频抖动/噪声；过小 = 收敛慢、相位滞后大。
 *       g_luen_emax_v               20      V        反电动势发散保护上限
 *                                                    |e_hat|
 *       g_luen_ih_clamp_a           40      A        观测器电流数值钳位
 *       g_luen_jdg_rpm_min          900     rpm      反电动势判定生效转速下限
 *       g_luen_jdg_ratio_min        0.85    -        C1 幅值比下限
 *       g_luen_jdg_ratio_max        1.08    -        C1 幅值比上限
 *       g_luen_jdg_phase_min_deg    -45     deg      C2 相位下限
 *       g_luen_jdg_phase_max_deg    5       deg      C2 相位上限
 *       g_luen_jdg_hold_ms          1000    ms       判定连续满足时长
 *       g_luen_jdg_filt_alpha       0.02    -        判定级慢速 EMA 系数
 *       g_pll_comp_en               0       -        1=PLL 补偿角生效（默认关）
 *   [!] PLL 补偿公式（g_pll_comp_en=1 时）按 SMO 的 LPF
 *       迟滞口径推导，对龙伯格不适用；默认关，确实要补相位请只调
 *       g_pll_comp_bias_deg 手填。
 * ------------------------------------------------------------------------------
 *   [只读] 关键观察变量
 * ------------------------------------------------------------------------------
 *       变量                           含义
 *       ------------------------------ -----------------------------------------
 *   (*) g_luen60_speed_filt_rpm        PI
 *                                      真实反馈；判超调/抖动只认它（ch14/ch1）
 *       g_luen60_speed_disp_rpm        显示强滤波 a=0.05，约 100ms 滞后
 *       g_luen60_speed_ramp_rpm        斜坡整形后给定 = PI setpoint
 *       g_luen60_speed_meas_rpm        速度原始测量（未滤波）
 *       g_luen60_speed_err_rpm         速度误差 = ramp - 反馈
 *       g_luen60_speed_out_ma          速度环输出 = iq 参考 (mA)
 *       g_luen60_id_ma                 d 轴电流反馈 (mA)
 *       g_luen60_iq_ma                 q 轴电流反馈 (mA)
 *       g_luen60_iq_filt_ma            滤波后 iq (mA)
 *       g_luen60_vd                    电流环输出 vd (V)
 *       g_luen60_vq                    电流环输出 vq (V)；贴 6.2V = 撞电压墙
 *   [!] g_luen60_vsat                  =1 时输出贴限幅，此段调参数据作废
 *       g_luen60_sl_active             无感锁存：1=Park 角与速度反馈已用 PLL
 *       g_pll_omega_out_rpm            无感锁存后喂速度环的转速估计 (rpm)
 *       g_luen_emf_ok                  龙伯格判定通过=1；无感切换的前置条件
 *       g_luen_jdg_fail                失败掩码 bit0 转速/bit1 幅值/bit2 相位
 *       g_luen_jdg_ratio_filt          幅值比慢速均值；龙伯格无滑模跌落应约 1.0
 *   (*) g_luen_diag_theta_err_filt_deg 龙伯格相位差滤波均值（极点滞后，非零正
 *                                      常）
 *   (*) g_pll_diag_theta_err_filt_deg  PLL 角差均值；comp_en=1 后应约 0
 *       g_luen_diag_e_mag_v            |e_hat| 实测幅值 (V)
 *       g_luen_diag_e_expect_v         理论 |e| = |omega_e| psi_f (V)
 *       g_luen_diag_e_ratio            实测/理论，收敛良好约 1.0
 *   (*) g_luen_diag_e_on_q_v           e_on_q 投影 (V)；匀速下必须是常数
 *       g_luen_diag_e_on_d_v           e_on_d 投影 (V)；负常数 = 滞后体现
 *       g_luen_diag_e_err_v            总误差范数 (V)，约 omega psi delta(rad)
 *   [!] g_luen_diverge                 发散保护触发次数；>0 说明 |e_hat|
 *                                      越限被复位
 *       g_luen60_state                 状态机：0 空闲 / 1 运行 / 2 过流
 *       g_luen60_evt                   事件码（evt=1 过流）
 *       g_luen60_rotor_deg             转子电角（编码器角，裁判用）(deg)
 *       g_luen60_running               1=本 mode 已起转，VOFA 走自持布局
 * ------------------------------------------------------------------------------
 *   VOFA 通道 -- 16ch，填充见 Foc_Luen60_VofaFill
 * ------------------------------------------------------------------------------
 *       通道  含义                   单位     备注
 *       ----- ---------------------- -------- ----------------------------------
 *       ch0   iq 反馈                A        q 轴电流实际值
 *       ch1   iq 滤波                A        滤波后 iq，alpha 由 Watch 调
 *       ch2   iq 给定                A        速度环输出 = iq 参考
 *       ch3   理论 e_alpha           V        编码器角生成的基准正弦
 *       ch4   理论 e_beta            V        编码器角生成的基准正弦
 *       ch5   观测器 i_alpha_hat     A        龙伯格电流状态，应跟随真实 i
 *       ch6   观测器 i_beta_hat      A        龙伯格电流状态，应跟随真实 i
 *       ch7   反电动势 e_alpha_hat   V        应与 ch3 逐点重叠
 *       ch8   反电动势 e_beta_hat    V        应与 ch4 逐点重叠
 *       ch9   相位误差               deg      原始值抖动；读均值请用 ch15
 *   (*) ch10  e_on_q 投影            V        主判据：0.9*ch13，匀速平直
 *       ch11  e_on_d 投影            V        负值恒定 = 滞后体现
 *       ch12  总误差范数             V        约 omega psi delta(rad)
 *   (*) ch13  理论幅值               V        主判据：omega psi_f
 *       ch14  实际转速               rpm      = speed_filt_rpm，PI 反馈
 *   (*) ch15  相位差滤波             deg      主判据：匀速下均值恒定
 *   [!] 本 mode 通道数会变 -- g_luen60_wave_mode = 1 时 改成 8ch：波形窄帧（32B
 *       窄帧，921600 下帧率约 2.9kHz）
 *       通道  含义                   单位     备注
 *       ch0   e_alpha_hat            V
 *       ch1   e_beta_hat             V
 *       ch2   理论 e_alpha           V
 *       ch3   理论 e_beta            V
 *       ch4   相位差原始             deg
 *   (*) ch5   相位差滤波             deg
 *       ch6   观测器 i_alpha_hat     A
 *       ch7   观测器 i_beta_hat      A
 *   [!] 这 8 个通道与主布局的 ch0..ch7
 *       含义完全不同，不是前缀关系；切过来时上位机通道数必须改成 8。
 *   [!] 本 mode 通道数会变 -- g_luen60_wave_mode = 2 时 改成 16ch：PLL /
 *       无感验收面板
 *       通道  含义                   单位     备注
 *       ch0   编码器原始转速         rpm
 *       ch1   编码器滤波转速         rpm
 *       ch2   omega_hat 原始         rpm
 *       ch3   omega_out 低通         rpm      无感时喂速度环
 *       ch4   转速差原始             rpm
 *   (*) ch5   转速差滤波             rpm      主判据：无感滞后量
 *       ch6   角差原始               deg
 *   (*) ch7   角差滤波               deg      主判据
 *       ch8   iq 滤波                A
 *       ch9   补偿角 delta_hat       deg      comp_en=1 才真正进 Park 角
 *       ch10  无感锁存               0/1
 *       ch11  |e_hat|                V
 *       ch12  龙伯格同拍角差         deg
 *       ch13  目标转速               rpm
 *       ch14  显示滤波转速           rpm
 *       ch15  斜坡输出转速           rpm
 *   [!] ch9 的补偿角只有 g_pll_comp_en=1 时才真正加进 Park 角（默认 0，此时 ch9
 *       照算但不生效）。
 * ------------------------------------------------------------------------------
 *   判据与坑
 * ------------------------------------------------------------------------------
 *   (*) 定位：与 mode
 *       45（SMO+PLL）同源，只把观测器内核从滑模换成龙伯格，便于直接 A/B
 *       对比。电机由自带编码器速度/电流双闭环拖动，龙伯格观测器纯旁观；Watch 置
 *       g_luen60_sensorless=1 且 g_luen_emf_ok=1 后，Park 角换 PLL
 *       补偿外推角、速度反馈换 PLL 低通转速估计。
 *   (*) 主判据（wave_mode=0 面板）是四条直流判据恒定：ch10 e_on_q 约
 *       0.9*ch13、ch11 e_on_d 负值恒定、ch12 总误差约 omega psi delta(rad)、ch15
 *       相位差滤波均值。与 SMO 的差别：龙伯格 e_hat 是观测器状态量、无滑模 LPF
 *       跌落，故幅值比 g_luen_jdg_ratio_filt 应更贴近 1.0（SMO 约
 *       0.98）。匀速下这四条必须是平直线；出现电频率纹波 = 死区签名。
 *   [!] 无感切换前置：g_luen60_sensorless=1 只是申请，必须 g_luen_emf_ok=1
 *       才锁存（g_luen60_sl_active 变 1；置回 0 才退出）。判定要求转速 >=
 *       g_luen_jdg_rpm_min（当前 900rpm）且幅值/相位连续满足
 *       1000ms；自动爬坡顶档正好 1000rpm，所以到速后还要约 1.5-2s 才出 OK（慢速
 *       EMA a=0.02 收敛 + hold 1s）。失败原因看 g_luen_jdg_fail：bit0
 *       转速未达（爬坡途中正常）/bit1 幅值/bit2 相位。
 *   [!] 调参顺序：先确认有感段速度环（kp 0.8 / ki 0.03）与电流环（kp 0.1 / ki
 *       300）已稳定，再调观测器。观测器看 g_luen_omega_o（默认 4712 rad/s，2~5
 *       倍电流环带宽）与三个模型参数 R/L/psi_f；omega_o 过大先出高频抖动甚至
 *       g_luen_diverge++。模型参数不准表现为 g_luen_diag_e_ratio 偏离
 *       1.0（幅值）与相位均值偏离（L/R 比例）。
 *   [!] 本 mode 是变长帧：wave_mode=0/2 为 16ch，wave_mode=1 为 8ch。切布局时
 *       VOFA+ 上位机通道数必须跟着改，否则整帧错位。未先跑 mode 24 时 mode 60
 *       起不来（g_luen60_running=0），固件回落到通用 20ch 帧而 VOFA 仍按 16ch
 *       解析，看起来像观测器炸了。
 *       发散保护：Foc_Luen_Step 内若 |e_hat| 平方超过 g_luen_emax_v
 *       平方即复位观测器状态并累加 g_luen_diverge（ISR 内不用
 *       sqrtf）。静止阶段反电动势为零、不可观测，e_hat
 *       由电流误差注入起振，故必须靠有感段先跑到 900rpm 以上才谈锁定。
 * ==============================================================================
 */

#ifndef __FOC_60_LUEN_H__
#define __FOC_60_LUEN_H__

#include <stdint.h>
#include "foc_core.h"
#include "../Utils/dev_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FOC60_DBG   1
#if FOC60_DBG
#define FOC60_LOG(fmt, ...)  MAIN_D("[LUEN60] " fmt, ##__VA_ARGS__)
#else
#define FOC60_LOG(fmt, ...)  ((void)0)
#endif

/* 内环电流 PI：与 mode 40 / mode 45 同源定稿值（kp 0.1 / ki 300）。
 *   kp=0.1 是压静止噪声的下限（mode 40 同硬件实测：0.5 明显噪声，0.05 与 0.1 无差别）；
 *   UMAX 6.2V（12V 母线 SVPWM 线性区上限 12/sqrt(3)=6.93V，留约 10% 余量）。 */
#define FOC60_PI_KP             0.1f
#define FOC60_PI_KI             300.0f
#define FOC60_PI_UMAX_V         6.2f
#define FOC60_ITERM_MAX_V       6.0f
#define FOC60_IQ_FILT_ALPHA     0.10f

/* 外环速度 PI：输出带符号 q 轴电流参考 (mA)。与 mode 40 定稿值一致
 *   （kp 0.8 / ki 0.03；mode 40 实测 0->1000rpm 阶跃：超调 4.1%、上升约 600ms）。
 *   注：运行时可用 Watch 改 g_luen60_pid_speed_cfg.kp/.ki，此处仅为上电默认值。 */
#define FOC60_SPD_KP_MA_PER_RPM       0.8f
#define FOC60_SPD_KI_MA_PER_RPM_S     0.03f
#define FOC60_SPD_IQ_LIMIT_MA         ((float)FOC_MOTOR_MAX_CURRENT_A \
                                      * 1000.0f * 0.20f)

/* 安全包络（取自 motor_config.h） */
#define FOC60_SPEED_REF_LIMIT_RPM     ((float)FOC_MOTOR_MAX_SPEED_RPM)
#define FOC60_ACCEL_LIMIT_RPM_S       ((float)FOC_MOTOR_MAX_SPEED_RPM \
                                      * 0.25f)

#define FOC60_SPD_WIN_MS              5u
#define FOC60_SPD_WIN_US              (FOC60_SPD_WIN_MS * 1000u)
#define FOC60_SPD_FILT_ALPHA          0.25f
/* 显示专用滤波（VOFA 曲线平滑用；alpha 越小越平滑越滞后，不进 PI 反馈） */
#define FOC60_SPD_DISP_ALPHA          0.05f
/* 编码器每拍增量限幅 = 单拍物理极限 × 1.35 裕度。物理极限 = 7800rpm 折算到每拍：
 *   20 kHz（ 50µs/拍）：26.5 counts -> 限幅 36（2026-09-23 由 10kHz 的 72 同步改）
 * ⚠ 若再改 MOTOR_PWM_FREQ_HZ，此值须按 1/f 同步缩放，否则高速测速削顶 */
#define FOC60_ENC_DELTA_MAX           36

/* 启动自动转速 profile（g_luen60_auto_ramp=1 时生效，到顶后停止写入）：
 * 秒 0/1/2/3 -> 0/200/500/1000 rpm，之后保持 1000（Watch 可接管） */
#define FOC60_AUTO_RAMP_DEFAULT       1u
#define FOC60_AUTO_RAMP_SECS          3u

#define FOC60_STEP_IDLE               0u
#define FOC60_STEP_RUN                1u
#define FOC60_STEP_FAULT_OC           2u

#define FOC60_EVT_OC                  1u

extern volatile uint8_t  g_luen60_running;
extern volatile uint8_t  g_luen60_state;
extern volatile uint8_t  g_luen60_evt;
extern volatile uint8_t  g_luen60_auto_ramp;
extern volatile uint8_t  g_luen60_wave_mode;   /* VOFA 帧布局：0=16ch 观测器验收面板，
                                                * 1=8ch 波形窄帧，2=16ch PLL/无感验收面板 */
extern volatile uint8_t  g_luen60_sensorless;  /* 无感切换开关：0=编码器角有感闭环，
                                                * 1=Park 角 + 速度反馈换 PLL 估计（需
                                                * g_luen_emf_ok=1 才锁存；回 0 退出） */
extern volatile float    g_luen60_ang_lead_ticks; /* 有感角度超前补偿拍数（默认 1.5，0=关） */
extern volatile float    g_luen60_speed_target_rpm;
extern volatile float    g_luen60_speed_ramp_rpm;
extern volatile float    g_luen60_speed_meas_rpm;
extern volatile float    g_luen60_speed_filt_rpm;
extern volatile float    g_luen60_speed_disp_rpm;    /* 显示专用强滤波转速（alpha=0.05，
                                              * 仅 VOFA/Watch 看趋势；PI 反馈仍用
                                              * speed_filt_rpm，勿混用防环路滞后） */
extern volatile float    g_luen60_speed_err_rpm;
extern volatile float    g_luen60_speed_out_ma;
extern volatile int32_t  g_luen60_rotor_count;
extern volatile int32_t  g_luen60_rotor_deg;
extern volatile float    g_luen60_id_ref_ma;
extern volatile float    g_luen60_iq_ref_ma;
extern volatile float    g_luen60_id_ma;
extern volatile float    g_luen60_iq_ma;
extern volatile float    g_luen60_iq_filt_ma;
extern volatile float    g_luen60_iq_filt_alpha;
extern volatile float    g_luen60_vd;
extern volatile float    g_luen60_vq;
extern volatile uint8_t  g_luen60_vsat;
extern volatile uint8_t  g_luen60_sl_active;   /* 无感锁存状态（只读观察）：1=当前 Park/速度
                                                * 反馈已用 PLL 无感量，0=编码器 */
extern volatile float    g_luen60_du;
extern volatile float    g_luen60_dv;
extern volatile float    g_luen60_dw;

extern pid_config_t g_luen60_pid_speed_cfg;
extern pid_config_t g_luen60_pid_id_cfg;
extern pid_config_t g_luen60_pid_iq_cfg;

void Foc_Luen60_InitPids(void);
void Foc_Luen60_SetTargetRPM(float target_rpm);
void Foc_Luen60_Start(void);
void Foc_Luen60_Stop(void);
void Foc_Luen60_Step(const stc_i_data_t *pData);
int  Foc_Luen60_VofaFill(int32_t *cur);   /* 模式自持 VOFA，见顶部速览卡 */

#ifdef __cplusplus
}
#endif

#endif /* __FOC_60_LUEN_H__ */