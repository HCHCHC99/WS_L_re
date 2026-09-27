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
 * ==============================================================================
 *   模式速览卡   MODE 45   SMO+PLL 无感 FOC 速度/电流双闭环（编码器角作裁判）
 *   本卡是本模式唯一事实源；代码内注释与本卡冲突时，以本卡为准
 * ==============================================================================
 *   入口   comm_mode = 45
 *   前置   必须先跑 mode 24 校准；否则 Foc_Smo45_Start 直接拒绝（RTT 打
 *          ERROR），g_smo45_running 保持 0
 *   结束   持续运行不自动停；过流自动停（g_smo45_state=2 / evt=1）；手动停
 *          comm_mode = 0
 *   标记   [!] 易错点   (*) 主判据   [可调] Watch 可写   [只读] 仅观察
 * ------------------------------------------------------------------------------
 *   [可调] Watch 变量 -- 改值即时生效，复位后回宏默认
 * ------------------------------------------------------------------------------
 *       变量                        当前值  单位     含义
 *       --------------------------- ------- -------- ---------------------------
 *       g_smo45_speed_target_rpm    0       rpm      目标转速；自动爬坡期间被固
 *                                                    件覆盖
 *       g_smo45_auto_ramp           1       -        启动自动爬坡；0=纯手动给目
 *                                                    标
 *   (*) 自动爬坡 profile：秒 0/1/2/3 -> 0/200/500/1000 rpm（表 s_auto_rpm）。t <
 *       4s 期间固件每拍写目标，Watch 改值会被立刻覆盖；t >= 4s 起停止写入，交还
 *       Watch。要全程手动就在进模式前把 g_smo45_auto_ramp 置 0。
 *       g_smo45_sensorless          0       -        1=切无感（需
 *                                                    g_smo_emf_ok=1）
 *       g_smo45_ang_lead_ticks      1.5     tick     有感角度超前补偿拍数，0=关
 *       g_smo45_iq_filt_alpha       0.10    -        iq 反馈滤波系数（显示用）
 *       g_smo45_wave_mode           0       -        VOFA 布局选择（见通道表）
 *       g_smo45_pid_speed_cfg.kp    0.8     mA/rpm   速度环 P（Watch
 *                                                    改立即生效）
 *       g_smo45_pid_speed_cfg.ki    0.03    mA/rpm/s 速度环 I
 *       g_smo45_pid_id_cfg.kp       0.1     V/A      电流环 P（d 轴）
 *       g_smo45_pid_iq_cfg.kp       0.1     V/A      电流环 P（q 轴）
 *       g_smo45_pid_id_cfg.ki       300     V/A/s    电流环 I（d 轴）
 *       g_smo45_pid_iq_cfg.ki       300     V/A/s    电流环 I（q 轴）
 *   (*) 上 6 行 2026-09-26 起 = mode 40 定稿值（电流 kp 0.1、速度 kp 0.8 / ki
 *       0.03）。原值 0.5 / 1.8 / 0.2 分别是 mode 29 家族默认与 mode 40
 *       调参前的起点。
 *   [!] id / iq 两个 kp 必须同步改，否则 dq 两轴响应不一致。kp
 *       是电流环限幅前增益，UMAX 已从 3.5 提到 6.2V（12V 母线 SVPWM 线性区上限
 *       12/sqrt(3)=6.93V，留约 10% 余量）。
 *   [!] 电流 kp=0.1 是压静止噪声的下限（mode 40 同硬件实测：0.5 明显噪声，0.05
 *       与 0.1 无差别）；带宽 0.1 -> 376Hz、0.5 -> 1.88kHz，压 kp
 *       的代价是内环变慢，不是稳定性问题。
 *       g_pll_comp_en               0       -        1=PLL 补偿角生效（默认关）
 *       g_pll_comp_bias_deg         0       deg      补偿角偏置微调；需
 *                                                    comp_en=1
 * ------------------------------------------------------------------------------
 *   [只读] 关键观察变量
 * ------------------------------------------------------------------------------
 *       变量                           含义
 *       ------------------------------ -----------------------------------------
 *   (*) g_smo45_speed_filt_rpm         PI 真实反馈；判超调/抖动只认它（ch14）
 *       g_smo45_speed_disp_rpm         显示强滤波 a=0.05，约 100ms 滞后
 *       g_smo45_speed_ramp_rpm         斜坡整形后给定 = PI setpoint
 *       g_smo45_speed_meas_rpm         速度原始测量（未滤波）
 *       g_smo45_speed_err_rpm          速度误差 = ramp - 反馈
 *       g_smo45_speed_out_ma           速度环输出 = iq 参考 (mA)
 *       g_smo45_id_ma                  d 轴电流反馈 (mA)
 *       g_smo45_id_ref_ma              d 轴电流给定（恒 0）
 *       g_smo45_iq_ma                  q 轴电流反馈 (mA)
 *       g_smo45_iq_filt_ma             滤波后 iq (mA)
 *       g_smo45_vd                     电流环输出 vd (V)
 *       g_smo45_vq                     电流环输出 vq (V)；贴 6.2V = 撞电压墙
 *   [!] g_smo45_vsat                   =1 时输出贴限幅，此段调参数据作废
 *       g_smo45_sl_active              无感锁存：1=Park 角与速度反馈已用 PLL
 *       g_pll_omega_out_rpm            无感锁存后喂速度环的转速估计 (rpm)
 *       g_smo_emf_ok                   SMO 判定通过=1；无感切换的前置条件
 *       g_smo_jdg_fail                 失败掩码 bit0 转速/bit1 幅值/bit2 相位
 *       g_smo_jdg_ratio_filt           幅值比慢速均值，实测中心约 0.98
 *   (*) g_smo_diag_theta_err_filt_deg  SMO 相位差滤波均值，1500rpm 约 -17.5deg
 *   (*) g_pll_diag_theta_err_filt_deg  PLL 角差均值；comp_en=1 后应约 0
 *       g_smo45_state                  状态机：0 空闲 / 1 运行 / 2 过流
 *       g_smo45_evt                    事件码（evt=1 过流）
 *       g_smo45_rotor_deg              转子电角（编码器角，裁判用）(deg)
 *       g_smo45_running                1=本 mode 已起转，VOFA 走自持布局
 * ------------------------------------------------------------------------------
 *   VOFA 通道 -- 16ch，填充见 Foc_Smo45_VofaFill
 * ------------------------------------------------------------------------------
 *       通道  含义                   单位     备注
 *       ----- ---------------------- -------- ----------------------------------
 *       ch0   iq 反馈                A        q 轴电流实际值（工况确认）
 *       ch1   iq 滤波                A        滤波后 iq，alpha 由 Watch 调
 *       ch2   iq 给定                A        速度环输出 = iq 参考
 *       ch3   理论 e_alpha           V        编码器角生成的基准正弦
 *       ch4   理论 e_beta            V        编码器角生成的基准正弦
 *       ch5   原始 z_alpha           V        SMO 滑模项，未滤波
 *       ch6   原始 z_beta            V        SMO 滑模项，未滤波
 *       ch7   滤波 e_alpha           V        应与 ch3 逐点重叠
 *       ch8   滤波 e_beta            V        应与 ch4 逐点重叠
 *       ch9   相位误差               deg      原始值抖动；读均值请用 ch15
 *   (*) ch10  e_on_q 投影            V        主判据：0.9*ch13，1500rpm 约 1.19V
 *       ch11  e_on_d 投影            V        负值恒定 = 滞后体现
 *       ch12  总误差范数             V        约 omega*psi*delta(rad)
 *   (*) ch13  理论幅值               V        主判据：omega*psi_f，1500rpm 1.32V
 *       ch14  实际转速               rpm      = speed_filt_rpm，PI 反馈
 *   (*) ch15  相位差滤波             deg      主判据：1500rpm 均值约 -17.5deg
 *   [!] 本 mode 通道数会变 -- g_smo45_wave_mode = 1 时 改成 8ch：波形窄帧（32B
 *       窄帧，921600 下帧率约 2.9kHz）
 *       通道  含义                   单位     备注
 *       ch0   z_alpha 原始           V
 *       ch1   z_beta 原始            V
 *       ch2   e_alpha_hat 滤波       V
 *       ch3   e_beta_hat 滤波        V
 *       ch4   理论 e_alpha           V
 *       ch5   理论 e_beta            V
 *       ch6   相位差原始             deg
 *   (*) ch7   相位差滤波             deg
 *   [!] 这 8 个通道与主布局的 ch0..ch7
 *       含义完全不同，不是前缀关系；切过来时上位机通道数必须改成 8。
 *   [!] 本 mode 通道数会变 -- g_smo45_wave_mode = 2 时 改成 16ch：PLL /
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
 *       ch12  SMO 同拍角差           deg
 *       ch13  目标转速               rpm
 *       ch14  显示滤波转速           rpm
 *       ch15  斜坡输出转速           rpm
 *   [!] ch9 的补偿角只有 g_pll_comp_en=1 时才真正加进 Park 角（默认 0，此时 ch9
 *       照算但不生效）。
 * ------------------------------------------------------------------------------
 *   判据与坑
 * ------------------------------------------------------------------------------
 *   (*) 主判据（wave_mode=0 面板）是四条直流判据恒定：ch10 e_on_q 约
 *       0.9*ch13、ch11 e_on_d 负值恒定、ch12 总误差约 omega*psi*delta(rad)、ch15
 *       相位差滤波均值。1500rpm 实测：ch13=1.32V、ch10=1.0~1.3V（中值 1.15，理论
 *       0.9*1.32=1.19V）、ch15 均值 -17.5deg（理论
 *       -17deg）。匀速下这四条必须是平直线；出现电频率纹波 = 死区签名。
 *   (*) 波形因串口带宽必然混叠，肉眼看的正弦形状不可信也不需要信 ----
 *       只看逐点重叠（ch3 vs ch7、ch4 vs ch8；wave_mode=1 时 ch2 vs ch4、ch3 vs
 *       ch5）。判跟踪：wave_mode=0 面板里没有目标和斜坡通道，要看跟踪得切
 *       wave_mode=2（ch13 目标 / ch15 斜坡 / ch1 反馈）或直接看
 *       Watch；本面板内只有 ch14 实际转速。
 *   [!] 本 mode 是变长帧：wave_mode=0/2 为 16ch，wave_mode=1 为 8ch。切布局时
 *       VOFA+ 上位机通道数必须跟着改，否则整帧错位。实测案例：未先跑 mode 24 时
 *       mode 45 起不来（g_smo45_running=0），固件回落到 Foc_Common_VofaFill 的
 *       20ch 通用帧而 VOFA 仍按 16ch 解析，ch13 读成 0.004、ch15 在 +/-100deg
 *       乱跳，看起来像观测器炸了。
 *   [!] 无感切换前置：g_smo45_sensorless=1 只是申请，必须 g_smo_emf_ok=1
 *       才锁存（g_smo45_sl_active 变 1；置回 0 才退出）。判定要求转速 >=
 *       g_smo_jdg_rpm_min（当前 900rpm）且幅值/相位连续满足
 *       1000ms；自动爬坡顶档正好 1000rpm，所以到速后还要约 1.5-2s 才出 OK（慢速
 *       EMA a=0.02 收敛 + hold 1s）。失败原因看 g_smo_jdg_fail：bit0
 *       转速未达（爬坡途中正常）/bit1 幅值/bit2 相位。
 *   [!] 2026-09-23 起 FOC_ISR_HZ=20kHz：SMO/PLL 的 alpha
 *       已同步改过（g_smo_lpf_alpha 0.39->0.21、g_pll_wout_lpf_alpha
 *       0.0125->0.00625）。若再动 MOTOR_PWM_FREQ_HZ，这些 alpha 与
 *       FOC45_ENC_DELTA_MAX(36) 必须按 1/f 同步缩放，否则相位滞后变大 /
 *       高速测速削顶（原 32 只支持到 4687rpm）。
 *   [!] 2026-09-26 起本模式速度/电流环增益 = mode 40 定稿值（电流 kp
 *       0.5->0.1、速度 kp 1.8->0.8、ki
 *       0.2->0.03）。理由：两者有感段同电机、同编码器、同一套 5ms
 *       速度环代码，而原值正是 mode 40 调参前的起点。
 *   [!] 无感段（g_smo45_sl_active=1）速度反馈换成 PLL 估计
 *       g_pll_omega_out_rpm，噪声特性与编码器不同：若切无感后出现抖动，应单独议
 *       外环参数，不要直接套有感段的结论。
 * ==============================================================================
 */

#ifndef __FOC_45_SMO_H__
#define __FOC_45_SMO_H__

#include <stdint.h>
#include "foc_core.h"
#include "../Utils/dev_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FOC45_DBG   1
#if FOC45_DBG
#define FOC45_LOG(fmt, ...)  MAIN_D("[SMO45] " fmt, ##__VA_ARGS__)
#else
#define FOC45_LOG(fmt, ...)  ((void)0)
#endif

/* Inner current PI: 2026-09-26 kp 由 mode 29 家族默认 0.5 改为 mode 40 的定稿值 0.1。
 *   依据：mode 40 与 mode 45 有感段是同一台电机、同一套电流采样、同一段环路代码，
 *   而 mode 40 实测：kp=0.5 静止（目标 0）时明显噪声（开关纹波 × P 增益 → 占空比被
 *   推偏 → 正反馈极限环）；kp=0.1 压住，0.05 与 0.1 已无差别。
 *   注意这不是"稳定性"问题：ωc = kp/L → 0.5 时 1.88 kHz、0.1 时 376 Hz，
 *   两者相位裕度都在 80° 上下；0.5 的代价是**噪声**，不是发散。
 *   ki 保持 300（噪声主要由 kp 决定）。要恢复带宽须先做电流采样 RC 抗混叠。
 * ⚠ UMAX 3.5→6.2（Step4 高速）：4800rpm 反电动势已 ≈3.4V，3.5V 限幅是
 *   高速第一电压墙。上限约束：12V 母线 SVPWM 线性区相电压峰值 = 12/√3
 *   = 6.93V，留 ~10% 余量防过调制。ITERM 同步放大防积分 early 饱和。 */
#define FOC45_PI_KP             0.1f
#define FOC45_PI_KI             300.0f
#define FOC45_PI_UMAX_V         6.2f
#define FOC45_ITERM_MAX_V       6.0f
#define FOC45_IQ_FILT_ALPHA     0.10f

/* Outer speed PI output is a signed q-axis current reference in mA.
 * 2026-09-26 对齐 mode 40 定稿值：kp 1.8 → **0.8**，ki 0.2 → **0.03**。
 *   起因：本模式原值（1.8 / 0.2）恰好是 mode 40 调参轨迹
 *     （kp 1.8 → 4.0 → 2.0 → 0.8；ki 0.2 → 0.45 → 0.01 → 0.03）的**起点**。
 *   同一台电机、同一编码器、同一套 5ms 速度环代码，没有理由各用一套。
 *   mode 40 实测（kp=0.8 / ki=0.03，0→1000rpm 阶跃）：超调 4.1%（峰值 1041rpm）、
 *   上升约 600ms（斜坡限幅物理下限 513ms）、稳态差收敛正常 ⇒ 已标"定稿"。
 *   ki 变化使 τi = kp/ki 由 9s 变 27s（积分介入更晚）；mode 40 实测证明稳态差
 *   仍能正常收敛（本机摩擦远小于早期估算）。
 * ⚠ 无感段（g_smo45_sl_active = 1）速度反馈换成 PLL 估计
 *   （g_pll_omega_out_rpm），噪声特性与编码器不同；若观测器噪声导致抖动，
 *   应单独议外环参数，不要直接外推有感段的结论。
 * 注：运行时可用 Watch 改 g_smo45_pid_speed_cfg.kp/.ki，此处仅为上电默认值。 */
#define FOC45_SPD_KP_MA_PER_RPM       0.8f
#define FOC45_SPD_KI_MA_PER_RPM_S     0.03f
#define FOC45_SPD_IQ_LIMIT_MA         ((float)FOC_MOTOR_MAX_CURRENT_A \
                                      * 1000.0f * 0.20f)

/* Safety envelope derived from motor_config.h. */
#define FOC45_SPEED_REF_LIMIT_RPM     ((float)FOC_MOTOR_MAX_SPEED_RPM)
#define FOC45_ACCEL_LIMIT_RPM_S       ((float)FOC_MOTOR_MAX_SPEED_RPM \
                                      * 0.25f)

#define FOC45_SPD_WIN_MS              5u
#define FOC45_SPD_WIN_US              (FOC45_SPD_WIN_MS * 1000u)
#define FOC45_SPD_FILT_ALPHA          0.25f
/* 显示专用滤波（VOFA 曲线平滑用；α 越小越平滑越滞后，不进 PI 反馈） */
#define FOC45_SPD_DISP_ALPHA          0.05f
/* 编码器每拍增量限幅 = 单拍物理极限 × 1.35 裕度。物理极限 = 7800rpm 折算到每拍：
 *   10 kHz（100µs/拍）：7800/60×4096×100µs = 53 counts → 限幅 72
 *   20 kHz（ 50µs/拍）：26.5 counts               → 限幅 36  ← 2026-09-23 同步改
 * ⚠ 原 32 只支持到 4687rpm（10kHz 时）：超限后测速削顶 + s_rotor_count 角度积分丢拍
 * （Park 角持续落后，高速转矩错位）——高速上不去的第一堵墙。
 * ⚠ 若再改 MOTOR_PWM_FREQ_HZ，此值须按 1/f 同步缩放 */
#define FOC45_ENC_DELTA_MAX           36

/* 启动自动转速 profile（g_smo45_auto_ramp=1 时生效，到顶后停止写入）：
 * 秒 0/1/2/3 -> 0/200/500/1000 rpm，之后保持 1000（Watch 可接管）
 * （Step 4 无感首切定在 1000rpm） */
#define FOC45_AUTO_RAMP_DEFAULT       1u
#define FOC45_AUTO_RAMP_SECS          3u

#define FOC45_STEP_IDLE               0u
#define FOC45_STEP_RUN                1u
#define FOC45_STEP_FAULT_OC           2u

#define FOC45_EVT_OC                  1u

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
