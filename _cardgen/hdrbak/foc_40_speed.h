/**
 *******************************************************************************
 * @file  foc_40_speed.h
 * @brief FOC mode 40 - encoder FOC with cascade speed/current loops.
 *
 * The speed PI runs on a fixed 5 ms tick and generates the q-axis current
 * reference.  The id reference is zero.  The independent id/iq current PIs
 * run every current-sample ISR.  Mode 40 requires a valid mode 24 calibration
 * before start.
 *
 * ==============================================================================
 *   模式速览卡   MODE 40   编码器 FOC 速度/电流双闭环
 *   本卡是本模式唯一事实源；代码内注释与本卡冲突时，以本卡为准
 * ==============================================================================
 *   入口   comm_mode = 40
 *   前置   先跑 mode 24 校准，否则 Start 拒绝启动
 *   结束   目标置 0 等减速完，再 comm_mode = 0
 *   标记   [!] 易错点   (*) 主判据   [可调] Watch 可写   [只读] 仅观察
 * ------------------------------------------------------------------------------
 *   [可调] Watch 变量 -- 改值即时生效，复位后回宏默认
 * ------------------------------------------------------------------------------
 *       变量                        当前值  单位     含义
 *       --------------------------- ------- -------- ---------------------------
 *       g_speed40_speed_target_rpm  0       rpm      目标转速（写它即给给定）
 *       g_speed40_iq_filt_alpha     0.10    -        iq 显示滤波系数
 *       g_speed40_pid_speed_cfg.kp  0.8     mA/rpm   速度环 P
 *       g_speed40_pid_speed_cfg.ki  0.03    mA/rpm/s 速度环 I
 *   (*) 上两行为 2026-09-23 定稿实测值：0->1000rpm 超调 4.1%、上升约
 *       600ms。勿随意改。
 *       g_speed40_pid_id_cfg.kp     0.1     V/A      电流环 P（d 轴）
 *       g_speed40_pid_iq_cfg.kp     0.1     V/A      电流环 P（q 轴）
 *       g_speed40_pid_id_cfg.ki     300     V/A/s    电流环 I（d 轴）
 *       g_speed40_pid_iq_cfg.ki     300     V/A/s    电流环 I（q 轴）
 *   [!] kp=0.1 是压静止噪声的串级下限（0.1 与 0.05
 *       实测无差别）；再降内环会慢过外环。
 *   [!] id / iq 两个 kp 必须同步改，否则 dq 两轴响应不一致。
 * ------------------------------------------------------------------------------
 *   [只读] 关键观察变量
 * ------------------------------------------------------------------------------
 *       变量                           含义
 *       ------------------------------ -----------------------------------------
 *   (*) g_speed40_speed_filt_rpm       PI 真实反馈；判超调/振铃只认它（ch17）
 *       g_speed40_speed_ramp_rpm       斜坡整形后的给定 = PI 的 setpoint
 *       g_speed40_speed_meas_rpm       速度原始测量值（未滤波）
 *       g_speed40_speed_err_rpm        速度误差 = ramp - filt
 *       g_speed40_speed_out_ma         速度环输出 = iq_ref (mA)
 *       g_speed40_speed_disp_rpm       显示强滤波 a=0.05 tau 约 100ms；只看趋势
 *       g_speed40_id_ma                d 轴电流反馈 (mA)
 *       g_speed40_iq_ma                q 轴电流反馈 (mA)
 *       g_speed40_iq_filt_ma           滤波后 iq (mA)
 *       g_speed40_id_ref_ma            d 轴电流给定（恒 0）
 *       g_speed40_iq_ref_ma            q 轴电流给定 = 速度环输出
 *       g_speed40_vd                   电流环输出 vd (V)
 *       g_speed40_vq                   电流环输出 vq (V)；贴 6.2V = 撞电压墙
 *   [!] g_speed40_vsat                 =1 时输出贴限幅，调参数据作废
 *       g_speed40_rotor_deg            转子电角 (deg)
 *       g_speed40_rotor_count          编码器累计计数（增量）
 *       g_speed40_state                状态机：0 空闲 / 1 运行 / 2 过流
 *       g_speed40_evt                  事件码（最近一次状态迁移原因）
 *       g_speed40_running              运行标志（1 = 已起转）
 * ------------------------------------------------------------------------------
 *   VOFA 通道 -- 18ch，填充见 Foc_Speed_VofaFill
 * ------------------------------------------------------------------------------
 *       通道  含义                   单位     备注
 *       ----- ---------------------- -------- ----------------------------------
 *       ch0   U 相电流               A
 *       ch1   V 相电流               A
 *       ch2   W 相电流               A
 *       ch3   静止系 ialpha          A
 *       ch4   静止系 ibeta           A
 *       ch5   iq 反馈                A        控制系 q 轴电流
 *       ch6   id 参考                A        恒 0
 *       ch7   iq 参考                A        = 速度环输出
 *       ch8   vd 输出                V
 *   (*) ch9   vq 输出                V        贴 6.2V = 撞电压墙
 *       ch10  静止系电流幅值         A
 *       ch11  预留                   -        恒 0
 *       ch12  预留                   -        恒 0
 *   (*) ch13  斜坡输出转速           rpm      内部斜坡整形后的真实给定
 *       ch14  目标转速               rpm
 *       ch15  实际转速（显示强滤波） rpm      a=0.05 tau 约 100ms，刻意滞后
 *       ch16  iq 滤波                A
 *   (*) ch17  实际转速（PI 反馈）    rpm      速度环真正吃的反馈
 * ------------------------------------------------------------------------------
 *   判据与坑
 * ------------------------------------------------------------------------------
 *   (*) 判跟踪只看两项：ch17（反馈）追 ch13（给定）。
 *   [!] 不要用 ch15 判跟踪：它是显示强滤波（tau 约 100ms），动态中必然落后
 *       ch17，属刻意设计不是故障；两者稳态重合。
 *   [!] ch13 落后 ch14 = 斜坡限幅卡住（宏 FOC40_ACCEL_LIMIT_RPM_S）；ch13 到位而
 *       ch17 落后，才是环路/滤波问题。
 *   [!] VOFA+ 上位机通道数必须同步配成 18，否则整帧错位。
 * ==============================================================================
 */

#ifndef __FOC_40_SPEED_H__
#define __FOC_40_SPEED_H__

#include <stdint.h>
#include "foc_core.h"
#include "../Utils/dev_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FOC40_DBG   1
#if FOC40_DBG
#define FOC40_LOG(fmt, ...)  MAIN_D("[SPEED40] " fmt, ##__VA_ARGS__)
#else
#define FOC40_LOG(fmt, ...)  ((void)0)
#endif

/* Inner current PI.
 * ⚠ 2026-09-23 实测调参：kp 0.5 → 0.1（= mode 40 静止噪声的处置，已在用）
 *
 * 现象：目标 0、静止时电机有嘈杂噪声，且电流反馈真的在振荡
 *       （VOFA ch5 iq 反馈 +1.21/−0.8A，ch8 vd ±0.3V、ch9 vq ±0.5V 不规则抖动）。
 * 机理：开关纹波 → 无抗混叠的电流反馈原样采入 → PI 的 P 项（kp × 纹波）放大
 *       → 占空比被推偏 → 纹波更大 = 正反馈极限环。kp 直接决定这条路的增益。
 * 实测：kp=0.5 → 明显噪声；kp=0.1 → 大幅下降；kp=0.05 → 与 0.1 无差别
 *       ⇒ 0.1 以下噪声已到底噪（PWM 开关本身），再降无收益。
 *
 * ⚠⚠ 0.1 是**结构下限，不要再降**：
 *   电流环带宽 ωc = kp/L = 0.1/42.3µH = 376 rad/s ≈ 60 Hz，
 *   仅比速度环（kp=2 → 约 36 Hz）快 1.7 倍，**已低于串级经验要求 5~10 倍**。
 *   再降内环会慢过外环，串级结构失效。
 *   要恢复带宽必须从硬件侧切断纹波（传感器输出 1kΩ+100nF → fc≈1.6kHz，
 *   10kHz 处 −16dB，且在环路之外不消耗相位裕度），之后 kp 可回到 0.3~0.5。
 *
 * 副产物：kp=0.1 时电流环 ζ = (R+kp)/(2√(ki·L)) = 0.2/(2×0.1127) ≈ 0.89，
 *   接近临界阻尼（ζ=1 对应 kp≈0.108），比原来的过阻尼(2.66)更"刚好"。
 *
 * 注：运行时可用 Watch 改 g_speed40_pid_id_cfg / iq_cfg .kp，此处为上电默认值。 */
#define FOC40_PI_KP             0.1f
#define FOC40_PI_KI             300.0f
/* ⚠ UMAX 3.5→6.2 / ITERM 3.2→6.0（与 mode 45 SMO45 同步，2026-09-23）：
 *   12V 母线 SVPWM 线性区相电压峰值 6.93V，原 3.5V 是高速电压墙（卡 ~4800rpm） */
#define FOC40_PI_UMAX_V         6.2f
#define FOC40_ITERM_MAX_V       6.0f
#define FOC40_IQ_FILT_ALPHA     0.10f

/* Outer speed PI output is a signed q-axis current reference in mA.
 * 2026-09-23 调参轨迹：kp 1.8 → 4.0 → 2.0 → **0.8**；
 *                      ki 0.2 → 0.45 → 0.01 → **0.03**
 *
 * ★ 实测结果（kp=0.8 / ki=0.03，0→1000rpm 阶跃）：
 *     超调 **4.1%**（峰值 1041rpm）— 反推阻尼比 ζ≈0.71，属"优秀"档（≤5%）
 *     上升 **约 600ms**          — 斜坡限幅物理下限 1000/1950 = 513ms，
 *                                  仅多 87ms，已接近最优
 *     稳态差 **收敛正常**        — 实测未出现积分过慢现象
 *   ⇒ **本组参数为定稿值，勿再改动。**
 *
 *   （注：按 kp/ki 推算积分时间常数 τi=27s，曾担心稳态差收敛需数十秒；
 *     实测证否——说明本机摩擦远小于估算、且积分器在爬坡段已积累足够电流。
 *     理论模型在此偏保守，以实测为准。）
 *
 * 理论要点（详见 md_record/mode40控制系统_纯理论计算手册.md）：
 *   kp 同时决定带宽与相位裕度（唯一带宽旋钮，主导超调与上升时间）；
 *   ki 主导稳态差收敛（且越大 PI 零点越高、相位裕度反而**提高**）。
 *
 * 注：运行时可用 Watch 改 g_speed40_pid_speed_cfg.kp/.ki，此处仅为上电默认值。 */
#define FOC40_SPD_KP_MA_PER_RPM       0.8f
#define FOC40_SPD_KI_MA_PER_RPM_S     0.03f
/* 速度环输出限幅 = 最大电流 × 20%（17A × 0.2 = 3400mA）。
 * 取 20% 而非更高：17A 是峰值能力（厂商规格书原文「最大电流」），
 * 按其 20% 折算已相当于一个合理的连续工作点。 */
#define FOC40_SPD_IQ_LIMIT_MA         ((float)FOC_MOTOR_MAX_CURRENT_A \
                                        * 1000.0f * 0.20f)

/* Safety envelope derived from motor_config.h. */
#define FOC40_SPEED_REF_LIMIT_RPM     ((float)FOC_MOTOR_MAX_SPEED_RPM)
#define FOC40_ACCEL_LIMIT_RPM_S       ((float)FOC_MOTOR_MAX_SPEED_RPM \
                                        * 0.25f)

#define FOC40_SPD_WIN_MS              5u
#define FOC40_SPD_WIN_US              (FOC40_SPD_WIN_MS * 1000u)
#define FOC40_SPD_FILT_ALPHA          0.25f
/* 显示专用滤波（VOFA 曲线平滑用；α 越小越平滑越滞后，不进 PI 反馈） */
#define FOC40_SPD_DISP_ALPHA          0.05f
/* 编码器每拍增量限幅 = 单拍物理极限 × 1.35 裕度。
 * 物理极限 = 7800rpm 折算到"每个 ISR 拍"的 counts 数。
 *   10 kHz（100µs/拍）：53 counts → 限幅 72
 *   20 kHz（ 50µs/拍）：26.5 counts → 限幅 36   ← 2026-09-23 随 PWM 频率提升同步改
 * 历史教训：原值 32 只支持 4687rpm（10kHz 时），超限导致测速削顶 +
 *   角度积分丢拍（高速下 Park 角落后）。限幅必须始终 ≥ 物理极限，
 *   否则高速时会削顶；但也不能过大，否则编码器毛刺被当真转速放行。
 * ⚠ 若再改 MOTOR_PWM_FREQ_HZ，此值须按 1/f 同步缩放 */
#define FOC40_ENC_DELTA_MAX           36

#define FOC40_STEP_IDLE               0u
#define FOC40_STEP_RUN                1u
#define FOC40_STEP_FAULT_OC           2u

#define FOC40_EVT_OC                  1u

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

void Foc_Speed_InitPids(void);
void Foc_Speed_SetTargetRPM(float target_rpm);
void Foc_Speed_Start(void);
void Foc_Speed_Stop(void);
void Foc_Speed_Step(const stc_i_data_t *pData);
int  Foc_Speed_VofaFill(int32_t *cur);  /* 模式自持 VOFA，见顶部速览卡 */

#ifdef __cplusplus
}
#endif

#endif /* __FOC_40_SPEED_H__ */
