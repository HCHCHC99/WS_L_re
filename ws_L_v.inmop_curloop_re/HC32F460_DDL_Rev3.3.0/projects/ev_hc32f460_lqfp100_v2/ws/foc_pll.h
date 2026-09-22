/**
 *******************************************************************************
 * @file  foc_pll.h
 * @brief 无感锁相环（PLL）—— Step 2：从反电动势估计提取角度/转速（纯旁观者）。
 *
 * 输入：SMO 滤波后反电动势 e_alpha_hat / e_beta_hat（静止 αβ 系正交信号）
 * 输出：theta_hat（转子电角估计）、omega_hat（电角速度估计）
 *
 * 结构（每拍与 SMO Step 同拍，FOC_ISR_HZ）：
 *   鉴相器：err = (−eα·sinθ̂ + eβ·cosθ̂) / |e| ≈ θ_e − θ̂  (rad)
 *           e 矢量相位 θ_e = θ_rotor + 90°（正转时）；
 *           除以 |e| 归一化 → 环路增益与转速无关，ω_n/ζ 恒定有效
 *   PI 环路：ω̂ += K_i·err·Ts
 *            θ̂ += (ω̂ + K_p·err)·Ts          ← θ̂ 锁定 e 矢量相位
 *   换算：K_p = 2ζω_n，K_i = ω_n²（与提示词公式 ω_n=√(K_i·|E|)、
 *         ζ=K_p/2·√(|E|/K_i) 对应，归一化后 |E| 已约掉）
 *   输出：θ_rotor = θ̂ − 90°·sign(ω̂)；ω̂_rpm = ω̂·60/(2π·p)（带符号）
 *
 * 预期效果（Step 2 验收核心）：PLL 窄带跟踪吃掉死区 6f 马鞍纹波
 * （带宽 200Hz 对 6f=1500Hz 衰减 ~98%）→ theta_err_pll 摆动 << ±9.5°。
 * 注意：PLL 锁定的是 e_hat 实际相位，SMO 的恒定滞后（1500rpm ≈ −17.5°）
 * **保留在均值里**（θ̂_rotor = θ_true − 17.5°），不是被"吃掉"；该滞后是
 * 已知确定性偏差，Step 4 无感闭环前用滞后补偿角（按 ω̂ 实时算）解决。
 *
 * 弱信号保护：|e_hat| < mag_min 时冻结积分（静止/极低速不乱锁），
 * 阈值以下 PLL 角度输出无效属预期。
 * 反转：按 ω̂ 符号输出（θ̂ + 90°）；Step 2 以正转（mode 45 爬坡）验证为主。
 *
 * ISR 约束：Foc_Pll_Step 在 ISR 调用（sinf/cosf/sqrtf 合计 ~2-3µs @168MHz）；
 *          Foc_Pll_Diag 在主循环调用（角度对比/EMA 滤波）。
 *******************************************************************************
 */

#ifndef __FOC_PLL_H__
#define __FOC_PLL_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------
 * Watch 可调参数
 *---------------------------------------------------------------------------*/
extern volatile float g_pll_omega_n;     /* PLL 自然频率 (rad/s)，默认 2π×200 ≈ 1257
                                          * 依据：电流环带宽 kp/(2πL)≈1880Hz，
                                          * PLL ≤ 其 1/5~1/10 → 取 200Hz */
extern volatile float g_pll_zeta;        /* 阻尼比，默认 0.9（0.707~1 稳妥区） */
extern volatile float g_pll_mag_min_v;   /* 弱信号冻结门限 (V)，默认 0.1 */

/*---------------------------------------------------------------------------
 * ISR 实时输出（每拍刷新）
 *---------------------------------------------------------------------------*/
extern volatile float g_pll_theta_rotor_deg; /* 转子电角估计 (deg,[0,360)) */
extern volatile float g_pll_omega_hat_rpm;   /* 机械转速估计 (rpm，带符号) */
extern volatile float g_pll_omega_hat_rad_s; /* 电角速度估计 (rad/s，带符号) */
extern volatile float g_pll_theta_e_deg;     /* e 矢量相位锁定角 (deg,[0,360)) */
extern volatile float g_pll_err_rad;         /* 归一化鉴相残差 ≈ 相位差 (rad) */

/*---------------------------------------------------------------------------
 * 诊断输出
 * ⚠ 角度误差必须在 ISR 同拍比较（Foc_Pll_Compare）：1500rpm 下 θ̂ 每拍
 *   转 9°电角，主循环分两次读 θ̂ 与 θ_enc 会被 ISR 打断出 ±9°/拍 假差
 *   （实测 4 拍循环马鞍序列即此撕裂）；主循环只做 EMA 滤波。
 *---------------------------------------------------------------------------*/
extern volatile float g_pll_diag_theta_err_deg;     /* ISR 同拍：θ̂_rotor − 编码器电角
                                                     * (deg,[-180,180))。锁定后均值 ≈ SMO
                                                     * 已知滞后（1500rpm ≈ −17.5°，PLL 锁
                                                     * e_hat 相位，滞后保留在均值里），
                                                     * 恒定滞后待补偿角解决 */
extern volatile float g_pll_diag_theta_err_filt_deg;/* 上项 EMA 滤波（主循环，看均值） */
extern volatile float g_pll_diag_err_filt_deg;      /* 鉴相残差 EMA 滤波 (deg)，
                                                     * 反映锁定质量（越小越稳） */
extern volatile float g_pll_diag_smo_err_deg;       /* ISR 同拍 SMO atan2 角差 (deg)：
                                                     * 与 e 同快照基准（每 10 拍算一次），
                                                     * 两链均值之差 = PLL 锁点偏置 */
extern volatile float g_pll_diag_filt_alpha;        /* 上两项 EMA 系数，默认 0.1 */

/*---------------------------------------------------------------------------
 * API
 *---------------------------------------------------------------------------*/
void Foc_Pll_Reset(void);
/* ISR 调用。e_alpha/e_beta = SMO 滤波后反电动势 (V)。 */
void Foc_Pll_Step(float e_alpha, float e_beta);
/* ISR 调用（紧跟 Foc_Pll_Step）：与编码器电角同拍比较，写 theta_err。
 * theta_enc_e_deg = 本拍编码器电角 (deg,[0,360))。 */
void Foc_Pll_Compare(float theta_enc_e_deg);
/* 主循环调用：对 ISR 写好的 theta_err 做滤波（不再比较角度）。 */
void Foc_Pll_Diag(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_PLL_H__ */
