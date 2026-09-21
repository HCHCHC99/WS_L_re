/**
 *******************************************************************************
 * @file  foc_smo.h
 * @brief 无感滑模观测器（SMO）—— 纯旁观者模块，不依赖任何模式。
 *
 * 静止 αβ 坐标系电流观测器：
 *   观测器：  L·dî/dt = v − R·î − z
 *   滑模项：  z = k·sat((î − i)/φ)     （饱和函数，边界层 φ）
 *   反电动势：滑模面上 z ≈ e（等效控制）→ e_hat = LPF(z)
 *
 *   定义 err = î − i，边界层内稳态：err_ss = e/(R + k/φ)，
 *   z_ss = e·(k/φ)/(R + k/φ) = e·(1 − R/(k/φ) + O(ωL·φ/k))
 *   → e_hat 幅值有确定性跌落 R/(k/φ)（默认 ≈6.9%），α/β 同跌落，
 *     该跌落实测值反过来可以校验模型 R 是否准确。
 *
 * 离散格式：半隐式（层内对 î_{k+1} 解析求解），Ts = 1/FOC_ISR_HZ
 *   （当前 valley 采样模式 = 10kHz，Ts=100µs）→ 无 k/φ 稳定上限；
 *   若改回显式 Euler，需满足 k/φ < 2L/Ts − R（10kHz 时仅 ≈0.75）。
 *   obs_gain 越大幅值跌落越小：跌落 ≈ R/(k/φ)，默认 1.45 → ≈6.9%。
 *
 * Step 1 用法（mode 45 旁观验证）：
 *   电压输入 = 前后两拍指令平均（中心对齐 PWM + 谷点采样下，采样区间上
 *   实际施加的电压各占一半；单喂上一拍指令会引入 ~1 拍传输延迟）；
 *   电流输入 = 与控制路径同源（sign × 零偏校正 → Foc_Clarke）。
 *   k 固定值整定：默认 4.6V = 2× e_peak@2600rpm
 *   （e_peak = ω_e·ψf = 2π×10×2600/60×0.00084 ≈ 2.29V）。
 *   固定 k 的可用验证窗 ≈ 1300~3400rpm（k/e_peak ≈ 1.5~3.5），
 *   低速档 e 信号小于切换纹波属预期，不是 bug。
 *
 * ISR 约束：Foc_Smo_Step 在 20kHz ISR 调用（无三角函数、无除法陷阱）；
 *          Foc_Smo_Diag 在主循环调用（sqrtf/atan2f 只在这里）。
 *******************************************************************************
 */

#ifndef __FOC_SMO_H__
#define __FOC_SMO_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------
 * Watch 可调参数（模型参数与增益全开放，波形不对按排查表逐个试）
 *---------------------------------------------------------------------------*/
extern volatile float g_smo_model_r_ohm;    /* SMO 模型电阻 (Ω)，默认 0.1 */
extern volatile float g_smo_model_l_h;      /* SMO 模型电感 (H)，默认 42.3µH */
extern volatile float g_smo_model_psi_wb;   /* SMO 模型磁链 (Wb)，默认 0.00084 */
extern volatile float g_smo_k_v;            /* 滑模增益 k (V)，默认 4.6 = 2×e_peak@2600rpm */
extern volatile float g_smo_obs_gain;       /* k/φ，默认 1.45，硬上限 ≈1.592 */
extern volatile float g_smo_lpf_alpha;      /* e_hat EMA 系数，默认 0.24（fc≈1kHz） */

/*---------------------------------------------------------------------------
 * ISR 实时输出（20kHz 刷新）
 *---------------------------------------------------------------------------*/
extern volatile float g_smo_e_alpha_hat_v;  /* e_hat α 分量 (V)，= LPF(z_α) */
extern volatile float g_smo_e_beta_hat_v;   /* e_hat β 分量 (V)，= LPF(z_β) */
extern volatile float g_smo_z_alpha_v;      /* 原始滑模项 z_α (V) */
extern volatile float g_smo_z_beta_v;       /* 原始滑模项 z_β (V) */
extern volatile float g_smo_ih_alpha_a;     /* 观测器电流 î_α (A) */
extern volatile float g_smo_ih_beta_a;      /* 观测器电流 î_β (A) */

/*---------------------------------------------------------------------------
 * 诊断输出（主循环刷新，Foc_Smo_Diag 计算）
 *---------------------------------------------------------------------------*/
extern volatile float g_smo_diag_e_mag_v;      /* |e_hat| (V) */
extern volatile float g_smo_diag_e_expect_v;   /* 理论 |e| = |ω_e|·ψf (V) */
extern volatile float g_smo_diag_e_ratio;      /* 实测/理论，理论 ≈ 1 − R/(k/φ) − LPF 损失 */
extern volatile float g_smo_diag_theta_hat_deg;/* 由 atan2(e_β,e_α) 提取的电角 (deg,[0,360)) */
extern volatile float g_smo_diag_theta_err_deg;/* theta_hat − 编码器电角 (deg,[-180,180))，
                                                 * 匀速下 = 已知滞后（LPF+观测器极点），非零正常 */
extern volatile float g_smo_diag_theta_err_filt_deg;/* theta_err 的 EMA 滤波值 (deg)——看均值用；
                                                 * 抖动幅度本身也是诊断量（突变 = 噪声/参数恶化） */
extern volatile float g_smo_diag_theta_err_filt_alpha;/* 上项 EMA 系数，默认 0.1（Watch 可调） */
/* 同步系投影（把"看波形"变成"看直流"——VOFA ~140Hz 无法显示 433Hz 电频率，
 * 混叠会让原始波形呈 max/min 交替假象；投影到编码器转子系后为慢变量）：
 *   稳态合格判据 = 两路都是常数（无电频率纹波）：
 *   e_on_q ≈ |e_hat|·cosδ，e_on_d ≈ −|e_hat|·sinδ
 *   （δ = LPF+观测器极点已知滞后，2600rpm 约 27°：q≈0.89|e|, d≈−0.45|e|） */
extern volatile float g_smo_diag_e_on_q_v;     /* e_hat 在编码器 q 轴投影 (V) */
extern volatile float g_smo_diag_e_on_d_v;     /* e_hat 在编码器 d 轴投影 (V) */
extern volatile float g_smo_diag_e_err_v;      /* 总误差矢量范数 (V) =
                                                 * sqrt((e_on_q−ωψf)² + e_on_d²)，
                                                 * 即 |E_hat − E_theory|（旋转系等价计算）。
                                                 * 健康值被已知滞后 δ 主导：≈ ωψf·δ(rad)
                                                 * （2600rpm/δ=27° ≈ 1.08V）；若显著大于
                                                 * 该值 → 幅值/波形/参数有问题 */
/* 理论反电动势（编码器 θ + 带符号 ω·ψf 生成，静止 αβ 系，主循环刷新）：
 * E_theory = ωψf·(−sinθ, cosθ)。与 SMO 输出同帧采样对比用。 */
extern volatile float g_smo_theory_alpha_v;    /* 理论 e_alpha (V) */
extern volatile float g_smo_theory_beta_v;     /* 理论 e_beta (V) */

/*---------------------------------------------------------------------------
 * API
 *---------------------------------------------------------------------------*/
void Foc_Smo_Reset(void);
/* 20kHz ISR 调用。v_alpha/v_beta = 上一拍指令电压 (V)（调用方缓冲一拍）；
 * i_alpha/i_beta = 与控制路径同源 Clarke 电流 (A)。 */
void Foc_Smo_Step(float v_alpha, float v_beta, float i_alpha, float i_beta);
/* 主循环调用。theta_enc_e_deg = 编码器电角 (deg,[0,360))；
 * omega_e_rad_s = 编码器电角速度 (rad/s，带符号)。 */
void Foc_Smo_Diag(float theta_enc_e_deg, float omega_e_rad_s);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_SMO_H__ */
