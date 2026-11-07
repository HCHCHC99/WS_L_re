/**
 *******************************************************************************
 * @file  foc_smo.h
 * @brief 无感滑模观测器（SMO）—— 纯旁观者模块，不依赖任何模式。
 *
 * 静止 αβ 坐标系，每轴：
 *   饱和函数：  sat(s) = s/Δ (|s| ≤ Δ)，否则 ±1
 *   电流观测：  Ls·dî/dt = v − Rs·î − ê − K_SM·sat(î − i)
 *   反电动势：  dê/dt = L_SM·sat(î − i)         ← ê 是独立积分状态
 *
 * 与旧实现（等效控制 + 后置 LPF）的区别：ê 不再是对 z 做低通，而是积分状态，
 * 稳态 ê → e，没有 R/(K_SM/Δ) 的确定性幅值跌落；滑模项只负责压住模型误差
 * 与死区，稳态 z → 0。反电动势下游不再做任何额外滤波（需要时在 PLL 消费端做）。
 *
 * 离散格式（Ts = 1/FOC_ISR_HZ，当前 20kHz / 50µs）：
 *   预测步：前向欧拉（含 ê 前馈）
 *   滑模层内：对 î_{k+1} 半隐式解析解，g = Ts·(K_SM/Δ)/Ls
 *            î_{k+1} = (î_p + g·i)/(1+g)，z = (K_SM/Δ)·(î_p − i)/(1+g)
 *            → 无 K_SM/Δ 稳定上限；若改回显式欧拉需 K_SM/Δ < 2Ls/Ts − Rs
 *   滑模层外：z = ±K_SM 饱和注入，î_{k+1} = î_p − Ts·z
 *            （该支路仍是显式欧拉，要求 Ts·(Rs + K_SM/Δ)/Ls < 2，
 *              当前默认 1.83，Δ 不得小于约 2.9A）
 *
 * 参数整定：
 *   K_SM / Δ 决定等效滑模增益 G = K_SM/Δ（默认 4.6V / 3.17A ≈ 1.45V/A）；
 *   L_SM 决定 ê 环带宽：默认 10000V/s 对应等效极点约 1.1e4 rad/s（≈1.8kHz），
 *   相位滞后远小于旧的 1kHz EMA；加大 L_SM → 滞后更小但纹波/振铃增大
 *   （默认值约 3e4 V/s 开始欠阻尼），减小 L_SM → 更平滑但滞后变大。
 *
 * Step 1 用法（mode 45/47 旁观验证）：
 *   电压输入 = 前后两拍指令平均（中心对齐 PWM + 谷点采样下，采样区间上
 *   实际施加的电压各占一半；单喂上一拍指令会引入 ~1 拍传输延迟）；
 *   电流输入 = 与控制路径同源（sign × 零偏校正 → Foc_Clarke）。
 *   K_SM 固定值整定：默认 4.6V = 2× e_peak@2600rpm
 *   （e_peak = ω_e·ψf = 2π×10×2600/60×0.00084 ≈ 2.29V）。
 *   低速档 e 信号小于切换纹波属预期，不是 bug（验证窗 ≈1300~3400rpm）。
 *
 * ISR 约束：Foc_Smo_Step 在 20kHz ISR 调用（无三角函数）；
 *          Foc_Smo_Diag 在主循环调用（sqrtf/atan2f/logf 只在这里）。
 *******************************************************************************
 */

#ifndef __FOC_SMO_H__
#define __FOC_SMO_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------
 * SMO 专用参数（唯一事实源；.c 里的 Watch 变量以此为默认值）
 * 当前电机：Rs=0.1Ω，Ls=42.3µH，ψf=0.00084Wb，p=10
 *---------------------------------------------------------------------------*/
#define SMO_RS_OHM          0.1f        /* 定子相电阻 Rs (Ω) */
#define SMO_LS_H            42.3e-6f    /* 定子同步电感 Ls (H) */
#define SMO_PSIF_WB         0.00084f    /* 永磁磁链 ψf (Wb)，仅诊断用 */
#define SMO_K_SM_V          4.6f        /* 电流滑模增益 K_SM (V) = 2×e_peak@2600rpm */
#define SMO_L_SM_V_PER_S    10000.0f    /* 反电动势滑模增益 L_SM (V/s)，ê 环 ≈1.8kHz */
#define SMO_DELTA_A         3.17f       /* 饱和函数边界层 Δ (A)；等效增益 K_SM/Δ ≈ 1.45V/A */

#define SMO_I_EST_CLAMP_A   40.0f       /* 观测电流安全钳位 (A)，数值保险 */
#define SMO_E_EST_CLAMP_V   15.0f       /* 反电动势积分状态钳位 (V)，覆盖 7800rpm 的 6.9V */
#define SMO_DELTA_MIN_A     0.05f       /* 边界层下限，防 K_SM/Δ 除零 */

/*---------------------------------------------------------------------------
 * Watch 可调参数（默认值见上面的宏）
 *---------------------------------------------------------------------------*/
extern volatile float g_smo_rs_ohm;         /* 模型电阻 Rs (Ω) */
extern volatile float g_smo_ls_h;           /* 模型电感 Ls (H) */
extern volatile float g_smo_psif_wb;        /* 模型磁链 ψf (Wb)，仅诊断分母用 */
extern volatile float g_smo_k_sm_v;         /* 电流滑模增益 K_SM (V) */
extern volatile float g_smo_l_sm_v_per_s;   /* 反电动势滑模增益 L_SM (V/s) */
extern volatile float g_smo_delta_a;        /* 饱和函数边界层 Δ (A) */

/* ê 环一阶等效极点 (rad/s)：由 foc_smo.c 按当前 Rs/Ls/K_SM/Δ 的离散线性化解出
 * （L_SM 只通过阻尼影响极点位置，不改变 |λ| 主项），主循环刷新；
 * 供 foc_pll.c 的滞后补偿 d1 = atan(|ω̂|/ωc) 使用 */
extern volatile float g_smo_emf_wc_rad_s;

/*---------------------------------------------------------------------------
 * ISR 实时输出（20kHz 刷新）
 *---------------------------------------------------------------------------*/
extern volatile float g_smo_e_alpha_hat_v;  /* ê_α (V)：反电动势积分状态 */
extern volatile float g_smo_e_beta_hat_v;   /* ê_β (V) */
extern volatile float g_smo_z_alpha_v;      /* 滑模注入 z_α (V)：稳态趋零 */
extern volatile float g_smo_z_beta_v;       /* 滑模注入 z_β (V) */
extern volatile float g_smo_ih_alpha_a;     /* 观测器电流 î_α (A) */
extern volatile float g_smo_ih_beta_a;      /* 观测器电流 î_β (A) */

/*---------------------------------------------------------------------------
 * 诊断输出（主循环刷新，Foc_Smo_Diag 计算）
 *---------------------------------------------------------------------------*/
extern volatile float g_smo_diag_e_mag_v;      /* |ê| (V) */
extern volatile float g_smo_diag_e_expect_v;   /* 理论 |e| = |ω_e|·ψf (V) */
extern volatile float g_smo_diag_e_ratio;      /* |ê|/理论，ê 为积分状态，稳态 ≈1 */
extern volatile float g_smo_diag_theta_hat_deg;/* 由 atan2(ê_β,ê_α) 提取的电角 (deg,[0,360)) */
extern volatile float g_smo_diag_theta_err_deg;/* theta_hat − 编码器电角 (deg,[-180,180))，
                                                 * 匀速下 = 已知滞后（ê 环极点），非零正常 */
extern volatile float g_smo_diag_theta_err_filt_deg;/* theta_err 的 EMA 滤波值 (deg)——看均值用；
                                                 * 抖动幅度本身也是诊断量（突变 = 噪声/参数恶化） */
extern volatile float g_smo_diag_theta_err_filt_alpha;/* 上项 EMA 系数，默认 0.1（Watch 可调） */
/* 同步系投影（把"看波形"变成"看直流"——VOFA ~140Hz 无法显示 433Hz 电频率，
 * 混叠会让原始波形呈 max/min 交替假象；投影到编码器转子系后为慢变量）：
 *   稳态合格判据 = 两路都是常数（无电频率纹波）：
 *   e_on_q ≈ |ê|·cosδ，e_on_d ≈ −|ê|·sinδ（δ = ê 环已知滞后） */
extern volatile float g_smo_diag_e_on_q_v;     /* ê 在编码器 q 轴投影 (V) */
extern volatile float g_smo_diag_e_on_d_v;     /* ê 在编码器 d 轴投影 (V) */
extern volatile float g_smo_diag_e_err_v;      /* 总误差矢量范数 (V) =
                                                 * sqrt((e_on_q−ωψf)² + e_on_d²)，
                                                 * 即 |ê − E_theory|（旋转系等价计算）。
                                                 * 健康值被已知滞后 δ 主导：≈ ωψf·δ(rad) */
/* 理论反电动势（编码器 θ + 带符号 ω·ψf 生成，静止 αβ 系，主循环刷新）：
 * E_theory = ωψf·(−sinθ, cosθ)。与 SMO 输出同帧采样对比用。 */
extern volatile float g_smo_theory_alpha_v;    /* 理论 e_alpha (V) */
extern volatile float g_smo_theory_beta_v;     /* 理论 e_beta (V) */

/*---------------------------------------------------------------------------
 * 反电动势自动判定（Foc_Smo_Diag 末尾执行，每次 Start 经 Foc_Smo_Reset 重新判定）
 *   前提：机械转速 >= rpm_min（固定 K_SM 的信噪比窗口下限，低于=未判定）
 *   C1：幅值比慢速均值 ∈ [ratio_min, ratio_max]
 *   C2：相位慢速均值 ∈ [phase_min, phase_max]（恒定滞后 PLL 会吃掉，只抓
 *       相位翻转/滞后过大，不卡"必须≈−17°"）
 *   慢速均值：两级 EMA（theta_err_filt α=0.1 → 判定级 α=jdg_filt_alpha），
 *   马鞍纹波/混叠拍频对判定免疫；恒定：三项连续满足 hold_ms 才置 OK，
 *   任一拍破窗立即清零重来。验收匀速稳态，不验收动态。
 *---------------------------------------------------------------------------*/
extern volatile float g_smo_jdg_rpm_min;       /* 判定生效转速下限 (rpm)，默认 900 */
extern volatile float g_smo_jdg_ratio_min;     /* C1 幅值比下限，默认 0.85 */
extern volatile float g_smo_jdg_ratio_max;     /* C1 幅值比上限，默认 1.08
                                                * （ê 为积分状态，中心应≈1；实测确认后收放） */
extern volatile float g_smo_jdg_phase_min_deg; /* C2 相位下限 (deg)，默认 −45 */
extern volatile float g_smo_jdg_phase_max_deg; /* C2 相位上限 (deg)，默认 +5 */
extern volatile float g_smo_jdg_hold_ms;       /* 持续满足时长 (ms)，默认 1000 */
extern volatile float g_smo_jdg_filt_alpha;    /* 判定级慢速 EMA 系数（幅值比/相位
                                                * 共用），默认 0.02 */

extern volatile uint8_t g_smo_emf_ok;          /* 判定结果：1=反电动势 OK */
extern volatile uint8_t g_smo_jdg_fail;        /* 失败原因位掩码（不 OK 时看这里）：
                                                *   bit0 = 转速未达生效下限（未判定）
                                                *   bit1 = C1 幅值比越界
                                                *   bit2 = C2 相位越界 */
extern volatile float g_smo_jdg_ratio_filt;    /* 慢速幅值比（判定与打印用） */
extern volatile float g_smo_jdg_phase_filt_deg;/* 慢速相位均值 (deg)，判定用 */

/*---------------------------------------------------------------------------
 * API
 *---------------------------------------------------------------------------*/
void Foc_Smo_Reset(void);
/* 20kHz ISR 调用。v_alpha/v_beta = 前后两拍指令电压平均 (V)；
 * i_alpha/i_beta = 与控制路径同源 Clarke 电流 (A)。 */
void Foc_Smo_Step(float v_alpha, float v_beta, float i_alpha, float i_beta);
/* 主循环调用。theta_enc_e_deg = 编码器电角 (deg,[0,360))；
 * omega_e_rad_s = 编码器电角速度 (rad/s，带符号)。 */
void Foc_Smo_Diag(float theta_enc_e_deg, float omega_e_rad_s);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_SMO_H__ */
