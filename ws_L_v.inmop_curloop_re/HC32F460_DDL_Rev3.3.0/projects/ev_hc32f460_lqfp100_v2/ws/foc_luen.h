/**
 *******************************************************************************
 * @file  foc_luen.h
 * @brief 无感龙伯格观测器（Luenberger）—— 纯旁观者模块，不依赖任何模式。
 *
 * 静止 αβ 坐标系"扩展反电动势"模型（表贴式 Ld = Lq = L，无凸极）：
 *   电压方程：  v = R·i + L·di/dt + e
 *               e = ω_e·ψf·(−sinθ, cosθ)   （扩展反电动势，θ = 转子电角）
 *   观测器状态：î_α/î_β（电流估计）、ê_α/ê_β（反电动势估计）
 *   误差注入型龙伯格：
 *     dî/dt = (−R/L)·î − (1/L)·ê + (1/L)·v + l1·(i − î)
 *     dê/dt = ω̂·J·ê − l2·(i − î)            ，J = [[0, −1], [1, 0]]
 *   增益（l1/l2 均为常数，只随 Watch 的 Ωo 变化）：
 *     l1 = 2·Ωo − R/L
 *     l2 = L·Ωo²
 *   误差动态（x_i = i−î，x_e = e−ê，并设 ω̂ = ωe）：
 *     dx_i/dt = −2·Ωo·x_i − (1/L)·x_e
 *     dx_e/dt = +l2·x_i + j·ωe·x_e
 *     特征方程 (s + 2·Ωo)(s − j·ωe) + l2/L = 0
 *     ⇒ ωe = 0 时双极点恰在 −Ωo；ωe 增大时极点右移但恒在左半平面。
 *   稳定域：l2 恒正 ⇒ 全转速段误差极点都在左半平面（ωe = 额定 8168 rad/s 时
 *     仍有约 −1160 rad/s 阻尼）。历史教训（两种写法都会发散，勿改回）：
 *       · 修正项取 +l2·(i−î)  → 0~Ωo 区间出现右半平面极点（emf_ok 永远攒不满）；
 *       · 用 l2 = L(Ωo²−ω̂²)   → ωe>Ωo 时 l2 变负、极点进入右半平面。
 *
 *   离散：î 用前向欧拉；ê 的旋转项用"精确旋转" R(ω̂·Ts)（Ts = 1/FOC_ISR_HZ，20kHz → 50µs）。
 *     · 电流内极点 (1 − Ts·R/L) 天然稳定，与 Ωo 无关；
 *     · 旋转项若用前向欧拉，离散极点 = 1 + Ts·(jω̂)，模 √(1+(Ts·ω̂)²) 恒 >1：
 *       高速时误差极点虚部 ≈ ω̂ (≈8k rad/s)，Ωo=4712、Ts=50µs 时约 5500rpm 以上
 *       就发散（而连续极点看着仍稳，极易误判）。改 Ωo 救不了 —— 它压不动虚部。
 *     · 现改为精确旋转 R(ω̂·Ts)=[[c,−s],[s,c]]（c=cos(ω̂Ts), s=sin(ω̂Ts)）：
 *       旋转项模恒为 1，再由 −l2·err 修正项把极点压进单位圆内；7800rpm 下
 *       谱半径 ≈0.985 < 1，全转速段稳定。
 *     · 若 ISR 算力紧张：sinf/cosf 可换查表/小角度近似（ω̂Ts ≤ ~0.41 rad @7800rpm，
 *       5 阶多项式足够）。
 *     · 经验：Ωo 取电流环带宽 (kp/L ≈ 2364 rad/s) 的 2~5 倍，只管收敛快慢/噪声。
 *
 * 角度/转速提取不在本模块：ê 直接喂共享 foc_pll（与 mode 45 的 SMO 输出同口径），
 *   由 PLL 出 θ̂/ω̂。本模块内部需要的 ω̂（ê 的旋转项与 l2 用）**由调用方每拍回灌**
 *   （mode 60 回灌 PLL 的 g_pll_omega_hat_rad_s，滞后一拍）。这样避免了
 *   "|ê|/ψf 只有幅值、丢失方向"的问题 —— 方向由 PLL 的带符号 ω̂ 提供。
 *
 * 符号约定：ê 收敛到 e = ωψf·(−sinθ, cosθ)，故转子电角 θ̂ = atan2(−ê_α, ê_β)。
 *
 * ISR 约束：Foc_Luen_Step 在 20kHz ISR 调用（无三角函数、无除法陷阱）；
 *          Foc_Luen_Diag 在主循环调用（sqrtf/atan2f 只在这里）。
 *******************************************************************************
 */

#ifndef __FOC_LUEN_H__
#define __FOC_LUEN_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------
 * Watch 可调参数（模型参数与带宽全开放）
 *---------------------------------------------------------------------------*/
extern volatile float g_luen_model_r_ohm;   /* 模型定子电阻 R (ohm)，默认 0.1 */
extern volatile float g_luen_model_l_h;     /* 模型定子电感 L (H)，默认 42.3µH */
extern volatile float g_luen_model_psi_wb;  /* 模型磁链 ψf (Wb)，默认 0.00084 */
extern volatile float g_luen_omega_o;       /* 观测器带宽 Ωo (rad/s)，默认 2π×750≈4712
                                             * （= 2× 电流环带宽 kp/L≈2364 rad/s）。
                                             * 连续极点全转速段都在左半平面，无硬上限；
                                             * Ωo 只管收敛快慢/噪声：过大抖、过小滞后。 */
extern volatile float g_luen_emax_v;        /* 反电动势发散保护上限 |ê| (V)，默认 20 */
extern volatile float g_luen_ih_clamp_a;    /* 观测器电流安全钳位 (A)，默认 40 */

/*---------------------------------------------------------------------------
 * ISR 实时输出（20kHz 刷新）
 *---------------------------------------------------------------------------*/
extern volatile float g_luen_e_alpha_hat_v;  /* ê_α (V) */
extern volatile float g_luen_e_beta_hat_v;   /* ê_β (V) */
extern volatile float g_luen_ih_alpha_a;     /* î_α (A) */
extern volatile float g_luen_ih_beta_a;      /* î_β (A) */
extern volatile uint8_t g_luen_diverge;      /* 发散保护触发次数（|ê| 越限复位累计） */

/*---------------------------------------------------------------------------
 * 诊断输出（主循环刷新，Foc_Luen_Diag 计算）
 *---------------------------------------------------------------------------*/
extern volatile float g_luen_diag_e_mag_v;        /* |ê| (V) */
extern volatile float g_luen_diag_e_expect_v;     /* 理论 |e| = |ω_e|·ψf (V) */
extern volatile float g_luen_diag_e_ratio;        /* 实测/理论，收敛良好应 ≈ 1.0 */
extern volatile float g_luen_diag_theta_hat_deg;  /* 由 atan2(−ê_α, ê_β) 提取的电角 (deg,[0,360)) */
extern volatile float g_luen_diag_theta_err_deg;  /* θ̂ − 编码器电角 (deg,[-180,180))，
                                                   * 匀速下 = 观测器已知极点滞后，非零正常 */
extern volatile float g_luen_diag_theta_err_filt_deg; /* theta_err 的 EMA 滤波（看均值用） */
extern volatile float g_luen_diag_theta_err_filt_alpha; /* 上项 EMA 系数，默认 0.1（Watch 可调） */
/* 同步系投影（把"看波形"变成"看直流"——VOFA 无法显示高速电频率，混叠会让原始波形呈
 * 假象；投影到编码器转子系后为慢变量）：
 *   稳态合格判据 = 两路都是常数（无电频率纹波）：
 *   e_on_q ≈ |ê|·cosδ，e_on_d ≈ −|ê|·sinδ（δ = 观测器极点滞后） */
extern volatile float g_luen_diag_e_on_q_v;       /* ê 在编码器 q 轴投影 (V) */
extern volatile float g_luen_diag_e_on_d_v;       /* ê 在编码器 d 轴投影 (V) */
extern volatile float g_luen_diag_e_err_v;        /* 总误差矢量范数 (V) =
                                                   * sqrt((e_on_q−ωψf)² + e_on_d²) */
/* 理论反电动势（编码器 θ + 带符号 ω·ψf 生成，静止 αβ 系，主循环刷新）：
 *   E_theory = ωψf·(−sinθ, cosθ)。与观测器输出同帧采样对比用。 */
extern volatile float g_luen_theory_alpha_v;      /* 理论 e_alpha (V) */
extern volatile float g_luen_theory_beta_v;       /* 理论 e_beta (V) */

/*---------------------------------------------------------------------------
 * 反电动势自动判定（Foc_Luen_Diag 末尾执行，每次 mode60 Start 经
 * Foc_Luen_Reset 重新判定）——"直流判据恒定"的代码化（口径与 foc_smo 判定一致）：
 *   前提：机械转速 >= rpm_min（低速反电动势小、不可观测，低于=未判定）
 *   C1：幅值比慢速均值 ∈ [ratio_min, ratio_max]
 *   C2：相位慢速均值 ∈ [phase_min, phase_max]（抓相位翻转/滞后过大）
 *   三项连续满足 hold_ms 才置 OK，任一拍破窗立即清零重来。
 *   验收匀速稳态，不验收动态。
 *---------------------------------------------------------------------------*/
extern volatile float g_luen_jdg_rpm_min;       /* 判定生效转速下限 (rpm)，默认 900 */
extern volatile float g_luen_jdg_ratio_min;     /* C1 幅值比下限，默认 0.85 */
extern volatile float g_luen_jdg_ratio_max;     /* C1 幅值比上限，默认 1.08 */
extern volatile float g_luen_jdg_phase_min_deg; /* C2 相位下限 (deg)，默认 −45 */
extern volatile float g_luen_jdg_phase_max_deg; /* C2 相位上限 (deg)，默认 +5 */
extern volatile float g_luen_jdg_hold_ms;       /* 持续满足时长 (ms)，默认 1000 */
extern volatile float g_luen_jdg_filt_alpha;    /* 判定级慢速 EMA 系数（幅值比/相位共用），默认 0.02 */

extern volatile uint8_t g_luen_emf_ok;          /* 判定结果：1=反电动势 OK */
extern volatile uint8_t g_luen_jdg_fail;        /* 失败原因位掩码（不 OK 时看这里）：
                                                 *   bit0 = 转速未达生效下限（未判定）
                                                 *   bit1 = C1 幅值比越界
                                                 *   bit2 = C2 相位越界 */
extern volatile float g_luen_jdg_ratio_filt;    /* 慢速幅值比（判定与打印用） */
extern volatile float g_luen_jdg_phase_filt_deg;/* 慢速相位均值 (deg)，判定用 */

/*---------------------------------------------------------------------------
 * API
 *---------------------------------------------------------------------------*/
void Foc_Luen_Reset(void);
/* 20kHz ISR 调用。
 *   v_alpha/v_beta = 上一拍指令电压 (V)（调用方缓冲/平均一拍）；
 *   i_alpha/i_beta = 与控制路径同源 Clarke 电流 (A)；
 *   omega_e_hat_rad_s = 电角速度估计 (rad/s，带符号)，由 PLL 回灌（滞后一拍），
 *                       启动静止阶段为 0（ê 由电流误差注入自行起振，PLL 锁定后闭环）。 */
void Foc_Luen_Step(float v_alpha, float v_beta, float i_alpha, float i_beta,
                   float omega_e_hat_rad_s);
/* 主循环调用。theta_enc_e_deg = 编码器电角 (deg,[0,360))；
 * omega_e_rad_s = 编码器电角速度 (rad/s，带符号)。 */
void Foc_Luen_Diag(float theta_enc_e_deg, float omega_e_rad_s);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_LUEN_H__ */