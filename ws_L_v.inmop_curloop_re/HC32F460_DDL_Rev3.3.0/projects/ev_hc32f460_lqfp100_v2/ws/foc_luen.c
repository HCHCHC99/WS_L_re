/**
 *******************************************************************************
 * @file  foc_luen.c
 * @brief 无感龙伯格观测器（Luenberger）实现 —— 纯旁观者，见 foc_luen.h 顶部说明。
 *
 * 每轴离散（Ts = 1/FOC_ISR_HZ，当前 50µs）：
 *   err     = i − î
 *   î_{k+1} = î + Ts·( −(R/L)·î − (1/L)·ê + (1/L)·v + l1·err )
 *   ê_{k+1} = R(ω̂·Ts)·ê + Ts·(−l2·err)     ，R(φ)=[[cosφ,−sinφ],[sinφ,cosφ]]
 *   旋转项用"精确旋转"代替前向欧拉：前向欧拉把 ±jω 映成 1+Ts·jω（模 √(1+(Ts·ω)²)
 *   恒 >1，7800rpm 时 ≈1.08 必然发散，与 Ωo 无关）；精确旋转模恒为 1，再由 −l2·err
 *   修正项把极点压进单位圆内（7800rpm、Ts=50µs 时谱半径 ≈0.985 < 1）。
 *   增益：l1 = 2·Ωo − R/L，l2 = L·Ωo²（恒正，全转速段极点都在左半平面）
 * 符号约定：ê → e = ωψf·(−sinθ, cosθ)，θ̂ = atan2(−ê_α, ê_β)。
 *******************************************************************************
 */

#include "foc_luen.h"
#include "motor_config.h"
#include "foc_core.h"      /* FOC_ISR_HZ（随采样模式 10k/20k 自适应） */
#include "TickTimer.h"     /* tickTimer_GetCount：判定 hold 计时（ms 级时基） */
#include <math.h>

#define LUEN_TS_S        (1.0f / (float)FOC_ISR_HZ)
#define LUEN_PHI_MIN     (1.0e-6f)

/* Watch 可调参数（默认 = 2× 电流环带宽起步） */
volatile float g_luen_model_r_ohm  = 0.1f;
volatile float g_luen_model_l_h    = 42.3e-6f;
volatile float g_luen_model_psi_wb = 0.00084f;
volatile float g_luen_omega_o      = 4712.0f;  /* 2π×750 rad/s = 2×(kp/L≈2364)。
                                                * 连续域全转速极点都在左半平面，无硬上限；
                                                * Ωo 只管收敛快慢/噪声：过大抖、过小滞后。 */
volatile float g_luen_emax_v       = 20.0f;    /* |ê| 发散保护上限（12V 母线反电动势
                                                * 峰值不超过 ~7V，20V 已是物理异常） */
volatile float g_luen_ih_clamp_a   = 40.0f;    /* î 数值保险钳位 */

/* ISR 输出 */
volatile float g_luen_e_alpha_hat_v = 0.0f;
volatile float g_luen_e_beta_hat_v  = 0.0f;
volatile float g_luen_ih_alpha_a    = 0.0f;
volatile float g_luen_ih_beta_a     = 0.0f;
volatile uint8_t g_luen_diverge     = 0u;

/* 诊断输出 */
volatile float g_luen_diag_e_mag_v       = 0.0f;
volatile float g_luen_diag_e_expect_v    = 0.0f;
volatile float g_luen_diag_e_ratio       = 0.0f;
volatile float g_luen_diag_theta_hat_deg = 0.0f;
volatile float g_luen_diag_theta_err_deg = 0.0f;
volatile float g_luen_diag_theta_err_filt_deg = 0.0f;
volatile float g_luen_diag_theta_err_filt_alpha = 0.1f;
volatile float g_luen_diag_e_on_q_v      = 0.0f;
volatile float g_luen_diag_e_on_d_v      = 0.0f;
volatile float g_luen_diag_e_err_v       = 0.0f;
volatile float g_luen_theory_alpha_v     = 0.0f;
volatile float g_luen_theory_beta_v      = 0.0f;

/* 反电动势自动判定（口径与 foc_smo 判定一致；门限可在 Watch 收放） */
volatile float g_luen_jdg_rpm_min        = 900.0f;
volatile float g_luen_jdg_ratio_min      = 0.85f;
volatile float g_luen_jdg_ratio_max      = 1.08f;
volatile float g_luen_jdg_phase_min_deg  = -45.0f;
volatile float g_luen_jdg_phase_max_deg  = 5.0f;
volatile float g_luen_jdg_hold_ms        = 1000.0f;
volatile float g_luen_jdg_filt_alpha     = 0.02f;  /* 判定用慢速 EMA（幅值比/相位共用） */

volatile uint8_t g_luen_emf_ok       = 0u;
volatile uint8_t g_luen_jdg_fail     = 0u;
volatile float g_luen_jdg_ratio_filt = 0.0f;
volatile float g_luen_jdg_phase_filt_deg = 0.0f;

/* 内部状态 */
static float s_ih_alpha;    /* 观测器电流 α (A) */
static float s_ih_beta;     /* 观测器电流 β (A) */
static float s_e_alpha;     /* 反电动势估计 α (V) */
static float s_e_beta;      /* 反电动势估计 β (V) */
static float s_theta_err_f; /* theta_err 滤波状态 (deg) */
static float s_ratio_f;     /* 幅值比慢速 EMA 状态（判定用） */
static float s_phase_f;     /* 相位慢速 EMA 状态（判定用） */
static uint64_t s_jdg_hold_t0; /* 判据开始连续满足的时刻 (ms tick)，0=未在满足中 */

void Foc_Luen_Reset(void)
{
    s_ih_alpha = 0.0f;
    s_ih_beta = 0.0f;
    s_e_alpha = 0.0f;
    s_e_beta = 0.0f;
    s_theta_err_f = 0.0f;
    s_ratio_f = 0.0f;
    s_phase_f = 0.0f;
    s_jdg_hold_t0 = 0u;
    g_luen_e_alpha_hat_v = 0.0f;
    g_luen_e_beta_hat_v = 0.0f;
    g_luen_ih_alpha_a = 0.0f;
    g_luen_ih_beta_a = 0.0f;
    g_luen_diverge = 0u;
    g_luen_diag_e_mag_v = 0.0f;
    g_luen_diag_e_expect_v = 0.0f;
    g_luen_diag_e_ratio = 0.0f;
    g_luen_diag_theta_hat_deg = 0.0f;
    g_luen_diag_theta_err_deg = 0.0f;
    g_luen_diag_theta_err_filt_deg = 0.0f;
    g_luen_diag_e_on_q_v = 0.0f;
    g_luen_diag_e_on_d_v = 0.0f;
    g_luen_diag_e_err_v = 0.0f;
    g_luen_theory_alpha_v = 0.0f;
    g_luen_theory_beta_v = 0.0f;
    g_luen_emf_ok = 0u;
    g_luen_jdg_fail = 0u;
    g_luen_jdg_ratio_filt = 0.0f;
    g_luen_jdg_phase_filt_deg = 0.0f;
}

void Foc_Luen_Step(float v_alpha, float v_beta, float i_alpha, float i_beta,
                   float omega_e_hat_rad_s)
{
    float ts, r, l, inv_l, rl, w, omega_o, l1, l2;
    float err_a, err_b, ea, eb, emax;

    /* 参数（每拍读 Watch，实时生效；防 0/负保护） */
    r = (g_luen_model_r_ohm > 0.0f) ? g_luen_model_r_ohm : 0.1f;
    l = (g_luen_model_l_h > LUEN_PHI_MIN) ? g_luen_model_l_h : 42.3e-6f;
    omega_o = (g_luen_omega_o > 1.0f) ? g_luen_omega_o : 1.0f;
    w = omega_e_hat_rad_s;

    ts = LUEN_TS_S;
    inv_l = 1.0f / l;
    rl = r * inv_l;                          /* R/L */
    l1 = 2.0f * omega_o - rl;                /* 电流误差极点 -> −2·Ωo */
    l2 = l * omega_o * omega_o;              /* 恒正增益：ωe=0 时双极点恰在 −Ωo，
                                              * 全转速段极点都在左半平面。
                                              * 勿改回 L(Ωo²−ω̂²)：ωe>Ωo 时变负 -> 发散 */

    /* 电流/反电动势误差 */
    err_a = i_alpha - s_ih_alpha;
    err_b = i_beta  - s_ih_beta;

    /* 前向欧拉一步（先算电流，再算反电动势，共用本拍误差） */
    s_ih_alpha += ts * ((-rl) * s_ih_alpha - inv_l * s_e_alpha
                        + inv_l * v_alpha + l1 * err_a);
    s_ih_beta  += ts * ((-rl) * s_ih_beta  - inv_l * s_e_beta
                        + inv_l * v_beta  + l1 * err_b);
    /* 反电动势修正项必须取负号：实测电流 i > î 说明实际 e < ê，应把 ê 减小。
     * 取正号会使误差特征方程出现右半平面极点（0~Ωo 区间发散，Ωo=4712 时
     * ωe=0 的根为 +0.414·Ωo），emf_ok 永远攒不满。详见 foc_luen.h 推导。 */
    /* 旋转项用精确旋转 R(ω̂·Ts)：前向欧拉把 ±jω 映成 1+Ts·jω，模 √(1+(Ts·ω)²) 恒 >1
     * （Ts=50µs、7800rpm 时 ≈1.08），高速天然发散，改 Ωo 救不了。精确旋转模恒为 1，
     * 再由 −l2·err 修正项压进单位圆内（7800rpm 谱半径 ≈0.985 < 1）。 */
    {
        float phi    = w * ts;
        float cc     = cosf(phi);
        float ss     = sinf(phi);
        float ea_new = cc * s_e_alpha - ss * s_e_beta + ts * (-l2 * err_a);
        float eb_new = ss * s_e_alpha + cc * s_e_beta + ts * (-l2 * err_b);
        s_e_alpha = ea_new;
        s_e_beta  = eb_new;
    }

    /* î 数值保险钳位 */
    {
        float ic = g_luen_ih_clamp_a > 0.0f ? g_luen_ih_clamp_a : 40.0f;
        if (s_ih_alpha > ic)       s_ih_alpha = ic;
        else if (s_ih_alpha < -ic) s_ih_alpha = -ic;
        if (s_ih_beta > ic)        s_ih_beta = ic;
        else if (s_ih_beta < -ic)  s_ih_beta = -ic;
    }

    /* 发散保护：|ê| 超物理上限 => 复位观测器状态（不用 sqrtf，比较平方） */
    ea = s_e_alpha;
    eb = s_e_beta;
    emax = (g_luen_emax_v > 1.0f) ? g_luen_emax_v : 1.0f;
    if (ea * ea + eb * eb > emax * emax) {
        s_ih_alpha = 0.0f;
        s_ih_beta = 0.0f;
        s_e_alpha = 0.0f;
        s_e_beta = 0.0f;
        if (g_luen_diverge < 255u) {
            g_luen_diverge++;
        }
    }

    g_luen_ih_alpha_a = s_ih_alpha;
    g_luen_ih_beta_a = s_ih_beta;
    g_luen_e_alpha_hat_v = s_e_alpha;
    g_luen_e_beta_hat_v = s_e_beta;
}

void Foc_Luen_Diag(float theta_enc_e_deg, float omega_e_rad_s)
{
    float ea = g_luen_e_alpha_hat_v;
    float eb = g_luen_e_beta_hat_v;
    float mag = sqrtf(ea * ea + eb * eb);
    float expect = fabsf(omega_e_rad_s) * g_luen_model_psi_wb;
    float th, err;

    g_luen_diag_e_mag_v = mag;
    g_luen_diag_e_expect_v = expect;
    g_luen_diag_e_ratio = (expect > 1.0e-4f) ? (mag / expect) : 0.0f;

    /* ê = ωψf·(−sinθ, cosθ)：θ̂ = atan2(−ê_α, ê_β) */
    th = atan2f(-ea, eb) * (180.0f / 3.14159265f);
    th -= (float)((int32_t)(th / 360.0f)) * 360.0f;
    if (th < 0.0f) {
        th += 360.0f;
    }
    g_luen_diag_theta_hat_deg = th;

    err = th - theta_enc_e_deg;
    if (err > 180.0f)       err -= 360.0f;
    else if (err < -180.0f) err += 360.0f;
    g_luen_diag_theta_err_deg = err;

    /* EMA 滤波（看均值；alpha Watch 可调，<=0 或 >1 视为 1=直通） */
    {
        float fa = g_luen_diag_theta_err_filt_alpha;
        if (fa <= 0.0f || fa > 1.0f) fa = 1.0f;
        s_theta_err_f += fa * (err - s_theta_err_f);
        g_luen_diag_theta_err_filt_deg = s_theta_err_f;
    }

    /* 理论反电动势（编码器 θ + 带符号 ω·ψf）：E_theory = ωψf·(−sinθ, cosθ)，
     * 与同步系投影共用 sin/cos */
    th = theta_enc_e_deg * (3.14159265f / 180.0f);
    {
        float sa = sinf(th);
        float ca = cosf(th);
        float eth = omega_e_rad_s * g_luen_model_psi_wb;
        g_luen_theory_alpha_v = -eth * sa;
        g_luen_theory_beta_v = eth * ca;

        /* 同步系投影：ê 投到编码器转子系（q̂=(−sinθ,cosθ), d̂=(cosθ,sinθ)）。
         * 稳态下为直流慢变量；出现电频率纹波 = 波形畸变/参数不对 */
        g_luen_diag_e_on_q_v = -ea * sa + eb * ca;
        g_luen_diag_e_on_d_v = ea * ca + eb * sa;
    }

    /* 总误差矢量范数 |ê − E_theory|（旋转系与静止系范数等价） */
    {
        float eq = g_luen_diag_e_on_q_v - expect;
        g_luen_diag_e_err_v = sqrtf(eq * eq
                                    + g_luen_diag_e_on_d_v * g_luen_diag_e_on_d_v);
    }

    /* ---- 反电动势自动判定（见 foc_luen.h 判定说明）----
     * 前提+C1+C2 全部连续满足 hold_ms → g_luen_emf_ok=1；
     * 任一拍破窗 → hold 计时清零、ok=0（恒定性要求，非粘滞锁存）。 */
    {
        float fa = g_luen_jdg_filt_alpha;
        uint8_t fail = 0u;

        if (fa <= 0.0f || fa > 1.0f) fa = 1.0f;
        s_ratio_f += fa * (g_luen_diag_e_ratio - s_ratio_f);
        g_luen_jdg_ratio_filt = s_ratio_f;
        s_phase_f += fa * (g_luen_diag_theta_err_filt_deg - s_phase_f);
        g_luen_jdg_phase_filt_deg = s_phase_f;

        /* 前提：机械转速（ω_e rad/s → rpm：×9.5493/极对数） */
        if (fabsf(omega_e_rad_s) * 9.5493f / (float)FOC_POLE_PAIRS < g_luen_jdg_rpm_min) {
            fail |= 0x01u;
        }
        /* C1：慢速幅值比（无滑模跌落，收敛良好应 ≈ 1.0） */
        if (s_ratio_f < g_luen_jdg_ratio_min || s_ratio_f > g_luen_jdg_ratio_max) {
            fail |= 0x02u;
        }
        /* C2：慢速相位均值（抓翻转/过大滞后） */
        if (s_phase_f < g_luen_jdg_phase_min_deg || s_phase_f > g_luen_jdg_phase_max_deg) {
            fail |= 0x04u;
        }

        if (fail == 0u) {
            uint64_t now = tickTimer_GetCount();
            if (s_jdg_hold_t0 == 0u) {
                s_jdg_hold_t0 = now;
            }
            if ((float)(now - s_jdg_hold_t0) >= g_luen_jdg_hold_ms) {
                g_luen_emf_ok = 1u;
            }
        } else {
            s_jdg_hold_t0 = 0u;
            g_luen_emf_ok = 0u;
        }
        g_luen_jdg_fail = fail;
    }
}