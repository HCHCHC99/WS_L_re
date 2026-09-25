/**
 *******************************************************************************
 * @file  foc_smo.c
 * @brief 无感滑模观测器（SMO）实现 —— 纯旁观者，见 foc_smo.h 顶部说明。
 *
 * 离散格式（每轴，Ts = 1/FOC_ISR_HZ；当前 VALLEY 采样 ⇒ Ts = 1/MOTOR_PWM_FREQ_HZ
 *   = 50µs @20kHz，2026-09-23 由 10kHz/100µs 提升）：
 *   预测：  î_p = î_k + (Ts/L)·(v_k − R·î_k)                （无滑模项）
 *   层内：  î_{k+1} = (î_p + g·i_k)/(1+g)，z = (k/φ)(î_{k+1} − i_k)，
 *           g = Ts·k/(φ·L) —— 对 î_{k+1} 半隐式解析解，无条件稳定
 *           （显式 Euler 则要求 k/φ < 2L/Ts − R，Ts=50µs 时阈值 ≈1.5）
 *   层外：  z = ±k 饱和注入，î_{k+1} = î_p − Ts·z
 *   e_hat = EMA(z)
 *   符号约定：err = î − i，滑模面上 z ≈ +e（若 e 矢量滞后转子 d 轴 90°
 *   而非超前，检查 Clarke/符号链路或整体翻转）。
 *******************************************************************************
 */

#include "foc_smo.h"
#include "motor_config.h"
#include "foc_core.h"      /* FOC_ISR_HZ（I.h 派生宏，随采样模式 10k/20k 自适应） */
#include "TickTimer.h"     /* tickTimer_GetCount：判定 hold 计时（ms 级时基） */
#include <math.h>

#define SMO_TS_S        (1.0f / (float)FOC_ISR_HZ)
#define SMO_IH_CLAMP_A  40.0f   /* 观测器电流安全钳位（数值保险） */
#define SMO_PHI_MIN_A   0.02f   /* 边界层下限（k≈0 时防除零） */

/* Watch 可调参数（默认 = 2600rpm 固定 k 整定） */
volatile float g_smo_model_r_ohm  = 0.1f;
volatile float g_smo_model_l_h    = 42.3e-6f;
volatile float g_smo_model_psi_wb = 0.00084f;
volatile float g_smo_k_v          = 4.6f;   /* 2× e_peak@2600rpm (2.29V) */
volatile float g_smo_obs_gain     = 1.45f;  /* k/φ：半隐式无稳定上限；越大跌落越小
                                              （幅值跌落 ≈ R/(k/φ) ≈ 6.9%） */
volatile float g_smo_lpf_alpha    = 0.21f;  /* fc ≈ 1kHz：α = 1 − e^(−2π·fc/fs)
                                              *   10 kHz ISR → 0.39（旧值）
                                              *   20 kHz ISR → 0.21  ← 2026-09-23 随
                                              *     MOTOR_PWM_FREQ_HZ 提升同步改
                                              *   （近似式 2π·fc/fs 给 0.24，精确式给 0.21）
                                              * ⚠ 改 PWM 频率后此值须按 1/fs 缩放 */

/* ISR 输出 */
volatile float g_smo_e_alpha_hat_v = 0.0f;
volatile float g_smo_e_beta_hat_v  = 0.0f;
volatile float g_smo_z_alpha_v     = 0.0f;
volatile float g_smo_z_beta_v      = 0.0f;
volatile float g_smo_ih_alpha_a    = 0.0f;
volatile float g_smo_ih_beta_a     = 0.0f;

/* 诊断输出 */
volatile float g_smo_diag_e_mag_v       = 0.0f;
volatile float g_smo_diag_e_expect_v    = 0.0f;
volatile float g_smo_diag_e_ratio       = 0.0f;
volatile float g_smo_diag_theta_hat_deg = 0.0f;
volatile float g_smo_diag_theta_err_deg = 0.0f;
volatile float g_smo_diag_theta_err_filt_deg = 0.0f;
volatile float g_smo_diag_theta_err_filt_alpha = 0.1f;
volatile float g_smo_diag_e_on_q_v      = 0.0f;
volatile float g_smo_diag_e_on_d_v      = 0.0f;
volatile float g_smo_diag_e_err_v       = 0.0f;
volatile float g_smo_theory_alpha_v     = 0.0f;
volatile float g_smo_theory_beta_v      = 0.0f;

/* 反电动势自动判定（窗口默认值依据 1500rpm 实测：ratio 中心≈0.98、phase 均值≈−16.3°。
 * ratio 实测比纯基波理论 0.90 高的原因：|e_hat|=sqrt(ea²+eb²) 把死区 6f 谐波与
 * 切换纹波能量也计入幅值，而分母 ωψf 是纯基波 → 分子虚高 ≈7%）。
 * 判定哲学：恒定滞后（−9°~−30°）PLL 都会吃掉，不妨碍可用；C2 只抓相位翻转
 * （大正角）与滞后过大（模型/滤波严重不对），不卡"必须≈−17°"。
 * rpm_min=900：Step 4 无感首切定在 1000rpm，判定需在该工况生效
 *（1000rpm 下 k/e_peak≈5.2 信噪比更差，若 C1/C2 误报按实测收放窗口） */
volatile float g_smo_jdg_rpm_min        = 900.0f;
volatile float g_smo_jdg_ratio_min      = 0.85f;
volatile float g_smo_jdg_ratio_max      = 1.08f;
volatile float g_smo_jdg_phase_min_deg  = -45.0f;
volatile float g_smo_jdg_phase_max_deg  = 5.0f;
volatile float g_smo_jdg_hold_ms        = 1000.0f;
volatile float g_smo_jdg_filt_alpha     = 0.02f;  /* 判定用慢速 EMA（幅值比/相位共用）：
                                                   * 一级滤波(theta_err_filt,0.1)后仍有
                                                   * ±9.5°马鞍纹波+混叠拍频驻留，点值进
                                                   * 窗口会来回破窗；二级慢速取均值免疫 */

volatile uint8_t g_smo_emf_ok       = 0u;
volatile uint8_t g_smo_jdg_fail     = 0u;
volatile float g_smo_jdg_ratio_filt = 0.0f;
volatile float g_smo_jdg_phase_filt_deg = 0.0f;

/* 内部状态 */
static float s_ih_alpha;    /* 观测器电流 α (A) */
static float s_ih_beta;     /* 观测器电流 β (A) */
static float s_e_f_alpha;   /* e_hat α 滤波状态 (V) */
static float s_e_f_beta;    /* e_hat β 滤波状态 (V) */
static float s_theta_err_f; /* theta_err 滤波状态 (deg)；稳态远离 ±180°，普通 EMA 即可 */
static float s_ratio_f;     /* 幅值比慢速 EMA 状态（判定用） */
static float s_phase_f;     /* 相位慢速 EMA 状态（判定用，级联在 theta_err_filt 之后） */
static uint64_t s_jdg_hold_t0; /* 判据开始连续满足的时刻 (ms tick)，0=未在满足中 */

void Foc_Smo_Reset(void)
{
    s_ih_alpha = 0.0f;
    s_ih_beta = 0.0f;
    s_e_f_alpha = 0.0f;
    s_e_f_beta = 0.0f;
    s_theta_err_f = 0.0f;
    g_smo_e_alpha_hat_v = 0.0f;
    g_smo_e_beta_hat_v = 0.0f;
    g_smo_z_alpha_v = 0.0f;
    g_smo_z_beta_v = 0.0f;
    g_smo_ih_alpha_a = 0.0f;
    g_smo_ih_beta_a = 0.0f;
    g_smo_diag_e_mag_v = 0.0f;
    g_smo_diag_e_expect_v = 0.0f;
    g_smo_diag_e_ratio = 0.0f;
    g_smo_diag_theta_hat_deg = 0.0f;
    g_smo_diag_theta_err_deg = 0.0f;
    g_smo_diag_theta_err_filt_deg = 0.0f;
    g_smo_diag_e_on_q_v = 0.0f;
    g_smo_diag_e_on_d_v = 0.0f;
    g_smo_diag_e_err_v = 0.0f;
    g_smo_theory_alpha_v = 0.0f;
    g_smo_theory_beta_v = 0.0f;
    s_ratio_f = 0.0f;
    s_phase_f = 0.0f;
    s_jdg_hold_t0 = 0u;
    g_smo_emf_ok = 0u;
    g_smo_jdg_fail = 0u;
    g_smo_jdg_ratio_filt = 0.0f;
    g_smo_jdg_phase_filt_deg = 0.0f;
}

void Foc_Smo_Step(float v_alpha, float v_beta, float i_alpha, float i_beta)
{
    float k, phi, ts_l, r, g, ihp_a, ihp_b, za_p, zb_p, za, zb;

    /* 参数（每拍读 Watch，实时生效；防 0/负保护） */
    r = (g_smo_model_r_ohm > 0.0f) ? g_smo_model_r_ohm : 0.1f;
    if (g_smo_model_l_h > 1.0e-6f) {
        ts_l = SMO_TS_S / g_smo_model_l_h;
    } else {
        ts_l = SMO_TS_S / 42.3e-6f;
    }
    k = (g_smo_k_v > 0.0f) ? g_smo_k_v : 0.0f;
    phi = (g_smo_obs_gain > 0.01f) ? (k / g_smo_obs_gain) : k;
    if (phi < SMO_PHI_MIN_A) {
        phi = SMO_PHI_MIN_A;
    }
    g = ts_l * k / phi;   /* 层内无量纲增益（半隐式格式下无稳定上限） */

    /* 预测：不含滑模项的观测器电流（极点 1−Ts·R/L≈0.99，天然稳定） */
    ihp_a = s_ih_alpha + ts_l * (v_alpha - r * s_ih_alpha);
    ihp_b = s_ih_beta + ts_l * (v_beta - r * s_ih_beta);

    /* 半隐式滑模项（α 轴）：
     * 层内 |z|≤k：î_{k+1} = (î_p + g·i)/(1+g)，z = (k/φ)(î_{k+1} − i)
     * 层外 |z|>k：z = ±k 饱和注入，î_{k+1} = î_p − Ts·z */
    za_p = (k / phi) * (ihp_a - i_alpha);
    if (za_p <= k * (1.0f + g) && za_p >= -k * (1.0f + g)) {
        s_ih_alpha = (ihp_a + g * i_alpha) / (1.0f + g);
        za = za_p / (1.0f + g);
    } else {
        za = (za_p > 0.0f) ? k : -k;
        s_ih_alpha = ihp_a - ts_l * za;
    }

    /* β 轴同 */
    zb_p = (k / phi) * (ihp_b - i_beta);
    if (zb_p <= k * (1.0f + g) && zb_p >= -k * (1.0f + g)) {
        s_ih_beta = (ihp_b + g * i_beta) / (1.0f + g);
        zb = zb_p / (1.0f + g);
    } else {
        zb = (zb_p > 0.0f) ? k : -k;
        s_ih_beta = ihp_b - ts_l * zb;
    }

    /* 数值保险钳位 */
    if (s_ih_alpha > SMO_IH_CLAMP_A)       s_ih_alpha = SMO_IH_CLAMP_A;
    else if (s_ih_alpha < -SMO_IH_CLAMP_A) s_ih_alpha = -SMO_IH_CLAMP_A;
    if (s_ih_beta > SMO_IH_CLAMP_A)        s_ih_beta = SMO_IH_CLAMP_A;
    else if (s_ih_beta < -SMO_IH_CLAMP_A)  s_ih_beta = -SMO_IH_CLAMP_A;

    /* e_hat = EMA(z)：α/β 同滤波 → 90° 关系与幅值比保持 */
    s_e_f_alpha += g_smo_lpf_alpha * (za - s_e_f_alpha);
    s_e_f_beta += g_smo_lpf_alpha * (zb - s_e_f_beta);

    g_smo_z_alpha_v = za;
    g_smo_z_beta_v = zb;
    g_smo_ih_alpha_a = s_ih_alpha;
    g_smo_ih_beta_a = s_ih_beta;
    g_smo_e_alpha_hat_v = s_e_f_alpha;
    g_smo_e_beta_hat_v = s_e_f_beta;
}

void Foc_Smo_Diag(float theta_enc_e_deg, float omega_e_rad_s)
{
    float ea = g_smo_e_alpha_hat_v;
    float eb = g_smo_e_beta_hat_v;
    float mag = sqrtf(ea * ea + eb * eb);
    float expect = fabsf(omega_e_rad_s) * g_smo_model_psi_wb;
    float th, err;

    g_smo_diag_e_mag_v = mag;
    g_smo_diag_e_expect_v = expect;
    g_smo_diag_e_ratio = (expect > 1.0e-4f) ? (mag / expect) : 0.0f;

    /* e = ωψf·(−sinθ, cosθ)：ω>0 时 atan2(e_β,e_α) = θ+90°；ω<0 时 = θ−90° */
    th = atan2f(eb, ea) * (180.0f / 3.14159265f);
    if (omega_e_rad_s >= 0.0f) {
        th -= 90.0f;
    } else {
        th += 90.0f;
    }
    th -= (float)((int32_t)(th / 360.0f)) * 360.0f;
    if (th < 0.0f) {
        th += 360.0f;
    }
    g_smo_diag_theta_hat_deg = th;

    err = th - theta_enc_e_deg;
    if (err > 180.0f)       err -= 360.0f;
    else if (err < -180.0f) err += 360.0f;
    g_smo_diag_theta_err_deg = err;

    /* EMA 滤波（看均值；alpha Watch 可调，<=0 或 >1 视为 1=直通） */
    {
        float fa = g_smo_diag_theta_err_filt_alpha;
        if (fa <= 0.0f || fa > 1.0f) fa = 1.0f;
        s_theta_err_f += fa * (err - s_theta_err_f);
        g_smo_diag_theta_err_filt_deg = s_theta_err_f;
    }

    /* 理论反电动势（编码器 θ + 带符号 ω·ψf）：E_theory = ωψf·(−sinθ, cosθ)，
     * 与同步系投影共用 sin/cos。稳态下为电频率正弦，VOFA 上同样混叠，
     * 与 ch7/8 同帧采样对照用 */
    th = theta_enc_e_deg * (3.14159265f / 180.0f);
    {
        float sa = sinf(th);
        float ca = cosf(th);
        float eth = omega_e_rad_s * g_smo_model_psi_wb;
        g_smo_theory_alpha_v = -eth * sa;
        g_smo_theory_beta_v = eth * ca;

        /* 同步系投影：e_hat 投到编码器转子系（q̂=(−sinθ,cosθ), d̂=(cosθ,sinθ)）。
         * 稳态下为直流慢变量，VOFA ~140Hz 无混叠可看；出现电频率纹波 = 波形畸变 */
        g_smo_diag_e_on_q_v = -ea * sa + eb * ca;
        g_smo_diag_e_on_d_v = ea * ca + eb * sa;
    }

    /* 总误差矢量范数 |E_hat − E_theory|（E_theory 在编码器系 = (ωψf, 0)，
     * 旋转系与静止系范数等价）。健康值 ≈ ωψf·δ(rad)，被已知滞后主导 */
    {
        float eq = g_smo_diag_e_on_q_v - expect;
        g_smo_diag_e_err_v = sqrtf(eq * eq + g_smo_diag_e_on_d_v * g_smo_diag_e_on_d_v);
    }

    /* ---- 反电动势自动判定（见 foc_smo.h 判定说明）----
     * 前提+C1+C2 全部连续满足 hold_ms → g_smo_emf_ok=1；
     * 任一拍破窗 → hold 计时清零、ok=0（恒定性要求，非粘滞锁存）。 */
    {
        float fa = g_smo_jdg_filt_alpha;
        uint8_t fail = 0u;

        if (fa <= 0.0f || fa > 1.0f) fa = 1.0f;
        /* 二级慢速 EMA 取"均值"：一级滤波(theta_err_filt,α=0.1)后仍有 ±9.5°
         * 马鞍纹波+混叠拍频驻留，点值进窗口会来回破窗；判定只关心均值 */
        s_ratio_f += fa * (g_smo_diag_e_ratio - s_ratio_f);
        g_smo_jdg_ratio_filt = s_ratio_f;
        s_phase_f += fa * (g_smo_diag_theta_err_filt_deg - s_phase_f);
        g_smo_jdg_phase_filt_deg = s_phase_f;

        /* 前提：机械转速（ω_e rad/s → rpm：×60/2π/极对数 ≈ ×9.5493/pp） */
        if (fabsf(omega_e_rad_s) * 9.5493f / (float)FOC_POLE_PAIRS < g_smo_jdg_rpm_min) {
            fail |= 0x01u;
        }
        /* C1：慢速幅值比（1500rpm 实测中心 ≈0.98） */
        if (s_ratio_f < g_smo_jdg_ratio_min || s_ratio_f > g_smo_jdg_ratio_max) {
            fail |= 0x02u;
        }
        /* C2：慢速相位均值（合理滞后带内即过；抓翻转/过大滞后，不卡必须≈−17°） */
        if (s_phase_f < g_smo_jdg_phase_min_deg || s_phase_f > g_smo_jdg_phase_max_deg) {
            fail |= 0x04u;
        }

        if (fail == 0u) {
            uint64_t now = tickTimer_GetCount();
            if (s_jdg_hold_t0 == 0u) {
                s_jdg_hold_t0 = now;
            }
            if ((float)(now - s_jdg_hold_t0) >= g_smo_jdg_hold_ms) {
                g_smo_emf_ok = 1u;
            }
        } else {
            s_jdg_hold_t0 = 0u;
            g_smo_emf_ok = 0u;
        }
        g_smo_jdg_fail = fail;
    }
}
