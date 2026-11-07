/**
 *******************************************************************************
 * @file  foc_smo.c
 * @brief 无感滑模观测器（SMO）实现 —— 纯旁观者，见 foc_smo.h 顶部说明。
 *
 * 每轴（Ts = 1/FOC_ISR_HZ = 50µs @20kHz）：
 *   预测   : î_p = î_k + (Ts/Ls)·(v_k − Rs·î_k − ê_k)
 *   误差   : s = î_p − i_k，sat = sat(s/Δ)
 *   电流   : 层内 î_{k+1} = (î_p + g·i_k)/(1+g)，z = (K_SM/Δ)·s/(1+g)
 *            层外 z = ±K_SM，î_{k+1} = î_p − Ts·z
 *   反电势 : ê_{k+1} = ê_k + Ts·L_SM·sat        ← 独立积分状态，稳态 ê→e
 *
 * 符号约定：s = î − i，滑模面上 z ≈ 模型误差。稳态 z→0 即 ê 收敛的直观标志。
 *******************************************************************************
 */

#include "foc_smo.h"
#include "motor_config.h"
#include "foc_core.h"      /* FOC_ISR_HZ（I.h 派生宏，随采样模式 10k/20k 自适应） */
#include "TickTimer.h"     /* tickTimer_GetCount：判定 hold 计时（ms 级时基） */
#include <math.h>

#define SMO_TS_S        (1.0f / (float)FOC_ISR_HZ)

/* Watch 可调参数（默认值来自 foc_smo.h 的宏） */
volatile float g_smo_rs_ohm         = SMO_RS_OHM;
volatile float g_smo_ls_h           = SMO_LS_H;
volatile float g_smo_psif_wb        = SMO_PSIF_WB;
volatile float g_smo_k_sm_v         = SMO_K_SM_V;
volatile float g_smo_l_sm_v_per_s   = SMO_L_SM_V_PER_S;
volatile float g_smo_delta_a        = SMO_DELTA_A;

volatile float g_smo_emf_wc_rad_s   = 0.0f;

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
 * ratio 实测比纯基波理论 0.90 高的原因：|ê|=sqrt(ea²+eb²) 把死区 6f 谐波与
 * 切换纹波能量也计入幅值，而分母 ωψf 是纯基波 → 分子虚高 ≈7%）。
 * 判定哲学：恒定滞后（−9°~−30°）PLL 都会吃掉，不妨碍可用；C2 只抓相位翻转
 * （大正角）与滞后过大（模型/滤波严重不对），不卡"必须≈−17°"。
 * rpm_min=900：Step 4 无感首切定在 1000rpm，判定需在该工况生效。 */
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
static float s_i_est_alpha;    /* 观测电流 î_α (A) */
static float s_i_est_beta;     /* 观测电流 î_β (A) */
static float s_e_est_alpha;    /* 反电动势积分状态 ê_α (V) */
static float s_e_est_beta;     /* 反电动势积分状态 ê_β (V) */
static float s_theta_err_f;    /* theta_err 滤波状态 (deg)；稳态远离 ±180°，普通 EMA 即可 */
static float s_ratio_f;        /* 幅值比慢速 EMA 状态（判定用） */
static float s_phase_f;        /* 相位慢速 EMA 状态（判定用，级联在 theta_err_filt 之后） */
static uint64_t s_jdg_hold_t0; /* 判据开始连续满足的时刻 (ms tick)，0=未在满足中 */

/* 每拍派生量（Step 刷新，轴函数与 Diag 共用） */
static float s_ts_l;           /* Ts/Ls */
static float s_rs;             /* 参与本拍计算的 Rs */
static float s_lin_det;        /* ê 环离散线性化的行列式（Diag 算 ωc 用） */
static float s_lin_tr;         /* ê 环离散线性化的迹 */

/*---------------------------------------------------------------------------
 * 饱和函数：|s| ≤ Δ 线性，否则 ±1
 *---------------------------------------------------------------------------*/
static float Smo_Sat(float s, float delta)
{
    if (s > delta) {
        return 1.0f;
    }
    if (s < -delta) {
        return -1.0f;
    }
    return s / delta;
}

/*---------------------------------------------------------------------------
 * 单轴观测器更新（α/β 各调一次）
 * 返回本拍滑模注入 z (V)
 *---------------------------------------------------------------------------*/
static float Smo_AxisUpdate(float v, float i_meas,
                            float *i_est, float *e_est,
                            float delta, float gain, float g)
{
    float l_sm, i_pred, s, sat, z, half_band;

    l_sm = (g_smo_l_sm_v_per_s > 0.0f) ? g_smo_l_sm_v_per_s : SMO_L_SM_V_PER_S;

    /* 1) 预测：含反电动势前馈 */
    i_pred = *i_est + s_ts_l * (v - s_rs * (*i_est) - (*e_est));

    /* 2) 电流误差与饱和函数 */
    s = i_pred - i_meas;
    sat = Smo_Sat(s, delta);

    /* 3) 电流状态：层内半隐式（无 K_SM/Δ 稳定上限），层外 ±K_SM 饱和注入 */
    half_band = delta * (1.0f + g);
    if (s <= half_band && s >= -half_band) {
        *i_est = (i_pred + g * i_meas) / (1.0f + g);
        z = gain * s / (1.0f + g);
    } else {
        z = (s > 0.0f) ? (gain * delta) : -(gain * delta);
        *i_est = i_pred - s_ts_l * z;
    }

    /* 4) 反电动势积分：ê += Ts·L_SM·sat */
    *e_est += SMO_TS_S * l_sm * sat;
    if (*e_est > SMO_E_EST_CLAMP_V) {
        *e_est = SMO_E_EST_CLAMP_V;
    } else if (*e_est < -SMO_E_EST_CLAMP_V) {
        *e_est = -SMO_E_EST_CLAMP_V;
    }

    return z;
}

void Foc_Smo_Reset(void)
{
    s_i_est_alpha = 0.0f;
    s_i_est_beta = 0.0f;
    s_e_est_alpha = 0.0f;
    s_e_est_beta = 0.0f;
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
    g_smo_emf_wc_rad_s = 0.0f;
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
    float rs, ls, ts_l, k_sm, delta, gain, g, l_sm;
    float z_alpha, z_beta;

    /* 参数（每拍读 Watch，实时生效；防 0/负保护） */
    rs = (g_smo_rs_ohm > 0.0f) ? g_smo_rs_ohm : SMO_RS_OHM;
    ls = (g_smo_ls_h > 1.0e-6f) ? g_smo_ls_h : SMO_LS_H;
    k_sm = (g_smo_k_sm_v > 0.0f) ? g_smo_k_sm_v : 0.0f;
    delta = (g_smo_delta_a > SMO_DELTA_MIN_A) ? g_smo_delta_a : SMO_DELTA_MIN_A;
    l_sm = (g_smo_l_sm_v_per_s > 0.0f) ? g_smo_l_sm_v_per_s : SMO_L_SM_V_PER_S;

    ts_l = SMO_TS_S / ls;
    gain = k_sm / delta;      /* 等效滑模增益 K_SM/Δ (V/A) */
    g = ts_l * gain;

    s_ts_l = ts_l;
    s_rs = rs;

    /* ê 环离散线性化（x = î−i, y = ê−e 的 2×2 状态矩阵）：
     *   A11 = (1−Ts·Rs/Ls)/(1+g)，A12 = −(Ts/Ls)/(1+g)，A21 = β·A11，A22 = 1−β·Ts/Ls
     *   β = Ts·L_SM/Δ；det = A11（恒等于 (1−Ts·Rs/Ls)/(1+g)，与 L_SM 无关）
     * 主循环 Diag 由 trace/det 解极点 → g_smo_emf_wc_rad_s（PLL 补偿用） */
    {
        float a = 1.0f - ts_l * rs;
        float beta = SMO_TS_S * l_sm / delta;

        s_lin_det = a / (1.0f + g);
        s_lin_tr  = s_lin_det + 1.0f - beta * ts_l;
    }

    z_alpha = Smo_AxisUpdate(v_alpha, i_alpha, &s_i_est_alpha, &s_e_est_alpha,
                             delta, gain, g);
    z_beta  = Smo_AxisUpdate(v_beta, i_beta, &s_i_est_beta, &s_e_est_beta,
                             delta, gain, g);

    /* 数值保险钳位 */
    if (s_i_est_alpha > SMO_I_EST_CLAMP_A)       s_i_est_alpha = SMO_I_EST_CLAMP_A;
    else if (s_i_est_alpha < -SMO_I_EST_CLAMP_A) s_i_est_alpha = -SMO_I_EST_CLAMP_A;
    if (s_i_est_beta > SMO_I_EST_CLAMP_A)        s_i_est_beta = SMO_I_EST_CLAMP_A;
    else if (s_i_est_beta < -SMO_I_EST_CLAMP_A)  s_i_est_beta = -SMO_I_EST_CLAMP_A;

    g_smo_z_alpha_v = z_alpha;
    g_smo_z_beta_v = z_beta;
    g_smo_ih_alpha_a = s_i_est_alpha;
    g_smo_ih_beta_a = s_i_est_beta;
    g_smo_e_alpha_hat_v = s_e_est_alpha;
    g_smo_e_beta_hat_v = s_e_est_beta;
}

void Foc_Smo_Diag(float theta_enc_e_deg, float omega_e_rad_s)
{
    float ea = g_smo_e_alpha_hat_v;
    float eb = g_smo_e_beta_hat_v;
    float mag = sqrtf(ea * ea + eb * eb);
    float expect = fabsf(omega_e_rad_s) * g_smo_psif_wb;
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

    /* ê 环一阶等效极点（rad/s）：由 Step 存下的离散线性化 trace/det 解极点。
     * 复极点取 |λ|=sqrt(det)；实极点取慢根。等效连续极点 = −ln|λ|/Ts。
     * 只在主循环算（logf 不进 ISR），供 foc_pll.c 的 d1 补偿用。 */
    {
        float disc = s_lin_tr * s_lin_tr - 4.0f * s_lin_det;
        float lam;

        if (disc >= 0.0f) {
            lam = 0.5f * (s_lin_tr + sqrtf(disc));
        } else {
            lam = sqrtf(s_lin_det);
        }
        if (lam > 0.0f && lam < 1.0f) {
            g_smo_emf_wc_rad_s = -logf(lam) / SMO_TS_S;
        } else {
            g_smo_emf_wc_rad_s = 0.0f;
        }
    }

    /* 理论反电动势（编码器 θ + 带符号 ω·ψf）：E_theory = ωψf·(−sinθ, cosθ)，
     * 与同步系投影共用 sin/cos。稳态下为电频率正弦，VOFA 上同样混叠，
     * 与 ch7/8 同帧采样对照用 */
    th = theta_enc_e_deg * (3.14159265f / 180.0f);
    {
        float sa = sinf(th);
        float ca = cosf(th);
        float eth = omega_e_rad_s * g_smo_psif_wb;
        g_smo_theory_alpha_v = -eth * sa;
        g_smo_theory_beta_v = eth * ca;

        /* 同步系投影：ê 投到编码器转子系（q̂=(−sinθ,cosθ), d̂=(cosθ,sinθ)）。
         * 稳态下为直流慢变量，VOFA ~140Hz 无混叠可看；出现电频率纹波 = 波形畸变 */
        g_smo_diag_e_on_q_v = -ea * sa + eb * ca;
        g_smo_diag_e_on_d_v = ea * ca + eb * sa;
    }

    /* 总误差矢量范数 |ê − E_theory|（E_theory 在编码器系 = (ωψf, 0)，
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
        /* C1：慢速幅值比（ê 为积分状态，中心应 ≈1） */
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
