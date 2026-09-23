/**
 *******************************************************************************
 * @file  foc_pll.c
 * @brief 无感锁相环（PLL）实现 —— 纯旁观者，见 foc_pll.h 顶部说明。
 *******************************************************************************
 */

#include "foc_pll.h"
#include "motor_config.h"
#include "foc_core.h"      /* FOC_ISR_HZ（与 SMO Step 同拍） */
#include "foc_smo.h"       /* 补偿角读 SMO 观测器参数（调参自动跟随） */
#include <math.h>

#define PLL_TS_S        (1.0f / (float)FOC_ISR_HZ)
#define PLL_TWO_PI      6.2831853f
#define PLL_RAD2DEG     57.2957795f

/* Watch 可调参数 */
volatile float g_pll_omega_n   = 1256.6f;  /* 2π×200 rad/s：电流环带宽 ~1880Hz 的 1/9 */
volatile float g_pll_zeta      = 0.9f;     /* 0.707~1 稳妥区 */
volatile float g_pll_mag_min_v = 0.10f;    /* 弱信号冻结门限（<1300rpm 验证窗外信号小） */

/* 补偿角 + 无感输出参数 */
volatile uint8_t g_pll_comp_en       = 0u;     /* 补偿角开关（Watch 置 1 开启） */
volatile float g_pll_comp_bias_deg   = 0.0f;   /* 补偿偏置：锁点偏置残留微调 */
volatile float g_pll_wout_lpf_alpha  = 0.0125f; /* ω̂ 输出低通：fc≈20Hz@10kHz，
                                                 * 喂速度环前压掉 25Hz 频段 PLL 超前 */

/* ISR 输出 */
volatile float g_pll_theta_rotor_deg = 0.0f;
volatile float g_pll_omega_hat_rpm   = 0.0f;
volatile float g_pll_omega_hat_rad_s = 0.0f;
volatile float g_pll_theta_e_deg     = 0.0f;
volatile float g_pll_err_rad         = 0.0f;

/* 补偿/无感输出 */
volatile float g_pll_theta_comp_deg  = 0.0f;
volatile float g_pll_theta_park_deg  = 0.0f;
volatile float g_pll_omega_out_rpm   = 0.0f;
volatile float g_pll_comp_deg_out    = 0.0f;

/* 诊断输出 */
volatile float g_pll_diag_theta_err_deg      = 0.0f;
volatile float g_pll_diag_theta_err_filt_deg = 0.0f;
volatile float g_pll_diag_err_filt_deg       = 0.0f;
volatile float g_pll_diag_filt_alpha         = 0.1f;
volatile float g_pll_diag_smo_err_deg        = 0.0f;  /* ISR 同拍 SMO atan2 角差 */

/* 内部状态 */
static float s_theta_e;   /* 锁定的 e 矢量相位 (rad,[0,2π)) */
static float s_omega;     /* 电角速度估计 (rad/s，带符号) */
static float s_sin;       /* sin(θ̂)（鉴相器复用） */
static float s_cos;       /* cos(θ̂) */
static float s_ea_last;   /* 本拍 e_hat 快照（同拍 SMO atan2 对比用） */
static float s_eb_last;
static uint16_t s_cmp_div; /* 同拍 SMO atan2 分频计数（每 10 拍算一次摊薄开销） */

/* 角度 wrap 到 [0,2π) */
static float Pll_Wrap2Pi(float a)
{
    a -= (float)((int32_t)(a / PLL_TWO_PI)) * PLL_TWO_PI;
    if (a < 0.0f)       a += PLL_TWO_PI;
    else if (a >= PLL_TWO_PI) a -= PLL_TWO_PI;
    return a;
}

/* 角度 wrap 到 [-180,180) */
static float Pll_Wrap180(float a)
{
    if (a > 180.0f)       a -= 360.0f;
    else if (a < -180.0f) a += 360.0f;
    return a;
}

void Foc_Pll_Reset(void)
{
    s_theta_e = 0.0f;
    s_omega = 0.0f;
    s_sin = 0.0f;
    s_cos = 1.0f;
    g_pll_theta_rotor_deg = 0.0f;
    g_pll_omega_hat_rpm = 0.0f;
    g_pll_omega_hat_rad_s = 0.0f;
    g_pll_theta_e_deg = 0.0f;
    g_pll_err_rad = 0.0f;
    g_pll_diag_theta_err_deg = 0.0f;
    g_pll_diag_theta_err_filt_deg = 0.0f;
    g_pll_diag_err_filt_deg = 0.0f;
    g_pll_diag_smo_err_deg = 0.0f;
    g_pll_theta_comp_deg = 0.0f;
    g_pll_theta_park_deg = 0.0f;
    g_pll_omega_out_rpm = 0.0f;
    g_pll_comp_deg_out = 0.0f;
    s_ea_last = 0.0f;
    s_eb_last = 0.0f;
    s_cmp_div = 0u;
}

void Foc_Pll_Step(float e_alpha, float e_beta)
{
    float mag, err, wn, zeta, kp, ki, th_deg, sub;

    s_ea_last = e_alpha;   /* 快照供 Compare 的同拍 SMO atan2 使用 */
    s_eb_last = e_beta;

    /* 弱信号保护：静止/极低速 e_hat 被纹波淹没，鉴相无意义 → 冻结
     * （残差清零，积分保持，角度输出在该区间无效属预期） */
    mag = sqrtf(e_alpha * e_alpha + e_beta * e_beta);
    if (mag < g_pll_mag_min_v) {
        g_pll_err_rad = 0.0f;
        return;
    }

    /* 归一化鉴相器：e·u(θ̂)/|e| = sin(θ_e − θ̂) ≈ θ_e − θ̂ (rad)
     * u(θ̂) = (−sinθ̂, cosθ̂) 为 θ̂+90° 方向单位矢量 */
    err = (-e_alpha * s_sin + e_beta * s_cos) / mag;
    g_pll_err_rad = err;

    /* Watch 参数 → 环路增益（每拍读，实时生效；防 0/负保护） */
    wn = (g_pll_omega_n > 1.0f) ? g_pll_omega_n : 1.0f;
    zeta = g_pll_zeta;
    if (zeta < 0.1f)  zeta = 0.1f;
    else if (zeta > 2.0f) zeta = 2.0f;
    kp = 2.0f * zeta * wn;
    ki = wn * wn;

    /* PI 环路：积分器估计角速度，比例项直接校正相位 */
    s_omega += ki * err * PLL_TS_S;
    s_theta_e = Pll_Wrap2Pi(s_theta_e + (s_omega + kp * err) * PLL_TS_S);
    s_sin = sinf(s_theta_e);
    s_cos = cosf(s_theta_e);

    /* 输出刷新 */
    g_pll_omega_hat_rad_s = s_omega;
    g_pll_omega_hat_rpm = s_omega * 9.5493f / (float)FOC_POLE_PAIRS; /* ×60/2π/极对数 */
    th_deg = s_theta_e * PLL_RAD2DEG;
    g_pll_theta_e_deg = th_deg;
    sub = (s_omega >= 0.0f) ? 90.0f : -90.0f;  /* e 相位 = 转子角 ± 90°（按旋向） */
    g_pll_theta_rotor_deg = Pll_Wrap180(th_deg - sub);
    if (g_pll_theta_rotor_deg < 0.0f) {
        g_pll_theta_rotor_deg += 360.0f;
    }

    /* ---- 滞后补偿 + 无感输出（Step 4，见 foc_pll.h 说明） ----
     * δ̂ = atan(|ω̂|/ω_c) + atan(|ω̂|·L/(k/φ)) + bias
     * ω_c = e_hat EMA 精确 −3dB 截止：cos w = (1+b²−2a²)/(2b)，b=1−α，
     *       fc = fs·w/2π（小 α 近式 620Hz 偏低，精确 ≈805Hz@α=0.39/10kHz）。
     * θ_comp = θ̂_rotor + δ̂（δ̂ 为滞后量，补偿=加回）；
     * θ_park = θ_comp + ω̂·Ts（外推一拍：Park 在下一拍首使用本值，
     *          消除一拍滞后的 9°@1500rpm）；
     * ω̂ 经低通输出 g_pll_omega_out_rpm（无感喂速度环，压 25Hz 频段超前）。 */
    {
        float w = fabsf(s_omega);
        float a = g_smo_lpf_alpha;
        float b, cosw, wc, kg, d1, d2, th_c;

        if (a <= 0.0f || a >= 1.0f) a = 0.39f;
        b = 1.0f - a;
        cosw = (1.0f + b * b - 2.0f * a * a) / (2.0f * b);
        if (cosw > 1.0f)        cosw = 1.0f;
        else if (cosw < -1.0f)  cosw = -1.0f;
        wc = (float)FOC_ISR_HZ * acosf(cosw);   /* ω_c = 2π·fc (rad/s) */

        kg = g_smo_obs_gain;
        if (kg < 0.1f) kg = 0.1f;
        d1 = atan2f(w, wc);
        d2 = atanf(w * g_smo_model_l_h / kg);
        g_pll_comp_deg_out = (d1 + d2) * PLL_RAD2DEG + g_pll_comp_bias_deg;

        th_c = g_pll_theta_rotor_deg + ((g_pll_comp_en != 0u) ? g_pll_comp_deg_out : 0.0f);
        th_c = Pll_Wrap180(th_c);
        if (th_c < 0.0f) th_c += 360.0f;
        g_pll_theta_comp_deg = th_c;

        th_c += s_omega * PLL_TS_S * PLL_RAD2DEG * 1.5f;   /* 外推 1.5 拍：
                                    * Park 在下一拍首使用本值，对齐 PWM 作用中心
                                    *（与有感侧 g_smo45_ang_lead_ticks=1.5 同口径） */
        th_c = Pll_Wrap180(th_c);
        if (th_c < 0.0f) th_c += 360.0f;
        g_pll_theta_park_deg = th_c;

        /* ω̂ 输出低通（α≤0 视为直通） */
        if (g_pll_wout_lpf_alpha <= 0.0f || g_pll_wout_lpf_alpha > 1.0f) {
            g_pll_omega_out_rpm = g_pll_omega_hat_rpm;
        } else {
            g_pll_omega_out_rpm += g_pll_wout_lpf_alpha
                                 * (g_pll_omega_hat_rpm - g_pll_omega_out_rpm);
        }
    }
}

void Foc_Pll_Compare(float theta_enc_e_deg)
{
    /* ISR 同拍比较：补偿后 Park 角（无感时真正进 Park 的角）与编码器角，
     * 同属本拍快照，无撕裂。补偿开启且锁定后均值应 ≈0（残差=bias+锁点偏置） */
    g_pll_diag_theta_err_deg = Pll_Wrap180(g_pll_theta_park_deg - theta_enc_e_deg);

    /* 同拍 SMO atan2 角差（每 10 拍算一次，atan2f ~1-2µs 摊薄后可忽略）：
     * 与 PLL 的 e 完全同快照基准 —— 两链均值之差 = PLL 锁点偏置 + 动态差，
     * 用于裁决 "PLL 均值 −1.3° vs SMO 理论 −24.5°" 的矛盾 */
    if (++s_cmp_div >= 10u) {
        float th_smo = atan2f(s_eb_last, s_ea_last) * PLL_RAD2DEG;
        s_cmp_div = 0u;
        th_smo += (s_omega >= 0.0f) ? -90.0f : 90.0f;
        if (th_smo < 0.0f)         th_smo += 360.0f;
        else if (th_smo >= 360.0f) th_smo -= 360.0f;
        g_pll_diag_smo_err_deg = Pll_Wrap180(th_smo - theta_enc_e_deg);
    }
}

void Foc_Pll_Diag(void)
{
    float fa;

    /* 只做滤波（差值已在 ISR 同拍算好，主循环禁止再比较角度） */
    fa = g_pll_diag_filt_alpha;
    if (fa <= 0.0f || fa > 1.0f) fa = 1.0f;
    g_pll_diag_theta_err_filt_deg += fa * (g_pll_diag_theta_err_deg
                                           - g_pll_diag_theta_err_filt_deg);
    g_pll_diag_err_filt_deg += fa * (g_pll_err_rad * PLL_RAD2DEG
                                     - g_pll_diag_err_filt_deg);
}
