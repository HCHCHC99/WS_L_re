/**
 *******************************************************************************
 * @file  foc_47_smoif.c
 * @brief FOC mode 47 - 开环强拖 + SMO/PLL 纯程序自检（无编码器 / 无 PI）。
 *
 * 结构与事实源见 foc_47_smoif.h 顶部速览卡。
 *
 * 控制路径（每 ISR 拍）：
 *   CALIB  三相零矢量 + Foc_Calib_Feed（200 拍 skip + 4000 拍平均 -> 锁零偏）
 *   ALIGN  θ_syn 冻结在 0，输出 V0 直流磁场吸附转子（f_cmd = 0）
 *   RAMP   f_cmd 以 g_smo47_f_ramp_hz_s 逼近 g_smo47_f_run_hz；
 *          θ_syn += 2π·f_cmd·Ts；V = min(v_max, v0 + kvf·f/1000)
 *   RUN    同 RAMP（f 已达目标，继续跟随 Watch 改动），稳态判定窗口打开
 *   输出   valpha/vbeta = V·(cosθ_syn, sinθ_syn) -> Foc_Svpwm -> SetDuty3Phase
 *   旁观   Foc_Smo_Step(前后两拍指令平均, 同源 Clarke 电流)
 *          -> Foc_Pll_Step(e_hat) -> Foc_Pll_Compare(θ_syn)   （RAMP/RUN 才喂）
 *
 * 全程不读编码器、不跑任何 PI；角度/频率/理论反电动势三个参照全部由程序
 * 给定（θ_syn / ω_syn / ω_syn·ψf），这就是"纯程序自检"的含义。
 *******************************************************************************
 */

#include "foc_47_smoif.h"
#include "foc_smo.h"
#include "foc_pll.h"
#include "foc_calib.h"
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "motor_config.h"

#define FOC47_ISR_DT_US     (1000000u / FOC_ISR_HZ)
#define FOC47_ALIGN_TICKS   ((uint32_t)FOC47_ALIGN_MS * (uint32_t)FOC_ISR_HZ / 1000u)
#define FOC47_STEADY_TICKS  ((uint32_t)FOC47_STEADY_MS * (uint32_t)FOC_ISR_HZ / 1000u)
#define FOC47_RAD2DEG       57.2958f

volatile uint8_t  g_smo47_running           = 0u;
volatile uint8_t  g_smo47_state             = FOC47_STEP_IDLE;
volatile uint8_t  g_smo47_evt               = 0u;

/* Watch 可调（初值 = 宏默认；MCU 复位回宏，Start 不重置） */
volatile float    g_smo47_f_run_hz          = FOC47_F_RUN_HZ;
volatile float    g_smo47_f_ramp_hz_s       = FOC47_F_RAMP_HZ_PER_S;
volatile float    g_smo47_v0_v              = FOC47_V0_V;
volatile float    g_smo47_kvf_v_khz         = FOC47_KVF_V_PER_KHZ;
volatile float    g_smo47_v_max_v           = FOC47_V_MAX_V;
volatile float    g_smo47_jdg_omega_err_pct = FOC47_JDG_OMEGA_ERR_PCT;
volatile float    g_smo47_jdg_theta_min_deg = FOC47_JDG_THETA_MIN_DEG;
volatile float    g_smo47_jdg_theta_max_deg = FOC47_JDG_THETA_MAX_DEG;

/* 只读观察量 */
volatile float    g_smo47_f_cmd_hz          = 0.0f;
volatile float    g_smo47_v_drag_v          = 0.0f;
volatile float    g_smo47_theta_syn_deg     = 0.0f;
volatile float    g_smo47_omega_ref_rpm     = 0.0f;
volatile float    g_smo47_id_ma             = 0.0f;
volatile float    g_smo47_iq_ma             = 0.0f;
volatile float    g_smo47_du                = 50.0f;
volatile float    g_smo47_dv                = 50.0f;
volatile float    g_smo47_dw                = 50.0f;
volatile uint8_t  g_smo47_steady            = 0u;
volatile uint8_t  g_smo47_jdg_ok            = 0u;   /* foc_obs.c 主循环维护 */
volatile uint8_t  g_smo47_jdg_fail          = 0u;

/* 运行内部状态 */
static uint32_t s_state_ticks;      /* 当前状态已过拍数 */
static float    s_theta_syn_rad;    /* 拖动磁场角 (rad, [0,2PI)) */
static float    s_v_alpha_prev;     /* 上一拍指令电压（SMO 喂入平均用） */
static float    s_v_beta_prev;
static float    s_zero_u_ma;        /* Foc_Calib 锁定的三相零偏 */
static float    s_zero_v_ma;
static float    s_zero_w_ma;
static float    s_f_run_prev;       /* 检测 Watch 改目标 -> 重开稳定窗 */

static void Smo47_ResetLoopState(void)
{
    s_state_ticks = 0u;
    s_theta_syn_rad = 0.0f;
    s_v_alpha_prev = 0.0f;
    s_v_beta_prev = 0.0f;
    s_zero_u_ma = 0.0f;
    s_zero_v_ma = 0.0f;
    s_zero_w_ma = 0.0f;
    s_f_run_prev = g_smo47_f_run_hz;
}

static void Smo47_ClearObservables(void)
{
    g_smo47_evt = 0u;
    g_smo47_f_cmd_hz = 0.0f;
    g_smo47_v_drag_v = 0.0f;
    g_smo47_theta_syn_deg = 0.0f;
    g_smo47_omega_ref_rpm = 0.0f;
    g_smo47_id_ma = 0.0f;
    g_smo47_iq_ma = 0.0f;
    g_smo47_du = 50.0f;
    g_smo47_dv = 50.0f;
    g_smo47_dw = 50.0f;
    g_smo47_steady = 0u;
    g_smo47_jdg_ok = 0u;
    g_smo47_jdg_fail = 0u;
}

static void Smo47_ClearCurrentFeedback(void)
{
    g_smo47_id_ma = 0.0f;
    g_smo47_iq_ma = 0.0f;
    g_foc_id_ma = 0.0f;
    g_foc_iq_ma = 0.0f;
    g_foc_vd = 0.0f;
    g_foc_vq = 0.0f;
}

static stc_i_data_t Smo47_CorrectedData(const stc_i_data_t *pData)
{
    stc_i_data_t data = *pData;
    data.i16IU_mA = (int16_t)((float)pData->i16IU_mA - s_zero_u_ma);
    data.i16IV_mA = (int16_t)((float)pData->i16IV_mA - s_zero_v_ma);
    data.i16IW_mA = (int16_t)((float)pData->i16IW_mA - s_zero_w_ma);
    return data;
}

void Foc_Smo47_Start(void)
{
    Foc_Core_ClearFault();
    Smo47_ResetLoopState();
    Smo47_ClearObservables();
    Foc_Smo_Reset();   /* SMO 旁观者状态清零（观测电流/e_hat/判定） */
    Foc_Pll_Reset();   /* PLL 旁观者状态清零（θ̂/ω̂/诊断） */
    Foc_Calib_Start(); /* 零偏自校准：CALIB 状态内三相零矢量窗口喂样 */

    g_foc_mode      = FOC_MODE_ALIGN;
    g_foc_phase     = 4u;
    g_foc_theta_rad = 0.0f;
    g_foc_id_ma     = 0.0f;
    g_foc_iq_ma     = 0.0f;
    g_foc_vd        = 0.0f;
    g_foc_vq        = 0.0f;
    Foc_Core_SetStateMachine(FOC_STATE_ALIGN);
    Foc_Core_ResetEma();
    g_foc_align_state = 1u;

    g_smo47_running = 1u;
    g_smo47_state = FOC47_STEP_CALIB;
    Foc_Core_PwmStart();   /* 50/50/50 零矢量，CALIB 直接复用 */
    FOC47_LOG("start f=%d mHz ramp=%d mHz/s v0=%d mV kvf=%d mV/khz vmax=%d mV",
              (int)(g_smo47_f_run_hz * 1000.0f),
              (int)(g_smo47_f_ramp_hz_s * 1000.0f),
              (int)(g_smo47_v0_v * 1000.0f),
              (int)(g_smo47_kvf_v_khz * 1000.0f),
              (int)(g_smo47_v_max_v * 1000.0f));
}

void Foc_Smo47_Stop(void)
{
    if (g_smo47_running) {
        g_smo47_running = 0u;
        g_foc_align_state = 0u;
        if (g_foc_active) {
            g_foc_active = 0u;
            Foc_Core_PwmStop();
            Foc_Core_SetStateMachine(FOC_STATE_IDLE);
        }
        FOC47_LOG("stopped");
    }
    g_smo47_state = FOC47_STEP_IDLE;
    Smo47_ClearCurrentFeedback();
}

void Foc_Smo47_Step(const stc_i_data_t *pData)
{
    stc_i_data_t data;
    float v, valpha, vbeta, du, dv, dw;
    float ialpha_s, ibeta_s, sgn;
    float id, iq, f_step, f_t;

    if (Foc_Core_OverCurrent(pData)) {
        g_smo47_state = FOC47_STEP_FAULT_OC;
        g_smo47_evt = FOC47_EVT_OC;
        g_smo47_running = 0u;
        Foc_Core_FaultStop(1u);
        Smo47_ClearCurrentFeedback();
        return;
    }

    if ((g_smo47_state == FOC47_STEP_IDLE)
            || (g_smo47_state == FOC47_STEP_FAULT_OC)) {
        return;
    }

    /* ---- 状态机推进 ---- */
    if (g_smo47_state == FOC47_STEP_CALIB) {
        /* 三相零矢量 + 零偏采样（foc_calib：skip 200 + avg 4000 拍） */
        TMR4_PWM_SetDuty3Phase(50.0f, 50.0f, 50.0f);
        if (Foc_Calib_Feed(pData) != 0u) {
            Foc_Calib_GetOffsetsMa(&s_zero_u_ma, &s_zero_v_ma, &s_zero_w_ma);
            g_smo47_evt = FOC47_EVT_CALIB_DONE;
            g_smo47_state = FOC47_STEP_ALIGN;
            s_state_ticks = 0u;
        }
        return;
    }

    s_state_ticks++;

    if (g_smo47_state == FOC47_STEP_ALIGN) {
        /* θ_syn 冻结在 0（f_cmd=0）：V0 直流磁场吸附转子 */
        if (s_state_ticks >= FOC47_ALIGN_TICKS) {
            g_smo47_state = FOC47_STEP_RAMP;
            s_state_ticks = 0u;
        }
    } else {
        /* RAMP/RUN：f_cmd 朝 f_run 限速逼近（RUN 中改 Watch 目标同样跟随） */
        if ((g_smo47_state == FOC47_STEP_RUN)
                && (g_smo47_f_run_hz != s_f_run_prev)) {
            s_f_run_prev = g_smo47_f_run_hz;   /* 改目标 -> 重开稳定窗 */
            g_smo47_steady = 0u;
            s_state_ticks = 0u;
        }
        f_t = g_smo47_f_run_hz;
        f_step = g_smo47_f_ramp_hz_s / (float)FOC_ISR_HZ;
        if (g_smo47_f_cmd_hz < f_t) {
            g_smo47_f_cmd_hz += f_step;
            if (g_smo47_f_cmd_hz > f_t) {
                g_smo47_f_cmd_hz = f_t;
            }
        } else if (g_smo47_f_cmd_hz > f_t) {
            g_smo47_f_cmd_hz -= f_step;
            if (g_smo47_f_cmd_hz < f_t) {
                g_smo47_f_cmd_hz = f_t;
            }
        }
        if (g_smo47_state == FOC47_STEP_RAMP) {
            if (g_smo47_f_cmd_hz >= f_t) {
                g_smo47_state = FOC47_STEP_RUN;
                s_state_ticks = 0u;
                g_smo47_evt = FOC47_EVT_RAMP_DONE;
            }
        } else if (g_smo47_state == FOC47_STEP_RUN) {
            if ((g_smo47_steady == 0u)
                    && (s_state_ticks >= FOC47_STEADY_TICKS)) {
                g_smo47_steady = 1u;   /* J1/J3 判定窗口从此打开 */
            }
        }
    }

    /* ---- 拖动磁场合成（θ_syn 积分 + V/f 幅值） ---- */
    s_theta_syn_rad += FOC_MATH_2PI * g_smo47_f_cmd_hz / (float)FOC_ISR_HZ;
    s_theta_syn_rad -= (float)((int32_t)(s_theta_syn_rad
                                         * (1.0f / FOC_MATH_2PI)))
                       * FOC_MATH_2PI;
    if (s_theta_syn_rad < 0.0f) {
        s_theta_syn_rad += FOC_MATH_2PI;
    }

    v = g_smo47_v0_v + g_smo47_kvf_v_khz * (g_smo47_f_cmd_hz * 0.001f);
    if (v > g_smo47_v_max_v) {
        v = g_smo47_v_max_v;
    }
    g_smo47_v_drag_v = v;

    valpha = v * Foc_Math_Cos(s_theta_syn_rad);
    vbeta  = v * Foc_Math_Sin(s_theta_syn_rad);
    Foc_Svpwm(valpha, vbeta, Foc_Vbus_GetV(), &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);

    /* ---- 同源 Clarke 电流（扣 Foc_Calib 零偏 × sign，SMO 输入与显示共用） ---- */
    data = Smo47_CorrectedData(pData);
    sgn = (float)g_foc_cur_sign;
    {
        float ia_s = (float)data.i16IU_mA * 0.001f * sgn;
        float ib_s = (float)data.i16IV_mA * 0.001f * sgn;
        float ic_s = (float)data.i16IW_mA * 0.001f * sgn;
        Foc_Clarke(ia_s, ib_s, ic_s, &ialpha_s, &ibeta_s);
    }

    /* ---- 拖动系 id/iq 观测（θ_syn 系，非转子系）：id≈磁化 / iq≈拖动转矩 ---- */
    Foc_Core_GetDq(&data, s_theta_syn_rad, &id, &iq);
    g_smo47_id_ma = id * 1000.0f;
    g_smo47_iq_ma = iq * 1000.0f;
    g_foc_id_ma = g_smo47_id_ma;
    g_foc_iq_ma = g_smo47_iq_ma;

    /* ---- SMO/PLL 旁观（有旋转才有 e；CALIB/ALIGN 期 e≈0 不喂） ----
     * 电压喂前后两拍指令平均（中心对齐 PWM + 谷点采样的传输延迟补偿，
     * 与 mode 45 同款）；PLL 角度比较用拖动磁场角 θ_syn（纯程序参照）。 */
    if ((g_smo47_state == FOC47_STEP_RAMP)
            || (g_smo47_state == FOC47_STEP_RUN)) {
        Foc_Smo_Step((s_v_alpha_prev + valpha) * 0.5f,
                     (s_v_beta_prev + vbeta) * 0.5f, ialpha_s, ibeta_s);
        Foc_Pll_Step(g_smo_e_alpha_hat_v, g_smo_e_beta_hat_v);
        /* 角度比较必须同拍新值（250Hz 时每拍 4.5°，喂上一拍 g_smo47_
         * theta_syn_deg 会引入固定 4.5° 假差），mode 45 同款处理 */
        Foc_Pll_Compare(s_theta_syn_rad * FOC47_RAD2DEG);
    }
    s_v_alpha_prev = valpha;
    s_v_beta_prev = vbeta;

    /* ---- 记录观测量 ---- */
    g_foc_theta_rad = s_theta_syn_rad;
    g_foc_valpha = valpha;
    g_foc_vbeta = vbeta;
    g_foc_du = du;
    g_foc_dv = dv;
    g_foc_dw = dw;
    g_smo47_du = du;
    g_smo47_dv = dv;
    g_smo47_dw = dw;
    g_smo47_theta_syn_deg = s_theta_syn_rad * FOC47_RAD2DEG;
    g_smo47_omega_ref_rpm = g_smo47_f_cmd_hz * 60.0f
                          / (float)FOC_POLE_PAIRS;
}

/*===========================================================================
 * 模式自持 VOFA：固定 16ch，通道含义见 foc_47_smoif.h 顶部速览卡。
 *===========================================================================*/
int Foc_Smo47_VofaFill(int32_t *cur)
{
    cur[0]  = (int32_t)(g_smo47_f_cmd_hz * 1000.0f);            /* mHz -> Hz */
    cur[1]  = (int32_t)(g_smo47_v_drag_v * 1000.0f);            /* mV -> V */
    cur[2]  = (int32_t)(g_smo47_theta_syn_deg * 1000.0f);       /* mdeg -> deg */
    cur[3]  = (int32_t)(g_pll_theta_rotor_deg * 1000.0f);       /* mdeg -> deg */
    cur[4]  = (int32_t)(g_pll_diag_theta_err_deg * 1000.0f);    /* mdeg -> deg */
    cur[5]  = (int32_t)(g_pll_diag_theta_err_filt_deg * 1000.0f);
    cur[6]  = (int32_t)(g_pll_omega_hat_rpm * 1000.0f);         /* mrpm -> rpm */
    cur[7]  = (int32_t)(g_pll_omega_out_rpm * 1000.0f);         /* mrpm -> rpm */
    cur[8]  = (int32_t)(g_smo47_omega_ref_rpm * 1000.0f);       /* mrpm -> rpm */
    cur[9]  = (int32_t)((g_pll_omega_out_rpm - g_smo47_omega_ref_rpm)
                        * 1000.0f);                             /* mrpm -> rpm */
    cur[10] = (int32_t)(g_smo_diag_e_mag_v * 1000.0f);          /* mV -> V */
    cur[11] = (int32_t)(g_smo_diag_e_expect_v * 1000.0f);       /* mV -> V */
    cur[12] = (int32_t)(g_smo_diag_e_on_q_v * 1000.0f);         /* mV -> V */
    cur[13] = (int32_t)(g_smo_diag_e_on_d_v * 1000.0f);         /* mV -> V */
    cur[14] = (int32_t)(g_smo47_jdg_ok * 1000.0f);              /* 0/1 */
    cur[15] = (int32_t)(g_smo47_iq_ma);                         /* mA -> A */
    return 16;
}
