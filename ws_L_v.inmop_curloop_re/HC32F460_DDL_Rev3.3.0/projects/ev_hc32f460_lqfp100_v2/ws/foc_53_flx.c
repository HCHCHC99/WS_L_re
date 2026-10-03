/**
 *******************************************************************************
 * @file  foc_53_flx.c
 * @brief FOC mode 53 - 永磁磁链 psi_f 辨识（编码器速度环 + 变转速 vq/omega_e 最小二乘）。
 *
 * 原理与判据见 foc_53_flx.h 顶部速览卡（唯一事实源）。关键实现点：
 *   1) 控制侧 = 直接复用 mode 40 的级联结构（5ms 速度 PI + 每拍电流 PI）：
 *      电机自己爬到 g_flx53_rpm[k]，不需要任何外力。
 *   2) 测量侧与环路解耦：对用于**测量**的 iq/vq 各加一级 EMA（α=0.05，τ≈1ms），
 *      绝不插入电流 PI 的反馈链（避免给环路加相位滞后）。
 *   3) omega_e 不做差值估计，用**窗内编码器计数增量 + Timer6 µs 时基**直算：
 *      分母是整窗平均转速，没有 5ms 速度窗的量化噪声。
 *   4) 求解取**斜率**：对 y = vq - R_eff·iq 做二参数最小二乘 y = psi_f·omega_e + V_dt。
 *      斜率对常数偏置免疫（R·iq 偏置、电流零偏、Vcc 增益误差、V_dt 都只影响截距）。
 *   5) Vbus 假设误差是本模式最大系统性误差源：拟合时电压用 vq × g_flx53_vbus_scale。
 *******************************************************************************
 */

#include "foc_53_flx.h"
#include "../Utils/dev_pid.h"  /* pid_config_t / pid_state_t / PID_Init / PID_UpdateUs */
#include "foc_24_dcal.h"       /* Foc_Dcal_GetResult */
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "hc32_ll_tmra.h"
#include "I.h"                 /* FOC_ISR_HZ, g_i_* */
#include "timer6_timebase.h"   /* Timer6 计数器：0.32us 分辨率时基，窗内 omega_e 直算 */
#include <math.h>

/*=============================================================================
 * 时长换算
 *=============================================================================*/
#define FOC53_ISR_DT_US      (1000000u / FOC_ISR_HZ)
#define FOC53_SPD_WIN_TICKS  (FOC53_SPD_WIN_MS * FOC_ISR_HZ / 1000u)
#define FOC53_MS_TO_TICKS(ms) ((uint32_t)(ms) * FOC_ISR_HZ / 1000u)

/*=============================================================================
 * Keil Watch 可调参数
 *=============================================================================*/
volatile float    g_flx53_rpm[FOC53_MAX_POINTS] = {800.0f, 1600.0f, 2400.0f, 3200.0f, 0.0f, 0.0f};
volatile uint32_t g_flx53_points      = 4u;
volatile float    g_flx53_r_used_ohm  = 0.098f;   /* mode 51 实测值 */
volatile float    g_flx53_vdead_v     = 0.160f;   /* mode 51 实测值 */
volatile uint32_t g_flx53_fit_vdt     = 1u;
volatile uint32_t g_flx53_avg_ms      = 300u;
volatile uint32_t g_flx53_settle_ms   = 300u;
volatile float    g_flx53_rpm_tol     = 15.0f;
volatile float    g_flx53_iq_filt_alpha = 0.05f;
volatile uint32_t g_flx53_reject_en   = 0u;
volatile float    g_flx53_rpm_max     = 4000.0f;
volatile float    g_flx53_vbus_scale  = 1.0f;

/*=============================================================================
 * 观测量（Watch / VOFA）
 *=============================================================================*/
volatile uint8_t  g_flx53_running     = 0u;
volatile uint8_t  g_flx53_state       = FOC53_STEP_IDLE;
volatile uint8_t  g_flx53_evt         = 0u;
volatile uint8_t  g_flx53_evt_seq     = 0u;
volatile float    g_flx53_psi_mwb     = 0.0f;
volatile float    g_flx53_psi_diff_mwb = 0.0f;
volatile float    g_flx53_vdt_fit_mv  = 0.0f;
volatile float    g_flx53_vdt_used_mv = 0.0f;
volatile float    g_flx53_ratio       = 0.0f;
volatile float    g_flx53_psi_alt_mwb = 0.0f;
volatile float    g_flx53_psi_spread_mwb = 0.0f;
volatile float    g_flx53_r2          = 0.0f;
volatile float    g_flx53_win_split_pct = 0.0f;
volatile float    g_flx53_psi_live_mwb = 0.0f;
volatile float    g_flx53_target_rpm  = 0.0f;
volatile float    g_flx53_rpm_meas  = 0.0f;
volatile float    g_flx53_rpm_disp    = 0.0f;
volatile float    g_flx53_vq          = 0.0f;
volatile float    g_flx53_iq          = 0.0f;
volatile float    g_flx53_iq_filt     = 0.0f;
volatile uint8_t  g_flx53_vsat        = 0u;
volatile float    g_flx53_omega_e     = 0.0f;
volatile uint32_t g_flx53_pts_done    = 0u;
volatile uint32_t g_flx53_elapsed_ms  = 0u;
volatile float    g_flx53_psi_pts[FOC53_MAX_POINTS];
volatile float    g_flx53_pts_rpm[FOC53_MAX_POINTS];
volatile float    g_flx53_pts_vq[FOC53_MAX_POINTS];
volatile float    g_flx53_pts_iq[FOC53_MAX_POINTS];
volatile float    g_flx53_pts_we[FOC53_MAX_POINTS];

/*=============================================================================
 * PID 配置（镜像 mode 40 已验证值；模块自持，不对外暴露）
 *=============================================================================*/
static pid_config_t s_cfg_speed = {
    .enabled = true, .p_valid = true, .i_valid = true, .d_valid = false,
    .kp = FOC53_SPD_KP_MA_PER_RPM, .ki = FOC53_SPD_KI_MA_PER_RPM_S, .kd = 0.0f,
    .output_min = -FOC53_SPD_IQ_LIMIT_MA, .output_max = FOC53_SPD_IQ_LIMIT_MA,
    .integral_max = FOC53_SPD_IQ_LIMIT_MA, .i_term_max = FOC53_SPD_IQ_LIMIT_MA,
    .update_ms = 0u,
};
static pid_config_t s_cfg_id = {
    .enabled = true, .p_valid = true, .i_valid = true, .d_valid = false,
    .kp = FOC53_PI_KP, .ki = FOC53_PI_KI, .kd = 0.0f,
    .output_min = -FOC53_PI_UMAX_V, .output_max = FOC53_PI_UMAX_V,
    .integral_max = FOC53_ITERM_MAX_V, .i_term_max = FOC53_ITERM_MAX_V,
    .update_ms = 0u,
};
static pid_config_t s_cfg_iq = {
    .enabled = true, .p_valid = true, .i_valid = true, .d_valid = false,
    .kp = FOC53_PI_KP, .ki = FOC53_PI_KI, .kd = 0.0f,
    .output_min = -FOC53_PI_UMAX_V, .output_max = FOC53_PI_UMAX_V,
    .integral_max = FOC53_ITERM_MAX_V, .i_term_max = FOC53_ITERM_MAX_V,
    .update_ms = 0u,
};

static pid_state_t s_pid_speed;
static pid_state_t s_pid_id;
static pid_state_t s_pid_iq;

/*=============================================================================
 * 内部状态
 *=============================================================================*/
static int32_t  s_rotor_count;
static uint16_t s_encoder_prev_hw;
static uint8_t  s_encoder_initialized;
static float    s_zero_u_ma, s_zero_v_ma, s_zero_w_ma;

static int32_t  s_speed_acc_cnt;
static uint32_t s_speed_win_tick;
static uint8_t  s_speed_filt_init, s_speed_disp_init;
static float    s_speed_ramp_rpm, s_speed_filt_rpm, s_speed_disp_rpm;
static float    s_speed_out_ma;

static uint32_t s_ms_tick;
static uint32_t s_state_ms;          /* 当前状态已持续 (ms) */
static uint32_t s_settle_ms_ok;      /* 稳态已连续保持 (ms) */
static uint8_t  s_fit_poor;          /* 1 = 拟合结果离散度大 */
static uint8_t  s_iq_stable;         /* 1 = iq 变化率小（稳态判据） */
static float    s_iq_slow;           /* 稳态判据慢 EMA (mA) */
static uint8_t  s_iq_slow_init;
static uint8_t  s_meas_init;         /* 测量侧 EMA 播种标志 */
static float    s_iq_filt;           /* 测量侧 iq EMA (mA) */
static float    s_vq_filt;           /* 测量侧 vq EMA (V) */

static uint32_t s_k;                 /* 当前转速点索引 */
static uint32_t s_split_n;           /* 已计入 win_split 平均的点数 */
static uint32_t s_avg_ticks;         /* 本点平均窗采样数 */
static uint32_t s_avg_skip_ticks;    /* 本点前段丢弃采样数 */
static uint32_t s_avg_skip;          /* 已丢弃采样数 */
static uint32_t s_avg_cnt;           /* 已接受采样数 */
static int64_t  s_sum_iq_ma;         /* Σ iq (mA) */
static int64_t  s_sum_vq_mv;         /* Σ vq (mV) */
static float    s_sum_vq_a, s_sum_vq_b;  /* 前半/后半 vq 和 (mV) */
static uint32_t s_cnt_a, s_cnt_b;
static int32_t  s_enc_avg_acc;       /* 窗内编码器计数增量（带符号） */
static uint32_t s_tb_avg_ticks;      /* 窗内 Timer6 计数增量（回绕已处理） */
static uint16_t s_tb_avg_prev;
static uint32_t s_wf_n;              /* Welford 计数 */
static float    s_wf_mean, s_wf_M2;  /* Welford 均值/二阶量（3σ 剔除用） */

/*=============================================================================
 * 助手
 *=============================================================================*/
static void Flx53_SetEvt(uint8_t e)
{
    g_flx53_evt = e;
    g_flx53_evt_seq++;
}

static void Flx53_ResetLoopState(void)
{
    s_rotor_count = 0;
    s_encoder_prev_hw = 0u;
    s_encoder_initialized = 0u;
    s_zero_u_ma = s_zero_v_ma = s_zero_w_ma = 0.0f;
    s_speed_acc_cnt = 0;
    s_speed_win_tick = 0u;
    s_speed_filt_init = 0u;
    s_speed_disp_init = 0u;
    s_speed_ramp_rpm = 0.0f;
    s_speed_filt_rpm = 0.0f;
    s_speed_disp_rpm = 0.0f;
    s_speed_out_ma = 0.0f;
    s_ms_tick = 0u;
    s_state_ms = 0u;
    s_settle_ms_ok = 0u;
    s_fit_poor = 0u;
    s_iq_stable = 0u;
    s_iq_slow = 0.0f;
    s_iq_slow_init = 0u;
    s_meas_init = 0u;
    s_iq_filt = 0.0f;
    s_vq_filt = 0.0f;
    s_k = 0u;
    s_split_n = 0u;
    s_avg_ticks = 1u;
    s_avg_skip_ticks = 0u;
    s_avg_skip = 0u;
    s_avg_cnt = 0u;
    s_sum_iq_ma = 0;
    s_sum_vq_mv = 0;
    s_sum_vq_a = 0.0f;
    s_sum_vq_b = 0.0f;
    s_cnt_a = 0u;
    s_cnt_b = 0u;
    s_enc_avg_acc = 0;
    s_tb_avg_ticks = 0u;
    s_tb_avg_prev = 0u;
    s_wf_n = 0u;
    s_wf_mean = 0.0f;
    s_wf_M2 = 0.0f;
}

static void Flx53_ClearResults(void)
{
    uint32_t i;

    g_flx53_evt = 0u;
    g_flx53_evt_seq = 0u;
    g_flx53_psi_mwb = 0.0f;
    g_flx53_psi_diff_mwb = 0.0f;
    g_flx53_vdt_fit_mv = 0.0f;
    g_flx53_vdt_used_mv = g_flx53_vdead_v * 1000.0f;
    g_flx53_ratio = 0.0f;
    g_flx53_psi_alt_mwb = 0.0f;
    g_flx53_psi_spread_mwb = 0.0f;
    g_flx53_r2 = 0.0f;
    g_flx53_win_split_pct = 0.0f;
    g_flx53_psi_live_mwb = 0.0f;
    g_flx53_target_rpm = 0.0f;
    g_flx53_rpm_meas = 0.0f;
    g_flx53_rpm_disp = 0.0f;
    g_flx53_vq = 0.0f;
    g_flx53_iq = 0.0f;
    g_flx53_iq_filt = 0.0f;
    g_flx53_vsat = 0u;
    g_flx53_omega_e = 0.0f;
    g_flx53_pts_done = 0u;
    g_flx53_elapsed_ms = 0u;
    for (i = 0u; i < FOC53_MAX_POINTS; i++) {
        g_flx53_psi_pts[i] = 0.0f;
        g_flx53_pts_rpm[i] = 0.0f;
        g_flx53_pts_vq[i] = 0.0f;
        g_flx53_pts_iq[i] = 0.0f;
        g_flx53_pts_we[i] = 0.0f;
    }
}

static stc_i_data_t Flx53_CorrectedData(const stc_i_data_t *pData)
{
    stc_i_data_t data = *pData;
    data.i16IU_mA = (int16_t)((float)pData->i16IU_mA - s_zero_u_ma);
    data.i16IV_mA = (int16_t)((float)pData->i16IV_mA - s_zero_v_ma);
    data.i16IW_mA = (int16_t)((float)pData->i16IW_mA - s_zero_w_ma);
    return data;
}

/* 关 PWM 前先写零矢量，避免最后一拍留下非零占空比 */
static void Flx53_StopPwm(void)
{
    float du, dv, dw;

    Foc_Svpwm(0.0f, 0.0f, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);
    if (g_foc_active) {
        g_foc_active = 0u;
        Foc_Core_PwmStop();
        Foc_Core_SetStateMachine(FOC_STATE_IDLE);
    }
}

/* 测速：5ms 窗 raw rpm -> EMA α=0.25（进 PI）+ 独立 α=0.05（显示专用） */
static void Flx53_UpdateSpeed(int32_t corrected_delta)
{
    float raw_rpm;

    s_speed_acc_cnt += corrected_delta;
    if (++s_speed_win_tick < FOC53_SPD_WIN_TICKS) {
        return;
    }
    raw_rpm = (float)s_speed_acc_cnt * 60.0f
            * (1000.0f / (float)FOC53_SPD_WIN_MS) / (float)ENCODER_CPR;
    if (s_speed_filt_init == 0u) {
        s_speed_filt_rpm = raw_rpm;
        s_speed_filt_init = 1u;
    } else {
        s_speed_filt_rpm += FOC53_SPD_FILT_ALPHA * (raw_rpm - s_speed_filt_rpm);
    }
    if (s_speed_disp_init == 0u) {
        s_speed_disp_rpm = raw_rpm;
        s_speed_disp_init = 1u;
    } else {
        s_speed_disp_rpm += FOC53_SPD_DISP_ALPHA * (raw_rpm - s_speed_disp_rpm);
    }
    g_flx53_rpm_meas = s_speed_filt_rpm;
    g_flx53_rpm_disp = s_speed_disp_rpm;
    s_speed_acc_cnt = 0;
    s_speed_win_tick = 0u;
}

/* 速度环：斜坡限幅整形后作 PI 给定，输出 iq_ref (mA)；id_ref 恒 0 */
static void Flx53_UpdateOuterLoop(void)
{
    float target_rpm = g_flx53_target_rpm;
    float ramp_step = FOC53_ACCEL_LIMIT_RPM_S
                    * ((float)FOC53_SPD_WIN_MS * 0.001f);

    if (target_rpm > FOC53_SPEED_REF_LIMIT_RPM) target_rpm = FOC53_SPEED_REF_LIMIT_RPM;
    if (target_rpm < -FOC53_SPEED_REF_LIMIT_RPM) target_rpm = -FOC53_SPEED_REF_LIMIT_RPM;
    if (s_speed_ramp_rpm < target_rpm) {
        s_speed_ramp_rpm += ramp_step;
        if (s_speed_ramp_rpm > target_rpm) s_speed_ramp_rpm = target_rpm;
    } else if (s_speed_ramp_rpm > target_rpm) {
        s_speed_ramp_rpm -= ramp_step;
        if (s_speed_ramp_rpm < target_rpm) s_speed_ramp_rpm = target_rpm;
    }
    s_speed_out_ma = PID_UpdateUs(&s_pid_speed, s_speed_ramp_rpm,
                                  s_speed_filt_rpm, FOC53_SPD_WIN_US);
}

/* 测量侧 EMA（只滤测量副本，不进电流 PI 反馈链） */
static void Flx53_UpdateMeasEma(float iq_a, float vq_eff)
{
    float a = g_flx53_iq_filt_alpha;

    if (a <= 0.0f || a > 1.0f) { a = 1.0f; }
    if (s_meas_init == 0u) {
        s_iq_filt = iq_a * 1000.0f;
        s_vq_filt = vq_eff;
        s_meas_init = 1u;
    } else {
        s_iq_filt += a * (iq_a * 1000.0f - s_iq_filt);
        s_vq_filt += a * (vq_eff - s_vq_filt);
    }
    g_flx53_iq_filt = s_iq_filt;
}

/* 稳态判据用的变化率检测：慢 EMA(τ≈5ms) 与快 EMA 之差小 => iq 基本不变 */
static void Flx53_UpdateSettleDetect(void)
{
    if (s_iq_slow_init == 0u) {
        s_iq_slow = s_iq_filt;
        s_iq_slow_init = 1u;
    } else {
        s_iq_slow += FOC53_IQ_SLOW_ALPHA * (s_iq_filt - s_iq_slow);
    }
    s_iq_stable = (fabsf(s_iq_filt - s_iq_slow) < (FOC53_IQ_STABLE_A * 1000.0f))
                ? 1u : 0u;
}

/*=============================== 测量状态机 ===============================*/
static void Flx53_StartAvg(void)
{
    s_avg_ticks = FOC53_MS_TO_TICKS(g_flx53_avg_ms);
    if (s_avg_ticks == 0u) { s_avg_ticks = 1u; }
    s_avg_skip_ticks = FOC53_MS_TO_TICKS(FOC53_AVG_SKIP_MS);
    s_avg_skip = 0u;
    s_avg_cnt = 0u;
    s_sum_iq_ma = 0;
    s_sum_vq_mv = 0;
    s_sum_vq_a = 0.0f;
    s_sum_vq_b = 0.0f;
    s_cnt_a = 0u;
    s_cnt_b = 0u;
    s_meas_init = 0u;          /* 每点重新播种测量 EMA */
    s_iq_slow_init = 0u;
    s_wf_n = 0u;
    s_wf_mean = 0.0f;
    s_wf_M2 = 0.0f;
    s_enc_avg_acc = 0;
    s_tb_avg_ticks = 0u;
    s_tb_avg_prev = (uint16_t)Timer6_Timebase_GetCounter();
    s_state_ms = 0u;
    g_flx53_state = FOC53_STEP_AVG;
}

/* 平均窗逐拍累加（在 Step 内调用，vq 已折算 vbus_scale） */
static void Flx53_AccumSample(void)
{
    if (s_avg_skip < s_avg_skip_ticks) {
        s_avg_skip++;
        return;
    }
    if (g_flx53_reject_en != 0u) {
        if (s_wf_n >= 30u) {
            float sd = sqrtf(s_wf_M2 / (float)(s_wf_n - 1u));
            if (fabsf(s_iq_filt - s_wf_mean) > (3.0f * sd)) {
                return;        /* 离群：不计入 */
            }
        }
        s_wf_n++;
        {   /* Welford 在线均值/二阶量 */
            float d = s_iq_filt - s_wf_mean;
            s_wf_mean += d / (float)s_wf_n;
            s_wf_M2 += d * (s_iq_filt - s_wf_mean);
        }
    }
    s_sum_iq_ma += (int64_t)(s_iq_filt * 1000.0f);
    s_sum_vq_mv += (int64_t)(s_vq_filt * 1000.0f);
    if (s_avg_cnt < (s_avg_ticks / 2u)) {
        s_sum_vq_a += s_vq_filt * 1000.0f;
        s_cnt_a++;
    } else {
        s_sum_vq_b += s_vq_filt * 1000.0f;
        s_cnt_b++;
    }
    s_avg_cnt++;
}

/* 本点收尾：算 omega_e / psi_f，落数组与观测量 */
static void Flx53_FinishPoint(void)
{
    uint32_t freq = Timer6_Timebase_GetFrequency();
    float iq_a = 0.0f, vq_v = 0.0f, rpm = 0.0f, we = 0.0f, psi_mwb = 0.0f;

    if ((s_avg_cnt > 0u) && (s_tb_avg_ticks > 0u) && (freq > 0u)) {
        iq_a = (float)s_sum_iq_ma / (1000.0f * (float)s_avg_cnt);
        vq_v = (float)s_sum_vq_mv / (1000.0f * (float)s_avg_cnt);
        rpm  = (float)s_enc_avg_acc / (float)ENCODER_CPR
             * (float)freq / (float)s_tb_avg_ticks * 60.0f;
        we   = rpm * (FOC_MATH_2PI * (float)FOC_POLE_PAIRS / 60.0f);
        if (we > 1.0f) {
            psi_mwb = (vq_v - g_flx53_r_used_ohm * iq_a) / we * 1000.0f;
            g_flx53_omega_e = we;
        }
    }
    g_flx53_pts_rpm[s_k] = rpm;
    g_flx53_pts_vq[s_k]  = vq_v;
    g_flx53_pts_iq[s_k]  = iq_a;
    g_flx53_pts_we[s_k]  = we;
    g_flx53_psi_pts[s_k] = psi_mwb;
    g_flx53_psi_live_mwb = psi_mwb;

    /* 窗内前半/后半 vq 均值相对差（诊断漂移/噪声） */
    if ((s_cnt_a > 0u) && (s_cnt_b > 0u)) {
        float ma = s_sum_vq_a / (float)s_cnt_a;
        float mb = s_sum_vq_b / (float)s_cnt_b;
        float den = (fabsf(ma) > fabsf(mb)) ? fabsf(ma) : fabsf(mb);
        float pct;

        if (den < 0.05f) { den = 0.05f; }
        pct = 100.0f * fabsf(ma - mb) / den;
        if (s_split_n == 0u) {
            g_flx53_win_split_pct = pct;
        } else {
            g_flx53_win_split_pct = (g_flx53_win_split_pct * (float)s_split_n + pct)
                                  / (float)(s_split_n + 1u);
        }
        s_split_n++;
    }

    g_flx53_pts_done++;
    s_k++;
    g_flx53_state = FOC53_STEP_NEXT;
}

/* 某点未稳 -> 跳过（数组留 0，不参与拟合） */
static void Flx53_SkipPoint(void)
{
    g_flx53_pts_rpm[s_k] = 0.0f;
    g_flx53_pts_vq[s_k]  = 0.0f;
    g_flx53_pts_iq[s_k]  = 0.0f;
    g_flx53_pts_we[s_k]  = 0.0f;
    g_flx53_psi_pts[s_k] = 0.0f;
    g_flx53_psi_live_mwb = 0.0f;
    g_flx53_pts_done++;
    Flx53_SetEvt(FOC53_EVT_TIMEOUT);
    s_k++;
    g_flx53_state = FOC53_STEP_NEXT;
}

/* 二参数最小二乘：y = psi_f·x + V_dt，x = omega_e，y = vq - R·iq */
static void Flx53_DoFit(void)
{
    float xv[FOC53_MAX_POINTS], yv[FOC53_MAX_POINTS], ya[FOC53_MAX_POINTS];
    uint32_t i, n = 0u, np = g_flx53_points;
    float sx = 0.0f, sy = 0.0f, xbar, ybar;
    float sdx2 = 0.0f, sdxdy = 0.0f, sdxdya = 0.0f;
    float slope = 0.0f, slope_alt = 0.0f, intercept = 0.0f;
    float ss_tot = 0.0f, ss_res = 0.0f, r2 = 0.0f;
    float pmin = 0.0f, pmax = 0.0f, spread = 0.0f;
    uint8_t have_p = 0u;

    if (np > FOC53_MAX_POINTS) { np = FOC53_MAX_POINTS; }

    for (i = 0u; i < np; i++) {
        if (g_flx53_pts_we[i] > 1.0f) {
            xv[n] = g_flx53_pts_we[i];
            yv[n] = g_flx53_pts_vq[i] - g_flx53_r_used_ohm * g_flx53_pts_iq[i];
            ya[n] = g_flx53_pts_vq[i] - FOC_MOTOR_RS_OHM * g_flx53_pts_iq[i];
            sx += xv[n];
            sy += yv[n];
            n++;
        }
    }

    g_flx53_psi_diff_mwb = 0.0f;
    if (n >= 2u) {
        xbar = sx / (float)n;
        ybar = sy / (float)n;
        for (i = 0u; i < n; i++) {
            float dx = xv[i] - xbar;
            sdx2   += dx * dx;
            sdxdy  += dx * (yv[i] - ybar);
            sdxdya += dx * (ya[i] - ybar);
        }
        if (sdx2 > 1.0e-6f) {
            slope     = sdxdy / sdx2;
            slope_alt = sdxdya / sdx2;
        }
        intercept = ybar - slope * xbar;
        for (i = 0u; i < n; i++) {
            float e = yv[i] - (slope * xv[i] + intercept);
            float d = yv[i] - ybar;
            ss_res += e * e;
            ss_tot += d * d;
        }
        if (ss_tot > 1.0e-9f) { r2 = 1.0f - ss_res / ss_tot; }
        if (fabsf(xv[n - 1u] - xv[0]) > 1.0e-3f) {
            g_flx53_psi_diff_mwb = (yv[n - 1u] - yv[0])
                                 / (xv[n - 1u] - xv[0]) * 1000.0f;
        }
    }
    for (i = 0u; i < np; i++) {
        if (g_flx53_pts_we[i] > 1.0f) {
            float p = g_flx53_psi_pts[i];
            if (have_p == 0u) { pmin = p; pmax = p; have_p = 1u; }
            else { if (p < pmin) pmin = p; if (p > pmax) pmax = p; }
        }
    }
    if (have_p != 0u) { spread = pmax - pmin; }

    g_flx53_psi_mwb        = slope * 1000.0f;
    g_flx53_psi_alt_mwb    = slope_alt * 1000.0f;
    g_flx53_vdt_fit_mv     = intercept * 1000.0f;
    g_flx53_vdt_used_mv    = (g_flx53_fit_vdt != 0u)
                           ? (intercept * 1000.0f)
                           : (g_flx53_vdead_v * 1000.0f);
    g_flx53_r2             = r2;
    g_flx53_psi_spread_mwb = spread;
    if (FOC_MOTOR_FLUX_VS > 0.0f) {
        g_flx53_ratio = g_flx53_psi_mwb / (FOC_MOTOR_FLUX_VS * 1000.0f);
    }
    {
        float spread_pct = (g_flx53_psi_mwb > 1.0e-6f)
                         ? (100.0f * spread / g_flx53_psi_mwb) : 100.0f;
        s_fit_poor = ((n < 2u) || (spread_pct >= FOC53_SPREAD_LIMIT_PCT)) ? 1u : 0u;
    }
}

static void Flx53_Finish(void)
{
    Flx53_SetEvt((s_fit_poor != 0u) ? FOC53_EVT_DONE_POOR : FOC53_EVT_DONE_OK);
    g_flx53_state = FOC53_STEP_DONE;
    Flx53_StopPwm();
}

/*=============================================================================
 * Foc_FlxId_Start（主循环上下文）
 *=============================================================================*/
void Foc_FlxId_Start(void)
{
    foc_dcal24_result_t cal;
    uint32_t i;

    /* ---- 前置：必须有 mode 24 的零点 ---- */
    if (Foc_Dcal_GetResult(&cal) == 0u) {
        g_flx53_running = 0u;
        g_flx53_state   = FOC53_STEP_IDLE;
        Flx53_SetEvt(FOC53_EVT_NO_CAL);
        FLX53_DBG("ERROR: no mode 24 calibration; run mode 24 first");
        return;
    }

    Foc_Core_ClearFault();
    Flx53_ResetLoopState();
    Flx53_ClearResults();

    /* ---- 参数护栏 ---- */
    if (g_flx53_points < 2u) { g_flx53_points = 2u; }
    if (g_flx53_points > FOC53_MAX_POINTS) { g_flx53_points = FOC53_MAX_POINTS; }
    for (i = 0u; i < FOC53_MAX_POINTS; i++) {
        float r = g_flx53_rpm[i];
        if (r < 0.0f) { r = -r; }
        if ((r > 0.0f) && (r < FOC53_RPM_MIN)) { r = FOC53_RPM_MIN; }
        if (r > FOC53_RPM_MAX_HARD) { r = FOC53_RPM_MAX_HARD; }
        g_flx53_rpm[i] = r;
    }
    if (g_flx53_r_used_ohm < 0.001f) { g_flx53_r_used_ohm = 0.001f; }
    if (g_flx53_vdead_v < 0.0f) { g_flx53_vdead_v = 0.0f; }
    if (g_flx53_avg_ms < FOC53_AVG_MIN_MS) { g_flx53_avg_ms = FOC53_AVG_MIN_MS; }
    if (g_flx53_avg_ms > FOC53_AVG_MAX_MS) { g_flx53_avg_ms = FOC53_AVG_MAX_MS; }
    if (g_flx53_settle_ms < FOC53_SETTLE_MIN_MS) { g_flx53_settle_ms = FOC53_SETTLE_MIN_MS; }
    if (g_flx53_settle_ms > FOC53_SETTLE_MAX_MS) { g_flx53_settle_ms = FOC53_SETTLE_MAX_MS; }
    if (g_flx53_rpm_tol < 1.0f) { g_flx53_rpm_tol = 1.0f; }
    if (g_flx53_rpm_max < FOC53_RPM_MIN) { g_flx53_rpm_max = FOC53_RPM_MIN; }
    if (g_flx53_rpm_max > FOC53_RPM_MAX_HARD) { g_flx53_rpm_max = FOC53_RPM_MAX_HARD; }
    if (g_flx53_vbus_scale < FOC53_VBUS_SCALE_MIN) { g_flx53_vbus_scale = FOC53_VBUS_SCALE_MIN; }
    if (g_flx53_vbus_scale > FOC53_VBUS_SCALE_MAX) { g_flx53_vbus_scale = FOC53_VBUS_SCALE_MAX; }

    /* ---- 复用 ALIGN 分发路径（按 g_flx53_running 认领） ---- */
    g_foc_mode      = FOC_MODE_ALIGN;
    g_foc_phase     = 4u;
    g_foc_theta_rad = 0.0f;
    g_foc_id_ma     = 0.0f;
    g_foc_iq_ma     = 0.0f;
    g_foc_vd        = 0.0f;
    g_foc_vq        = 0.0f;
    Foc_Core_SetStateMachine(FOC_STATE_ALIGN);
    Foc_Core_ResetVoltageEnvelope();
    Foc_Core_ResetEma();
    g_foc_align_state = 1u;

    s_zero_u_ma = cal.zero_u_ma;
    s_zero_v_ma = cal.zero_v_ma;
    s_zero_w_ma = cal.zero_w_ma;

    PID_Init(&s_pid_speed, &s_cfg_speed);
    PID_Init(&s_pid_id, &s_cfg_id);
    PID_Init(&s_pid_iq, &s_cfg_iq);
    PID_Reset(&s_pid_speed);
    PID_Reset(&s_pid_id);
    PID_Reset(&s_pid_iq);

    g_flx53_target_rpm = g_flx53_rpm[0];
    g_flx53_running = 1u;
    g_flx53_state   = FOC53_STEP_SPINUP;
    Foc_Core_PwmStart();

    FLX53_DBG("start: n=%u r0=%drpm r1=%drpm r2=%drpm r3=%drpm R=%dmohm avg=%ums fit_vdt=%u",
              (unsigned)g_flx53_points,
              (int)g_flx53_rpm[0], (int)g_flx53_rpm[1],
              (int)g_flx53_rpm[2], (int)g_flx53_rpm[3],
              (int)(g_flx53_r_used_ohm * 1000.0f),
              (unsigned)g_flx53_avg_ms, (unsigned)g_flx53_fit_vdt);
}

/*=============================================================================
 * Foc_FlxId_Stop
 *=============================================================================*/
void Foc_FlxId_Stop(void)
{
    if (g_flx53_running) {
        g_flx53_running   = 0u;
        g_foc_align_state = 0u;
        g_flx53_state     = FOC53_STEP_IDLE;
        Flx53_StopPwm();
        FLX53_DBG("stopped");
    }
}

/*=============================================================================
 * Foc_FlxId_Step（20 kHz ISR，禁止打印）
 *=============================================================================*/
void Foc_FlxId_Step(const stc_i_data_t *pData)
{
    stc_i_data_t data;
    float id, iq, vd, vq, vq_eff, valpha, vbeta, du, dv, dw;
    float cos_r, sin_r, rotor_rad;
    uint16_t hardware_count;
    int32_t hardware_delta, corrected_delta;
    uint8_t ms_tick_flag = 0u;

    /* ===== 过流保护 ===== */
    if (Foc_Core_OverCurrent(pData)) {
        g_flx53_state = FOC53_STEP_FAULT_OC;
        Flx53_SetEvt(FOC53_EVT_OC);
        Foc_Core_FaultStop(1u);
        return;
    }

    /* 终态/故障：PWM 已关，不再驱动 */
    if ((g_flx53_state == FOC53_STEP_IDLE) ||
        (g_flx53_state == FOC53_STEP_DONE) ||
        (g_flx53_state == FOC53_STEP_FAULT_OC) ||
        (g_flx53_state == FOC53_STEP_FAULT_OVRPM)) {
        return;
    }

    /* ===== 点间推进（单拍过渡，本拍不输出） ===== */
    if (g_flx53_state == FOC53_STEP_NEXT) {
        if (s_k >= g_flx53_points) {
            Flx53_DoFit();
            g_flx53_target_rpm = 0.0f;
            s_state_ms = 0u;
            g_flx53_state = FOC53_STEP_RAMPDOWN;
        } else {
            g_flx53_target_rpm = g_flx53_rpm[s_k];
            s_state_ms = 0u;
            s_settle_ms_ok = 0u;
            g_flx53_state = FOC53_STEP_SPINUP;
        }
        return;
    }

    /* ===== 1ms 计时 ===== */
    if (++s_ms_tick >= (FOC_ISR_HZ / 1000u)) {
        s_ms_tick = 0u;
        ms_tick_flag = 1u;
        g_flx53_elapsed_ms++;
        s_state_ms++;
    }

    /* ===== 编码器 / 测速 / 速度环 ===== */
    hardware_count = TMRA_GetCountValue(CM_TMRA_1);
    if (s_encoder_initialized == 0u) {
        s_encoder_prev_hw = hardware_count;
        s_encoder_initialized = 1u;
    }
    hardware_delta = (int32_t)(int16_t)((uint16_t)hardware_count - s_encoder_prev_hw);
    s_encoder_prev_hw = hardware_count;
    if (hardware_delta > FOC53_ENC_DELTA_MAX) {
        hardware_delta = FOC53_ENC_DELTA_MAX;
    }
    if (hardware_delta < -FOC53_ENC_DELTA_MAX) {
        hardware_delta = -FOC53_ENC_DELTA_MAX;
    }
    corrected_delta = hardware_delta * (int32_t)g_foc_enc_dir;
    s_rotor_count = Foc_Core_ModPos(s_rotor_count + corrected_delta, (int32_t)ENCODER_CPR);

    if (g_flx53_state == FOC53_STEP_RAMPDOWN) {
        g_flx53_target_rpm = 0.0f;
    } else {
        g_flx53_target_rpm = g_flx53_rpm[s_k];
    }
    Flx53_UpdateSpeed(corrected_delta);
    Flx53_UpdateOuterLoop();

    rotor_rad = (float)s_rotor_count
              * (FOC_MATH_2PI * (float)FOC_POLE_PAIRS / (float)ENCODER_CPR);
    rotor_rad -= (float)((int32_t)(rotor_rad * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
    if (rotor_rad < 0.0f) { rotor_rad += FOC_MATH_2PI; }

    data = Flx53_CorrectedData(pData);
    Foc_Core_GetDq(&data, rotor_rad, &id, &iq);
    g_flx53_iq = iq * 1000.0f;
    g_foc_id_ma = id * 1000.0f;
    g_foc_iq_ma = iq * 1000.0f;

    vd = PID_UpdateUs(&s_pid_id, 0.0f, id, FOC53_ISR_DT_US);
    vq = PID_UpdateUs(&s_pid_iq, s_speed_out_ma * 0.001f, iq, FOC53_ISR_DT_US);
    vq_eff = vq * g_flx53_vbus_scale;    /* Vbus 假设误差的校正：拟合用"真实电压" */
    g_foc_vd = vd;
    g_foc_vq = vq;
    g_flx53_vq = vq_eff;
    g_flx53_vsat = ((vd <= -FOC53_PI_UMAX_V + 0.01f) ||
                    (vd >=  FOC53_PI_UMAX_V - 0.01f) ||
                    (vq <= -FOC53_PI_UMAX_V + 0.01f) ||
                    (vq >=  FOC53_PI_UMAX_V - 0.01f)) ? 1u : 0u;

    Flx53_UpdateMeasEma(iq, vq_eff);
    Flx53_UpdateSettleDetect();

    /* ===== 超速保护 ===== */
    if (s_speed_filt_rpm > g_flx53_rpm_max) {
        g_flx53_state = FOC53_STEP_FAULT_OVRPM;
        Flx53_SetEvt(FOC53_EVT_OVRPM);
        Flx53_StopPwm();
        return;
    }

    /* ===== 状态机 ===== */
    switch (g_flx53_state) {
    case FOC53_STEP_SPINUP:
        if (fabsf(g_flx53_target_rpm - s_speed_filt_rpm) < g_flx53_rpm_tol) {
            g_flx53_state = FOC53_STEP_SETTLE;
            s_state_ms = 0u;
            s_settle_ms_ok = 0u;
        } else if (s_state_ms >= FOC53_SPINUP_TIMEOUT_MS) {
            Flx53_SkipPoint();
        }
        break;

    case FOC53_STEP_SETTLE:
        if (ms_tick_flag != 0u) {
            if ((s_iq_stable != 0u) &&
                (fabsf(g_flx53_target_rpm - s_speed_filt_rpm) < g_flx53_rpm_tol)) {
                s_settle_ms_ok++;
            } else {
                s_settle_ms_ok = 0u;
            }
        }
        if (s_settle_ms_ok >= g_flx53_settle_ms) {
            Flx53_StartAvg();
        } else if (s_state_ms >= FOC53_SETTLE_TIMEOUT_MS) {
            Flx53_SkipPoint();
        }
        break;

    case FOC53_STEP_AVG: {
        uint16_t now = (uint16_t)Timer6_Timebase_GetCounter();
        s_tb_avg_ticks += (uint32_t)(uint16_t)(now - s_tb_avg_prev);  /* 16 位相减天然处理回绕 */
        s_tb_avg_prev = now;
        s_enc_avg_acc += corrected_delta;
        Flx53_AccumSample();
        if (s_avg_cnt >= s_avg_ticks) {
            Flx53_FinishPoint();
        }
        break;
    }

    case FOC53_STEP_RAMPDOWN:
        if ((fabsf(s_speed_filt_rpm) < FOC53_RPM_STOP) ||
            (s_state_ms >= FOC53_RAMPDOWN_TIMEOUT_MS)) {
            Flx53_Finish();
            return;
        }
        break;

    default:
        break;
    }

    /* ===== 输出 ===== */
    cos_r = Foc_Math_Cos(rotor_rad);
    sin_r = Foc_Math_Sin(rotor_rad);
    valpha = vd * cos_r - vq * sin_r;
    vbeta  = vd * sin_r + vq * cos_r;
    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);

    g_foc_valpha = valpha;
    g_foc_vbeta = vbeta;
    g_foc_theta_rad = rotor_rad;
    g_foc_du = du;
    g_foc_dv = dv;
    g_foc_dw = dw;
}

/*===========================================================================
 * 模式自持 VOFA：固定 16ch，单元换算"毫单位"（SendScaled 内部 ×0.001，
 * 即"传 物理量×1000、显示物理量"）；通道含义见 foc_53_flx.h 顶部速览卡。
 *===========================================================================*/
int Foc_FlxId_VofaFill(int32_t *cur)
{
    cur[0]  = (int32_t)(g_flx53_state);                        /* ch0 状态码 */
    cur[1]  = (int32_t)(g_flx53_target_rpm * 1000.0f);         /* ch1 目标转速 (rpm) */
    cur[2]  = (int32_t)(g_flx53_rpm_meas * 1000.0f);           /* ch2 实际转速-PI 反馈 (rpm) */
    cur[3]  = (int32_t)(g_flx53_vq * 1000.0f);                 /* ch3 vq (V) */
    cur[4]  = (int32_t)(g_flx53_iq_filt);                      /* ch4 iq 滤波 (A) */
    cur[5]  = (int32_t)(g_flx53_psi_live_mwb * 1000.0f);       /* ch5 psi_f 实时估计 (mWb) */
    cur[6]  = (int32_t)(g_flx53_psi_mwb * 1000.0f);            /* ch6 psi_f 最终值 (mWb) */
    cur[7]  = (int32_t)(g_flx53_psi_spread_mwb * 1000.0f);     /* ch7 离散度 (mWb) */
    cur[8]  = (int32_t)(g_flx53_pts_done);                     /* ch8 已完成点数 */
    cur[9]  = (int32_t)(g_flx53_r2 * 1000.0f);                 /* ch9 r2 */
    cur[10] = (int32_t)(g_flx53_win_split_pct * 1000.0f);      /* ch10 窗内前后半差 (%) */
    cur[11] = (int32_t)(g_flx53_vdt_used_mv);                  /* ch11 V_dt 用值 (mV -> V) */
    cur[12] = (int32_t)(g_flx53_psi_diff_mwb * 1000.0f);       /* ch12 psi_f 差分校验 (mWb) */
    cur[13] = (int32_t)(g_flx53_ratio * 1000.0f);              /* ch13 psi_f / 手册 */
    cur[14] = (int32_t)(g_flx53_vsat);                         /* ch14 Vsat 0/1 */
    cur[15] = (int32_t)(g_flx53_elapsed_ms * 1000.0f);         /* ch15 累计时间 (ms) */
    return 16;
}
