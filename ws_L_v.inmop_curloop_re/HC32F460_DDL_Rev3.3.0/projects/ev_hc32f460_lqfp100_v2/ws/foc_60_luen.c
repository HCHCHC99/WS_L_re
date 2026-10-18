/**
 *******************************************************************************
 * @file  foc_60_luen.c
 * @brief FOC mode 60 - 龙伯格观测器无感速度/电流双闭环（编码器角作裁判）。
 *
 * 与 mode 45（foc_45_smo.c）**同源**，唯一区别是观测器内核：
 *   mode 45：滑模观测器 foc_smo.c（z = k·sat，e_hat = LPF(z)）
 *   mode 60：龙伯格观测器 foc_luen.c（ê 为观测器状态，极点配置到 −Ωo）
 * 角度/转速提取两模式共用 foc_pll.c，便于直接 A/B 对比。
 *
 * 有感段：编码器转子角做 Park/电流环，观测器纯旁观；
 * 无感段：g_luen60_sensorless=1 且 g_luen_emf_ok=1 锁存后，Park 角换
 *         g_pll_theta_park_deg、速度反馈换 g_pll_omega_out_rpm。
 *******************************************************************************
 */

#include "foc_60_luen.h"
#include "foc_luen.h"
#include "foc_pll.h"
#include "foc_24_dcal.h"
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "hc32_ll_tmra.h"
#include <math.h>   /* sqrtf */

#define FOC60_ISR_DT_US     (1000000u / FOC_ISR_HZ)
#define FOC60_SPD_WIN_TICKS (FOC60_SPD_WIN_MS * FOC_ISR_HZ / 1000u)
#define FOC60_RAD2DEG       57.2958f

volatile uint8_t  g_luen60_running          = 0u;
volatile uint8_t  g_luen60_state            = FOC60_STEP_IDLE;
volatile uint8_t  g_luen60_evt              = 0u;
volatile uint8_t  g_luen60_auto_ramp        = FOC60_AUTO_RAMP_DEFAULT;
volatile uint8_t  g_luen60_wave_mode        = 0u;
volatile uint8_t  g_luen60_sensorless       = 0u;   /* 无感切换开关（0=有感） */
volatile float    g_luen60_ang_lead_ticks   = 1.5f; /* 高速角度超前补偿拍数（0=关） */
volatile float    g_luen60_speed_target_rpm = 0.0f;
volatile float    g_luen60_speed_ramp_rpm   = 0.0f;
volatile float    g_luen60_speed_meas_rpm   = 0.0f;
volatile float    g_luen60_speed_filt_rpm   = 0.0f;
volatile float    g_luen60_speed_disp_rpm   = 0.0f;   /* 显示专用强滤波转速 */
volatile float    g_luen60_speed_err_rpm    = 0.0f;
volatile float    g_luen60_speed_out_ma     = 0.0f;
volatile int32_t  g_luen60_rotor_count      = 0;
volatile int32_t  g_luen60_rotor_deg        = 0;
volatile float    g_luen60_id_ref_ma        = 0.0f;
volatile float    g_luen60_iq_ref_ma        = 0.0f;
volatile float    g_luen60_id_ma            = 0.0f;
volatile float    g_luen60_iq_ma            = 0.0f;
volatile float    g_luen60_iq_filt_ma       = 0.0f;
volatile float    g_luen60_iq_filt_alpha    = FOC60_IQ_FILT_ALPHA;
volatile float    g_luen60_vd               = 0.0f;
volatile float    g_luen60_vq               = 0.0f;
volatile uint8_t  g_luen60_vsat             = 0u;
volatile float    g_luen60_du               = 50.0f;
volatile float    g_luen60_dv               = 50.0f;
volatile float    g_luen60_dw               = 50.0f;

/* 无感锁存状态（全局供 Watch/打印观察）：开关置 1 且 emf_ok=1 时进入，
 * 进入后不因 emf_ok 瞬时掉 0 跌回有感（防角度源来回跳变）；关开关/停机退出 */
volatile uint8_t  g_luen60_sl_active        = 0u;

pid_config_t g_luen60_pid_speed_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = FOC60_SPD_KP_MA_PER_RPM,
    .ki           = FOC60_SPD_KI_MA_PER_RPM_S,
    .kd           = 0.0f,
    .output_min   = -FOC60_SPD_IQ_LIMIT_MA,
    .output_max   =  FOC60_SPD_IQ_LIMIT_MA,
    .integral_max =  FOC60_SPD_IQ_LIMIT_MA,
    .i_term_max   =  FOC60_SPD_IQ_LIMIT_MA,
    .update_ms    = 0u,
};

pid_config_t g_luen60_pid_id_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = FOC60_PI_KP,
    .ki           = FOC60_PI_KI,
    .kd           = 0.0f,
    .output_min   = -FOC60_PI_UMAX_V,
    .output_max   =  FOC60_PI_UMAX_V,
    .integral_max =  FOC60_ITERM_MAX_V,
    .i_term_max   =  FOC60_ITERM_MAX_V,
    .update_ms    = 0u,
};

pid_config_t g_luen60_pid_iq_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = FOC60_PI_KP,
    .ki           = FOC60_PI_KI,
    .kd           = 0.0f,
    .output_min   = -FOC60_PI_UMAX_V,
    .output_max   =  FOC60_PI_UMAX_V,
    .integral_max =  FOC60_ITERM_MAX_V,
    .i_term_max   =  FOC60_ITERM_MAX_V,
    .update_ms    = 0u,
};

static pid_state_t s_speed_pid;
static pid_state_t s_pid_id;
static pid_state_t s_pid_iq;

static int32_t  s_rotor_count;
static uint16_t s_encoder_prev_hw;
static uint8_t  s_encoder_initialized;
static float    s_zero_u_ma;
static float    s_zero_v_ma;
static float    s_zero_w_ma;
static uint8_t  s_iq_filt_init;

static int32_t  s_speed_acc_cnt;
static uint32_t s_speed_win_tick;
static uint8_t  s_speed_filt_init;
static float    s_speed_ramp_rpm;
static float    s_speed_filt_rpm;
static uint8_t  s_speed_disp_init;
static float    s_speed_disp_rpm;

/* 启动自动转速 profile：秒 0/1/2/3 -> 0/200/500/1000 rpm，之后保持 1000
 * （无感首切定在 1000rpm：PLL 信噪比窗口内、打摆扰动更小） */
static uint32_t s_run_ticks;
static const uint16_t s_auto_rpm[FOC60_AUTO_RAMP_SECS + 1u] = {
    0u, 200u, 500u, 1000u
};

/* 观测器电压输入缓冲：采样时刻实际施加的是上一拍指令电压 */
static float s_v_alpha_prev;
static float s_v_beta_prev;

static void Luen60_ResetLoopState(void)
{
    s_rotor_count = 0;
    s_encoder_prev_hw = 0u;
    s_encoder_initialized = 0u;
    s_zero_u_ma = 0.0f;
    s_zero_v_ma = 0.0f;
    s_zero_w_ma = 0.0f;
    s_iq_filt_init = 0u;
    s_speed_acc_cnt = 0;
    s_speed_win_tick = 0u;
    s_speed_filt_init = 0u;
    s_speed_ramp_rpm = 0.0f;
    s_speed_filt_rpm = 0.0f;
    s_speed_disp_rpm = 0.0f;
    s_speed_disp_init = 0u;
    s_run_ticks = 0u;
    s_v_alpha_prev = 0.0f;
    s_v_beta_prev = 0.0f;
}

static void Luen60_ClearObservables(void)
{
    g_luen60_evt = 0u;
    g_luen60_speed_ramp_rpm = 0.0f;
    g_luen60_speed_meas_rpm = 0.0f;
    g_luen60_speed_filt_rpm = 0.0f;
    g_luen60_speed_disp_rpm = 0.0f;
    g_luen60_speed_err_rpm = 0.0f;
    g_luen60_speed_out_ma = 0.0f;
    g_luen60_rotor_count = 0;
    g_luen60_rotor_deg = 0;
    g_luen60_id_ref_ma = 0.0f;
    g_luen60_iq_ref_ma = 0.0f;
    g_luen60_id_ma = 0.0f;
    g_luen60_iq_ma = 0.0f;
    g_luen60_iq_filt_ma = 0.0f;
    g_luen60_vd = 0.0f;
    g_luen60_vq = 0.0f;
    g_luen60_vsat = 0u;
    g_luen60_du = 50.0f;
    g_luen60_dv = 50.0f;
    g_luen60_dw = 50.0f;
}

static void Luen60_ClearCurrentFeedback(void)
{
    g_luen60_id_ma = 0.0f;
    g_luen60_iq_ma = 0.0f;
    g_luen60_iq_filt_ma = 0.0f;
    g_luen60_id_ref_ma = 0.0f;
    g_luen60_iq_ref_ma = 0.0f;
    g_luen60_vd = 0.0f;
    g_luen60_vq = 0.0f;
    g_foc_id_ma = 0.0f;
    g_foc_iq_ma = 0.0f;
    g_foc_vd = 0.0f;
    g_foc_vq = 0.0f;
}

static stc_i_data_t Luen60_CorrectedData(const stc_i_data_t *pData)
{
    stc_i_data_t data = *pData;
    data.i16IU_mA = (int16_t)((float)pData->i16IU_mA - s_zero_u_ma);
    data.i16IV_mA = (int16_t)((float)pData->i16IV_mA - s_zero_v_ma);
    data.i16IW_mA = (int16_t)((float)pData->i16IW_mA - s_zero_w_ma);
    return data;
}

static float Luen60_LimitSpeedRPM(float rpm)
{
    if (rpm > FOC60_SPEED_REF_LIMIT_RPM) {
        rpm = FOC60_SPEED_REF_LIMIT_RPM;
    }
    if (rpm < -FOC60_SPEED_REF_LIMIT_RPM) {
        rpm = -FOC60_SPEED_REF_LIMIT_RPM;
    }
    return rpm;
}

/* 启动自动转速 profile：前 4s（秒 0/1/2/3 -> 0/200/500/1000 rpm）每拍写目标，
 * 之后停止写入（Watch 接管）。表长 = FOC60_AUTO_RAMP_SECS + 1。 */
static void Luen60_AutoRamp(void)
{
    uint32_t sec;

    if (g_luen60_auto_ramp == 0u) {
        return;
    }
    sec = s_run_ticks / (uint32_t)FOC_ISR_HZ;
    if (sec <= FOC60_AUTO_RAMP_SECS) {
        g_luen60_speed_target_rpm = (float)s_auto_rpm[sec];
    }
}

static void Luen60_UpdateSpeed(int32_t corrected_delta)
{
    float raw_rpm;

    s_speed_acc_cnt += corrected_delta;
    if (++s_speed_win_tick < FOC60_SPD_WIN_TICKS) {
        return;
    }

    raw_rpm = (float)s_speed_acc_cnt * 60.0f
            * (1000.0f / (float)FOC60_SPD_WIN_MS)
            / (float)ENCODER_CPR;
    if (s_speed_filt_init == 0u) {
        s_speed_filt_rpm = raw_rpm;
        s_speed_filt_init = 1u;
    } else {
        s_speed_filt_rpm += FOC60_SPD_FILT_ALPHA
                          * (raw_rpm - s_speed_filt_rpm);
    }
    /* 显示专用强滤波（独立于 PI 反馈链，alpha=0.05 仅平滑曲线） */
    if (s_speed_disp_init == 0u) {
        s_speed_disp_rpm = raw_rpm;
        s_speed_disp_init = 1u;
    } else {
        s_speed_disp_rpm += FOC60_SPD_DISP_ALPHA
                          * (raw_rpm - s_speed_disp_rpm);
    }

    g_luen60_speed_meas_rpm = raw_rpm;
    g_luen60_speed_filt_rpm = s_speed_filt_rpm;
    g_luen60_speed_disp_rpm = s_speed_disp_rpm;
    s_speed_acc_cnt = 0;
    s_speed_win_tick = 0u;
}

static void Luen60_UpdateOuterLoop(void)
{
    float target_rpm;
    float ramp_step;
    float speed_error;
    float speed_out;

    target_rpm = Luen60_LimitSpeedRPM(g_luen60_speed_target_rpm);
    ramp_step = FOC60_ACCEL_LIMIT_RPM_S
              * ((float)FOC60_SPD_WIN_MS * 0.001f);
    if (s_speed_ramp_rpm < target_rpm) {
        s_speed_ramp_rpm += ramp_step;
        if (s_speed_ramp_rpm > target_rpm) s_speed_ramp_rpm = target_rpm;
    } else if (s_speed_ramp_rpm > target_rpm) {
        s_speed_ramp_rpm -= ramp_step;
        if (s_speed_ramp_rpm < target_rpm) s_speed_ramp_rpm = target_rpm;
    }

    speed_error = s_speed_ramp_rpm - s_speed_filt_rpm;
    /* 无感锁存时速度反馈换 PLL 低通转速估计（其余逻辑不变） */
    {
        float speed_fb = s_speed_filt_rpm;

        if (g_luen60_sensorless != 0u && g_luen_emf_ok != 0u) {
            g_luen60_sl_active = 1u;
        }
        if (g_luen60_sensorless == 0u) {
            g_luen60_sl_active = 0u;
        }
        if (g_luen60_sl_active != 0u) {
            speed_fb = g_pll_omega_out_rpm;
        }
        speed_error = s_speed_ramp_rpm - speed_fb;
        speed_out = PID_UpdateUs(&s_speed_pid, s_speed_ramp_rpm,
                                 speed_fb, FOC60_SPD_WIN_US);
    }

    g_luen60_speed_ramp_rpm = s_speed_ramp_rpm;
    g_luen60_speed_err_rpm = speed_error;
    g_luen60_speed_out_ma = speed_out;
    g_luen60_iq_ref_ma = speed_out;
    g_luen60_id_ref_ma = 0.0f;
}

void Foc_Luen60_InitPids(void)
{
    PID_Init(&s_speed_pid, &g_luen60_pid_speed_cfg);
    PID_Init(&s_pid_id, &g_luen60_pid_id_cfg);
    PID_Init(&s_pid_iq, &g_luen60_pid_iq_cfg);
}

void Foc_Luen60_SetTargetRPM(float target_rpm)
{
    g_luen60_speed_target_rpm = Luen60_LimitSpeedRPM(target_rpm);
}

void Foc_Luen60_Start(void)
{
    foc_dcal24_result_t calibration;
    uint16_t hardware_count;
    int32_t encoder_dir;

    if (Foc_Dcal_GetResult(&calibration) == 0u) {
        g_luen60_running = 0u;
        g_luen60_state = FOC60_STEP_IDLE;
        FOC60_LOG("ERROR: no mode 24 calibration; run mode 24 first");
        return;
    }

    Foc_Core_ClearFault();
    Luen60_ResetLoopState();
    Luen60_ClearObservables();
    Foc_Luen_Reset();   /* 龙伯格观测器状态清零（î/ê/诊断/判定） */
    Foc_Pll_Reset();    /* PLL 旁观者状态清零（θ̂/ω̂/诊断） */
    g_luen60_sl_active = 0u;  /* 无感锁存退出（每次 Start 从有感闭环起步） */
    s_zero_u_ma = calibration.zero_u_ma;
    s_zero_v_ma = calibration.zero_v_ma;
    s_zero_w_ma = calibration.zero_w_ma;

    hardware_count = TMRA_GetCountValue(CM_TMRA_1);
    encoder_dir = (int32_t)g_foc_enc_dir;
    s_rotor_count = Foc_Core_ModPos((int32_t)hardware_count * encoder_dir
                                    - calibration.offset,
                                    (int32_t)ENCODER_CPR);
    s_encoder_prev_hw = hardware_count;
    s_encoder_initialized = 1u;
    g_luen60_rotor_count = s_rotor_count;

    g_foc_mode      = FOC_MODE_ALIGN;
    g_foc_phase     = 4u;
    g_foc_theta_rad = FOC_MATH_HALF_PI;
    g_foc_id_ma     = 0.0f;
    g_foc_iq_ma     = 0.0f;
    g_foc_vd        = 0.0f;
    g_foc_vq        = 0.0f;
    Foc_Core_SetStateMachine(FOC_STATE_ALIGN);
    Foc_Core_ResetEma();
    g_foc_align_state = 1u;
    PID_Reset(&s_speed_pid);
    PID_Reset(&s_pid_id);
    PID_Reset(&s_pid_iq);

    g_luen60_speed_target_rpm = 0.0f;
    g_luen60_running = 1u;
    g_luen60_state = FOC60_STEP_RUN;
    Foc_Core_PwmStart();
    FOC60_LOG("start rot=%d cnt limit=%.0frpm accel=%.0frpm/s iq=%.0fmA auto=%u",
              (int)s_rotor_count,
              (double)FOC60_SPEED_REF_LIMIT_RPM,
              (double)FOC60_ACCEL_LIMIT_RPM_S,
              (double)FOC60_SPD_IQ_LIMIT_MA,
              (unsigned)g_luen60_auto_ramp);
}

void Foc_Luen60_Stop(void)
{
    if (g_luen60_running) {
        g_luen60_running = 0u;
        g_foc_align_state = 0u;
        if (g_foc_active) {
            g_foc_active = 0u;
            Foc_Core_PwmStop();
            Foc_Core_SetStateMachine(FOC_STATE_IDLE);
        }
        FOC60_LOG("stopped");
    }
    g_luen60_sl_active = 0u;   /* 停机退出无感锁存（下次 Start 从有感起步） */
    Luen60_ClearCurrentFeedback();
}

void Foc_Luen60_Step(const stc_i_data_t *pData)
{
    stc_i_data_t data;
    float id, iq, vd, vq, valpha, vbeta, du, dv, dw;
    float cos_r, sin_r, rotor_rad, rotor_use;
    float ialpha_s, ibeta_s;   /* 观测器输入电流（Clarke 后，跨块传递到 SVPWM 后调用点） */
    uint16_t hardware_count;
    int32_t hardware_delta, corrected_delta, rotor_deg;

    if (Foc_Core_OverCurrent(pData)) {
        g_luen60_state = FOC60_STEP_FAULT_OC;
        g_luen60_evt = FOC60_EVT_OC;
        g_luen60_running = 0u;
        Foc_Core_FaultStop(1u);
        Luen60_ClearCurrentFeedback();
        return;
    }

    if (g_luen60_state != FOC60_STEP_RUN) {
        return;
    }

    hardware_count = TMRA_GetCountValue(CM_TMRA_1);
    if (s_encoder_initialized == 0u) {
        s_encoder_prev_hw = hardware_count;
        s_encoder_initialized = 1u;
    }
    hardware_delta = (int32_t)(int16_t)((uint16_t)hardware_count
                                        - s_encoder_prev_hw);
    s_encoder_prev_hw = hardware_count;
    if (hardware_delta > FOC60_ENC_DELTA_MAX) {
        hardware_delta = FOC60_ENC_DELTA_MAX;
    }
    if (hardware_delta < -FOC60_ENC_DELTA_MAX) {
        hardware_delta = -FOC60_ENC_DELTA_MAX;
    }
    corrected_delta = hardware_delta * (int32_t)g_foc_enc_dir;
    s_rotor_count = Foc_Core_ModPos(s_rotor_count + corrected_delta,
                                    (int32_t)ENCODER_CPR);
    g_luen60_rotor_count = s_rotor_count;
    s_run_ticks++;
    Luen60_AutoRamp();
    Luen60_UpdateSpeed(corrected_delta);
    Luen60_UpdateOuterLoop();

    rotor_rad = (float)s_rotor_count
              * (FOC_MATH_2PI * (float)FOC_POLE_PAIRS / (float)ENCODER_CPR);
    rotor_rad -= (float)((int32_t)(rotor_rad * (1.0f / FOC_MATH_2PI)))
               * FOC_MATH_2PI;
    if (rotor_rad < 0.0f) rotor_rad += FOC_MATH_2PI;

    /* 高速角度超前补偿：Park 角要对齐"电压矢量作用中心"= 采样后 ~1.5 拍。
     * g_luen60_ang_lead_ticks Watch 可调（0=关，1.5=默认），低速无影响。 */
    if (g_luen60_ang_lead_ticks > 0.0f && g_luen60_sl_active == 0u) {
        float w_e = g_luen60_speed_filt_rpm * 0.10472f * (float)FOC_POLE_PAIRS;
        rotor_rad += w_e * FOC60_ISR_DT_US * 1.0e-6f * g_luen60_ang_lead_ticks;
        rotor_rad -= (float)((int32_t)(rotor_rad * (1.0f / FOC_MATH_2PI)))
                   * FOC_MATH_2PI;
        if (rotor_rad < 0.0f) rotor_rad += FOC_MATH_2PI;
    }

    /* 无感锁存时 Park 角换 PLL 补偿外推角 g_pll_theta_park_deg
     *（含滞后补偿+一拍外推）。编码器角 rotor_rad 保留作裁判。 */
    rotor_use = rotor_rad;
    if (g_luen60_sensorless != 0u && g_luen_emf_ok != 0u) {
        g_luen60_sl_active = 1u;
    }
    if (g_luen60_sensorless == 0u) {
        g_luen60_sl_active = 0u;
    }
    if (g_luen60_sl_active != 0u) {
        rotor_use = g_pll_theta_park_deg * (FOC_MATH_2PI / 360.0f);
    }

    data = Luen60_CorrectedData(pData);
    Foc_Core_GetDq(&data, rotor_use, &id, &iq);
    g_luen60_id_ma = id * 1000.0f;
    g_luen60_iq_ma = iq * 1000.0f;
    {
        float alpha = g_luen60_iq_filt_alpha;
        if (alpha <= 0.0f || alpha > 1.0f) {
            alpha = 1.0f;
        }
        if (s_iq_filt_init == 0u) {
            g_luen60_iq_filt_ma = g_luen60_iq_ma;
            s_iq_filt_init = 1u;
        } else {
            g_luen60_iq_filt_ma += alpha
                                 * (g_luen60_iq_ma - g_luen60_iq_filt_ma);
        }
    }
    g_foc_id_ma = g_luen60_id_ma;
    g_foc_iq_ma = g_luen60_iq_ma;

    /* 观测器输入电流：与控制路径完全同源（sign × 零偏校正副本 -> Foc_Clarke）。
     * 电压在下方 SVPWM 之后取"前后两拍指令平均"再喂观测器 */
    {
        float sgn = (float)g_foc_cur_sign;
        float ia_s = (float)data.i16IU_mA * 0.001f * sgn;
        float ib_s = (float)data.i16IV_mA * 0.001f * sgn;
        float ic_s = (float)data.i16IW_mA * 0.001f * sgn;
        Foc_Clarke(ia_s, ib_s, ic_s, &ialpha_s, &ibeta_s);
    }

    vd = PID_UpdateUs(&s_pid_id, g_luen60_id_ref_ma * 0.001f, id,
                      FOC60_ISR_DT_US);
    vq = PID_UpdateUs(&s_pid_iq, g_luen60_iq_ref_ma * 0.001f, iq,
                      FOC60_ISR_DT_US);
    g_foc_vd = vd;
    g_foc_vq = vq;
    g_luen60_vd = vd;
    g_luen60_vq = vq;
    g_luen60_vsat = ((vd <= -FOC60_PI_UMAX_V + 0.01f) ||
                     (vd >=  FOC60_PI_UMAX_V - 0.01f) ||
                     (vq <= -FOC60_PI_UMAX_V + 0.01f) ||
                     (vq >=  FOC60_PI_UMAX_V - 0.01f)) ? 1u : 0u;

    cos_r = Foc_Math_Cos(rotor_use);
    sin_r = Foc_Math_Sin(rotor_use);
    valpha = vd * cos_r - vq * sin_r;
    vbeta  = vd * sin_r + vq * cos_r;
    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);

    g_foc_valpha = valpha;
    g_foc_vbeta = vbeta;
    /* 观测器调用：中心对齐 PWM + 谷点采样下，采样区间 [t_k, t_k+1] 上实际施加的
     * 电压 = 前后两拍指令各占一半，喂平均值。ω̂ 回灌 PLL 的电角速度估计
     * （带符号，滞后一拍；启动静止阶段为 0，ê 由电流误差注入自行起振）。 */
    Foc_Luen_Step((s_v_alpha_prev + valpha) * 0.5f,
                  (s_v_beta_prev + vbeta) * 0.5f,
                  ialpha_s, ibeta_s,
                  g_pll_omega_hat_rad_s);
    /* PLL：消费龙伯格滤波后 ê（foc_pll 与 mode 45 共用）；
     * 紧跟同拍角度比较（θ̂ 每拍转 ~9°电角，跨拍比较会撕裂出 ±9° 假差） */
    Foc_Pll_Step(g_luen_e_alpha_hat_v, g_luen_e_beta_hat_v);
    Foc_Pll_Compare(rotor_rad * FOC60_RAD2DEG);
    s_v_alpha_prev = valpha;   /* 缓冲本拍指令，下一拍参与平均 */
    s_v_beta_prev = vbeta;
    g_foc_theta_rad = rotor_rad;
    g_foc_du = du;
    g_foc_dv = dv;
    g_foc_dw = dw;
    g_luen60_du = du;
    g_luen60_dv = dv;
    g_luen60_dw = dw;

    rotor_deg = (int32_t)(rotor_rad * FOC60_RAD2DEG);
    if (rotor_deg >= 360) rotor_deg -= 360;
    g_luen60_rotor_deg = rotor_deg;
}

/*===========================================================================
 * 模式自持 VOFA：三种布局由 g_luen60_wave_mode 选择，
 * 通道含义见 foc_60_luen.h 顶部速览卡（唯一事实源）。
 *===========================================================================*/
int Foc_Luen60_VofaFill(int32_t *cur)
{
    if (g_luen60_wave_mode == 2u) {
        /*===== wave_mode=2：PLL/无感验收面板（16ch）=====*/
        float luen_emag = sqrtf(g_luen_e_alpha_hat_v * g_luen_e_alpha_hat_v
                              + g_luen_e_beta_hat_v * g_luen_e_beta_hat_v);

        cur[0]  = (int32_t)(g_luen60_speed_meas_rpm * 1000.0f);   /* mrpm -> rpm */
        cur[1]  = (int32_t)(g_luen60_speed_filt_rpm * 1000.0f);   /* mrpm -> rpm */
        cur[2]  = (int32_t)(g_pll_omega_hat_rpm * 1000.0f);       /* mrpm -> rpm */
        cur[3]  = (int32_t)(g_pll_omega_out_rpm * 1000.0f);       /* mrpm -> rpm */
        cur[4]  = (int32_t)((g_pll_omega_hat_rpm
                           - g_luen60_speed_meas_rpm) * 1000.0f); /* mrpm -> rpm */
        cur[5]  = (int32_t)((g_pll_omega_out_rpm
                           - g_luen60_speed_filt_rpm) * 1000.0f); /* mrpm -> rpm */
        cur[6]  = (int32_t)(g_pll_diag_theta_err_deg * 1000.0f);  /* mdeg -> deg */
        cur[7]  = (int32_t)(g_pll_diag_theta_err_filt_deg * 1000.0f);
        cur[8]  = (int32_t)(g_luen60_iq_filt_ma);                 /* mA -> A */
        cur[9]  = (int32_t)(g_pll_comp_deg_out * 1000.0f);        /* mdeg -> deg */
        cur[10] = (int32_t)(g_luen60_sl_active * 1000.0f);        /* 0/1 */
        cur[11] = (int32_t)(luen_emag * 1000.0f);                 /* mV -> V */
        cur[12] = (int32_t)(g_pll_diag_smo_err_deg * 1000.0f);    /* mdeg -> deg */
        cur[13] = (int32_t)(g_luen60_speed_target_rpm * 1000.0f); /* mrpm -> rpm */
        cur[14] = (int32_t)(g_luen60_speed_disp_rpm * 1000.0f);   /* 显示滤波转速 */
        cur[15] = (int32_t)(g_luen60_speed_ramp_rpm * 1000.0f);   /* 斜坡输出转速 */
        return 16;
    } else if (g_luen60_wave_mode != 0u) {
        /*===== wave_mode=1：波形窄帧（8ch=32B）=====*/
        cur[0] = (int32_t)(g_luen_e_alpha_hat_v * 1000.0f);  /* mV -> V */
        cur[1] = (int32_t)(g_luen_e_beta_hat_v * 1000.0f);   /* mV -> V */
        cur[2] = (int32_t)(g_luen_theory_alpha_v * 1000.0f); /* mV -> V */
        cur[3] = (int32_t)(g_luen_theory_beta_v * 1000.0f);  /* mV -> V */
        cur[4] = (int32_t)(g_luen_diag_theta_err_deg * 1000.0f);      /* mdeg -> deg */
        cur[5] = (int32_t)(g_luen_diag_theta_err_filt_deg * 1000.0f); /* mdeg -> deg */
        cur[6] = (int32_t)(g_luen_ih_alpha_a * 1000.0f);     /* mA -> A */
        cur[7] = (int32_t)(g_luen_ih_beta_a * 1000.0f);      /* mA -> A */
        return 8;
    } else {
        /*===== wave_mode=0：观测器验收面板（16ch）=====*/
        cur[0]  = (int32_t)(g_luen60_iq_ma);                    /* ch0 iq 反馈 (mA -> A) */
        cur[1]  = (int32_t)(g_luen60_iq_filt_ma);               /* ch1 滤波后 iq (mA -> A) */
        cur[2]  = (int32_t)(g_luen60_iq_ref_ma);                /* ch2 目标 iq (mA -> A) */
        cur[3]  = (int32_t)(g_luen_theory_alpha_v * 1000.0f);   /* ch3 理论 e_alpha (mV -> V) */
        cur[4]  = (int32_t)(g_luen_theory_beta_v * 1000.0f);    /* ch4 理论 e_beta (mV -> V) */
        cur[5]  = (int32_t)(g_luen_ih_alpha_a * 1000.0f);       /* ch5 观测器 î_alpha (mA -> A) */
        cur[6]  = (int32_t)(g_luen_ih_beta_a * 1000.0f);        /* ch6 观测器 î_beta (mA -> A) */
        cur[7]  = (int32_t)(g_luen_e_alpha_hat_v * 1000.0f);    /* ch7 ê_alpha (mV -> V) */
        cur[8]  = (int32_t)(g_luen_e_beta_hat_v * 1000.0f);     /* ch8 ê_beta (mV -> V) */
        cur[9]  = (int32_t)(g_luen_diag_theta_err_deg * 1000.0f);/* ch9 相位误差 (mdeg -> deg) */
        cur[10] = (int32_t)(g_luen_diag_e_on_q_v * 1000.0f);    /* ch10 e_on_q 投影 (mV -> V) */
        cur[11] = (int32_t)(g_luen_diag_e_on_d_v * 1000.0f);    /* ch11 e_on_d 投影 (mV -> V) */
        cur[12] = (int32_t)(g_luen_diag_e_err_v * 1000.0f);     /* ch12 总误差范数 (mV -> V) */
        cur[13] = (int32_t)(g_luen_diag_e_expect_v * 1000.0f);  /* ch13 理论幅值 ωψf (mV -> V) */
        cur[14] = (int32_t)(g_luen60_speed_filt_rpm * 1000.0f); /* ch14 实际转速 (mrpm -> rpm) */
        cur[15] = (int32_t)(g_luen_diag_theta_err_filt_deg * 1000.0f); /* ch15 相位差滤波 (mdeg -> deg) */
        return 16;
    }
}