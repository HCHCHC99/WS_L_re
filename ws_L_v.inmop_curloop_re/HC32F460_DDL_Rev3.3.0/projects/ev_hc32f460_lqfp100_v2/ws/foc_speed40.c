/**
 *******************************************************************************
 * @file  foc_speed40.c
 * @brief FOC mode 40 - cascade speed/current control.
 *
 * The speed PI is decimated to 5 ms.  Its signed mA output is the q-axis
 * current reference.  The id reference remains zero.  The inner id/iq PIs
 * execute on every current-sample ISR with the rotor angle from TMRA_1.
 *******************************************************************************
 */

#include "foc_speed40.h"
#include "foc_dcal24.h"
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "hc32_ll_tmra.h"

#define SPEED40_ISR_DT_US     (1000000u / FOC_ISR_HZ)
#define SPEED40_SPD_WIN_TICKS (SPEED40_SPD_WIN_MS * FOC_ISR_HZ / 1000u)
#define SPEED40_RAD2DEG       57.2958f

volatile uint8_t  g_speed40_running           = 0u;
volatile uint8_t  g_speed40_state             = SPEED40_STEP_IDLE;
volatile uint8_t  g_speed40_evt               = 0u;
volatile float    g_speed40_speed_target_rpm  = 0.0f;
volatile float    g_speed40_speed_ramp_rpm    = 0.0f;
volatile float    g_speed40_speed_meas_rpm    = 0.0f;
volatile float    g_speed40_speed_filt_rpm    = 0.0f;
volatile float    g_speed40_speed_disp_rpm    = 0.0f;   /* 显示专用强滤波转速 */
volatile float    g_speed40_speed_err_rpm     = 0.0f;
volatile float    g_speed40_speed_out_ma      = 0.0f;
volatile int32_t  g_speed40_rotor_count       = 0;
volatile int32_t  g_speed40_rotor_deg         = 0;
volatile float    g_speed40_id_ref_ma         = 0.0f;
volatile float    g_speed40_iq_ref_ma         = 0.0f;
volatile float    g_speed40_id_ma             = 0.0f;
volatile float    g_speed40_iq_ma             = 0.0f;
volatile float    g_speed40_iq_filt_ma        = 0.0f;
volatile float    g_speed40_iq_filt_alpha     = SPEED40_IQ_FILT_ALPHA;
volatile float    g_speed40_vd                = 0.0f;
volatile float    g_speed40_vq                = 0.0f;
volatile uint8_t  g_speed40_vsat              = 0u;
volatile float    g_speed40_du                = 50.0f;
volatile float    g_speed40_dv                = 50.0f;
volatile float    g_speed40_dw                = 50.0f;

pid_config_t g_speed40_pid_speed_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = SPEED40_SPD_KP_MA_PER_RPM,
    .ki           = SPEED40_SPD_KI_MA_PER_RPM_S,
    .kd           = 0.0f,
    .output_min   = -SPEED40_SPD_IQ_LIMIT_MA,
    .output_max   =  SPEED40_SPD_IQ_LIMIT_MA,
    .integral_max =  SPEED40_SPD_IQ_LIMIT_MA,
    .i_term_max   =  SPEED40_SPD_IQ_LIMIT_MA,
    .update_ms    = 0u,
};

pid_config_t g_speed40_pid_id_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = SPEED40_PI_KP,
    .ki           = SPEED40_PI_KI,
    .kd           = 0.0f,
    .output_min   = -SPEED40_PI_UMAX_V,
    .output_max   =  SPEED40_PI_UMAX_V,
    .integral_max =  SPEED40_ITERM_MAX_V,
    .i_term_max   =  SPEED40_ITERM_MAX_V,
    .update_ms    = 0u,
};

pid_config_t g_speed40_pid_iq_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = SPEED40_PI_KP,
    .ki           = SPEED40_PI_KI,
    .kd           = 0.0f,
    .output_min   = -SPEED40_PI_UMAX_V,
    .output_max   =  SPEED40_PI_UMAX_V,
    .integral_max =  SPEED40_ITERM_MAX_V,
    .i_term_max   =  SPEED40_ITERM_MAX_V,
    .update_ms    = 0u,
};

static pid_state_t s_speed_pid;
static pid_state_t s_pid_id;
static pid_state_t s_pid_iq;
static foc_dcal24_result_t s_calibration;

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
static uint8_t  s_speed_disp_init;
static float    s_speed_ramp_rpm;
static float    s_speed_filt_rpm;
static float    s_speed_disp_rpm;
static float    s_speed_out_ma;

static void Speed40_ResetLoopState(void)
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
    s_speed_out_ma = 0.0f;
}

static void Speed40_ClearObservables(void)
{
    g_speed40_evt = 0u;
    g_speed40_speed_ramp_rpm = 0.0f;
    g_speed40_speed_meas_rpm = 0.0f;
    g_speed40_speed_filt_rpm = 0.0f;
    g_speed40_speed_disp_rpm = 0.0f;
    g_speed40_speed_err_rpm = 0.0f;
    g_speed40_speed_out_ma = 0.0f;
    g_speed40_rotor_count = 0;
    g_speed40_rotor_deg = 0;
    g_speed40_id_ref_ma = 0.0f;
    g_speed40_iq_ref_ma = 0.0f;
    g_speed40_id_ma = 0.0f;
    g_speed40_iq_ma = 0.0f;
    g_speed40_iq_filt_ma = 0.0f;
    g_speed40_vd = 0.0f;
    g_speed40_vq = 0.0f;
    g_speed40_vsat = 0u;
    g_speed40_du = 50.0f;
    g_speed40_dv = 50.0f;
    g_speed40_dw = 50.0f;
}

static void Speed40_ClearCurrentFeedback(void)
{
    g_speed40_id_ma = 0.0f;
    g_speed40_iq_ma = 0.0f;
    g_speed40_iq_filt_ma = 0.0f;
    g_speed40_id_ref_ma = 0.0f;
    g_speed40_iq_ref_ma = 0.0f;
    g_speed40_vd = 0.0f;
    g_speed40_vq = 0.0f;
    g_foc_id_ma = 0.0f;
    g_foc_iq_ma = 0.0f;
    g_foc_vd = 0.0f;
    g_foc_vq = 0.0f;
}

static stc_i_data_t Speed40_CorrectedData(const stc_i_data_t *pData)
{
    stc_i_data_t data = *pData;
    data.i16IU_mA = (int16_t)((float)pData->i16IU_mA - s_zero_u_ma);
    data.i16IV_mA = (int16_t)((float)pData->i16IV_mA - s_zero_v_ma);
    data.i16IW_mA = (int16_t)((float)pData->i16IW_mA - s_zero_w_ma);
    return data;
}

static float Speed40_LimitSpeedRPM(float rpm)
{
    if (rpm > SPEED40_SPEED_REF_LIMIT_RPM) {
        rpm = SPEED40_SPEED_REF_LIMIT_RPM;
    }
    if (rpm < -SPEED40_SPEED_REF_LIMIT_RPM) {
        rpm = -SPEED40_SPEED_REF_LIMIT_RPM;
    }
    return rpm;
}

static void Speed40_UpdateSpeed(int32_t corrected_delta)
{
    float raw_rpm;

    s_speed_acc_cnt += corrected_delta;
    if (++s_speed_win_tick < SPEED40_SPD_WIN_TICKS) {
        return;
    }

    raw_rpm = (float)s_speed_acc_cnt * 60.0f
            * (1000.0f / (float)SPEED40_SPD_WIN_MS)
            / (float)ENCODER_CPR;
    if (s_speed_filt_init == 0u) {
        s_speed_filt_rpm = raw_rpm;
        s_speed_filt_init = 1u;
    } else {
        s_speed_filt_rpm += SPEED40_SPD_FILT_ALPHA
                          * (raw_rpm - s_speed_filt_rpm);
    }
    /* 显示专用强滤波（独立于 PI 反馈链，α=0.05 仅平滑曲线） */
    if (s_speed_disp_init == 0u) {
        s_speed_disp_rpm = raw_rpm;
        s_speed_disp_init = 1u;
    } else {
        s_speed_disp_rpm += SPEED40_SPD_DISP_ALPHA
                          * (raw_rpm - s_speed_disp_rpm);
    }

    g_speed40_speed_meas_rpm = raw_rpm;
    g_speed40_speed_filt_rpm = s_speed_filt_rpm;
    g_speed40_speed_disp_rpm = s_speed_disp_rpm;
    s_speed_acc_cnt = 0;
    s_speed_win_tick = 0u;
}

static void Speed40_UpdateOuterLoop(void)
{
    float target_rpm;
    float ramp_step;
    float speed_error;
    float speed_out;

    target_rpm = Speed40_LimitSpeedRPM(g_speed40_speed_target_rpm);
    ramp_step = SPEED40_ACCEL_LIMIT_RPM_S
              * ((float)SPEED40_SPD_WIN_MS * 0.001f);
    if (s_speed_ramp_rpm < target_rpm) {
        s_speed_ramp_rpm += ramp_step;
        if (s_speed_ramp_rpm > target_rpm) s_speed_ramp_rpm = target_rpm;
    } else if (s_speed_ramp_rpm > target_rpm) {
        s_speed_ramp_rpm -= ramp_step;
        if (s_speed_ramp_rpm < target_rpm) s_speed_ramp_rpm = target_rpm;
    }

    speed_error = s_speed_ramp_rpm - s_speed_filt_rpm;
    speed_out = PID_UpdateUs(&s_speed_pid, s_speed_ramp_rpm,
                             s_speed_filt_rpm, SPEED40_SPD_WIN_US);

    g_speed40_speed_ramp_rpm = s_speed_ramp_rpm;
    g_speed40_speed_err_rpm = speed_error;
    g_speed40_speed_out_ma = speed_out;
    g_speed40_iq_ref_ma = speed_out;
    g_speed40_id_ref_ma = 0.0f;
}

void Foc_Speed40_InitPids(void)
{
    PID_Init(&s_speed_pid, &g_speed40_pid_speed_cfg);
    PID_Init(&s_pid_id, &g_speed40_pid_id_cfg);
    PID_Init(&s_pid_iq, &g_speed40_pid_iq_cfg);
}

void Foc_Speed40_SetTargetRPM(float target_rpm)
{
    g_speed40_speed_target_rpm = Speed40_LimitSpeedRPM(target_rpm);
}

void Foc_Speed40_Start(void)
{
    foc_dcal24_result_t calibration;
    uint16_t hardware_count;
    int32_t encoder_dir;

    if (Foc_Dcal24_GetResult(&calibration) == 0u) {
        g_speed40_running = 0u;
        g_speed40_state = SPEED40_STEP_IDLE;
        SPEED40_LOG("ERROR: no mode 24 calibration; run mode 24 first");
        return;
    }

    Foc_Core_ClearFault();
    Speed40_ResetLoopState();
    Speed40_ClearObservables();
    s_calibration = calibration;
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
    g_speed40_rotor_count = s_rotor_count;

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

    g_speed40_speed_target_rpm = 0.0f;
    g_speed40_running = 1u;
    g_speed40_state = SPEED40_STEP_RUN;
    Foc_Core_PwmStart();
    SPEED40_LOG("start rot=%d cnt limit=%.0frpm accel=%.0frpm/s iq=%.0fmA",
                (int)s_rotor_count,
                (double)SPEED40_SPEED_REF_LIMIT_RPM,
                (double)SPEED40_ACCEL_LIMIT_RPM_S,
                (double)SPEED40_SPD_IQ_LIMIT_MA);
}

void Foc_Speed40_Stop(void)
{
    if (g_speed40_running) {
        g_speed40_running = 0u;
        g_foc_align_state = 0u;
        if (g_foc_active) {
            g_foc_active = 0u;
            Foc_Core_PwmStop();
            Foc_Core_SetStateMachine(FOC_STATE_IDLE);
        }
        SPEED40_LOG("stopped");
    }
    Speed40_ClearCurrentFeedback();
}

void Foc_Speed40_Step(const stc_i_data_t *pData)
{
    stc_i_data_t data;
    float id, iq, vd, vq, valpha, vbeta, du, dv, dw;
    float cos_r, sin_r, rotor_rad;
    uint16_t hardware_count;
    int32_t hardware_delta, corrected_delta, rotor_deg;

    if (Foc_Core_OverCurrent(pData)) {
        g_speed40_state = SPEED40_STEP_FAULT_OC;
        g_speed40_evt = SPEED40_EVT_OC;
        g_speed40_running = 0u;
        Foc_Core_FaultStop(1u);
        Speed40_ClearCurrentFeedback();
        return;
    }

    if (g_speed40_state != SPEED40_STEP_RUN) {
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
    if (hardware_delta > SPEED40_ENC_DELTA_MAX) {
        hardware_delta = SPEED40_ENC_DELTA_MAX;
    }
    if (hardware_delta < -SPEED40_ENC_DELTA_MAX) {
        hardware_delta = -SPEED40_ENC_DELTA_MAX;
    }
    corrected_delta = hardware_delta * (int32_t)g_foc_enc_dir;
    s_rotor_count = Foc_Core_ModPos(s_rotor_count + corrected_delta,
                                    (int32_t)ENCODER_CPR);
    g_speed40_rotor_count = s_rotor_count;
    Speed40_UpdateSpeed(corrected_delta);
    Speed40_UpdateOuterLoop();

    rotor_rad = (float)s_rotor_count
              * (FOC_MATH_2PI * (float)FOC_POLE_PAIRS / (float)ENCODER_CPR);
    rotor_rad -= (float)((int32_t)(rotor_rad * (1.0f / FOC_MATH_2PI)))
               * FOC_MATH_2PI;
    if (rotor_rad < 0.0f) rotor_rad += FOC_MATH_2PI;

    data = Speed40_CorrectedData(pData);
    Foc_Core_GetDq(&data, rotor_rad, &id, &iq);
    g_speed40_id_ma = id * 1000.0f;
    g_speed40_iq_ma = iq * 1000.0f;
    {
        float alpha = g_speed40_iq_filt_alpha;
        if (alpha <= 0.0f || alpha > 1.0f) {
            alpha = 1.0f;
        }
        if (s_iq_filt_init == 0u) {
            g_speed40_iq_filt_ma = g_speed40_iq_ma;
            s_iq_filt_init = 1u;
        } else {
            g_speed40_iq_filt_ma += alpha
                                    * (g_speed40_iq_ma - g_speed40_iq_filt_ma);
        }
    }
    g_foc_id_ma = g_speed40_id_ma;
    g_foc_iq_ma = g_speed40_iq_ma;

    vd = PID_UpdateUs(&s_pid_id, g_speed40_id_ref_ma * 0.001f, id,
                      SPEED40_ISR_DT_US);
    vq = PID_UpdateUs(&s_pid_iq, g_speed40_iq_ref_ma * 0.001f, iq,
                      SPEED40_ISR_DT_US);
    g_foc_vd = vd;
    g_foc_vq = vq;
    g_speed40_vd = vd;
    g_speed40_vq = vq;
    g_speed40_vsat = ((vd <= -SPEED40_PI_UMAX_V + 0.01f) ||
                      (vd >=  SPEED40_PI_UMAX_V - 0.01f) ||
                      (vq <= -SPEED40_PI_UMAX_V + 0.01f) ||
                      (vq >=  SPEED40_PI_UMAX_V - 0.01f)) ? 1u : 0u;

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
    g_speed40_du = du;
    g_speed40_dv = dv;
    g_speed40_dw = dw;

    rotor_deg = (int32_t)(rotor_rad * SPEED40_RAD2DEG);
    if (rotor_deg >= 360) rotor_deg -= 360;
    g_speed40_rotor_deg = rotor_deg;
}
