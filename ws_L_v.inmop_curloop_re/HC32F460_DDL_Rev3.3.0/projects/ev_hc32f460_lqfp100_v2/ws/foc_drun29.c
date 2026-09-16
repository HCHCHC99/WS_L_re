/**
 *******************************************************************************
 * @file  foc_drun29.c
 * @brief FOC mode 29 - independent current-loop state machine.
 *
 * The mode takes only a value copy of the mode 24 calibration snapshot.  At
 * Start it reads the live TMRA_1 count, converts it to the current rotor
 * angle, and integrates all later motion in a private frame.
 *******************************************************************************
 */

#include "foc_drun29.h"
#include "foc_dcal24.h"
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "timer6_timebase.h"
#include "hc32_ll_tmra.h"

#define DRUN29_ISR_DT_US  (1000000u / FOC_ISR_HZ)

#define DRUN29_SPEED_WIN_TICKS  ((uint32_t)DRUN29_SPEED_WIN_MS * FOC_ISR_HZ / 1000u)
#define DRUN29_PP_WIN_TICKS     ((uint32_t)DRUN29_PP_WIN_MS * FOC_ISR_HZ / 1000u)
#define DRUN29_MEAN_WIN_TICKS   ((uint32_t)DRUN29_MEAN_WIN_MS * FOC_ISR_HZ / 1000u)
#define DRUN29_ERR_WIN_TICKS    ((uint32_t)DRUN29_ERR_WIN_MS * FOC_ISR_HZ / 1000u)

#define DRUN29_RAD2DEG  57.2958f
#define DRUN29_DEG2RAD  0.0174533f

volatile uint8_t  g_drun29_running       = 0u;
volatile uint8_t  g_drun29_state         = DRUN29_STEP_IDLE;
volatile uint8_t  g_drun29_evt           = 0u;
volatile float    g_drun29_dlt_init_deg  = 90.0f;
volatile float    g_drun29_dlt_targ_deg  = 90.0f;
volatile uint32_t g_drun29_dlt_tr_ms     = 0u;
volatile float    g_drun29_dlt_now_deg   = 0.0f;
volatile float    g_drun29_speed_hz      = 0.0f;
volatile int32_t  g_drun29_field_deg     = 0;
volatile int32_t  g_drun29_rotor_deg     = 0;
volatile int32_t  g_drun29_diff_deg      = 0;
volatile int32_t  g_drun29_rotor_count   = 0;
volatile float    g_drun29_id_ma         = 0.0f;
volatile float    g_drun29_iq_ma         = 0.0f;
volatile float    g_drun29_id_pp_ma      = 0.0f;
volatile float    g_drun29_iq_pp_ma      = 0.0f;
volatile float    g_drun29_id_mean_ma    = 0.0f;
volatile float    g_drun29_iq_mean_ma    = 0.0f;
volatile float    g_drun29_i_ref_ma      = DRUN29_I_REF_MA;
volatile float    g_drun29_id_ref_ma     = 0.0f;
volatile float    g_drun29_iq_ref_ma     = 0.0f;
volatile float    g_drun29_i_ramp_ma_s   = DRUN29_I_RAMP_MA_S;
volatile uint8_t  g_drun29_vsat          = 0u;
volatile float    g_drun29_ed_mean_ma    = 0.0f;
volatile float    g_drun29_eq_mean_ma    = 0.0f;
volatile float    g_drun29_ed_pp_ma      = 0.0f;
volatile float    g_drun29_eq_pp_ma      = 0.0f;
volatile float    g_drun29_du            = 50.0f;
volatile float    g_drun29_dv            = 50.0f;
volatile float    g_drun29_dw            = 50.0f;
volatile uint32_t g_drun29_time_us       = 0u;
volatile float    g_drun29_step_frac_pct = 90.0f;
volatile uint8_t  g_drun29_id_step_state = DRUN29_STEP_ST_IDLE;
volatile uint8_t  g_drun29_iq_step_state = DRUN29_STEP_ST_IDLE;
volatile float    g_drun29_id_step_target_ma = 0.0f;
volatile float    g_drun29_iq_step_target_ma = 0.0f;
volatile uint32_t g_drun29_id_step_t90_us = DRUN29_STEP_TIME_TIMEOUT;
volatile uint32_t g_drun29_iq_step_t90_us = DRUN29_STEP_TIME_TIMEOUT;

pid_config_t g_drun29_pid_id_cfg = {
    .enabled = true,
    .p_valid = true,
    .i_valid = false,
    .d_valid = false,
    .kp = DRUN29_PI_KP,
    .ki = 0.0f,
    .kd = 0.0f,
    .output_min = -DRUN29_PI_UMAX_V,
    .output_max =  DRUN29_PI_UMAX_V,
    .integral_max = DRUN29_ITERM_MAX_V,
    .i_term_max = DRUN29_ITERM_MAX_V,
    .update_ms = 0,
};

pid_config_t g_drun29_pid_iq_cfg = {
    .enabled = true,
    .p_valid = true,
    .i_valid = false,
    .d_valid = false,
    .kp = DRUN29_PI_KP,
    .ki = 0.0f,
    .kd = 0.0f,
    .output_min = -DRUN29_PI_UMAX_V,
    .output_max =  DRUN29_PI_UMAX_V,
    .integral_max = DRUN29_ITERM_MAX_V,
    .i_term_max = DRUN29_ITERM_MAX_V,
    .update_ms = 0,
};

static pid_state_t s_pid_id;
static pid_state_t s_pid_iq;
static foc_dcal24_result_t s_calibration;

static int32_t s_rotor_count;
static uint16_t s_encoder_prev_hw;
static uint8_t s_encoder_initialized;
static float s_zero_u_ma;
static float s_zero_v_ma;
static float s_zero_w_ma;

static uint32_t s_run_tick;
static uint8_t s_ramp_done;
static float s_i_ref_ramp;
static uint32_t s_speed_tick;
static int32_t s_speed_acc;

static uint32_t s_pp_tick;
static uint8_t s_pp_init;
static float s_id_min;
static float s_id_max;
static float s_iq_min;
static float s_iq_max;

static uint32_t s_mean_tick;
static int32_t s_id_sum_ma;
static int32_t s_iq_sum_ma;

static uint32_t s_err_tick;
static uint8_t s_err_init;
static int32_t s_ed_sum_ma;
static int32_t s_eq_sum_ma;
static float s_ed_min;
static float s_ed_max;
static float s_eq_min;
static float s_eq_max;

typedef struct {
    uint8_t  state;
    uint8_t  entry_armed;
    float    target_ma;
    float    prev_ref_ma;
    uint32_t start_us;
    uint32_t cross_us;
    uint8_t  cross_tick;
} drun29_step_track_t;

static drun29_step_track_t s_id_step;
static drun29_step_track_t s_iq_step;

static uint16_t s_timer_last_count;
static uint64_t s_time_elapsed_us;
static uint8_t  s_timer_initialized;

static void Drun29_ResetLoopState(void)
{
    s_rotor_count = 0;
    s_encoder_prev_hw = 0u;
    s_encoder_initialized = 0u;
    s_zero_u_ma = 0.0f;
    s_zero_v_ma = 0.0f;
    s_zero_w_ma = 0.0f;
    s_run_tick = 0u;
    s_ramp_done = 0u;
    s_i_ref_ramp = 0.0f;
    s_speed_tick = 0u;
    s_speed_acc = 0;
    s_pp_tick = 0u;
    s_pp_init = 0u;
    s_id_min = 0.0f;
    s_id_max = 0.0f;
    s_iq_min = 0.0f;
    s_iq_max = 0.0f;
    s_mean_tick = 0u;
    s_id_sum_ma = 0;
    s_iq_sum_ma = 0;
    s_err_tick = 0u;
    s_err_init = 0u;
    s_ed_sum_ma = 0;
    s_eq_sum_ma = 0;
    s_ed_min = 0.0f;
    s_ed_max = 0.0f;
    s_eq_min = 0.0f;
    s_eq_max = 0.0f;
    s_id_step.state = DRUN29_STEP_ST_IDLE;
    s_id_step.entry_armed = 1u;
    s_id_step.target_ma = 0.0f;
    s_id_step.prev_ref_ma = 0.0f;
    s_id_step.start_us = 0u;
    s_id_step.cross_us = 0u;
    s_id_step.cross_tick = 0u;
    s_iq_step = s_id_step;
    s_timer_last_count = 0u;
    s_time_elapsed_us = 0u;
    s_timer_initialized = 0u;
}

static void Drun29_ClearObservables(void)
{
    g_drun29_evt = 0u;
    g_drun29_dlt_now_deg = g_drun29_dlt_init_deg;
    g_drun29_speed_hz = 0.0f;
    g_drun29_field_deg = 0;
    g_drun29_rotor_deg = 0;
    g_drun29_diff_deg = 0;
    g_drun29_rotor_count = 0;
    g_drun29_id_ma = 0.0f;
    g_drun29_iq_ma = 0.0f;
    g_drun29_id_pp_ma = 0.0f;
    g_drun29_iq_pp_ma = 0.0f;
    g_drun29_id_mean_ma = 0.0f;
    g_drun29_iq_mean_ma = 0.0f;
    g_drun29_id_ref_ma = 0.0f;
    g_drun29_iq_ref_ma = 0.0f;
    g_drun29_vsat = 0u;
    g_drun29_ed_mean_ma = 0.0f;
    g_drun29_eq_mean_ma = 0.0f;
    g_drun29_ed_pp_ma = 0.0f;
    g_drun29_eq_pp_ma = 0.0f;
    g_drun29_du = 50.0f;
    g_drun29_dv = 50.0f;
    g_drun29_dw = 50.0f;
    g_drun29_time_us = 0u;
    g_drun29_step_frac_pct = 75.0f;
    g_drun29_id_step_state = DRUN29_STEP_ST_IDLE;
    g_drun29_iq_step_state = DRUN29_STEP_ST_IDLE;
    g_drun29_id_step_target_ma = 0.0f;
    g_drun29_iq_step_target_ma = 0.0f;
    g_drun29_id_step_t90_us = DRUN29_STEP_TIME_TIMEOUT;
    g_drun29_iq_step_t90_us = DRUN29_STEP_TIME_TIMEOUT;
}

static void Drun29_PpFeed(float id, float iq)
{
    if (s_pp_init == 0u) {
        s_id_min = s_id_max = id;
        s_iq_min = s_iq_max = iq;
        s_pp_init = 1u;
    } else {
        if (id < s_id_min) s_id_min = id;
        if (id > s_id_max) s_id_max = id;
        if (iq < s_iq_min) s_iq_min = iq;
        if (iq > s_iq_max) s_iq_max = iq;
    }

    if (++s_pp_tick >= DRUN29_PP_WIN_TICKS) {
        s_pp_tick = 0u;
        g_drun29_id_pp_ma = (s_id_max - s_id_min) * 1000.0f;
        g_drun29_iq_pp_ma = (s_iq_max - s_iq_min) * 1000.0f;
        s_id_min = s_id_max = id;
        s_iq_min = s_iq_max = iq;
    }
}

static stc_i_data_t Drun29_CorrectedData(const stc_i_data_t *pData)
{
    stc_i_data_t data = *pData;
    data.i16IU_mA = (int16_t)((float)pData->i16IU_mA - s_zero_u_ma);
    data.i16IV_mA = (int16_t)((float)pData->i16IV_mA - s_zero_v_ma);
    data.i16IW_mA = (int16_t)((float)pData->i16IW_mA - s_zero_w_ma);
    return data;
}

static void Drun29_ClearCurrentFeedback(void)
{
    g_drun29_id_ma = 0.0f;
    g_drun29_iq_ma = 0.0f;
    g_drun29_id_ref_ma = 0.0f;
    g_drun29_iq_ref_ma = 0.0f;
    g_foc_id_ma = 0.0f;
    g_foc_iq_ma = 0.0f;
}

static uint32_t Drun29_TimeUpdate(void)
{
    uint32_t now_count;
    int32_t timer_delta;
    uint32_t timer_freq;

    now_count = Timer6_Timebase_GetCounter();
    if (s_timer_initialized == 0u) {
        s_timer_last_count = (uint16_t)now_count;
        s_time_elapsed_us = 0u;
        s_timer_initialized = 1u;
        g_drun29_time_us = 0u;
        return 0u;
    }

    timer_delta = (int32_t)(int16_t)((uint16_t)now_count - s_timer_last_count);
    s_timer_last_count = (uint16_t)now_count;
    timer_freq = Timer6_Timebase_GetFrequency();
    if (timer_freq != 0u) {
        s_time_elapsed_us += ((uint64_t)timer_delta * 1000000u
                              + (uint64_t)(timer_freq / 2u)) / (uint64_t)timer_freq;
    }
    g_drun29_time_us = (uint32_t)s_time_elapsed_us;
    return (uint32_t)s_time_elapsed_us;
}

static void Drun29_StepFeed(drun29_step_track_t *track,
                            volatile uint8_t *state_out,
                            volatile float *target_out,
                            volatile uint32_t *time_out,
                            float ref_ma,
                            float actual_ma,
                            uint32_t now_us)
{
    float fraction;
    float threshold_ma;
    uint8_t crossed;

    fraction = g_drun29_step_frac_pct * 0.01f;
    if ((fraction <= 0.0f) || (fraction > 2.0f)) {
        fraction = 0.9f;
    }

    if (track->state == DRUN29_STEP_ST_WAIT) {
        /* The measurement is armed only by the first ISR after entering
         * mode 29. Later Watch changes do not restart this timer. */
        if ((now_us - track->start_us) >= DRUN29_STEP_TIMEOUT_US) {
            track->state = DRUN29_STEP_ST_TIMEOUT;
            *time_out = DRUN29_STEP_TIME_TIMEOUT;
        } else {
            threshold_ma = track->target_ma * fraction;
            if (track->target_ma >= 0.0f) {
                crossed = (actual_ma >= threshold_ma) ? 1u : 0u;
            } else {
                crossed = (actual_ma <= threshold_ma) ? 1u : 0u;
            }

            if (crossed != 0u) {
                if (track->cross_tick == 0u) {
                    track->cross_us = now_us;
                }
                track->cross_tick++;
                if (track->cross_tick >= DRUN29_STEP_CONFIRM_TICK) {
                    track->state = DRUN29_STEP_ST_DONE;
                    *time_out = track->cross_us;
                }
            } else {
                track->cross_tick = 0u;
                track->cross_us = 0u;
            }
        }
    } else if ((track->entry_armed != 0u)
               && ((ref_ma >= DRUN29_STEP_MIN_MA)
                   || (ref_ma <= -DRUN29_STEP_MIN_MA))) {
        track->state = DRUN29_STEP_ST_WAIT;
        track->entry_armed = 0u;
        track->target_ma = ref_ma;
        track->start_us = now_us;
        track->cross_us = 0u;
        track->cross_tick = 0u;
        *time_out = 0u;
        *target_out = ref_ma;
    }

    track->prev_ref_ma = ref_ma;
    *state_out = track->state;
}

void Foc_Drun29_InitPids(void)
{
    PID_Init(&s_pid_id, &g_drun29_pid_id_cfg);
    PID_Init(&s_pid_iq, &g_drun29_pid_iq_cfg);
}

void Foc_Drun29_Start(void)
{
    foc_dcal24_result_t calibration;
    uint16_t hardware_count;
    int32_t encoder_dir;

    if (Foc_Dcal24_GetResult(&calibration) == 0u) {
        g_drun29_running = 0u;
        g_drun29_state = DRUN29_STEP_IDLE;
        DRUN29_LOG("ERROR: no mode 24 calibration; run mode 24 first");
        return;
    }

    Foc_Core_ClearFault();
    Drun29_ResetLoopState();
    Drun29_ClearObservables();
    s_calibration = calibration;
    s_zero_u_ma = calibration.zero_u_ma;
    s_zero_v_ma = calibration.zero_v_ma;
    s_zero_w_ma = calibration.zero_w_ma;
    s_timer_last_count = (uint16_t)Timer6_Timebase_GetCounter();
    s_time_elapsed_us = 0u;
    s_timer_initialized = 1u;
    g_drun29_time_us = 0u;

    hardware_count = TMRA_GetCountValue(CM_TMRA_1);
    encoder_dir = (int32_t)g_foc_enc_dir;
    s_rotor_count = Foc_Core_ModPos((int32_t)hardware_count * encoder_dir - calibration.offset,
                                    (int32_t)ENCODER_CPR);
    s_encoder_prev_hw = hardware_count;
    s_encoder_initialized = 1u;
    g_drun29_rotor_count = s_rotor_count;

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
    PID_Reset(&s_pid_id);
    PID_Reset(&s_pid_iq);

    g_drun29_running = 1u;
    g_drun29_state = DRUN29_STEP_RUN;
    Foc_Core_PwmStart();
    DRUN29_LOG("start rot=%d cnt dlt=%d->%d deg tr=%ums Iref=%dmA kp=%dm iValid=%u",
               (int)s_rotor_count,
               (int)g_drun29_dlt_init_deg, (int)g_drun29_dlt_targ_deg,
               (unsigned)g_drun29_dlt_tr_ms, (int)g_drun29_i_ref_ma,
               (int)(g_drun29_pid_iq_cfg.kp * 1000.0f),
               (unsigned)g_drun29_pid_iq_cfg.i_valid);
}

void Foc_Drun29_Stop(void)
{
    if (g_drun29_running) {
        g_drun29_running = 0u;
        g_foc_align_state = 0u;
        if (g_foc_active) {
            g_foc_active = 0u;
            Foc_Core_PwmStop();
            Foc_Core_SetStateMachine(FOC_STATE_IDLE);
        }
        DRUN29_LOG("stopped");
    }
    Drun29_ClearCurrentFeedback();
}

void Foc_Drun29_Step(const stc_i_data_t *pData)
{
    stc_i_data_t data;
    float id, iq, ramp_step, delta_deg, delta_rad, id_ref, iq_ref, vd, vq;
    float valpha, vbeta, du, dv, dw, cos_r, sin_r, rotor_rad, progress;
    uint16_t hardware_count;
    uint32_t ramp_ticks;
    int32_t hardware_delta, corrected_delta, field_deg, rotor_deg, angle_diff;
    uint32_t now_us;

    if (Foc_Core_OverCurrent(pData)) {
        g_drun29_state = DRUN29_STEP_FAULT_OC;
        g_drun29_evt = DRUN29_EVT_OC;
        g_drun29_running = 0u;
        Foc_Core_FaultStop(1u);
        Drun29_ClearCurrentFeedback();
        return;
    }

    if (g_drun29_state != DRUN29_STEP_RUN) {
        return;
    }

    now_us = Drun29_TimeUpdate();
    ramp_ticks = g_drun29_dlt_tr_ms * (FOC_ISR_HZ / 1000u);
    if (ramp_ticks == 0u) {
        progress = 1.0f;
    } else {
        progress = (float)s_run_tick / (float)ramp_ticks;
        if (progress > 1.0f) progress = 1.0f;
    }
    delta_deg = g_drun29_dlt_init_deg
              + (g_drun29_dlt_targ_deg - g_drun29_dlt_init_deg) * progress;
    g_drun29_dlt_now_deg = delta_deg;
    if ((s_ramp_done == 0u) && (s_run_tick >= ramp_ticks)) {
        s_ramp_done = 1u;
        g_drun29_evt = DRUN29_EVT_RAMP_DONE;
    }
    s_run_tick++;

    hardware_count = TMRA_GetCountValue(CM_TMRA_1);
    if (s_encoder_initialized == 0u) {
        s_encoder_prev_hw = hardware_count;
        s_encoder_initialized = 1u;
    }
    hardware_delta = (int32_t)(int16_t)((uint16_t)hardware_count - s_encoder_prev_hw);
    s_encoder_prev_hw = hardware_count;
    if (hardware_delta > DRUN29_ENC_DELTA_MAX) {
        hardware_delta = DRUN29_ENC_DELTA_MAX;
    }
    if (hardware_delta < -DRUN29_ENC_DELTA_MAX) {
        hardware_delta = -DRUN29_ENC_DELTA_MAX;
    }
    corrected_delta = hardware_delta * (int32_t)g_foc_enc_dir;
    s_rotor_count = Foc_Core_ModPos(s_rotor_count + corrected_delta,
                                    (int32_t)ENCODER_CPR);
    g_drun29_rotor_count = s_rotor_count;

    rotor_rad = (float)s_rotor_count * (FOC_MATH_2PI * (float)FOC_POLE_PAIRS
                                       / (float)ENCODER_CPR);
    rotor_rad -= (float)((int32_t)(rotor_rad * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
    if (rotor_rad < 0.0f) rotor_rad += FOC_MATH_2PI;

    data = Drun29_CorrectedData(pData);
    Foc_Core_GetDq(&data, rotor_rad, &id, &iq);
    g_drun29_id_ma = id * 1000.0f;
    g_drun29_iq_ma = iq * 1000.0f;
    g_foc_id_ma = g_drun29_id_ma;
    g_foc_iq_ma = g_drun29_iq_ma;
    Drun29_PpFeed(id, iq);

    s_id_sum_ma += (int32_t)(id * 1000.0f);
    s_iq_sum_ma += (int32_t)(iq * 1000.0f);
    if (++s_mean_tick >= DRUN29_MEAN_WIN_TICKS) {
        s_mean_tick = 0u;
        g_drun29_id_mean_ma = (float)s_id_sum_ma / (float)DRUN29_MEAN_WIN_TICKS;
        g_drun29_iq_mean_ma = (float)s_iq_sum_ma / (float)DRUN29_MEAN_WIN_TICKS;
        s_id_sum_ma = 0;
        s_iq_sum_ma = 0;
    }

    if (g_drun29_i_ramp_ma_s > 0.0f) {
        ramp_step = g_drun29_i_ramp_ma_s / (float)FOC_ISR_HZ;
        if (s_i_ref_ramp < g_drun29_i_ref_ma) {
            s_i_ref_ramp += ramp_step;
            if (s_i_ref_ramp > g_drun29_i_ref_ma) s_i_ref_ramp = g_drun29_i_ref_ma;
        } else if (s_i_ref_ramp > g_drun29_i_ref_ma) {
            s_i_ref_ramp -= ramp_step;
            if (s_i_ref_ramp < g_drun29_i_ref_ma) s_i_ref_ramp = g_drun29_i_ref_ma;
        }
    } else {
        s_i_ref_ramp = g_drun29_i_ref_ma;
    }

    delta_rad = delta_deg * DRUN29_DEG2RAD;
    id_ref = (s_i_ref_ramp * 0.001f) * Foc_Math_Cos(delta_rad);
    iq_ref = (s_i_ref_ramp * 0.001f) * Foc_Math_Sin(delta_rad);
    g_drun29_id_ref_ma = id_ref * 1000.0f;
    g_drun29_iq_ref_ma = iq_ref * 1000.0f;
    Drun29_StepFeed(&s_id_step, &g_drun29_id_step_state,
                    &g_drun29_id_step_target_ma,
                    &g_drun29_id_step_t90_us,
                    g_drun29_id_ref_ma, g_drun29_id_ma, now_us);
    Drun29_StepFeed(&s_iq_step, &g_drun29_iq_step_state,
                    &g_drun29_iq_step_target_ma,
                    &g_drun29_iq_step_t90_us,
                    g_drun29_iq_ref_ma, g_drun29_iq_ma, now_us);

    vd = PID_UpdateUs(&s_pid_id, id_ref, id, DRUN29_ISR_DT_US);
    vq = PID_UpdateUs(&s_pid_iq, iq_ref, iq, DRUN29_ISR_DT_US);
    g_foc_vd = vd;
    g_foc_vq = vq;
    g_drun29_vsat = ((vd <= -DRUN29_PI_UMAX_V + 0.01f) ||
                     (vd >=  DRUN29_PI_UMAX_V - 0.01f) ||
                     (vq <= -DRUN29_PI_UMAX_V + 0.01f) ||
                     (vq >=  DRUN29_PI_UMAX_V - 0.01f)) ? 1u : 0u;

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
    g_drun29_du = du;
    g_drun29_dv = dv;
    g_drun29_dw = dw;

    {
        int32_t error_d = (int32_t)((id_ref - id) * 1000.0f);
        int32_t error_q = (int32_t)((iq_ref - iq) * 1000.0f);

        if (s_err_init == 0u) {
            s_ed_min = s_ed_max = (float)error_d;
            s_eq_min = s_eq_max = (float)error_q;
            s_err_init = 1u;
        } else {
            if (error_d < s_ed_min) s_ed_min = (float)error_d;
            if (error_d > s_ed_max) s_ed_max = (float)error_d;
            if (error_q < s_eq_min) s_eq_min = (float)error_q;
            if (error_q > s_eq_max) s_eq_max = (float)error_q;
        }
        s_ed_sum_ma += error_d;
        s_eq_sum_ma += error_q;
        if (++s_err_tick >= DRUN29_ERR_WIN_TICKS) {
            s_err_tick = 0u;
            g_drun29_ed_mean_ma = (float)s_ed_sum_ma / (float)DRUN29_ERR_WIN_TICKS;
            g_drun29_eq_mean_ma = (float)s_eq_sum_ma / (float)DRUN29_ERR_WIN_TICKS;
            g_drun29_ed_pp_ma = s_ed_max - s_ed_min;
            g_drun29_eq_pp_ma = s_eq_max - s_eq_min;
            s_ed_sum_ma = 0;
            s_eq_sum_ma = 0;
            s_ed_min = s_ed_max = (float)error_d;
            s_eq_min = s_eq_max = (float)error_q;
        }
    }

    s_speed_acc += corrected_delta;
    if (++s_speed_tick >= DRUN29_SPEED_WIN_TICKS) {
        g_drun29_speed_hz = (float)s_speed_acc * (float)FOC_POLE_PAIRS
                          * (1000.0f / (float)DRUN29_SPEED_WIN_MS)
                          / (float)ENCODER_CPR;
        s_speed_tick = 0u;
        s_speed_acc = 0;
    }

    field_deg = (int32_t)((rotor_rad + delta_rad) * DRUN29_RAD2DEG);
    if (field_deg >= 360) field_deg -= 360;
    rotor_deg = (int32_t)(rotor_rad * DRUN29_RAD2DEG);
    angle_diff = field_deg - rotor_deg;
    angle_diff = 180 - Foc_Core_ModPos(180 - angle_diff, 360);
    g_drun29_field_deg = field_deg;
    g_drun29_rotor_deg = rotor_deg;
    g_drun29_diff_deg = angle_diff;
}
