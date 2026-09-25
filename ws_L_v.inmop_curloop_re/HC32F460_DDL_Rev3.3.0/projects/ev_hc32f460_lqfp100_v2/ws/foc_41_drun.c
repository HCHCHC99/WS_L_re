/**
 *******************************************************************************
 * @file  foc_41_drun.c
 * @brief FOC mode 41 - current loop with runtime-switchable dq feed-forward.
 *
 * The mode takes only a value copy of the mode 24 calibration snapshot.  At
 * Start it reads the live TMRA_1 count, converts it to the current rotor
 * angle, and integrates all later motion in a private frame.
 *******************************************************************************
 */

#include "foc_41_drun.h"
#include "foc_24_dcal.h"
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "timer6_timebase.h"
#include "hc32_ll_tmra.h"
#include "I.h"   /* g_i_iu/iv/iw_ma（VOFA 三相电流通道） */
#include "foc_30_ramp.h"  /* g_foc_ialpha/ibeta（静止系观测，VOFA 用）
                           * ⚠ 这三个量定义在 mode 30 模块里却被多模式共用，
                           *   属架构不洁点；后续宜迁至 foc_core（阶段 3 待办） */
#include <math.h>

#define DRUN41_ISR_DT_US  (1000000u / FOC_ISR_HZ)

#define DRUN41_SPEED_WIN_TICKS  ((uint32_t)DRUN41_SPEED_WIN_MS * FOC_ISR_HZ / 1000u)
#define DRUN41_PP_WIN_TICKS     ((uint32_t)DRUN41_PP_WIN_MS * FOC_ISR_HZ / 1000u)
#define DRUN41_MEAN_WIN_TICKS   ((uint32_t)DRUN41_MEAN_WIN_MS * FOC_ISR_HZ / 1000u)
#define DRUN41_ERR_WIN_TICKS    ((uint32_t)DRUN41_ERR_WIN_MS * FOC_ISR_HZ / 1000u)

#define DRUN41_RAD2DEG  57.2958f
#define DRUN41_DEG2RAD  0.0174533f

volatile uint8_t  g_drun41_running       = 0u;
volatile uint8_t  g_drun41_state         = DRUN41_STEP_IDLE;
volatile uint8_t  g_drun41_evt           = 0u;
volatile float    g_drun41_dlt_init_deg  = 90.0f;
volatile float    g_drun41_dlt_targ_deg  = 90.0f;
volatile uint32_t g_drun41_dlt_tr_ms     = 0u;
volatile float    g_drun41_dlt_now_deg   = 0.0f;
volatile float    g_drun41_speed_hz      = 0.0f;
volatile int32_t  g_drun41_field_deg     = 0;
volatile int32_t  g_drun41_rotor_deg     = 0;
volatile int32_t  g_drun41_diff_deg      = 0;
volatile int32_t  g_drun41_rotor_count   = 0;
volatile float    g_drun41_id_ma         = 0.0f;
volatile float    g_drun41_iq_ma         = 0.0f;
volatile float    g_drun41_iq_filt_ma    = 0.0f;
volatile float    g_drun41_iq_filt_alpha = DRUN41_IQ_FILT_ALPHA;
volatile float    g_drun41_id_pp_ma      = 0.0f;
volatile float    g_drun41_iq_pp_ma      = 0.0f;
volatile float    g_drun41_id_mean_ma    = 0.0f;
volatile float    g_drun41_iq_mean_ma    = 0.0f;
volatile float    g_drun41_i_ref_ma      = DRUN41_I_REF_MA;
volatile float    g_drun41_id_ref_ma     = 0.0f;
volatile float    g_drun41_iq_ref_ma     = 0.0f;
volatile float    g_drun41_i_ramp_ma_s   = DRUN41_I_RAMP_MA_S;
volatile uint8_t  g_drun41_vsat          = 0u;
volatile float    g_drun41_ed_mean_ma    = 0.0f;
volatile float    g_drun41_eq_mean_ma    = 0.0f;
volatile float    g_drun41_ed_pp_ma      = 0.0f;
volatile float    g_drun41_eq_pp_ma      = 0.0f;
volatile float    g_drun41_du            = 50.0f;
volatile float    g_drun41_dv            = 50.0f;
volatile float    g_drun41_dw            = 50.0f;
volatile uint8_t  g_drun41_ff_enable     = 0u;
volatile float    g_drun41_ff_bemf_gain  = 1.0f;
volatile float    g_drun41_ff_cross_gain = 1.0f;
volatile float    g_drun41_ff_res_gain   = 1.0f;
volatile float    g_drun41_omega_e_rad_s = 0.0f;
volatile float    g_drun41_vd_pi_v       = 0.0f;
volatile float    g_drun41_vq_pi_v       = 0.0f;
volatile float    g_drun41_vd_ff_v       = 0.0f;
volatile float    g_drun41_vq_ff_v       = 0.0f;
volatile float    g_drun41_vmax_v        = DRUN41_VMAX_DEFAULT_V;
volatile uint32_t g_drun41_time_us       = 0u;
volatile float    g_drun41_step_frac_pct = 90.0f;
volatile uint8_t  g_drun41_id_step_state = DRUN41_STEP_ST_IDLE;
volatile uint8_t  g_drun41_iq_step_state = DRUN41_STEP_ST_IDLE;
volatile float    g_drun41_id_step_target_ma = 0.0f;
volatile float    g_drun41_iq_step_target_ma = 0.0f;
volatile uint32_t g_drun41_id_step_t90_us = DRUN41_STEP_TIME_TIMEOUT;
volatile uint32_t g_drun41_iq_step_t90_us = DRUN41_STEP_TIME_TIMEOUT;

pid_config_t g_drun41_pid_id_cfg = {
    .enabled = true,
    .p_valid = true,
    .i_valid = true,
    .d_valid = false,
    .kp = DRUN41_PI_KP,
    .ki = DRUN41_PI_KI,
    .kd = 0.0f,
    .output_min = -DRUN41_PI_UMAX_V,
    .output_max =  DRUN41_PI_UMAX_V,
    .integral_max = DRUN41_ITERM_MAX_V,
    .i_term_max = DRUN41_ITERM_MAX_V,
    .update_ms = 0,
};

pid_config_t g_drun41_pid_iq_cfg = {
    .enabled = true,
    .p_valid = true,
    .i_valid = true,
    .d_valid = false,
    .kp = DRUN41_PI_KP,
    .ki = DRUN41_PI_KI,
    .kd = 0.0f,
    .output_min = -DRUN41_PI_UMAX_V,
    .output_max =  DRUN41_PI_UMAX_V,
    .integral_max = DRUN41_ITERM_MAX_V,
    .i_term_max = DRUN41_ITERM_MAX_V,
    .update_ms = 0,
};

static pid_state_t s_pid_id;
static pid_state_t s_pid_iq;

static int32_t s_rotor_count;
static uint16_t s_encoder_prev_hw;
static uint8_t s_encoder_initialized;
static float s_zero_u_ma;
static float s_zero_v_ma;
static float s_zero_w_ma;
static uint8_t s_iq_filt_init;

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
} drun41_step_track_t;

static drun41_step_track_t s_id_step;
static drun41_step_track_t s_iq_step;

static uint16_t s_timer_last_count;
static uint64_t s_time_elapsed_us;
static uint8_t  s_timer_initialized;

static void Drun41_ResetLoopState(void)
{
    s_rotor_count = 0;
    s_encoder_prev_hw = 0u;
    s_encoder_initialized = 0u;
    s_zero_u_ma = 0.0f;
    s_zero_v_ma = 0.0f;
    s_zero_w_ma = 0.0f;
    s_iq_filt_init = 0u;
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
    s_id_step.state = DRUN41_STEP_ST_IDLE;
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

static void Drun41_ClearObservables(void)
{
    g_drun41_evt = 0u;
    g_drun41_dlt_now_deg = g_drun41_dlt_init_deg;
    g_drun41_speed_hz = 0.0f;
    g_drun41_field_deg = 0;
    g_drun41_rotor_deg = 0;
    g_drun41_diff_deg = 0;
    g_drun41_rotor_count = 0;
    g_drun41_id_ma = 0.0f;
    g_drun41_iq_ma = 0.0f;
    g_drun41_iq_filt_ma = 0.0f;
    g_drun41_id_pp_ma = 0.0f;
    g_drun41_iq_pp_ma = 0.0f;
    g_drun41_id_mean_ma = 0.0f;
    g_drun41_iq_mean_ma = 0.0f;
    g_drun41_id_ref_ma = 0.0f;
    g_drun41_iq_ref_ma = 0.0f;
    g_drun41_vsat = 0u;
    g_drun41_ed_mean_ma = 0.0f;
    g_drun41_eq_mean_ma = 0.0f;
    g_drun41_ed_pp_ma = 0.0f;
    g_drun41_eq_pp_ma = 0.0f;
    g_drun41_du = 50.0f;
    g_drun41_dv = 50.0f;
    g_drun41_dw = 50.0f;
    g_drun41_omega_e_rad_s = 0.0f;
    g_drun41_vd_pi_v = 0.0f;
    g_drun41_vq_pi_v = 0.0f;
    g_drun41_vd_ff_v = 0.0f;
    g_drun41_vq_ff_v = 0.0f;
    g_drun41_time_us = 0u;
    g_drun41_step_frac_pct = 75.0f;
    g_drun41_id_step_state = DRUN41_STEP_ST_IDLE;
    g_drun41_iq_step_state = DRUN41_STEP_ST_IDLE;
    g_drun41_id_step_target_ma = 0.0f;
    g_drun41_iq_step_target_ma = 0.0f;
    g_drun41_id_step_t90_us = DRUN41_STEP_TIME_TIMEOUT;
    g_drun41_iq_step_t90_us = DRUN41_STEP_TIME_TIMEOUT;
}

static void Drun41_PpFeed(float id, float iq)
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

    if (++s_pp_tick >= DRUN41_PP_WIN_TICKS) {
        s_pp_tick = 0u;
        g_drun41_id_pp_ma = (s_id_max - s_id_min) * 1000.0f;
        g_drun41_iq_pp_ma = (s_iq_max - s_iq_min) * 1000.0f;
        s_id_min = s_id_max = id;
        s_iq_min = s_iq_max = iq;
    }
}

static stc_i_data_t Drun41_CorrectedData(const stc_i_data_t *pData)
{
    stc_i_data_t data = *pData;
    data.i16IU_mA = (int16_t)((float)pData->i16IU_mA - s_zero_u_ma);
    data.i16IV_mA = (int16_t)((float)pData->i16IV_mA - s_zero_v_ma);
    data.i16IW_mA = (int16_t)((float)pData->i16IW_mA - s_zero_w_ma);
    return data;
}

static void Drun41_ClearCurrentFeedback(void)
{
    g_drun41_id_ma = 0.0f;
    g_drun41_iq_ma = 0.0f;
    g_drun41_iq_filt_ma = 0.0f;
    g_drun41_id_ref_ma = 0.0f;
    g_drun41_iq_ref_ma = 0.0f;
    g_foc_id_ma = 0.0f;
    g_foc_iq_ma = 0.0f;
}

static uint32_t Drun41_TimeUpdate(void)
{
    uint32_t now_count;
    int32_t timer_delta;
    uint32_t timer_freq;

    now_count = Timer6_Timebase_GetCounter();
    if (s_timer_initialized == 0u) {
        s_timer_last_count = (uint16_t)now_count;
        s_time_elapsed_us = 0u;
        s_timer_initialized = 1u;
        g_drun41_time_us = 0u;
        return 0u;
    }

    timer_delta = (int32_t)(int16_t)((uint16_t)now_count - s_timer_last_count);
    s_timer_last_count = (uint16_t)now_count;
    timer_freq = Timer6_Timebase_GetFrequency();
    if (timer_freq != 0u) {
        s_time_elapsed_us += ((uint64_t)timer_delta * 1000000u
                              + (uint64_t)(timer_freq / 2u)) / (uint64_t)timer_freq;
    }
    g_drun41_time_us = (uint32_t)s_time_elapsed_us;
    return (uint32_t)s_time_elapsed_us;
}

static void Drun41_StepFeed(drun41_step_track_t *track,
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

    fraction = g_drun41_step_frac_pct * 0.01f;
    if ((fraction <= 0.0f) || (fraction > 2.0f)) {
        fraction = 0.9f;
    }

    if (track->state == DRUN41_STEP_ST_WAIT) {
        /* The measurement is armed only by the first ISR after entering
         * mode 29. Later Watch changes do not restart this timer. */
        if ((now_us - track->start_us) >= DRUN41_STEP_TIMEOUT_US) {
            track->state = DRUN41_STEP_ST_TIMEOUT;
            *time_out = DRUN41_STEP_TIME_TIMEOUT;
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
                if (track->cross_tick >= DRUN41_STEP_CONFIRM_TICK) {
                    track->state = DRUN41_STEP_ST_DONE;
                    *time_out = track->cross_us;
                }
            } else {
                track->cross_tick = 0u;
                track->cross_us = 0u;
            }
        }
    } else if ((track->entry_armed != 0u)
               && ((ref_ma >= DRUN41_STEP_MIN_MA)
                   || (ref_ma <= -DRUN41_STEP_MIN_MA))) {
        track->state = DRUN41_STEP_ST_WAIT;
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

void Foc_Drun41_InitPids(void)
{
    PID_Init(&s_pid_id, &g_drun41_pid_id_cfg);
    PID_Init(&s_pid_iq, &g_drun41_pid_iq_cfg);
}

void Foc_Drun41_Start(void)
{
    foc_dcal24_result_t calibration;
    uint16_t hardware_count;
    int32_t encoder_dir;

    if (Foc_Dcal24_GetResult(&calibration) == 0u) {
        g_drun41_running = 0u;
        g_drun41_state = DRUN41_STEP_IDLE;
        DRUN41_LOG("ERROR: no mode 24 calibration; run mode 24 first");
        return;
    }

    Foc_Core_ClearFault();
    Drun41_ResetLoopState();
    Drun41_ClearObservables();
    s_zero_u_ma = calibration.zero_u_ma;
    s_zero_v_ma = calibration.zero_v_ma;
    s_zero_w_ma = calibration.zero_w_ma;
    s_timer_last_count = (uint16_t)Timer6_Timebase_GetCounter();
    s_time_elapsed_us = 0u;
    s_timer_initialized = 1u;
    g_drun41_time_us = 0u;

    hardware_count = TMRA_GetCountValue(CM_TMRA_1);
    encoder_dir = (int32_t)g_foc_enc_dir;
    s_rotor_count = Foc_Core_ModPos((int32_t)hardware_count * encoder_dir - calibration.offset,
                                    (int32_t)ENCODER_CPR);
    s_encoder_prev_hw = hardware_count;
    s_encoder_initialized = 1u;
    g_drun41_rotor_count = s_rotor_count;

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

    g_drun41_running = 1u;
    g_drun41_state = DRUN41_STEP_RUN;
    Foc_Core_PwmStart();
    DRUN41_LOG("start rot=%d cnt dlt=%d->%d deg tr=%ums Iref=%dmA kp=%dm iValid=%u",
               (int)s_rotor_count,
               (int)g_drun41_dlt_init_deg, (int)g_drun41_dlt_targ_deg,
               (unsigned)g_drun41_dlt_tr_ms, (int)g_drun41_i_ref_ma,
               (int)(g_drun41_pid_iq_cfg.kp * 1000.0f),
               (unsigned)g_drun41_pid_iq_cfg.i_valid);
}

void Foc_Drun41_Stop(void)
{
    if (g_drun41_running) {
        g_drun41_running = 0u;
        g_foc_align_state = 0u;
        if (g_foc_active) {
            g_foc_active = 0u;
            Foc_Core_PwmStop();
            Foc_Core_SetStateMachine(FOC_STATE_IDLE);
        }
        DRUN41_LOG("stopped");
    }
    Drun41_ClearCurrentFeedback();
}

void Foc_Drun41_Step(const stc_i_data_t *pData)
{
    stc_i_data_t data;
    float id, iq, ramp_step, delta_deg, delta_rad, id_ref, iq_ref, vd, vq;
    float valpha, vbeta, du, dv, dw, cos_r, sin_r, rotor_rad, progress;
    uint16_t hardware_count;
    uint32_t ramp_ticks;
    int32_t hardware_delta, corrected_delta, field_deg, rotor_deg, angle_diff;
    uint32_t now_us;

    if (Foc_Core_OverCurrent(pData)) {
        g_drun41_state = DRUN41_STEP_FAULT_OC;
        g_drun41_evt = DRUN41_EVT_OC;
        g_drun41_running = 0u;
        Foc_Core_FaultStop(1u);
        Drun41_ClearCurrentFeedback();
        return;
    }

    if (g_drun41_state != DRUN41_STEP_RUN) {
        return;
    }

    now_us = Drun41_TimeUpdate();
    ramp_ticks = g_drun41_dlt_tr_ms * (FOC_ISR_HZ / 1000u);
    if (ramp_ticks == 0u) {
        progress = 1.0f;
    } else {
        progress = (float)s_run_tick / (float)ramp_ticks;
        if (progress > 1.0f) progress = 1.0f;
    }
    delta_deg = g_drun41_dlt_init_deg
              + (g_drun41_dlt_targ_deg - g_drun41_dlt_init_deg) * progress;
    g_drun41_dlt_now_deg = delta_deg;
    if ((s_ramp_done == 0u) && (s_run_tick >= ramp_ticks)) {
        s_ramp_done = 1u;
        g_drun41_evt = DRUN41_EVT_RAMP_DONE;
    }
    s_run_tick++;

    hardware_count = TMRA_GetCountValue(CM_TMRA_1);
    if (s_encoder_initialized == 0u) {
        s_encoder_prev_hw = hardware_count;
        s_encoder_initialized = 1u;
    }
    hardware_delta = (int32_t)(int16_t)((uint16_t)hardware_count - s_encoder_prev_hw);
    s_encoder_prev_hw = hardware_count;
    if (hardware_delta > DRUN41_ENC_DELTA_MAX) {
        hardware_delta = DRUN41_ENC_DELTA_MAX;
    }
    if (hardware_delta < -DRUN41_ENC_DELTA_MAX) {
        hardware_delta = -DRUN41_ENC_DELTA_MAX;
    }
    corrected_delta = hardware_delta * (int32_t)g_foc_enc_dir;
    s_rotor_count = Foc_Core_ModPos(s_rotor_count + corrected_delta,
                                    (int32_t)ENCODER_CPR);
    g_drun41_rotor_count = s_rotor_count;

    rotor_rad = (float)s_rotor_count * (FOC_MATH_2PI * (float)FOC_POLE_PAIRS
                                       / (float)ENCODER_CPR);
    rotor_rad -= (float)((int32_t)(rotor_rad * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
    if (rotor_rad < 0.0f) rotor_rad += FOC_MATH_2PI;

    data = Drun41_CorrectedData(pData);
    Foc_Core_GetDq(&data, rotor_rad, &id, &iq);
    g_drun41_id_ma = id * 1000.0f;
    g_drun41_iq_ma = iq * 1000.0f;
    {
        float alpha = g_drun41_iq_filt_alpha;
        if (alpha <= 0.0f || alpha > 1.0f) {
            alpha = 1.0f;
        }
        if (s_iq_filt_init == 0u) {
            g_drun41_iq_filt_ma = g_drun41_iq_ma;
            s_iq_filt_init = 1u;
        } else {
            g_drun41_iq_filt_ma += alpha
                                   * (g_drun41_iq_ma - g_drun41_iq_filt_ma);
        }
    }
    g_foc_id_ma = g_drun41_id_ma;
    g_foc_iq_ma = g_drun41_iq_ma;
    Drun41_PpFeed(id, iq);

    s_id_sum_ma += (int32_t)(id * 1000.0f);
    s_iq_sum_ma += (int32_t)(iq * 1000.0f);
    if (++s_mean_tick >= DRUN41_MEAN_WIN_TICKS) {
        s_mean_tick = 0u;
        g_drun41_id_mean_ma = (float)s_id_sum_ma / (float)DRUN41_MEAN_WIN_TICKS;
        g_drun41_iq_mean_ma = (float)s_iq_sum_ma / (float)DRUN41_MEAN_WIN_TICKS;
        s_id_sum_ma = 0;
        s_iq_sum_ma = 0;
    }

    if (g_drun41_i_ramp_ma_s > 0.0f) {
        ramp_step = g_drun41_i_ramp_ma_s / (float)FOC_ISR_HZ;
        if (s_i_ref_ramp < g_drun41_i_ref_ma) {
            s_i_ref_ramp += ramp_step;
            if (s_i_ref_ramp > g_drun41_i_ref_ma) s_i_ref_ramp = g_drun41_i_ref_ma;
        } else if (s_i_ref_ramp > g_drun41_i_ref_ma) {
            s_i_ref_ramp -= ramp_step;
            if (s_i_ref_ramp < g_drun41_i_ref_ma) s_i_ref_ramp = g_drun41_i_ref_ma;
        }
    } else {
        s_i_ref_ramp = g_drun41_i_ref_ma;
    }

    delta_rad = delta_deg * DRUN41_DEG2RAD;
    id_ref = (s_i_ref_ramp * 0.001f) * Foc_Math_Cos(delta_rad);
    iq_ref = (s_i_ref_ramp * 0.001f) * Foc_Math_Sin(delta_rad);
    g_drun41_id_ref_ma = id_ref * 1000.0f;
    g_drun41_iq_ref_ma = iq_ref * 1000.0f;
    Drun41_StepFeed(&s_id_step, &g_drun41_id_step_state,
                    &g_drun41_id_step_target_ma,
                    &g_drun41_id_step_t90_us,
                    g_drun41_id_ref_ma, g_drun41_id_ma, now_us);
    Drun41_StepFeed(&s_iq_step, &g_drun41_iq_step_state,
                    &g_drun41_iq_step_target_ma,
                    &g_drun41_iq_step_t90_us,
                    g_drun41_iq_ref_ma, g_drun41_iq_ma, now_us);

    vd = PID_UpdateUs(&s_pid_id, id_ref, id, DRUN41_ISR_DT_US);
    vq = PID_UpdateUs(&s_pid_iq, iq_ref, iq, DRUN41_ISR_DT_US);
    g_drun41_vd_pi_v = vd;
    g_drun41_vq_pi_v = vq;

    /* g_drun41_speed_hz is electrical speed, refreshed by the 200 ms speed
     * window.  Keep the first feed-forward updates at zero until enough
     * encoder samples have accumulated. */
    g_drun41_omega_e_rad_s = FOC_MATH_2PI * g_drun41_speed_hz;
    if (g_drun41_ff_enable != 0u) {
        const float ls_h = (float)FOC_MOTOR_LS_UH * 0.000001f;

        g_drun41_vd_ff_v = -g_drun41_ff_cross_gain
                           * g_drun41_omega_e_rad_s * ls_h * iq;
        g_drun41_vq_ff_v = g_drun41_ff_cross_gain
                           * g_drun41_omega_e_rad_s * ls_h * id
                           + g_drun41_ff_bemf_gain
                           * g_drun41_omega_e_rad_s * (float)FOC_MOTOR_FLUX_VS
                           + g_drun41_ff_res_gain
                           * (float)FOC_MOTOR_RS_OHM * iq_ref;
        vd += g_drun41_vd_ff_v;
        vq += g_drun41_vq_ff_v;

        /* PI outputs are independently clamped.  After adding feed-forward,
         * limit the total dq vector so SVPWM is not asked for an excessive
         * circular voltage. */
        if (g_drun41_vmax_v > 0.0f) {
            float v2 = vd * vd + vq * vq;
            float vmax2 = g_drun41_vmax_v * g_drun41_vmax_v;
            if (v2 > vmax2) {
                float scale = g_drun41_vmax_v / sqrtf(v2);
                vd *= scale;
                vq *= scale;
                g_drun41_vsat = 1u;
            } else {
                g_drun41_vsat = ((vd <= -DRUN41_PI_UMAX_V + 0.01f) ||
                                 (vd >=  DRUN41_PI_UMAX_V - 0.01f) ||
                                 (vq <= -DRUN41_PI_UMAX_V + 0.01f) ||
                                 (vq >=  DRUN41_PI_UMAX_V - 0.01f)) ? 1u : 0u;
            }
        } else {
            g_drun41_vsat = ((vd <= -DRUN41_PI_UMAX_V + 0.01f) ||
                             (vd >=  DRUN41_PI_UMAX_V - 0.01f) ||
                             (vq <= -DRUN41_PI_UMAX_V + 0.01f) ||
                             (vq >=  DRUN41_PI_UMAX_V - 0.01f)) ? 1u : 0u;
        }
    } else {
        g_drun41_vd_ff_v = 0.0f;
        g_drun41_vq_ff_v = 0.0f;
        g_drun41_vsat = ((vd <= -DRUN41_PI_UMAX_V + 0.01f) ||
                         (vd >=  DRUN41_PI_UMAX_V - 0.01f) ||
                         (vq <= -DRUN41_PI_UMAX_V + 0.01f) ||
                         (vq >=  DRUN41_PI_UMAX_V - 0.01f)) ? 1u : 0u;
    }

    g_foc_vd = vd;
    g_foc_vq = vq;

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
    g_drun41_du = du;
    g_drun41_dv = dv;
    g_drun41_dw = dw;

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
        if (++s_err_tick >= DRUN41_ERR_WIN_TICKS) {
            s_err_tick = 0u;
            g_drun41_ed_mean_ma = (float)s_ed_sum_ma / (float)DRUN41_ERR_WIN_TICKS;
            g_drun41_eq_mean_ma = (float)s_eq_sum_ma / (float)DRUN41_ERR_WIN_TICKS;
            g_drun41_ed_pp_ma = s_ed_max - s_ed_min;
            g_drun41_eq_pp_ma = s_eq_max - s_eq_min;
            s_ed_sum_ma = 0;
            s_eq_sum_ma = 0;
            s_ed_min = s_ed_max = (float)error_d;
            s_eq_min = s_eq_max = (float)error_q;
        }
    }

    s_speed_acc += corrected_delta;
    if (++s_speed_tick >= DRUN41_SPEED_WIN_TICKS) {
        g_drun41_speed_hz = (float)s_speed_acc * (float)FOC_POLE_PAIRS
                          * (1000.0f / (float)DRUN41_SPEED_WIN_MS)
                          / (float)ENCODER_CPR;
        s_speed_tick = 0u;
        s_speed_acc = 0;
    }

    field_deg = (int32_t)((rotor_rad + delta_rad) * DRUN41_RAD2DEG);
    if (field_deg >= 360) field_deg -= 360;
    rotor_deg = (int32_t)(rotor_rad * DRUN41_RAD2DEG);
    angle_diff = field_deg - rotor_deg;
    angle_diff = 180 - Foc_Core_ModPos(180 - angle_diff, 360);
    g_drun41_field_deg = field_deg;
    g_drun41_rotor_deg = rotor_deg;
    g_drun41_diff_deg = angle_diff;
}

/*===========================================================================
 * mode 41 自持 VOFA：固定 21ch 布局
 * 通道含义速览卡 = foc_41_drun.h 顶部【模式速览卡】（唯一事实源）
 *
 * ch0~18 与 mode 29 完全一致（便于 A/B 对比），ch19/20 是本模式特有的
 * **前馈电压分量** —— 前馈实验的核心观测量，必须能直接看到。
 * 单位换算：传"毫单位"，SendScaled 内部 ×0.001（电流 mA→A，电压 mV→V）
 *===========================================================================*/
int Foc_Drun41_VofaFill(int32_t *cur)
{
    cur[0]  = (int32_t)(g_i_iu_ma);                  /* ch0  U 相电流 (mA -> A) */
    cur[1]  = (int32_t)(g_i_iv_ma);                  /* ch1  V 相电流 */
    cur[2]  = (int32_t)(g_i_iw_ma);                  /* ch2  W 相电流 */
    cur[3]  = (int32_t)(g_foc_ialpha * 1000.0f);     /* ch3  静止系 ialpha */
    cur[4]  = (int32_t)(g_foc_ibeta  * 1000.0f);     /* ch4  静止系 ibeta */
    cur[5]  = (int32_t)(g_foc_iq_ma);                /* ch5  iq 反馈 ← 环反馈 */
    cur[6]  = (int32_t)(g_foc_id_ma);                /* ch6  id 反馈 ← 环反馈 */
    cur[7]  = (int32_t)(g_drun41_iq_ref_ma);         /* ch7  iq 参考 */
    cur[8]  = (int32_t)(g_drun41_id_ref_ma);         /* ch8  id 参考（不恒为 0！） */
    cur[9]  = (int32_t)(g_foc_vq * 1000.0f);         /* ch9  vq 总输出（含前馈） */
    cur[10] = (int32_t)(g_foc_vd * 1000.0f);         /* ch10 vd 总输出（含前馈） */
    cur[11] = (int32_t)(g_drun41_eq_mean_ma);        /* ch11 q 轴误差均值 ← 主判据 */
    cur[12] = (int32_t)(g_drun41_ed_mean_ma);        /* ch12 d 轴误差均值 */
    cur[13] = (int32_t)(g_drun41_iq_mean_ma);        /* ch13 iq 3s 均值 */
    cur[14] = (int32_t)(g_drun41_id_mean_ma);        /* ch14 id 3s 均值 */
    cur[15] = (int32_t)(g_drun41_eq_pp_ma);          /* ch15 q 轴误差峰峰值（振铃判据） */
    cur[16] = (int32_t)(g_drun41_iq_filt_ma);        /* ch16 显示用滤波 iq（PI 不用） */
    cur[17] = (int32_t)(g_drun41_diff_deg * 1000);   /* ch17 功角 delta (mdeg -> deg) */
    cur[18] = (int32_t)(g_drun41_speed_hz * 1000.0f);/* ch18 实测电频率 (mHz -> Hz) */
    cur[19] = (int32_t)(g_drun41_vd_ff_v * 1000.0f); /* ch19 **d 轴前馈电压** ← 本模式专属 */
    cur[20] = (int32_t)(g_drun41_vq_ff_v * 1000.0f); /* ch20 **q 轴前馈电压** ← 本模式专属 */
    return 21;
}
