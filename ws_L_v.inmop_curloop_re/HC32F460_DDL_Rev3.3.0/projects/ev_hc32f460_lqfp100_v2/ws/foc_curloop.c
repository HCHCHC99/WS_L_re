/**
 *******************************************************************************
 * @file  foc_curloop.c
 * @brief FOC 模式22 — 编码器 FOC 电流环实现（I-F 启动 + 同步交接 + RUN）。
 *
 *        所有实现均自 foc.c 原样迁移，控制行为不变。
 *******************************************************************************
 */

#include "foc_curloop.h"
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"

/* ISR period in us */
#define FOC_ISR_DT_US  (1000000u / FOC_ISR_HZ)
#define FOC_IF_HOLD_MAX_CNT ((uint32_t)FOC_IF_HOLD_MAX_MS * FOC_ISR_HZ / 1000u)
#define FOC_RUN_BLEND_CNT    200u

/*******************************************************************************
 * 模式22 观测量 / Keil Watch 调参量
 ******************************************************************************/
volatile int32_t  g_foc_pi_off_180      = (int32_t)FOC_PI_OFF_180;
volatile float    g_foc_iq_ref_cmd_ma   = (float)FOC_IQ_REF_MA;
volatile float    g_foc_iq_ref_ma       = 0.0f;
volatile float    g_foc_iq_ramp_ma_s    = (float)FOC_IQ_RAMP_MA_S;
volatile float    g_foc_iq_pi_ma        = 0.0f;
volatile float    g_foc_run_target_rpm  = -30.0f;
volatile float    g_foc_anchor_deg      = (float)FOC_RUN_ANCHOR_DEG;
volatile float    g_foc_run_iq_sign     = (float)FOC_RUN_IQ_SIGN;

/* I-F start observables */
volatile float    g_foc_if_freq_hz  = 0.0f;
volatile float    g_foc_if_sweep_cHz = 0.0f;
volatile float    g_foc_if_hold_iq_ma = (float)FOC_IF_HOLD_IQ_MA;
volatile float    g_foc_if_sync_band_rad = FOC_IF_SYNC_BAND_RAD;
volatile uint32_t g_foc_if_sync_win_cnt = FOC_IF_SYNC_WIN_CNT;
volatile uint32_t g_foc_if_sync_good_wins = FOC_IF_SYNC_GOOD_WINS;
volatile float    g_foc_if_lock_diff_rad = 0.0f;
volatile float    g_foc_if_rel_diff_rad = 0.0f;
volatile float    g_foc_if_win_min_rad  = 0.0f;
volatile float    g_foc_if_win_max_rad  = 0.0f;
volatile uint32_t g_foc_if_win_cnt     = 0u;
volatile uint32_t g_foc_if_good_cnt    = 0u;
volatile uint8_t  g_foc_if_sync     = 0u;
volatile uint8_t  g_foc_if_evt    = 0u;
volatile int32_t  g_foc_if_evt_v1 = 0;
volatile int32_t  g_foc_if_evt_v2 = 0;
volatile int32_t  g_foc_if_evt_v3 = 0;
volatile int32_t  g_foc_if_evt_v4 = 0;

/* Current-loop PI configs */
pid_config_t g_foc_pid_id_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = FOC_PI_KP,
    .ki           = FOC_PI_KI,
    .kd           = 0.0f,
    .output_min   = -FOC_PI_UMAX_V,
    .output_max   =  FOC_PI_UMAX_V,
    .integral_max = 0.5f,
    .i_term_max   = 0.0f,
    .update_ms    = 0,
};

pid_config_t g_foc_pid_iq_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = FOC_PI_KP,
    .ki           = FOC_PI_KI,
    .kd           = 0.0f,
    .output_min   = -FOC_PI_UMAX_V,
    .output_max   =  FOC_PI_UMAX_V,
    .integral_max = 0.5f,
    .i_term_max   = 0.0f,
    .update_ms    = 0,
};

pid_config_t g_foc_pid_spd_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = 50.0f,
    .ki           = 10.0f,
    .kd           = 0.0f,
    .output_min   = 0.0f,
    .output_max   = (float)FOC_IQ_REF_MA,
    .integral_max = 50.0f,
    .i_term_max   = 0.0f,
    .update_ms    = 0,
};

/* PI runtime states */
static pid_state_t s_pid_id;
static pid_state_t s_pid_iq;
static pid_state_t s_pid_spd;

/* I-F sync detection 内部状态 */
static float    s_if_diff_min   = 0.0f;
static float    s_if_diff_max   = 0.0f;
static uint32_t s_if_win_cnt    = 0u;
static uint32_t s_if_hold_tick  = 0u;
static uint8_t  s_if_hold_done  = 0u;
static float    s_if_last_diff = 0.0f;
static uint32_t s_if_wrap_cnt  = 0u;
static uint32_t s_if_sweep_last_wrap = 0u;
static uint32_t s_if_sweep_last_tick = 0u;
static float    s_if_diff_unwrapped = 0.0f;
static uint8_t  s_if_diff_unwrapped_valid = 0u;
static uint32_t s_if_good_wins  = 0u;
static uint32_t s_if_tick       = 0u;
static uint8_t  s_run_blend_cnt  = 0u;
static float    s_run_blend_from = 0.0f;

/*******************************************************************************
 * Foc_CurLoop_InitPids - 绑定 PI 实例与配置
 ******************************************************************************/
void Foc_CurLoop_InitPids(void)
{
    PID_Init(&s_pid_id, &g_foc_pid_id_cfg);
    PID_Init(&s_pid_iq, &g_foc_pid_iq_cfg);
    PID_Init(&s_pid_spd, &g_foc_pid_spd_cfg);
}

/*******************************************************************************
 * Foc_StartCurrentLoop - mode 22: I-F current-controlled start
 ******************************************************************************/
void Foc_StartCurrentLoop(void)
{
    Foc_Core_ClearFault();
    g_foc_mode        = FOC_MODE_CURLOOP;
    g_foc_phase       = 1u;
    g_foc_theta_rad   = 0.0f;
    g_foc_iq_ref_ma   = 0.0f;
    g_foc_id_ma       = 0.0f;
    g_foc_iq_ma       = 0.0f;
    g_foc_vd          = 0.0f;
    g_foc_vq          = 0.0f;
    g_foc_if_freq_hz  = 0.0f;
    g_foc_if_diff_rad = 0.0f;
    g_foc_if_sync     = 0u;

    Foc_Core_SetStateMachine(FOC_STATE_IF_START);
    s_if_tick         = 0u;
    s_if_hold_tick    = 0u;
    s_if_hold_done    = 0u;
    s_if_last_diff    = 0.0f;
    s_if_wrap_cnt     = 0u;
    g_foc_if_evt      = 0u;
    g_foc_if_evt_v4   = 0;
    g_foc_if_sweep_cHz = 0.0f;
    s_if_sweep_last_wrap = 0u;
    s_if_sweep_last_tick = 0u;
    s_if_diff_unwrapped = 0.0f;
    s_if_diff_unwrapped_valid = 0u;
    g_foc_if_lock_diff_rad = 0.0f;
    g_foc_if_rotor_rad   = 0.0f;
    g_foc_if_rel_diff_rad = 0.0f;
    g_foc_if_win_min_rad  = 0.0f;
    g_foc_if_win_max_rad  = 0.0f;
    g_foc_if_win_cnt      = 0u;
    g_foc_if_good_cnt     = 0u;
    s_if_win_cnt      = 0u;
    s_if_good_wins    = 0u;
    s_if_diff_min     = 0.0f;
    s_if_diff_max     = 0.0f;
    Foc_Core_ResetVoltageEnvelope();
    Foc_Core_ResetEma();
    g_foc_align_state = 1u;

    PID_Reset(&s_pid_id);
    PID_Reset(&s_pid_iq);
    PID_Reset(&s_pid_spd);

    Foc_Core_PwmStart();
}

/*******************************************************************************
 * I-F start step
 ******************************************************************************/
static void Foc_IfHandover(const stc_i_data_t *pData);

static void Foc_IfStartStep(const stc_i_data_t *pData)
{
    float theta, ctrl_theta, id, iq, iq_ref_a, vd, vq, valpha, vbeta, du, dv, dw;
    float enc_elec, diff;

    if (Foc_Core_OverCurrent(pData)) {
        Foc_Core_FaultStop(1u);
        return;
    }

    g_foc_phase = (s_if_hold_done != 0u) ? 2u : 1u;

    if ((g_foc_iq_ref_ma < g_foc_if_hold_iq_ma) &&
        (s_if_hold_tick < FOC_IF_HOLD_MAX_CNT)) {
        s_if_hold_tick++;
        theta = g_foc_theta_rad;
    } else {
        if (!s_if_hold_done) {
            s_if_hold_done = 1u;
            g_foc_if_evt    = 1u;
            g_foc_if_evt_v1 = (int32_t)(g_foc_if_freq_hz * 100.0f);
            g_foc_if_evt_v2 = (int32_t)g_foc_iq_ref_ma;
            g_foc_if_evt_v3 = 0;
            {
                float phi_hold = FOC_MATH_HALF_PI
                               + ((g_foc_pi_off_180 != 0) ? FOC_MATH_PI : 0.0f)
                               + (g_foc_anchor_deg * FOC_MATH_PI / 180.0f);
                float per_cnt = FOC_MATH_2PI * (float)FOC_POLE_PAIRS / (float)ENCODER_CPR;
                int32_t off = Foc_Core_ModPos(((int32_t)g_enc_count * (int32_t)g_foc_enc_dir)
                                              - (int32_t)(phi_hold / per_cnt),
                                              (int32_t)ENCODER_CPR);
                Foc_Core_SetAlignOffset(off);
            }
        }
        if (g_foc_if_freq_hz < g_foc_openloop_freq_hz) {
            g_foc_if_freq_hz += FOC_IF_FREQ_RAMP_HZ_S / (float)FOC_ISR_HZ;
            if (g_foc_if_freq_hz > g_foc_openloop_freq_hz) {
                g_foc_if_freq_hz = g_foc_openloop_freq_hz;
            }
        }

        theta = g_foc_theta_rad
              + (FOC_MATH_2PI * g_foc_if_freq_hz / (float)FOC_ISR_HZ);
        if (theta >= FOC_MATH_2PI) {
            theta -= FOC_MATH_2PI;
        }
        g_foc_theta_rad = theta;
    }

    ctrl_theta = theta + ((g_foc_pi_off_180 != 0) ? FOC_MATH_PI : 0.0f);

    Foc_Core_GetDq(pData, ctrl_theta, &id, &iq);
    Foc_Core_EmaFilter(&id, &iq);
    g_foc_id_ma = id * 1000.0f;
    g_foc_iq_ma = iq * 1000.0f;

    {
        float step = g_foc_iq_ramp_ma_s / (float)FOC_ISR_HZ;
        if (g_foc_iq_ref_ma < g_foc_iq_ref_cmd_ma) {
            g_foc_iq_ref_ma += step;
            if (g_foc_iq_ref_ma > g_foc_iq_ref_cmd_ma) {
                g_foc_iq_ref_ma = g_foc_iq_ref_cmd_ma;
            }
        } else if (g_foc_iq_ref_ma > g_foc_iq_ref_cmd_ma) {
            g_foc_iq_ref_ma -= step;
            if (g_foc_iq_ref_ma < g_foc_iq_ref_cmd_ma) {
                g_foc_iq_ref_ma = g_foc_iq_ref_cmd_ma;
            }
        }
    }

    iq_ref_a = g_foc_iq_ref_ma * 0.001f;
    vd = PID_UpdateUs(&s_pid_id, 0.0f,     id, FOC_ISR_DT_US);
    vq = PID_UpdateUs(&s_pid_iq, iq_ref_a, iq, FOC_ISR_DT_US);
    g_foc_vd = vd;
    g_foc_vq = vq;

    Foc_Core_ApplyVoltageEnvelope(&vd, &vq);

    Foc_InvPark(vd, vq, ctrl_theta, &valpha, &vbeta);
    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);
    g_foc_valpha = valpha;
    g_foc_vbeta  = vbeta;
    g_foc_du = du;
    g_foc_dv = dv;
    g_foc_dw = dw;

    s_if_tick++;
    enc_elec = (float)Foc_Core_ModPos(((int32_t)g_enc_count * (int32_t)g_foc_enc_dir),
                                      (int32_t)ENCODER_CPR)
             * (FOC_MATH_2PI * (float)FOC_POLE_PAIRS / (float)ENCODER_CPR);
    enc_elec -= (float)((int32_t)(enc_elec * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
    if (enc_elec < 0.0f) {
        enc_elec += FOC_MATH_2PI;
    }
    diff = enc_elec - theta;
    if (diff >  FOC_MATH_PI) diff -= FOC_MATH_2PI;
    if (diff < -FOC_MATH_PI) diff += FOC_MATH_2PI;
    g_foc_if_diff_rad = diff;
    g_foc_if_rotor_rad = enc_elec;

    {
        float dd = diff - s_if_last_diff;
        if (dd >  FOC_MATH_PI) { s_if_wrap_cnt++; dd -= FOC_MATH_2PI; }
        if (dd < -FOC_MATH_PI) { s_if_wrap_cnt++; dd += FOC_MATH_2PI; }
        s_if_last_diff = diff;
        if (s_if_diff_unwrapped_valid) {
            s_if_diff_unwrapped += dd;
        } else {
            s_if_diff_unwrapped = diff;
            s_if_diff_unwrapped_valid = 1u;
        }
    }

    if (!s_if_hold_done) {
        g_foc_if_lock_diff_rad = diff;
        s_if_diff_unwrapped = 0.0f;
        s_if_diff_unwrapped_valid = 1u;
    }
    g_foc_if_rel_diff_rad = s_if_diff_unwrapped;

    if ((s_if_tick - s_if_sweep_last_tick) >= (uint32_t)(FOC_ISR_HZ / 10u)) {
        uint32_t dwrap = s_if_wrap_cnt - s_if_sweep_last_wrap;
        g_foc_if_sweep_cHz = (float)dwrap * 1000.0f;
        s_if_sweep_last_wrap = s_if_wrap_cnt;
        s_if_sweep_last_tick = s_if_tick;
    }

    if (g_foc_if_freq_hz >= FOC_IF_SYNC_MIN_HZ) {
        float    band      = g_foc_if_sync_band_rad;
        uint32_t win_cnt   = g_foc_if_sync_win_cnt;
        uint32_t good_wins = g_foc_if_sync_good_wins;

        if (band < 0.01f)    band = 0.01f;
        if (win_cnt == 0u)   win_cnt = 1u;
        if (good_wins == 0u) good_wins = 1u;

        if (s_if_win_cnt == 0u) {
            s_if_diff_min = s_if_diff_unwrapped;
            s_if_diff_max = s_if_diff_unwrapped;
        } else {
            if (s_if_diff_unwrapped < s_if_diff_min) s_if_diff_min = s_if_diff_unwrapped;
            if (s_if_diff_unwrapped > s_if_diff_max) s_if_diff_max = s_if_diff_unwrapped;
        }
        s_if_win_cnt++;
        if (s_if_win_cnt >= win_cnt) {
            if ((s_if_diff_max - s_if_diff_min) < band) {
                if (++s_if_good_wins >= good_wins) {
                    Foc_IfHandover(pData);
                    return;
                }
            } else {
                s_if_good_wins = 0u;
            }
            s_if_win_cnt = 0u;
        }

        g_foc_if_win_cnt     = s_if_win_cnt;
        g_foc_if_good_cnt    = s_if_good_wins;
        g_foc_if_win_min_rad = s_if_diff_min;
        g_foc_if_win_max_rad = s_if_diff_max;
    } else {
        g_foc_if_win_cnt     = 0u;
        g_foc_if_good_cnt    = 0u;
        g_foc_if_win_min_rad = s_if_diff_unwrapped;
        g_foc_if_win_max_rad = s_if_diff_unwrapped;
    }

    if (s_if_tick > ((uint32_t)FOC_IF_TIMEOUT_MS * FOC_ISR_HZ / 1000u)) {
        g_foc_if_evt    = 3u;
        g_foc_if_evt_v1 = (int32_t)(g_foc_if_freq_hz * 100.0f);
        g_foc_if_evt_v2 = (int32_t)g_foc_iq_ma;
        g_foc_if_evt_v3 = (int32_t)(g_foc_if_diff_rad * 1000.0f);
        g_foc_if_evt_v4 = (s_if_tick > 0u)
                        ? (int32_t)((float)s_if_wrap_cnt * (float)FOC_ISR_HZ
                                    / (float)s_if_tick * 100.0f)
                        : 0;
        Foc_Core_FaultStop(2u);
    }
}

static void Foc_IfHandover(const stc_i_data_t *pData)
{
    float theta = g_foc_theta_rad;
    float ctrl_theta = theta + ((g_foc_pi_off_180 != 0) ? FOC_MATH_PI : 0.0f);
    float id, iq;
    float vd_seed, vq_seed;

    s_run_blend_cnt  = 0u;
    s_run_blend_from = g_foc_theta_rad;

    Foc_Core_GetDq(pData, ctrl_theta, &id, &iq);

    vd_seed =  g_foc_valpha * Foc_Math_Cos(ctrl_theta) + g_foc_vbeta * Foc_Math_Sin(ctrl_theta);
    vq_seed = -g_foc_valpha * Foc_Math_Sin(ctrl_theta) + g_foc_vbeta * Foc_Math_Cos(ctrl_theta);
    if (vd_seed >  FOC_PI_UMAX_V) vd_seed =  FOC_PI_UMAX_V;
    if (vd_seed < -FOC_PI_UMAX_V) vd_seed = -FOC_PI_UMAX_V;
    if (vq_seed >  FOC_PI_UMAX_V) vq_seed =  FOC_PI_UMAX_V;
    if (vq_seed < -FOC_PI_UMAX_V) vq_seed = -FOC_PI_UMAX_V;

    PID_Seed(&s_pid_id, 0.0f,                     id, vd_seed);
    {
        float iq_seed = g_foc_run_iq_sign * (g_foc_pid_spd_cfg.kp * (g_enc_speed_rpm - g_foc_run_target_rpm));
        if (iq_seed >  (float)FOC_IQ_REF_MA) iq_seed =  (float)FOC_IQ_REF_MA;
        if (iq_seed < -(float)FOC_IQ_REF_MA) iq_seed = -(float)FOC_IQ_REF_MA;
        PID_Seed(&s_pid_iq, iq_seed * 0.001f, iq, vq_seed);
    }

    Foc_Core_ResetEma();
    PID_Reset(&s_pid_spd);

    g_foc_align_offset = Foc_Core_GetAlignOffset();

    Foc_Core_SetStateMachine(FOC_STATE_RUN);
    g_foc_phase       = 3u;
    g_foc_align_state = 2u;
    g_foc_if_sync     = 1u;
    g_foc_if_evt      = 2u;
    g_foc_if_evt_v1   = (int32_t)g_foc_align_offset;
    g_foc_if_evt_v2   = (int32_t)g_foc_iq_ma;
    g_foc_if_evt_v3   = (int32_t)(g_foc_if_freq_hz * 100.0f);
}

/*******************************************************************************
 * RUN: encoder-angle FOC current loop
 ******************************************************************************/
static void Foc_CurrentLoopStep(const stc_i_data_t *pData)
{
    float theta, ctrl_theta, id, iq, iq_ref_a, vd, vq, valpha, vbeta, du, dv, dw;

    if (Foc_Core_OverCurrent(pData)) {
        Foc_Core_FaultStop(1u);
        return;
    }

    {
        float step = g_foc_iq_ramp_ma_s / (float)FOC_ISR_HZ;
        if (g_foc_iq_ref_ma < g_foc_iq_ref_cmd_ma) {
            g_foc_iq_ref_ma += step;
            if (g_foc_iq_ref_ma > g_foc_iq_ref_cmd_ma) {
                g_foc_iq_ref_ma = g_foc_iq_ref_cmd_ma;
            }
        } else if (g_foc_iq_ref_ma > g_foc_iq_ref_cmd_ma) {
            g_foc_iq_ref_ma -= step;
            if (g_foc_iq_ref_ma < g_foc_iq_ref_cmd_ma) {
                g_foc_iq_ref_ma = g_foc_iq_ref_cmd_ma;
            }
        }
    }

    theta = Foc_Core_CurLoopTheta();
    if (s_run_blend_cnt < FOC_RUN_BLEND_CNT) {
        float d = theta - s_run_blend_from;
        if (d >  FOC_MATH_PI) d -= FOC_MATH_2PI;
        if (d < -FOC_MATH_PI) d += FOC_MATH_2PI;
        theta = s_run_blend_from + d * ((float)s_run_blend_cnt / (float)FOC_RUN_BLEND_CNT);
        s_run_blend_cnt++;
    }
    g_foc_theta_rad = theta;
    {
        float enc = (float)Foc_Core_ModPos(((int32_t)g_enc_count * (int32_t)g_foc_enc_dir),
                                           (int32_t)ENCODER_CPR)
                  * (FOC_MATH_2PI * (float)FOC_POLE_PAIRS / (float)ENCODER_CPR);
        enc -= (float)((int32_t)(enc * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
        if (enc < 0.0f) enc += FOC_MATH_2PI;
        g_foc_if_rotor_rad = enc;
        g_foc_if_diff_rad = enc - theta;
        while (g_foc_if_diff_rad >  FOC_MATH_PI) g_foc_if_diff_rad -= FOC_MATH_2PI;
        while (g_foc_if_diff_rad < -FOC_MATH_PI) g_foc_if_diff_rad += FOC_MATH_2PI;
        g_foc_if_rel_diff_rad = 0.0f;
    }
    ctrl_theta = theta + ((g_foc_pi_off_180 != 0) ? FOC_MATH_PI : 0.0f);

    Foc_Core_GetDq(pData, ctrl_theta, &id, &iq);
    Foc_Core_EmaFilter(&id, &iq);
    g_foc_id_ma = id * 1000.0f;
    g_foc_iq_ma = iq * 1000.0f;

    if (g_foc_pid_spd_cfg.kp > 0.0f) {
        iq_ref_a = g_foc_run_iq_sign * PID_UpdateUs(&s_pid_spd, g_enc_speed_rpm, g_foc_run_target_rpm,
                                                     FOC_ISR_DT_US) * 0.001f;
        g_foc_iq_pi_ma = iq_ref_a * 1000.0f;
    } else {
        iq_ref_a = g_foc_run_iq_sign * g_foc_iq_ref_ma * 0.001f;
        g_foc_iq_pi_ma = iq_ref_a * 1000.0f;
    }
    vd = PID_UpdateUs(&s_pid_id, 0.0f,     id, FOC_ISR_DT_US);
    vq = PID_UpdateUs(&s_pid_iq, iq_ref_a, iq, FOC_ISR_DT_US);
    g_foc_vd = vd;
    g_foc_vq = vq;

    Foc_Core_ApplyVoltageEnvelope(&vd, &vq);

    Foc_InvPark(vd, vq, ctrl_theta, &valpha, &vbeta);
    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);

    g_foc_valpha = valpha;
    g_foc_vbeta  = vbeta;
    g_foc_du = du;
    g_foc_dv = dv;
    g_foc_dw = dw;
}

/*******************************************************************************
 * Foc_CurLoop_Step - 模式22 状态机分派（20 kHz ISR 中调用）
 ******************************************************************************/
void Foc_CurLoop_Step(const stc_i_data_t *pData)
{
    switch (Foc_Core_GetStateMachine()) {
    case FOC_STATE_IF_START:
        Foc_IfStartStep(pData);
        break;
    case FOC_STATE_RUN:
        Foc_CurrentLoopStep(pData);
        break;
    default:
        break;
    }
}
