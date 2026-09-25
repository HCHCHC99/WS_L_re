/**
 *******************************************************************************
 * @file  foc_45_smo.c
 * @brief FOC mode 45 - SMO+PLL 无感速度/电流双闭环（分步开发中）。
 *
 * 第 1 步（当前，已完成）：完全复刻 mode 40 串级结构 —— 速度 PI 抽稀到 5ms，
 * 输出带符号 mA 作为 q 轴电流参考；id 参考 = 0；id/iq 电流 PI 在每个
 * 电流采样 ISR 用 TMRA_1 编码器转子角执行。
 * 新增 1：启动自动转速 profile（0/200/500/1000/1500/2000 rpm，每秒一档，
 * 5s 后保持；g_smo45_auto_ramp=0 可关闭，此后 Watch 直接改目标）。
 * 新增 2：SMO 纯旁观（foc_smo.c，见其头文件）—— 编码器闭环控制路径
 * 零改动，SMO 输入 = 上一拍指令电压 + 同源 Clarke 电流，输出
 * e_alpha_hat/e_beta_hat 与诊断量（|e| vs ωψf、atan2 角 vs 编码器角）。
 *
 * 第 2 步（当前）：PLL 旁观（foc_pll.c，独立模块）—— 消费 SMO 滤波后
 * e_hat，输出 theta_hat/omega_hat 与编码器对比；预期吃掉 SMO 的恒定
 * 滞后与 6f 马鞍纹波（theta_err_pll ≈ 0）。编码器闭环控制路径仍零改动。
 *******************************************************************************
 */

#include "foc_45_smo.h"
#include "foc_smo.h"
#include "foc_pll.h"
#include "foc_24_dcal.h"
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "hc32_ll_tmra.h"
#include <math.h>   /* sqrtf */

#define SMO45_ISR_DT_US     (1000000u / FOC_ISR_HZ)
#define SMO45_SPD_WIN_TICKS (SMO45_SPD_WIN_MS * FOC_ISR_HZ / 1000u)
#define SMO45_RAD2DEG       57.2958f

volatile uint8_t  g_smo45_running           = 0u;
volatile uint8_t  g_smo45_state             = SMO45_STEP_IDLE;
volatile uint8_t  g_smo45_evt               = 0u;
volatile uint8_t  g_smo45_auto_ramp         = SMO45_AUTO_RAMP_DEFAULT;
volatile uint8_t  g_smo45_wave_mode         = 0u;
volatile uint8_t  g_smo45_sensorless        = 0u;   /* Step 4：无感切换开关（0=有感） */
volatile float    g_smo45_ang_lead_ticks    = 1.5f; /* 高速角度超前补偿拍数（0=关） */
volatile float    g_smo45_speed_target_rpm  = 0.0f;
volatile float    g_smo45_speed_ramp_rpm    = 0.0f;
volatile float    g_smo45_speed_meas_rpm    = 0.0f;
volatile float    g_smo45_speed_filt_rpm    = 0.0f;
volatile float    g_smo45_speed_disp_rpm    = 0.0f;   /* 显示专用强滤波转速 */
volatile float    g_smo45_speed_err_rpm     = 0.0f;
volatile float    g_smo45_speed_out_ma      = 0.0f;
volatile int32_t  g_smo45_rotor_count       = 0;
volatile int32_t  g_smo45_rotor_deg         = 0;
volatile float    g_smo45_id_ref_ma         = 0.0f;
volatile float    g_smo45_iq_ref_ma         = 0.0f;
volatile float    g_smo45_id_ma             = 0.0f;
volatile float    g_smo45_iq_ma             = 0.0f;
volatile float    g_smo45_iq_filt_ma        = 0.0f;
volatile float    g_smo45_iq_filt_alpha     = SMO45_IQ_FILT_ALPHA;
volatile float    g_smo45_vd                = 0.0f;
volatile float    g_smo45_vq                = 0.0f;
volatile uint8_t  g_smo45_vsat              = 0u;
volatile float    g_smo45_du                = 50.0f;
volatile float    g_smo45_dv                = 50.0f;
volatile float    g_smo45_dw                = 50.0f;

pid_config_t g_smo45_pid_speed_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = SMO45_SPD_KP_MA_PER_RPM,
    .ki           = SMO45_SPD_KI_MA_PER_RPM_S,
    .kd           = 0.0f,
    .output_min   = -SMO45_SPD_IQ_LIMIT_MA,
    .output_max   =  SMO45_SPD_IQ_LIMIT_MA,
    .integral_max =  SMO45_SPD_IQ_LIMIT_MA,
    .i_term_max   =  SMO45_SPD_IQ_LIMIT_MA,
    .update_ms    = 0u,
};

pid_config_t g_smo45_pid_id_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = SMO45_PI_KP,
    .ki           = SMO45_PI_KI,
    .kd           = 0.0f,
    .output_min   = -SMO45_PI_UMAX_V,
    .output_max   =  SMO45_PI_UMAX_V,
    .integral_max =  SMO45_ITERM_MAX_V,
    .i_term_max   =  SMO45_ITERM_MAX_V,
    .update_ms    = 0u,
};

pid_config_t g_smo45_pid_iq_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = SMO45_PI_KP,
    .ki           = SMO45_PI_KI,
    .kd           = 0.0f,
    .output_min   = -SMO45_PI_UMAX_V,
    .output_max   =  SMO45_PI_UMAX_V,
    .integral_max =  SMO45_ITERM_MAX_V,
    .i_term_max   =  SMO45_ITERM_MAX_V,
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
static float    s_speed_ramp_rpm;
static float    s_speed_filt_rpm;
static uint8_t  s_speed_disp_init;
static float    s_speed_disp_rpm;
static float    s_speed_out_ma;

/* 启动自动转速 profile：秒 0/1/2/3 -> 0/200/500/1000 rpm，之后保持 1000
 * （Step 4 无感首切定在 1000rpm：PLL 信噪比窗口内、打摆扰动更小） */
static uint32_t s_run_ticks;
static const uint16_t s_auto_rpm[SMO45_AUTO_RAMP_SECS + 1u] = {
    0u, 200u, 500u, 1000u
};

/* SMO 旁观者电压输入缓冲：采样时刻实际施加的是上一拍指令电压 */
static float s_v_alpha_prev;
static float s_v_beta_prev;

/* Step 4 无感锁存状态（全局供 Watch/打印观察）：开关置 1 且 emf_ok=1 时进入，
 * 进入后不因 emf_ok 瞬时掉 0 跌回有感（防角度源来回跳变）；关开关/停机退出 */
volatile uint8_t g_smo45_sl_active = 0u;

static void Smo45_ResetLoopState(void)
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
    s_run_ticks = 0u;
    s_v_alpha_prev = 0.0f;
    s_v_beta_prev = 0.0f;
}

static void Smo45_ClearObservables(void)
{
    g_smo45_evt = 0u;
    g_smo45_speed_ramp_rpm = 0.0f;
    g_smo45_speed_meas_rpm = 0.0f;
    g_smo45_speed_filt_rpm = 0.0f;
    g_smo45_speed_disp_rpm = 0.0f;
    g_smo45_speed_err_rpm = 0.0f;
    g_smo45_speed_out_ma = 0.0f;
    g_smo45_rotor_count = 0;
    g_smo45_rotor_deg = 0;
    g_smo45_id_ref_ma = 0.0f;
    g_smo45_iq_ref_ma = 0.0f;
    g_smo45_id_ma = 0.0f;
    g_smo45_iq_ma = 0.0f;
    g_smo45_iq_filt_ma = 0.0f;
    g_smo45_vd = 0.0f;
    g_smo45_vq = 0.0f;
    g_smo45_vsat = 0u;
    g_smo45_du = 50.0f;
    g_smo45_dv = 50.0f;
    g_smo45_dw = 50.0f;
}

static void Smo45_ClearCurrentFeedback(void)
{
    g_smo45_id_ma = 0.0f;
    g_smo45_iq_ma = 0.0f;
    g_smo45_iq_filt_ma = 0.0f;
    g_smo45_id_ref_ma = 0.0f;
    g_smo45_iq_ref_ma = 0.0f;
    g_smo45_vd = 0.0f;
    g_smo45_vq = 0.0f;
    g_foc_id_ma = 0.0f;
    g_foc_iq_ma = 0.0f;
    g_foc_vd = 0.0f;
    g_foc_vq = 0.0f;
}

static stc_i_data_t Smo45_CorrectedData(const stc_i_data_t *pData)
{
    stc_i_data_t data = *pData;
    data.i16IU_mA = (int16_t)((float)pData->i16IU_mA - s_zero_u_ma);
    data.i16IV_mA = (int16_t)((float)pData->i16IV_mA - s_zero_v_ma);
    data.i16IW_mA = (int16_t)((float)pData->i16IW_mA - s_zero_w_ma);
    return data;
}

static float Smo45_LimitSpeedRPM(float rpm)
{
    if (rpm > SMO45_SPEED_REF_LIMIT_RPM) {
        rpm = SMO45_SPEED_REF_LIMIT_RPM;
    }
    if (rpm < -SMO45_SPEED_REF_LIMIT_RPM) {
        rpm = -SMO45_SPEED_REF_LIMIT_RPM;
    }
    return rpm;
}

/* 启动自动转速 profile：前 5s 每秒抬一档目标，之后停止写入（Watch 接管）。
 * s_run_ticks 每 ISR 拍 +1，20kHz -> 整秒 = s_run_ticks / FOC_ISR_HZ */
static void Smo45_AutoRamp(void)
{
    uint32_t sec;

    if (g_smo45_auto_ramp == 0u) {
        return;
    }
    sec = s_run_ticks / (uint32_t)FOC_ISR_HZ;
    if (sec <= SMO45_AUTO_RAMP_SECS) {
        g_smo45_speed_target_rpm = (float)s_auto_rpm[sec];
    }
}

static void Smo45_UpdateSpeed(int32_t corrected_delta)
{
    float raw_rpm;

    s_speed_acc_cnt += corrected_delta;
    if (++s_speed_win_tick < SMO45_SPD_WIN_TICKS) {
        return;
    }

    raw_rpm = (float)s_speed_acc_cnt * 60.0f
            * (1000.0f / (float)SMO45_SPD_WIN_MS)
            / (float)ENCODER_CPR;
    if (s_speed_filt_init == 0u) {
        s_speed_filt_rpm = raw_rpm;
        s_speed_filt_init = 1u;
    } else {
        s_speed_filt_rpm += SMO45_SPD_FILT_ALPHA
                          * (raw_rpm - s_speed_filt_rpm);
    }
    /* 显示专用强滤波（独立于 PI 反馈链，α=0.05 仅平滑曲线） */
    if (s_speed_disp_init == 0u) {
        s_speed_disp_rpm = raw_rpm;
        s_speed_disp_init = 1u;
    } else {
        s_speed_disp_rpm += SMO45_SPD_DISP_ALPHA
                          * (raw_rpm - s_speed_disp_rpm);
    }

    g_smo45_speed_meas_rpm = raw_rpm;
    g_smo45_speed_filt_rpm = s_speed_filt_rpm;
    g_smo45_speed_disp_rpm = s_speed_disp_rpm;
    s_speed_acc_cnt = 0;
    s_speed_win_tick = 0u;
}

static void Smo45_UpdateOuterLoop(void)
{
    float target_rpm;
    float ramp_step;
    float speed_error;
    float speed_out;

    target_rpm = Smo45_LimitSpeedRPM(g_smo45_speed_target_rpm);
    ramp_step = SMO45_ACCEL_LIMIT_RPM_S
              * ((float)SMO45_SPD_WIN_MS * 0.001f);
    if (s_speed_ramp_rpm < target_rpm) {
        s_speed_ramp_rpm += ramp_step;
        if (s_speed_ramp_rpm > target_rpm) s_speed_ramp_rpm = target_rpm;
    } else if (s_speed_ramp_rpm > target_rpm) {
        s_speed_ramp_rpm -= ramp_step;
        if (s_speed_ramp_rpm < target_rpm) s_speed_ramp_rpm = target_rpm;
    }

    speed_error = s_speed_ramp_rpm - s_speed_filt_rpm;
    /* Step 4：无感锁存时速度反馈换 PLL 低通转速估计（其余逻辑不变） */
    {
        float speed_fb = s_speed_filt_rpm;

        if (g_smo45_sensorless != 0u && g_smo_emf_ok != 0u) {
            g_smo45_sl_active = 1u;
        }
        if (g_smo45_sensorless == 0u) {
            g_smo45_sl_active = 0u;
        }
        if (g_smo45_sl_active != 0u) {
            speed_fb = g_pll_omega_out_rpm;
        }
        speed_error = s_speed_ramp_rpm - speed_fb;
        speed_out = PID_UpdateUs(&s_speed_pid, s_speed_ramp_rpm,
                                 speed_fb, SMO45_SPD_WIN_US);
    }

    g_smo45_speed_ramp_rpm = s_speed_ramp_rpm;
    g_smo45_speed_err_rpm = speed_error;
    g_smo45_speed_out_ma = speed_out;
    g_smo45_iq_ref_ma = speed_out;
    g_smo45_id_ref_ma = 0.0f;
}

void Foc_Smo45_InitPids(void)
{
    PID_Init(&s_speed_pid, &g_smo45_pid_speed_cfg);
    PID_Init(&s_pid_id, &g_smo45_pid_id_cfg);
    PID_Init(&s_pid_iq, &g_smo45_pid_iq_cfg);
}

void Foc_Smo45_SetTargetRPM(float target_rpm)
{
    g_smo45_speed_target_rpm = Smo45_LimitSpeedRPM(target_rpm);
}

void Foc_Smo45_Start(void)
{
    foc_dcal24_result_t calibration;
    uint16_t hardware_count;
    int32_t encoder_dir;

    if (Foc_Dcal24_GetResult(&calibration) == 0u) {
        g_smo45_running = 0u;
        g_smo45_state = SMO45_STEP_IDLE;
        SMO45_LOG("ERROR: no mode 24 calibration; run mode 24 first");
        return;
    }

    Foc_Core_ClearFault();
    Smo45_ResetLoopState();
    Smo45_ClearObservables();
    Foc_Smo_Reset();   /* SMO 旁观者状态清零（观测电流/e_hat/诊断） */
    Foc_Pll_Reset();   /* PLL 旁观者状态清零（θ̂/ω̂/诊断） */
    g_smo45_sl_active = 0u;  /* 无感锁存退出（每次 Start 从有感闭环起步） */
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
    g_smo45_rotor_count = s_rotor_count;

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

    g_smo45_speed_target_rpm = 0.0f;
    g_smo45_running = 1u;
    g_smo45_state = SMO45_STEP_RUN;
    Foc_Core_PwmStart();
    SMO45_LOG("start rot=%d cnt limit=%.0frpm accel=%.0frpm/s iq=%.0fmA auto=%u",
              (int)s_rotor_count,
              (double)SMO45_SPEED_REF_LIMIT_RPM,
              (double)SMO45_ACCEL_LIMIT_RPM_S,
              (double)SMO45_SPD_IQ_LIMIT_MA,
              (unsigned)g_smo45_auto_ramp);
}

void Foc_Smo45_Stop(void)
{
    if (g_smo45_running) {
        g_smo45_running = 0u;
        g_foc_align_state = 0u;
        if (g_foc_active) {
            g_foc_active = 0u;
            Foc_Core_PwmStop();
            Foc_Core_SetStateMachine(FOC_STATE_IDLE);
        }
        SMO45_LOG("stopped");
    }
    g_smo45_sl_active = 0u;   /* 停机退出无感锁存（下次 Start 从有感起步） */
    Smo45_ClearCurrentFeedback();
}

void Foc_Smo45_Step(const stc_i_data_t *pData)
{
    stc_i_data_t data;
    float id, iq, vd, vq, valpha, vbeta, du, dv, dw;
    float cos_r, sin_r, rotor_rad, rotor_use;
    float ialpha_s, ibeta_s;   /* SMO 输入电流（Clarke 后，跨块传递到 SVPWM 后调用点） */
    uint16_t hardware_count;
    int32_t hardware_delta, corrected_delta, rotor_deg;

    if (Foc_Core_OverCurrent(pData)) {
        g_smo45_state = SMO45_STEP_FAULT_OC;
        g_smo45_evt = SMO45_EVT_OC;
        g_smo45_running = 0u;
        Foc_Core_FaultStop(1u);
        Smo45_ClearCurrentFeedback();
        return;
    }

    if (g_smo45_state != SMO45_STEP_RUN) {
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
    if (hardware_delta > SMO45_ENC_DELTA_MAX) {
        hardware_delta = SMO45_ENC_DELTA_MAX;
    }
    if (hardware_delta < -SMO45_ENC_DELTA_MAX) {
        hardware_delta = -SMO45_ENC_DELTA_MAX;
    }
    corrected_delta = hardware_delta * (int32_t)g_foc_enc_dir;
    s_rotor_count = Foc_Core_ModPos(s_rotor_count + corrected_delta,
                                    (int32_t)ENCODER_CPR);
    g_smo45_rotor_count = s_rotor_count;
    s_run_ticks++;
    Smo45_AutoRamp();
    Smo45_UpdateSpeed(corrected_delta);
    Smo45_UpdateOuterLoop();

    rotor_rad = (float)s_rotor_count
              * (FOC_MATH_2PI * (float)FOC_POLE_PAIRS / (float)ENCODER_CPR);
    rotor_rad -= (float)((int32_t)(rotor_rad * (1.0f / FOC_MATH_2PI)))
               * FOC_MATH_2PI;
    if (rotor_rad < 0.0f) rotor_rad += FOC_MATH_2PI;

    /* 高速角度超前补偿：Park 角要对齐"电压矢量作用中心"= 采样后 ~1.5 拍
     * （中心对齐 PWM + 谷点采样）。4500rpm 电频率 750Hz 下一拍=27°，不补
     * 则功角偏差 27~40° → 转矩折损+失步（实测 6000 掉 3000 的帮凶）。
     * g_smo45_ang_lead_ticks Watch 可调（0=关，1.5=默认），低速无影响。 */
    if (g_smo45_ang_lead_ticks > 0.0f && g_smo45_sl_active == 0u) {
        float w_e = g_smo45_speed_filt_rpm * 0.10472f * (float)FOC_POLE_PAIRS;
        rotor_rad += w_e * SMO45_ISR_DT_US * 1.0e-6f * g_smo45_ang_lead_ticks;
        rotor_rad -= (float)((int32_t)(rotor_rad * (1.0f / FOC_MATH_2PI)))
                   * FOC_MATH_2PI;
        if (rotor_rad < 0.0f) rotor_rad += FOC_MATH_2PI;
    }

    /* Step 4：无感锁存时 Park 角换 PLL 补偿外推角 g_pll_theta_park_deg
     *（含滞后补偿+一拍外推）。编码器角 rotor_rad 保留作裁判（Compare 入参）。
     * 锁存进条件：开关置 1 且 SMO 判定 ok；退出：开关回 0（防 emf_ok 瞬时
     * 掉 0 造成角度源来回跳变）。 */
    rotor_use = rotor_rad;
    if (g_smo45_sensorless != 0u && g_smo_emf_ok != 0u) {
        g_smo45_sl_active = 1u;
    }
    if (g_smo45_sensorless == 0u) {
        g_smo45_sl_active = 0u;
    }
    if (g_smo45_sl_active != 0u) {
        rotor_use = g_pll_theta_park_deg * (FOC_MATH_2PI / 360.0f);
    }

    data = Smo45_CorrectedData(pData);
    Foc_Core_GetDq(&data, rotor_use, &id, &iq);
    g_smo45_id_ma = id * 1000.0f;
    g_smo45_iq_ma = iq * 1000.0f;
    {
        float alpha = g_smo45_iq_filt_alpha;
        if (alpha <= 0.0f || alpha > 1.0f) {
            alpha = 1.0f;
        }
        if (s_iq_filt_init == 0u) {
            g_smo45_iq_filt_ma = g_smo45_iq_ma;
            s_iq_filt_init = 1u;
        } else {
            g_smo45_iq_filt_ma += alpha
                                    * (g_smo45_iq_ma - g_smo45_iq_filt_ma);
        }
    }
    g_foc_id_ma = g_smo45_id_ma;
    g_foc_iq_ma = g_smo45_iq_ma;

    /* SMO 输入电流：与控制路径完全同源（sign × 零偏校正副本 -> Foc_Clarke）。
     * 电压在下方 SVPWM 之后取"前后两拍指令平均"再调用 SMO */
    {
        float sgn = (float)g_foc_cur_sign;
        float ia_s = (float)data.i16IU_mA * 0.001f * sgn;
        float ib_s = (float)data.i16IV_mA * 0.001f * sgn;
        float ic_s = (float)data.i16IW_mA * 0.001f * sgn;
        Foc_Clarke(ia_s, ib_s, ic_s, &ialpha_s, &ibeta_s);
    }

    vd = PID_UpdateUs(&s_pid_id, g_smo45_id_ref_ma * 0.001f, id,
                      SMO45_ISR_DT_US);
    vq = PID_UpdateUs(&s_pid_iq, g_smo45_iq_ref_ma * 0.001f, iq,
                      SMO45_ISR_DT_US);
    g_foc_vd = vd;
    g_foc_vq = vq;
    g_smo45_vd = vd;
    g_smo45_vq = vq;
    g_smo45_vsat = ((vd <= -SMO45_PI_UMAX_V + 0.01f) ||
                    (vd >=  SMO45_PI_UMAX_V - 0.01f) ||
                    (vq <= -SMO45_PI_UMAX_V + 0.01f) ||
                    (vq >=  SMO45_PI_UMAX_V - 0.01f)) ? 1u : 0u;

    cos_r = Foc_Math_Cos(rotor_use);
    sin_r = Foc_Math_Sin(rotor_use);
    valpha = vd * cos_r - vq * sin_r;
    vbeta  = vd * sin_r + vq * cos_r;
    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);

    g_foc_valpha = valpha;
    g_foc_vbeta = vbeta;
    /* SMO 旁观者调用：中心对齐 PWM + 谷点采样下，采样区间 [t_k, t_k+1] 上
     * 实际施加的电压 = 前后两拍指令各占一半（前半旧占空比/后半新占空比），
     * 喂平均值——单喂 v_prev 会引入 ~1 拍的传输延迟（1500rpm 时 ~9° 系统滞后） */
    Foc_Smo_Step((s_v_alpha_prev + valpha) * 0.5f,
                 (s_v_beta_prev + vbeta) * 0.5f, ialpha_s, ibeta_s);
    /* PLL 旁观者：消费 SMO 滤波后 e_hat（独立模块，见 foc_pll.h）；
     * 紧跟同拍角度比较（θ̂ 每拍转 ~9°电角，跨拍比较会撕裂出 ±9° 假差） */
    Foc_Pll_Step(g_smo_e_alpha_hat_v, g_smo_e_beta_hat_v);
    Foc_Pll_Compare(rotor_rad * SMO45_RAD2DEG);
    s_v_alpha_prev = valpha;   /* 缓冲本拍指令，下一拍参与平均 */
    s_v_beta_prev = vbeta;
    g_foc_theta_rad = rotor_rad;
    g_foc_du = du;
    g_foc_dv = dv;
    g_foc_dw = dw;
    g_smo45_du = du;
    g_smo45_dv = dv;
    g_smo45_dw = dw;

    rotor_deg = (int32_t)(rotor_rad * SMO45_RAD2DEG);
    if (rotor_deg >= 360) rotor_deg -= 360;
    g_smo45_rotor_deg = rotor_deg;
}

/*===========================================================================
 * 模式自持 VOFA：三种布局由 g_smo45_wave_mode 选择，
 * 通道含义见 foc_45_smo.h 顶部速览卡（唯一事实源）。
 *===========================================================================*/
int Foc_Smo45_VofaFill(int32_t *cur)
{
    if (g_smo45_wave_mode == 2u) {
        /*===== wave_mode=2：PLL/无感验收面板（16ch）=====*/
        float pll_emag = sqrtf(g_smo_e_alpha_hat_v * g_smo_e_alpha_hat_v
                             + g_smo_e_beta_hat_v * g_smo_e_beta_hat_v);

        cur[0]  = (int32_t)(g_smo45_speed_meas_rpm * 1000.0f);   /* mrpm -> rpm */
        cur[1]  = (int32_t)(g_smo45_speed_filt_rpm * 1000.0f);   /* mrpm -> rpm */
        cur[2]  = (int32_t)(g_pll_omega_hat_rpm * 1000.0f);      /* mrpm -> rpm */
        cur[3]  = (int32_t)(g_pll_omega_out_rpm * 1000.0f);      /* mrpm -> rpm */
        cur[4]  = (int32_t)((g_pll_omega_hat_rpm
                           - g_smo45_speed_meas_rpm) * 1000.0f); /* mrpm -> rpm */
        cur[5]  = (int32_t)((g_pll_omega_out_rpm
                           - g_smo45_speed_filt_rpm) * 1000.0f); /* mrpm -> rpm */
        cur[6]  = (int32_t)(g_pll_diag_theta_err_deg * 1000.0f); /* mdeg -> deg */
        cur[7]  = (int32_t)(g_pll_diag_theta_err_filt_deg * 1000.0f);
        cur[8]  = (int32_t)(g_smo45_iq_filt_ma);                 /* mA -> A */
        cur[9]  = (int32_t)(g_pll_comp_deg_out * 1000.0f);       /* mdeg -> deg */
        cur[10] = (int32_t)(g_smo45_sl_active * 1000.0f);        /* 0/1 */
        cur[11] = (int32_t)(pll_emag * 1000.0f);                 /* mV -> V */
        cur[12] = (int32_t)(g_pll_diag_smo_err_deg * 1000.0f);   /* mdeg -> deg */
        cur[13] = (int32_t)(g_smo45_speed_target_rpm * 1000.0f); /* mrpm -> rpm */
        cur[14] = (int32_t)(g_smo45_speed_disp_rpm * 1000.0f);   /* 显示滤波转速 */
        cur[15] = (int32_t)(g_smo45_speed_ramp_rpm * 1000.0f);   /* 斜坡输出转速
                                              * (mrpm -> rpm)：与 ch13 目标对比看
                                              * 斜坡限幅段，与 ch1 反馈对比看环路 */
        return 16;
    } else if (g_smo45_wave_mode != 0u) {
        /*===== wave_mode=1：波形窄帧（8ch=32B，~2.9kHz 帧率）=====*/
        cur[0] = (int32_t)(g_smo_z_alpha_v * 1000.0f);      /* mV -> V */
        cur[1] = (int32_t)(g_smo_z_beta_v * 1000.0f);       /* mV -> V */
        cur[2] = (int32_t)(g_smo_e_alpha_hat_v * 1000.0f);  /* mV -> V */
        cur[3] = (int32_t)(g_smo_e_beta_hat_v * 1000.0f);   /* mV -> V */
        cur[4] = (int32_t)(g_smo_theory_alpha_v * 1000.0f); /* mV -> V */
        cur[5] = (int32_t)(g_smo_theory_beta_v * 1000.0f);  /* mV -> V */
        cur[6] = (int32_t)(g_smo_diag_theta_err_deg * 1000.0f);      /* mdeg -> deg */
        cur[7] = (int32_t)(g_smo_diag_theta_err_filt_deg * 1000.0f); /* mdeg -> deg */
        return 8;
    } else {
        /*===== wave_mode=0：SMO 验收面板（16ch）=====*/
        cur[0] = (int32_t)(g_smo45_iq_ma);                      /* ch0 当前 iq (mA -> A) */
        cur[1] = (int32_t)(g_smo45_iq_filt_ma);                 /* ch1 滤波后 iq (mA -> A) */
        cur[2] = (int32_t)(g_smo45_iq_ref_ma);                  /* ch2 目标 iq (mA -> A) */
        cur[3] = (int32_t)(g_smo_theory_alpha_v * 1000.0f);     /* ch3 理论 e_alpha (mV -> V) */
        cur[4] = (int32_t)(g_smo_theory_beta_v * 1000.0f);      /* ch4 理论 e_beta (mV -> V) */
        cur[5] = (int32_t)(g_smo_z_alpha_v * 1000.0f);          /* ch5 SMO 原始 z_alpha (mV -> V) */
        cur[6] = (int32_t)(g_smo_z_beta_v * 1000.0f);           /* ch6 SMO 原始 z_beta (mV -> V) */
        cur[7] = (int32_t)(g_smo_e_alpha_hat_v * 1000.0f);      /* ch7 滤波 e_alpha_hat (mV -> V) */
        cur[8] = (int32_t)(g_smo_e_beta_hat_v * 1000.0f);       /* ch8 滤波 e_beta_hat (mV -> V) */
        cur[9] = (int32_t)(g_smo_diag_theta_err_deg * 1000.0f); /* ch9 相位误差 (mdeg -> deg) */
        cur[10] = (int32_t)(g_smo_diag_e_on_q_v * 1000.0f);     /* ch10 e_on_q 投影 (mV -> V) */
        cur[11] = (int32_t)(g_smo_diag_e_on_d_v * 1000.0f);     /* ch11 e_on_d 投影 (mV -> V) */
        cur[12] = (int32_t)(g_smo_diag_e_err_v * 1000.0f);      /* ch12 总误差范数 (mV -> V) */
        cur[13] = (int32_t)(g_smo_diag_e_expect_v * 1000.0f);   /* ch13 理论幅值 ωψf (mV -> V) */
        cur[14] = (int32_t)(g_smo45_speed_filt_rpm * 1000.0f);  /* ch14 实际转速 (mrpm -> rpm) */
        cur[15] = (int32_t)(g_smo_diag_theta_err_filt_deg * 1000.0f); /* ch15 相位差滤波 (mdeg -> deg) */
        return 16;
    }
}
