/**
 *******************************************************************************
 * @file  foc_iq_pi.c
 * @brief FOC 模式31 — 编码器转子角度 PI 电流环实现。
 *
 *        转子电角度：直接读取 TIMERA_1 硬件计数（不依赖主循环
 *        Encoder_Update()），并扣除 mode 30 (ZIZENG) 锁定的基线偏移，
 *        得到绝对化转子电角度用于 Park/InvPark。
 *
 *        电流路径与 ZIZENG 一致：foc_calib 残余零偏扣除 (dataCal 副本)
 *        -> Clarke/Park -> EMA -> PI；过流保护仍用原始 pData。
 *******************************************************************************
 */

#include "foc_iq_pi.h"
#include "foc_math.h"
#include "foc_calib.h"
#include "foc_zizeng.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "rtt_log.h"
#include "hc32_ll_tmra.h"   /* 直接读取 TIMERA_1 计数器 */

/* ISR period in us */
#define FOC_ISR_DT_US  (1000000u / FOC_ISR_HZ)

/*******************************************************************************
 * Keil Watch 可调变量 / 观测量
 ******************************************************************************/
volatile uint8_t g_iqpi_running        = 0;
volatile float   g_iqpi_iq_ref_ma      = IQPI_IQ_REF_MA;
volatile float   g_iqpi_iq_ref_ramp_ma = 0.0f;
volatile float   g_iqpi_iq_ramp_ma_s   = IQPI_IQ_RAMP_MA_S;
volatile float   g_iqpi_theta_rad      = 0.0f;

/*******************************************************************************
 * d/q 轴 PI 配置（初值来自 foc_iq_pi.h 宏，字段 volatile 可 Watch 实时修改）
 ******************************************************************************/
pid_config_t g_iqpi_pid_id_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = IQPI_PI_KP,
    .ki           = IQPI_PI_KI,
    .kd           = 0.0f,
    .output_min   = -IQPI_PI_UMAX_V,
    .output_max   =  IQPI_PI_UMAX_V,
    .integral_max = IQPI_INTEGRAL_MAX,
    .i_term_max   = 0.0f,
    .update_ms    = 0,
};

pid_config_t g_iqpi_pid_iq_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = IQPI_PI_KP,
    .ki           = IQPI_PI_KI,
    .kd           = 0.0f,
    .output_min   = -IQPI_PI_UMAX_V,
    .output_max   =  IQPI_PI_UMAX_V,
    .integral_max = IQPI_INTEGRAL_MAX,
    .i_term_max   = 0.0f,
    .update_ms    = 0,
};

/* PI runtime states */
static pid_state_t s_pid_id;
static pid_state_t s_pid_iq;

/* 编码器硬件计数跟踪（与 foc_zizeng 相同的回绕处理，独立状态） */
static int32_t s_prev_hw_cnt = 0;
static int32_t s_enc_pos = 0;
static uint8_t s_enc_initialized = 0;

/* 启动时捕获的 ZIZENG 偏移基线 (rad) */
static float s_zizeng_off_rad = 0.0f;

/*******************************************************************************
 * Foc_IqPi_InitPids - 绑定 PI 实例与配置（Foc_Init 调用一次）
 ******************************************************************************/
void Foc_IqPi_InitPids(void)
{
    PID_Init(&s_pid_id, &g_iqpi_pid_id_cfg);
    PID_Init(&s_pid_iq, &g_iqpi_pid_iq_cfg);
}

/*******************************************************************************
 * Foc_StartIqPi - mode 31 启动（需 mode 30 已锁定偏移）
 ******************************************************************************/
void Foc_StartIqPi(void)
{
    if (!Foc_Zizeng_GetOffsetRad(&s_zizeng_off_rad)) {
        MAIN_D("[IQPI] ERROR: ZIZENG offset not locked, run mode 30 first");
        return;
    }

    Foc_Core_ClearFault();
    g_iqpi_running      = 1;
    g_iqpi_theta_rad    = 0.0f;
    g_iqpi_iq_ref_ramp_ma = 0.0f;

    s_enc_initialized = 0;
    s_enc_pos = 0;

    TMR4_PWM_SetFocMode(FOC_DEADTIME_NS);
    g_foc_du = 50.0f;
    g_foc_dv = 50.0f;
    g_foc_dw = 50.0f;
    TMR4_PWM_SetDuty3Phase(50.0f, 50.0f, 50.0f);
    TMR4_PWM_StartOutput();
    g_foc_active = 1u;
    g_foc_mode   = FOC_MODE_NONE;

    /* 相电流 DC 零偏自校准：本函数返回后的 ~210ms 内强制零矢量，
     * 锁定后 iq_ref 从 0 斜坡启动 */
    Foc_Calib_Start();

    Foc_Core_ResetEma();
    PID_Reset(&s_pid_id);
    PID_Reset(&s_pid_iq);

    MAIN_D("[IQPI] Started: off=%.3f rad, iq_ref=%d mA, kp=%d m, ki=%d",
           s_zizeng_off_rad, (int)g_iqpi_iq_ref_ma,
           (int)(g_iqpi_pid_iq_cfg.kp * 1000.0f),
           (int)g_iqpi_pid_iq_cfg.ki);
}

/*******************************************************************************
 * Foc_StopIqPi - mode 31 停止
 ******************************************************************************/
void Foc_StopIqPi(void)
{
    g_iqpi_running = 0;
    g_foc_active   = 0u;
    TMR4_PWM_EmergencyStop();
    g_foc_du = 0.0f;
    g_foc_dv = 0.0f;
    g_foc_dw = 0.0f;
    MAIN_D("[IQPI] Stopped");
}

/*******************************************************************************
 * Foc_IqPi_Step - 模式31 单步运算（20 kHz ISR 中调用）
 ******************************************************************************/
void Foc_IqPi_Step(const stc_i_data_t *pData)
{
    float enc_elec, id, iq, iq_ref_a, vd, vq, valpha, vbeta, du, dv, dw;
    float off_u, off_v, off_w;
    uint32_t hw_cnt;
    int16_t delta;
    stc_i_data_t dataCal;   /* 零偏校正后的电流采样副本 */

    if (!g_iqpi_running) {
        return;
    }

    /* 过流保护（原始 pData） */
    if (Foc_Core_OverCurrent(pData)) {
        Foc_Core_FaultStop(1u);
        g_iqpi_running = 0;
        return;
    }

    /* ===== 0. 编码器硬件计数（与 ZIZENG 相同的回绕处理） ===== */
    hw_cnt = TMRA_GetCountValue(CM_TMRA_1);
    if (!s_enc_initialized) {
        s_prev_hw_cnt = (int32_t)hw_cnt;
        s_enc_pos = 0;
        s_enc_initialized = 1;
    }
    delta = (int16_t)((uint16_t)hw_cnt - (uint16_t)s_prev_hw_cnt);
    s_prev_hw_cnt = (int32_t)hw_cnt;
    s_enc_pos += (int32_t)delta;
    g_enc_count = s_enc_pos;
    g_enc_count_f = (float)s_enc_pos;

    /* ===== 1. 相电流 DC 零偏自校准（零矢量窗口，非阻塞） =====
     * 锁定前三相 50% 占空比（零电流），锁定后才开始闭环。 */
    if (!Foc_Calib_IsLocked()) {
        Foc_Calib_Feed(pData);
        TMR4_PWM_SetDuty3Phase(50.0f, 50.0f, 50.0f);
        g_foc_du = 50.0f;
        g_foc_dv = 50.0f;
        g_foc_dw = 50.0f;
        g_foc_id_ma = 0.0f;
        g_foc_iq_ma = 0.0f;
        return;
    }

    /* ===== 2. 转子电角度（扣 ZIZENG 偏移基线，绝对化） ===== */
    enc_elec = (float)Foc_Core_ModPos(s_enc_pos * (int32_t)g_foc_enc_dir,
                                      (int32_t)ENCODER_CPR)
             * (FOC_MATH_2PI * (float)FOC_POLE_PAIRS / (float)ENCODER_CPR);
    enc_elec -= (float)((int32_t)(enc_elec * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
    if (enc_elec < 0.0f) {
        enc_elec += FOC_MATH_2PI;
    }
    enc_elec -= s_zizeng_off_rad;
    /* ZIZENG q 轴拖动锁定瞬间：电流矢量在 theta+90°，转子 d 轴滞后一个负载
     * 角 delta，故锁定基线 offset = c + 90° - delta，减基线后角度
     * = 转子角 - 90° + delta。这里补回 90° 使 Park 角对准真实转子 d 轴
     * （残余误差仅为 delta 几度）。若不补，iq 电流矢量仅超前转子 delta，
     * 转矩不足以克服静摩擦 -> 堵转（iq 有读数但转子不动）。 */
    enc_elec += FOC_MATH_HALF_PI;
    enc_elec -= (float)((int32_t)(enc_elec * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
    if (enc_elec < 0.0f) {
        enc_elec += FOC_MATH_2PI;
    }
    g_iqpi_theta_rad = enc_elec;
    g_foc_theta_rad    = enc_elec;
    g_foc_if_rotor_rad = enc_elec;

    /* ===== 3. 相电流零偏扣除 -> Clarke/Park -> EMA =====
     * 过流保护等仍使用原始 pData，不受影响。 */
    dataCal = *pData;
    Foc_Calib_GetOffsetsMa(&off_u, &off_v, &off_w);
    dataCal.i16IU_mA = (int16_t)((float)pData->i16IU_mA - off_u);
    dataCal.i16IV_mA = (int16_t)((float)pData->i16IV_mA - off_v);
    dataCal.i16IW_mA = (int16_t)((float)pData->i16IW_mA - off_w);

    Foc_Core_GetDq(&dataCal, enc_elec, &id, &iq);
    Foc_Core_EmaFilter(&id, &iq);
    g_foc_id_ma = id * 1000.0f;
    g_foc_iq_ma = iq * 1000.0f;

    /* ===== 4. iq 软启动斜坡（Watch 改 g_iqpi_iq_ref_ma 实时生效） ===== */
    {
        float step = g_iqpi_iq_ramp_ma_s / (float)FOC_ISR_HZ;
        if (g_iqpi_iq_ref_ramp_ma < g_iqpi_iq_ref_ma) {
            g_iqpi_iq_ref_ramp_ma += step;
            if (g_iqpi_iq_ref_ramp_ma > g_iqpi_iq_ref_ma) {
                g_iqpi_iq_ref_ramp_ma = g_iqpi_iq_ref_ma;
            }
        } else if (g_iqpi_iq_ref_ramp_ma > g_iqpi_iq_ref_ma) {
            g_iqpi_iq_ref_ramp_ma -= step;
            if (g_iqpi_iq_ref_ramp_ma < g_iqpi_iq_ref_ma) {
                g_iqpi_iq_ref_ramp_ma = g_iqpi_iq_ref_ma;
            }
        }
    }

    /* ===== 5. 双 PI 闭环：vd = PI_id(0, id)，vq = PI_iq(iq_ref, iq) ===== */
    iq_ref_a = g_iqpi_iq_ref_ramp_ma * 0.001f;
    vd = PID_UpdateUs(&s_pid_id, 0.0f,     id, FOC_ISR_DT_US);
    vq = PID_UpdateUs(&s_pid_iq, iq_ref_a, iq, FOC_ISR_DT_US);
    g_foc_vd = vd;
    g_foc_vq = vq;

    /* ===== 6. InvPark -> SVPWM 输出 ===== */
    Foc_InvPark(vd, vq, enc_elec, &valpha, &vbeta);
    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);

    g_foc_valpha = valpha;
    g_foc_vbeta  = vbeta;
    g_foc_du = du;
    g_foc_dv = dv;
    g_foc_dw = dw;
}
