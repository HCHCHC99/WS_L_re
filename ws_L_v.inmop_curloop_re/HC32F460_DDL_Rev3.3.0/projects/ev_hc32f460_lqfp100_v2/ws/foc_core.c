/**
 *******************************************************************************
 * @file  foc_core.c
 * @brief FOC 公共核心实现 — 共享观测量定义、过流保护、PWM 启停封装、
 *        公共控制助手（GetDq / EMA / 电压包络 / 编码器电角度）。
 *
 *        所有实现均自 foc.c 原样迁移，控制行为不变。
 *******************************************************************************
 */

#include "foc_core.h"
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "hc32_ll_tmra.h"   /* Foc_Core_UpdateAngleObs 读 TMRA_1 原始计数 */
#include <math.h>

/*******************************************************************************
 * 跨模式共享观测量 / Keil Watch 调参量
 ******************************************************************************/
volatile float   g_foc_theta_rad        = 0.0f;
volatile float   g_foc_du               = 50.0f;
volatile float   g_foc_dv               = 50.0f;
volatile float   g_foc_dw               = 50.0f;
volatile float   g_foc_valpha           = 0.0f;
volatile float   g_foc_vbeta            = 0.0f;
volatile uint8_t g_foc_active           = 0u;

volatile uint8_t g_foc_mode             = FOC_MODE_NONE;
volatile uint8_t g_foc_phase            = 0u;
volatile int32_t g_foc_cur_sign         = (int32_t)FOC_CUR_SIGN;
volatile int32_t g_foc_enc_dir          = (int32_t)FOC_ENC_DIR;  /* encoder direction, Watch tunable */

volatile float   g_foc_id_ma            = 0.0f;
volatile float   g_foc_iq_ma            = 0.0f;
volatile float   g_foc_vd               = 0.0f;
volatile float   g_foc_vq               = 0.0f;

volatile uint8_t g_foc_align_state      = 0u;
volatile uint8_t g_foc_fault            = 0u;

/* Over-current limit (Watch tunable) + fault diagnostic */
volatile float   g_foc_oc_limit_a       = (float)FOC_OC_LIMIT_A;
volatile float   g_foc_fault_i_ma       = 0.0f;
volatile uint8_t g_foc_fault_stage      = 0u;
volatile int16_t g_foc_fault_iu_ma      = 0;
volatile int16_t g_foc_fault_iv_ma      = 0;
volatile int16_t g_foc_fault_iw_ma      = 0;

/* Voltage envelope / feedback filter (all Watch tunable) */
volatile float   g_foc_vmax_v           = (float)FOC_VMAX_V;
volatile float   g_foc_vramp_v_s        = (float)FOC_VRAMP_V_S;
volatile float   g_foc_cur_fb_alpha     = FOC_CUR_FB_ALPHA;
volatile float   g_foc_vlim_v           = 0.0f;

/* 开环调参量 */
volatile float   g_foc_openloop_freq_hz = FOC_OPENLOOP_FREQ_HZ;
volatile float   g_foc_openloop_volt_v  = FOC_OPENLOOP_VOLT_V;

/* 共享角度观测量 */
volatile float   g_foc_if_rotor_rad     = 0.0f;
volatile float   g_foc_if_diff_rad      = 0.0f;
volatile float   g_foc_mech_rad         = 0.0f;   /* 机械角度 [0,2PI)，mode 0 观测更新 */
volatile int32_t g_foc_mech_deg         = 0;      /* 机械角度 [0,360)，mode 0 观测更新 */
volatile int32_t g_foc_elec_deg         = 0;      /* 电角度 [0,360*极对数)，mode 0 观测更新 */

/* 对齐电零点 */
volatile int32_t g_foc_align_offset     = 0;

/*******************************************************************************
 * 模块内部状态
 ******************************************************************************/

/* OC debounce */
#define FOC_OC_DEBOUNCE_SAMPLES  4u
static uint16_t s_oc_cnt = 0u;

/* 内部状态机 */
static foc_state_t s_state = FOC_STATE_IDLE;

/* 对齐零点（模式23 记录 / 模式22 I-F 交接沿用） */
static int32_t s_align_offset = 0;

/* EMA 滤波器状态 */
static float s_id_f = 0.0f;
static float s_iq_f = 0.0f;

/* 电压包络状态 */
static float s_vlim = 0.0f;

/*******************************************************************************
 * 状态机 / 对齐零点托管
 ******************************************************************************/
foc_state_t Foc_Core_GetStateMachine(void)
{
    return s_state;
}

void Foc_Core_SetStateMachine(foc_state_t state)
{
    s_state = state;
}

int32_t Foc_Core_GetAlignOffset(void)
{
    return s_align_offset;
}

void Foc_Core_SetAlignOffset(int32_t offset)
{
    s_align_offset    = offset;
    g_foc_align_offset = offset;
}

/*******************************************************************************
 * Foc_Core_UpdateAngleObs - mode 0 / 空闲时实时刷新角度观测量（主循环调用）
 *
 * 直接读 TMRA_1 原始硬件计数（free-run 16 位）。CPR=4096 整除 65536，
 * "原始计数 mod CPR" 是连续合法的圈内位置，回绕无缝，无需累积器。
 * 扣对齐零点 offset（mode 20/23 锁定）后：
 *   g_foc_mech_rad     = 机械角度 [0, 2π)          （rad，控制框架）
 *   g_foc_if_rotor_rad = 电角度   [0, 2π)（折叠）  （rad，控制框架）
 *   g_foc_mech_deg     = 机械角度 [0, 360)          （deg，显示/Watch）
 *   g_foc_elec_deg     = 电角度   [0, 360*极对数)   （deg，显示/Watch）
 * 活跃模式下各 step 函数会覆盖 g_foc_if_rotor_rad，不冲突。
 ******************************************************************************/
void Foc_Core_UpdateAngleObs(void)
{
    int32_t cnt;
    int32_t diff;
    float   mech;
    float   elec;

    cnt  = (int32_t)TMRA_GetCountValue(CM_TMRA_1);
    diff = Foc_Core_ModPos(cnt * (int32_t)g_foc_enc_dir - s_align_offset,
                           (int32_t)ENCODER_CPR);

    mech = (float)diff * (FOC_MATH_2PI / (float)ENCODER_CPR);   /* 已在 [0,2π) */
    g_foc_mech_rad = mech;

    /* deg 观测量（纯整数运算）：机械角 0-360，电角度 = 机械 x 极对数，
     * 直接从 counts 展开（不经 mech_deg 放大截断误差） */
    g_foc_mech_deg = diff * 360 / (int32_t)ENCODER_CPR;
    g_foc_elec_deg = (diff * 360 * (int32_t)FOC_POLE_PAIRS / (int32_t)ENCODER_CPR)
                   % (360 * (int32_t)FOC_POLE_PAIRS);

    elec = mech * (float)FOC_POLE_PAIRS;
    elec -= (float)((int32_t)(elec * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
    if (elec < 0.0f) {
        elec += FOC_MATH_2PI;   /* 浮点截断误差防负 */
    }
    g_foc_if_rotor_rad = elec;
}

/*******************************************************************************
 * 过流保护
 ******************************************************************************/
void Foc_Core_ClearFault(void)
{
    g_foc_fault      = 0u;
    g_foc_fault_i_ma = 0.0f;
    s_oc_cnt         = 0u;
}

uint8_t Foc_Core_OverCurrent(const stc_i_data_t *pData)
{
    float iu, iv, iw, imax;

    if (pData == NULL) {
        return 0u;
    }

    iu  = (float)pData->i16IU_mA;
    iv  = (float)pData->i16IV_mA;
    iw  = (float)pData->i16IW_mA;

    imax = (iu > iv) ? iu : iv;
    if (iw > imax) imax = iw;
    {
        float imin = (iu < iv) ? iu : iv;
        if (iw < imin) imin = iw;
        if (-imin > imax) imax = -imin;
    }

    if (imax > g_foc_oc_limit_a * 1000.0f) {
        if (++s_oc_cnt >= FOC_OC_DEBOUNCE_SAMPLES) {
            s_oc_cnt          = 0u;
            g_foc_fault_i_ma  = imax;
            g_foc_fault_stage = g_foc_phase;
            g_foc_fault_iu_ma = pData->i16IU_mA;
            g_foc_fault_iv_ma = pData->i16IV_mA;
            g_foc_fault_iw_ma = pData->i16IW_mA;
            return 1u;
        }
    } else {
        s_oc_cnt = 0u;
    }
    return 0u;
}

void Foc_Core_FaultStop(uint8_t u8Code)
{
    g_foc_fault       = u8Code;
    g_foc_active      = 0u;
    s_state           = FOC_STATE_IDLE;
    g_foc_align_state = 0u;
    s_oc_cnt          = 0u;
    TMR4_PWM_EmergencyStop();
    g_foc_du = 0.0f;
    g_foc_dv = 0.0f;
    g_foc_dw = 0.0f;
}

/*******************************************************************************
 * PWM 输出启停封装
 ******************************************************************************/
void Foc_Core_PwmStart(void)
{
    TMR4_PWM_SetFocMode(FOC_DEADTIME_NS);
    g_foc_du = 50.0f;
    g_foc_dv = 50.0f;
    g_foc_dw = 50.0f;
    TMR4_PWM_SetDuty3Phase(50.0f, 50.0f, 50.0f);
    TMR4_PWM_StartOutput();
    g_foc_active = 1u;
}

void Foc_Core_PwmStop(void)
{
    TMR4_PWM_EmergencyStop();
    g_foc_du = 0.0f;
    g_foc_dv = 0.0f;
    g_foc_dw = 0.0f;
}

/*******************************************************************************
 * 公共控制助手
 ******************************************************************************/
int32_t Foc_Core_ModPos(int32_t x, int32_t n)
{
    return ((x % n) + n) % n;
}

void Foc_Core_ClampOpenLoopVolt(void)
{
    if (g_foc_openloop_volt_v > FOC_OPENLOOP_VOLT_MAX) {
        g_foc_openloop_volt_v = FOC_OPENLOOP_VOLT_MAX;
    }
}

float Foc_Core_CurLoopTheta(void)
{
    int32_t diff = Foc_Core_ModPos(((int32_t)g_enc_count * (int32_t)g_foc_enc_dir) - s_align_offset,
                                   (int32_t)ENCODER_CPR);
    return (float)diff * (FOC_MATH_2PI / (float)ENCODER_CPR)
         * (float)FOC_POLE_PAIRS;
}

void Foc_Core_GetDq(const stc_i_data_t *pData, float theta, float *id, float *iq)
{
    float ia, ib, ic, ialpha, ibeta;
    float sign = (float)g_foc_cur_sign;

    if (pData != NULL) {
        ia = (float)pData->i16IU_mA * 0.001f * sign;
        ib = (float)pData->i16IV_mA * 0.001f * sign;
        ic = (float)pData->i16IW_mA * 0.001f * sign;
    } else {
        ia = ib = ic = 0.0f;
    }
    Foc_Clarke(ia, ib, ic, &ialpha, &ibeta);
    Foc_Park(ialpha, ibeta, theta, id, iq);
}

void Foc_Core_ResetEma(void)
{
    s_id_f = 0.0f;
    s_iq_f = 0.0f;
}

void Foc_Core_EmaFilter(float *id, float *iq)
{
    float alpha = g_foc_cur_fb_alpha;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    s_id_f += alpha * (*id - s_id_f);
    s_iq_f += alpha * (*iq - s_iq_f);
    *id = s_id_f;
    *iq = s_iq_f;
}

void Foc_Core_ResetVoltageEnvelope(void)
{
    s_vlim = 0.0f;
}

void Foc_Core_ApplyVoltageEnvelope(float *vd, float *vq)
{
    s_vlim += g_foc_vramp_v_s / (float)FOC_ISR_HZ;
    if (s_vlim > g_foc_vmax_v) {
        s_vlim = g_foc_vmax_v;
    }
    g_foc_vlim_v = s_vlim;
    {
        float v2    = (*vd) * (*vd) + (*vq) * (*vq);
        float vmax2 = s_vlim * s_vlim;
        if (v2 > vmax2) {
            float k = s_vlim / sqrtf(v2);
            *vd *= k;
            *vq *= k;
        }
    }
}

/*******************************************************************************
 * 原有对外 API（自 foc.c 迁移）
 ******************************************************************************/
void Foc_Stop(void)
{
    g_foc_active      = 0u;
    g_foc_mode        = FOC_MODE_NONE;
    g_foc_phase       = 0u;
    s_state           = FOC_STATE_IDLE;
    g_foc_align_state = 0u;
    TMR4_PWM_EmergencyStop();
    g_foc_du = 0.0f;
    g_foc_dv = 0.0f;
    g_foc_dw = 0.0f;
    g_foc_valpha = 0.0f;
    g_foc_vbeta  = 0.0f;
    s_vlim       = 0.0f;
    g_foc_vlim_v = 0.0f;
}

uint8_t Foc_GetState(void)
{
    return g_foc_align_state;
}

uint8_t Foc_IsRunning(void)
{
    return g_foc_active;
}

float Foc_EncoderElecAngleRad(void)
{
    float mech_rad = (float)g_enc_count * (FOC_MATH_2PI / (float)ENCODER_CPR);
    return mech_rad * (float)FOC_POLE_PAIRS;
}
