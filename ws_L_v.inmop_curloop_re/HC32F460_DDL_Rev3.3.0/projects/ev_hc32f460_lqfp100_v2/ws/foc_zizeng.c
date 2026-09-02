/**
 *******************************************************************************
 * @file  foc_zizeng.c
 * @brief FOC 模式30 — 磁场角度自增拖动实现（自 foc.c 原样迁移）。
 *
 *        关键点：直接读取 TIMERA_1 硬件编码器计数，不依赖主循环的
 *        Encoder_Update()，在 20 kHz ISR 中实时更新。
 *******************************************************************************
 */

#include "foc_zizeng.h"
#include "foc_math.h"
#include "foc_calib.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "rtt_log.h"
#include "hc32_ll_tmra.h"   /* 直接读取 TIMERA_1 计数器 */
#include <math.h>           /* sqrtf */

/* ============================================================================
 * ZIZENG 自动偏移补偿参数
 * ==========================================================================*/
#define ZIZENG_OFFSET_WAIT_MS     1000           /* 启动后等待 1000ms 再采样 */
#define ZIZENG_OFFSET_SAMPLES     2000           /* 采样 2000 次（约 100ms @20kHz） */

/* Keil Watch 可调变量 */
volatile float   g_zizeng_theta_rad  = 0.0f;
volatile float   g_zizeng_freq_hz    = 3.0f;
volatile float   g_zizeng_volt_v     = 0.6f;
volatile float   g_zizeng_du         = 50.0f;
volatile float   g_zizeng_dv         = 50.0f;
volatile float   g_zizeng_dw         = 50.0f;
volatile uint8_t g_zizeng_running    = 0;

/* 转子系电流观测（偏移锁定后，用补偿后的转子电角度做 Park 变换） */
volatile float   g_foc_id_rotor_ma   = 0.0f;
volatile float   g_foc_iq_rotor_ma   = 0.0f;

/* ZIZENG 自动偏移补偿状态 */
static uint8_t  s_zizeng_offset_state = 0;      /* 0=IDLE, 1=SAMPLING, 2=LOCKED */
static float    s_zizeng_offset_sum = 0.0f;
static uint32_t s_zizeng_offset_cnt = 0;
static float    s_zizeng_offset_locked = 0.0f;
static uint32_t s_zizeng_start_cnt = 0;          /* 用于延迟启动采样 */

/* 转子系电流独立 EMA 状态（不与磁场系 Foc_Core_EmaFilter() 共用） */
static float    s_id_rotor_f = 0.0f;
static float    s_iq_rotor_f = 0.0f;

/* 静止两相系电流观测（A，瞬时值）：Clarke(dataCal) 输出与幅值 */
volatile float   g_foc_ialpha  = 0.0f;
volatile float   g_foc_ibeta   = 0.0f;
volatile float   g_foc_iab_mag = 0.0f;

/**
 * @brief 启动 ZIZENG 模式
 */
void Foc_StartZizeng(void)
{
    g_zizeng_theta_rad = 0.0f;
    g_zizeng_running   = 1;

    /* 重置自动偏移补偿状态 */
    s_zizeng_offset_state = 0;
    s_zizeng_offset_sum = 0.0f;
    s_zizeng_offset_cnt = 0;
    s_zizeng_offset_locked = 0.0f;
    s_zizeng_start_cnt = 0;

    if (g_zizeng_volt_v <= 0.0f) {
        g_zizeng_volt_v = 0.6f;  /* ±10A 传感器：0.6V→≈5A，量程内 */
    }
    if (g_zizeng_freq_hz <= 0.0f) {
        g_zizeng_freq_hz = 5.0f;
    }

    TMR4_PWM_SetFocMode(FOC_DEADTIME_NS);
    g_zizeng_du = 50.0f;
    g_zizeng_dv = 50.0f;
    g_zizeng_dw = 50.0f;
    TMR4_PWM_SetDuty3Phase(50.0f, 50.0f, 50.0f);

    TMR4_PWM_StartOutput();
    g_foc_active = 1u;
    g_foc_mode   = FOC_MODE_NONE;

    /* 相电流 DC 零偏自校准（温漂残余）：本函数返回后的前 ~210ms 内
     * Foc_Zizeng_Step 会强制零矢量（零电流），校准锁定后才开始拖动 */
    Foc_Calib_Start();

    /* 转子系电流观测复位 */
    s_id_rotor_f = 0.0f;
    s_iq_rotor_f = 0.0f;
    g_foc_id_rotor_ma = 0.0f;
    g_foc_iq_rotor_ma = 0.0f;

    MAIN_D("[ZIZENG] Started: freq=%.1f Hz, volt=%.2f V",
           g_zizeng_freq_hz, g_zizeng_volt_v);
}

/**
 * @brief 停止 ZIZENG 模式
 */
void Foc_StopZizeng(void)
{
    g_zizeng_running = 0;
    g_foc_active     = 0u;
    TMR4_PWM_EmergencyStop();
    g_zizeng_du = 0.0f;
    g_zizeng_dv = 0.0f;
    g_zizeng_dw = 0.0f;

    /* 重置偏移补偿状态 */
    s_zizeng_offset_state = 0;
    MAIN_D("[ZIZENG] Stopped");
}

/**
 * @brief ZIZENG 模式步进函数（在 20kHz ISR 中调用）
 *
 * 关键修复：直接读取 TIMERA_1 硬件计数器，不依赖主循环的 Encoder_Update()
 */
void Foc_Zizeng_Step(const stc_i_data_t *pData)
{
    float theta, valpha, vbeta, du, dv, dw;
    float enc_elec, diff;
    float id, iq;
    float off_u, off_v, off_w;
    uint32_t hw_cnt;
    int16_t delta;
    stc_i_data_t dataCal;   /* 零偏校正后的电流采样副本 */
    static int32_t s_prev_hw_cnt = 0;
    static int32_t s_enc_pos = 0;
    static uint8_t s_enc_initialized = 0;

    if (!g_zizeng_running) {
        return;
    }

    /* ===== ★★★ 关键修复：直接读取 TIMERA_1 硬件编码器计数 ★★★ =====
     * 不依赖主循环的 Encoder_Update()，在 20kHz ISR 中实时更新
     */
    hw_cnt = TMRA_GetCountValue(CM_TMRA_1);

    if (!s_enc_initialized) {
        /* 首次运行：初始化历史值 */
        s_prev_hw_cnt = (int32_t)hw_cnt;
        s_enc_pos = 0;
        s_enc_initialized = 1;
    }

    /* 计算增量（正确处理 16 位计数器回绕） */
    delta = (int16_t)((uint16_t)hw_cnt - (uint16_t)s_prev_hw_cnt);
    s_prev_hw_cnt = (int32_t)hw_cnt;
    s_enc_pos += (int32_t)delta;

    /* 同步到全局变量，供其他模块使用 */
    g_enc_count = s_enc_pos;
    g_enc_count_f = (float)s_enc_pos;

    /* ===== 0. 相电流 DC 零偏自校准（零矢量窗口，非阻塞） =====
     * 锁定前强制三相 50% 占空比（线电压为 0，绕组无电流），
     * theta 保持不动，锁定后才开始自增拖动。
     * 1000ms 角度基线等待在锁定后重新计数。 */
    if (!Foc_Calib_IsLocked()) {
        Foc_Calib_Feed(pData);
        TMR4_PWM_SetDuty3Phase(50.0f, 50.0f, 50.0f);
        g_zizeng_du = 50.0f;
        g_zizeng_dv = 50.0f;
        g_zizeng_dw = 50.0f;
        g_foc_id_ma = 0.0f;
        g_foc_iq_ma = 0.0f;
        g_foc_ialpha  = 0.0f;
        g_foc_ibeta   = 0.0f;
        g_foc_iab_mag = 0.0f;
        s_zizeng_start_cnt = 0;
        return;
    }

    /* ===== 1. 角度积分 ===== */
    theta = g_zizeng_theta_rad
          + (FOC_MATH_2PI * g_zizeng_freq_hz / (float)FOC_ISR_HZ);

    if (theta >= FOC_MATH_2PI) {
        theta -= FOC_MATH_2PI;
    }
    if (theta < 0.0f) {
        theta += FOC_MATH_2PI;
    }
    g_zizeng_theta_rad = theta;
    g_foc_theta_rad = theta;

    /* ===== 2. 计算 id/iq（控制坐标系） =====
     * 先扣除校准锁定的三相残余零偏（温漂），再做 Clarke/Park。
     * 过流保护等仍使用原始 pData，不受影响。 */
    dataCal = *pData;
    Foc_Calib_GetOffsetsMa(&off_u, &off_v, &off_w);
    dataCal.i16IU_mA = (int16_t)((float)pData->i16IU_mA - off_u);
    dataCal.i16IV_mA = (int16_t)((float)pData->i16IV_mA - off_v);
    dataCal.i16IW_mA = (int16_t)((float)pData->i16IW_mA - off_w);

    /* 2.5 静止两相系观测：与 GetDq 同源的 Clarke（同符号约定），瞬时值无 EMA */
    {
        float sign = (float)g_foc_cur_sign;
        float ia = (float)dataCal.i16IU_mA * 0.001f * sign;
        float ib = (float)dataCal.i16IV_mA * 0.001f * sign;
        float ic = (float)dataCal.i16IW_mA * 0.001f * sign;
        Foc_Clarke(ia, ib, ic, &g_foc_ialpha, &g_foc_ibeta);
        g_foc_iab_mag = sqrtf(g_foc_ialpha * g_foc_ialpha
                            + g_foc_ibeta  * g_foc_ibeta);
    }

    Foc_Core_GetDq(&dataCal, theta, &id, &iq);
    Foc_Core_EmaFilter(&id, &iq);

    g_foc_id_ma = id * 1000.0f;
    g_foc_iq_ma = iq * 1000.0f;

    /* ===== 3. 编码器电角度（使用刚读取的硬件值） ===== */
    enc_elec = (float)Foc_Core_ModPos(s_enc_pos * (int32_t)g_foc_enc_dir,
                                      (int32_t)ENCODER_CPR)
             * (FOC_MATH_2PI * (float)FOC_POLE_PAIRS / (float)ENCODER_CPR);
    enc_elec -= (float)((int32_t)(enc_elec * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
    if (enc_elec < 0.0f) {
        enc_elec += FOC_MATH_2PI;
    }

    /* 偏移基线锁定后：直接扣进转子电角度，使其对齐磁场参考系（绝对化）。
     * 锁定瞬间 g_foc_if_rotor_rad 会跳变一次（跳变量 = 基线值），属预期行为。 */
    if (s_zizeng_offset_state == 2) {
        enc_elec -= s_zizeng_offset_locked;
        enc_elec -= (float)((int32_t)(enc_elec * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
        if (enc_elec < 0.0f) {
            enc_elec += FOC_MATH_2PI;
        }
    }
    g_foc_if_rotor_rad = enc_elec;

    /* ===== 3.5 转子系电流观测：用补偿后的转子电角度做 Park 变换 =====
     * iq_rotor = 真实力矩电流，id_rotor = 磁链方向分量。
     * 锁定前转子角度未绝对化，观测无意义，输出保持 0。 */
    if (s_zizeng_offset_state == 2) {
        float id_r, iq_r, alpha;
        Foc_Core_GetDq(&dataCal, enc_elec, &id_r, &iq_r);
        alpha = g_foc_cur_fb_alpha;
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;
        s_id_rotor_f += alpha * (id_r - s_id_rotor_f);
        s_iq_rotor_f += alpha * (iq_r - s_iq_rotor_f);
        g_foc_id_rotor_ma = s_id_rotor_f * 1000.0f;
        g_foc_iq_rotor_ma = s_iq_rotor_f * 1000.0f;
    } else {
        s_id_rotor_f = 0.0f;
        s_iq_rotor_f = 0.0f;
        g_foc_id_rotor_ma = 0.0f;
        g_foc_iq_rotor_ma = 0.0f;
    }

    /* ===== 4. 差值（锁定前为原始差值；锁定后因转子角度已补偿，即为补偿后差值） ===== */
    diff = enc_elec - theta;
    if (diff > FOC_MATH_PI) diff -= FOC_MATH_2PI;
    if (diff < -FOC_MATH_PI) diff += FOC_MATH_2PI;
    g_foc_if_diff_rad = diff;
    if (s_zizeng_offset_state == 0) {
        /* IDLE: 等待电机启动并稳定运行 */
        s_zizeng_start_cnt++;
        if (s_zizeng_start_cnt >= (ZIZENG_OFFSET_WAIT_MS * FOC_ISR_HZ / 1000)) {
            s_zizeng_offset_state = 1;
            s_zizeng_offset_sum = 0.0f;
            s_zizeng_offset_cnt = 0;
            s_zizeng_start_cnt = 0;
            MAIN_D("[ZIZENG] Offset sampling started...");
        }
    }
    else if (s_zizeng_offset_state == 1) {
        /* SAMPLING: 采集 diff 样本 */
        s_zizeng_offset_sum += diff;
        s_zizeng_offset_cnt++;

        if (s_zizeng_offset_cnt >= ZIZENG_OFFSET_SAMPLES) {
            float avg_diff = s_zizeng_offset_sum / (float)s_zizeng_offset_cnt;

            s_zizeng_offset_locked = avg_diff;
            s_zizeng_offset_state = 2;

            MAIN_D("[ZIZENG] Offset LOCKED: %.3f rad (%.1f deg), samples=%lu",
                   s_zizeng_offset_locked,
                   s_zizeng_offset_locked * 57.2958f,
                   (unsigned long)s_zizeng_offset_cnt);
        }
    }
    /* else state == 2: LOCKED, 偏移已直接作用于转子角度（见第 3 步） */

    /* ===== 5. SVPWM 输出 =====
     * 加压轴由 ZIZENG_VOLT_ON_Q_AXIS（foc_zizeng.h）编译期选择：
     *   q 轴加压: Vd=0, Vq=V -> valpha=-V·sinθ, vbeta=V·cosθ
     *   d 轴加压: Vd=V, Vq=0 -> valpha= V·cosθ, vbeta=V·sinθ
     */
#if ZIZENG_VOLT_ON_Q_AXIS
    valpha = -g_zizeng_volt_v * Foc_Math_Sin(theta);
    vbeta  =  g_zizeng_volt_v * Foc_Math_Cos(theta);
#else
    valpha =  g_zizeng_volt_v * Foc_Math_Cos(theta);
    vbeta  =  g_zizeng_volt_v * Foc_Math_Sin(theta);
#endif

    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);

    g_zizeng_du = du;
    g_zizeng_dv = dv;
    g_zizeng_dw = dw;
    g_foc_valpha = valpha;
    g_foc_vbeta  = vbeta;
}
