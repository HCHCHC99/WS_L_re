/**
 *******************************************************************************
 * @file  foc_scope.c
 * @brief MotorScope RTT 遥测实现（自 foc.c 原样迁移）。
 *
 *        发送策略：
 *          - 模式/阶段/同步/事件变化 -> 立即发送
 *          - 转速/vd/vq 变化超阈值   -> 立即发送
 *          - iq/id/theta 任一变化    -> 受限速（MOTOR_SCOPE_LOCK_MS_FAST）
 *          - 保活超时（MOTOR_SCOPE_LOCK_MS_KEEPALIVE）-> 发送
 *******************************************************************************
 */

#include "foc_scope.h"
#include "foc_core.h"
#include "foc_curloop.h"    /* 读取 I-F 频率/同步/事件观测量 */
#include "foc_math.h"
#include "encoder.h"
#include "motor_config.h"
#include "SEGGER_RTT.h"
#include <math.h>
#include <stdio.h>

volatile int32_t g_motor_scope = (int32_t)MOTOR_SCOPE_KEY;

#ifndef FOC_RTT_ENABLE
#define FOC_RTT_ENABLE      1u
#endif
#ifndef FOC_RTT_CH
#define FOC_RTT_CH          0u
#endif
#ifndef FOC_RTT_RATE_HZ
#define FOC_RTT_RATE_HZ     1000u
#endif

#if FOC_RTT_ENABLE && (FOC_RTT_RATE_HZ > 0u) && (FOC_RTT_RATE_HZ <= FOC_ISR_HZ)
#define FOC_RTT_DIV         ((uint16_t)(FOC_ISR_HZ / FOC_RTT_RATE_HZ))

#if FOC_RTT_RATE_HZ > 2000u
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t ms;
    int32_t  rotor_deg;        /* 转子电角度 [0,360*极对数) */
    int32_t  theta_deg;        /* 磁场角 (deg, mode30 拖动递增) */
    int32_t  iq_ma;
    int32_t  id_ma;
    int32_t  vq_mv;
    int32_t  vd_mv;
    int32_t  spd_rpm;
    int32_t  diff_deg;         /* rotor - theta (deg, 带符号小值) */
    int32_t  freq_cHz;
    uint8_t  mode;
    uint8_t  phase;
    uint8_t  sync;
    uint8_t  rsv;
    int32_t  mech_deg;         /* 机械角度 [0,360) */
    int32_t  is_ma;
    int32_t  is_angle_deg;     /* 电流矢量角 (-180,180] */
    int32_t  v_mv;
    int32_t  v_angle_deg;      /* 电压矢量角 (-180,180] */
    int32_t  theta_mech_deg;   /* 磁场角折算机械当量 (deg) */
    int32_t  cnt;
} foc_rtt_frame_t;
#endif

static float Foc_RotorAngleFromEncoderRad(void)
{
    int32_t cnt = Foc_Core_ModPos(((int32_t)g_enc_count * (int32_t)g_foc_enc_dir),
                                  (int32_t)ENCODER_CPR);
    float enc = (float)cnt * (FOC_MATH_2PI * (float)FOC_POLE_PAIRS / (float)ENCODER_CPR);
    enc -= (float)((int32_t)(enc * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
    if (enc < 0.0f) {
        enc += FOC_MATH_2PI;
    }
    return enc;
}

/* 角度通道统一用 deg：机械角 [0,360)，电角度 [0,360*极对数)，
 * 直接从编码器 counts 整数换算（电角度 = 圈内位置 x 极对数） */
static int32_t Foc_MechAngleDeg(void)
{
    return Foc_Core_ModPos((int32_t)g_enc_count * (int32_t)g_foc_enc_dir,
                           (int32_t)ENCODER_CPR) * 360 / (int32_t)ENCODER_CPR;
}

/* 转子电角度 = 机械圈内位置 x 极对数，[0, 360*极对数) */
static int32_t Foc_RotorElecDeg(void)
{
    return Foc_Core_ModPos((int32_t)g_enc_count * (int32_t)g_foc_enc_dir,
                           (int32_t)ENCODER_CPR) * 360 * (int32_t)FOC_POLE_PAIRS
         / (int32_t)ENCODER_CPR;
}

static int32_t Foc_IsMagMa(void)      { return (int32_t)sqrtf(g_foc_id_ma * g_foc_id_ma + g_foc_iq_ma * g_foc_iq_ma); }
static int32_t Foc_IsAngleDeg(void)   { return (int32_t)(atan2f(g_foc_iq_ma, g_foc_id_ma) * 57.2958f); }
static int32_t Foc_VMagMv(void)       { return (int32_t)(sqrtf(g_foc_vd * g_foc_vd + g_foc_vq * g_foc_vq) * 1000.0f); }
static int32_t Foc_VAngleDeg(void)    { return (int32_t)(atan2f(g_foc_vq, g_foc_vd) * 57.2958f); }
static int32_t Foc_ThetaMechDeg(void) { return (int32_t)(g_foc_theta_rad * (57.2958f / (float)FOC_POLE_PAIRS)); }

static uint64_t s_mot_scope_last_us    = 0u;
static int32_t  s_mot_scope_last_mode  = -1;
static int32_t  s_mot_scope_last_phase = -1;
static int32_t  s_mot_scope_last_sync  = -1;
static int32_t  s_mot_scope_last_evt   = -1;
static float    s_mot_scope_last_iq    = 0.0f;
static float    s_mot_scope_last_id    = 0.0f;
static float    s_mot_scope_last_th    = 0.0f;
static float    s_mot_scope_last_spd   = 0.0f;
static float    s_mot_scope_last_vd    = 0.0f;
static float    s_mot_scope_last_vq    = 0.0f;

void Foc_RttSend(uint64_t now_us)
{
    uint32_t ms;
    uint64_t since_us;
    int32_t  mode, phase, sync, evt;
    float    iq, id, theta, spd, vd, vq;
    uint8_t  send;

    if (g_motor_scope == 0) {
        return;
    }

    mode  = (int32_t)g_foc_mode;
    phase = (int32_t)g_foc_phase;
    sync  = (int32_t)g_foc_if_sync;
    evt   = (int32_t)g_foc_if_evt;
    iq    = g_foc_iq_ma;
    id    = g_foc_id_ma;
    theta = g_foc_theta_rad;
    spd   = g_enc_speed_rpm;
    vd    = g_foc_vd;
    vq    = g_foc_vq;
    since_us = now_us - s_mot_scope_last_us;

    send = 0u;
    if ((mode != s_mot_scope_last_mode) || (phase != s_mot_scope_last_phase) ||
        (sync != s_mot_scope_last_sync) || (evt != s_mot_scope_last_evt)) {
        send = 1u;
    }
    if (!send) {
        if ((fabsf(spd - s_mot_scope_last_spd) > (float)MOTOR_SCOPE_CHG_SPD_RPM) ||
            (fabsf(vd - s_mot_scope_last_vd)    > (float)MOTOR_SCOPE_CHG_VD_V) ||
            (fabsf(vq - s_mot_scope_last_vq)    > (float)MOTOR_SCOPE_CHG_VQ_V)) {
            send = 1u;
        }
    }
    if (!send) {
        if ((iq != s_mot_scope_last_iq) || (id != s_mot_scope_last_id) ||
            (theta != s_mot_scope_last_th)) {
            if (since_us >= ((uint64_t)MOTOR_SCOPE_LOCK_MS_FAST * 1000u)) {
                send = 1u;
            }
        }
    }
    if (!send) {
        if (since_us >= ((uint64_t)MOTOR_SCOPE_LOCK_MS_KEEPALIVE * 1000u)) {
            send = 1u;
        }
    }
    if (!send) {
        return;
    }

    s_mot_scope_last_us    = now_us;
    s_mot_scope_last_mode  = mode;
    s_mot_scope_last_phase = phase;
    s_mot_scope_last_sync  = sync;
    s_mot_scope_last_evt   = evt;
    s_mot_scope_last_iq    = iq;
    s_mot_scope_last_id    = id;
    s_mot_scope_last_th    = theta;
    s_mot_scope_last_spd   = spd;
    s_mot_scope_last_vd    = vd;
    s_mot_scope_last_vq    = vq;
    ms = (uint32_t)(now_us / 1000u);

    g_foc_if_rotor_rad = Foc_RotorAngleFromEncoderRad();

#if FOC_RTT_RATE_HZ > 2000u
    {
        foc_rtt_frame_t fr;
        fr.magic      = 0x46544F4Du;
        fr.ms         = ms;
        fr.rotor_deg  = Foc_RotorElecDeg();
        fr.theta_deg  = (int32_t)(g_foc_theta_rad  * 57.2958f);
        fr.iq_ma      = (int32_t)g_foc_iq_ma;
        fr.id_ma      = (int32_t)g_foc_id_ma;
        fr.vq_mv      = (int32_t)(g_foc_vq * 1000.0f);
        fr.vd_mv      = (int32_t)(g_foc_vd * 1000.0f);
        fr.spd_rpm    = (int32_t)g_enc_speed_rpm;
        fr.diff_deg   = (int32_t)(g_foc_if_diff_rad * 57.2958f);
        fr.freq_cHz   = (int32_t)(g_foc_if_freq_hz * 100.0f);
        fr.mode       = g_foc_mode;
        fr.phase      = g_foc_phase;
        fr.sync       = g_foc_if_sync;
        fr.rsv        = 0u;
        fr.mech_deg        = Foc_MechAngleDeg();
        fr.is_ma           = Foc_IsMagMa();
        fr.is_angle_deg    = Foc_IsAngleDeg();
        fr.v_mv            = Foc_VMagMv();
        fr.v_angle_deg     = Foc_VAngleDeg();
        fr.theta_mech_deg  = Foc_ThetaMechDeg();
        fr.cnt            = (int32_t)g_enc_count;
        SEGGER_RTT_Write(FOC_RTT_CH, (const char *)&fr, (unsigned)sizeof(fr));
    }
#else
    {
        char buf[224];
        int  n;
        n = snprintf(buf, sizeof(buf),
            "MOTF,%u,%u,%d,%d,%d,%d,%d,%d,%d,%u,%d,%d,%u,%d,%d,%d,%d,%d,%d,%d\r\n",
            (unsigned)g_foc_mode, (unsigned)g_foc_phase,
            (int)Foc_RotorElecDeg(),
            (int)(g_foc_theta_rad  * 57.2958f),
            (int)g_foc_iq_ma, (int)g_foc_id_ma,
            (int)(g_foc_vq * 1000.0f), (int)(g_foc_vd * 1000.0f),
            (int)g_enc_speed_rpm,
            (unsigned)g_foc_if_sync,
            (int)(g_foc_if_diff_rad * 57.2958f),
            (int)(g_foc_if_freq_hz * 100.0f),
            (unsigned)ms,
            (int)Foc_MechAngleDeg(),
            (int)Foc_IsMagMa(),
            (int)Foc_IsAngleDeg(),
            (int)Foc_VMagMv(),
            (int)Foc_VAngleDeg(),
            (int)Foc_ThetaMechDeg(),
            (int)g_enc_count);
        if (n > 0 && n < (int)sizeof(buf)) {
            SEGGER_RTT_Write(FOC_RTT_CH, buf, (unsigned)n);
        }
    }
#endif
}
#else
void Foc_RttSend(uint64_t now_us) { (void)now_us; }
#endif /* FOC_RTT_ENABLE */
