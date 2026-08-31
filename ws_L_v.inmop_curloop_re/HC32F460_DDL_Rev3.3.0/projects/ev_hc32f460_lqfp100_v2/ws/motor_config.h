/**
 *******************************************************************************
 * @file  motor_config.h
 * @brief 电机控制参数主配置（精简版 — 仅保留模式23和30）
 *******************************************************************************
 */

#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__

/* ============================================================================
 * 电机控制频率主配置
 * ==========================================================================*/
#define MOTOR_PWM_FREQ_HZ   10000u

/* 霍尔使能：0 = 关闭霍尔（PA8/9/10 释放给编码器 ABZ 用） */
#define MOTOR_HALL_ENABLE   0

/* ============================================================================
 * 环拓扑配置（精简版 — 模式23和30不需要速度环/电流环）
 * ==========================================================================*/
#define MOTOR_LOOP_POSITION_ENABLE   0
#define MOTOR_LOOP_SPEED_ENABLE      0
#define MOTOR_LOOP_CURRENT_ENABLE    0
#define MOTOR_DUTY_SRC               DUTY_DIRECT   /* 直接控制，不用电流环 */

/* 占空比来源（仅供编译通过） */
#define DUTY_DIRECT                  0
#define DUTY_FROM_CURRENT            1

/* 电流参考源（仅供编译通过） */
#define CUR_REF_FIXED                0
#define CUR_REF_FROM_SPEED           1

/* 速度参考源（仅供编译通过） */
#define SPD_REF_TARGET_RPM           0
#define SPD_REF_FROM_POS             1
#define POS_REF_EXTERNAL             0
#define MOTOR_POS_REF_SRC            POS_REF_EXTERNAL
#define MOTOR_SPD_REF_SRC            SPD_REF_TARGET_RPM
#define MOTOR_CUR_REF_SRC            CUR_REF_FIXED

/* ============================================================================
 * 电机电气参数（3505-KV650 云台/外转子电机）
 * ==========================================================================*/
#define FOC_MOTOR_RS_OHM            0.1f
#define FOC_MOTOR_LS_UH             42.3f
#define FOC_MOTOR_FLUX_VS           0.00084f
#define FOC_MOTOR_KV_RPM_V          650u
#define FOC_MOTOR_RATED_CURRENT_A   17.0f
#define FOC_MOTOR_RATED_TORQUE_NM   0.2f
#define FOC_MOTOR_MAX_SPEED_RPM     7800u

/* ============================================================================
 * MotorScope RTT 心跳总开关
 * ==========================================================================*/
#define MOTOR_SCOPE_KEY      1
#define MOTOR_SCOPE_LOCK_MS_FAST       15u
#define MOTOR_SCOPE_LOCK_MS_KEEPALIVE  300u
#define MOTOR_SCOPE_CHG_SPD_RPM        50
#define MOTOR_SCOPE_CHG_VD_V           0.3f
#define MOTOR_SCOPE_CHG_VQ_V           0.3f

/* ============================================================================
 * FOC 参数（模式23 对齐校准 / 模式30 自增拖动）
 * ==========================================================================*/
#define MOTOR_FOC_ENABLE        1
#define FOC_POLE_PAIRS          10
#define FOC_VBUS_V              12.0f
#define FOC_ISR_HZ              20000
#define FOC_DEADTIME_NS         500u

/* 对齐校准（comm_mode 23） */
#define FOC_ALIGN_VOLT_V        0.4f
#define FOC_ALIGN_BETA_MS       1000
#define FOC_ALIGN_STABLE_MS     300
#define FOC_ALIGN_TIMEOUT_MS    3000
#define FOC_ALIGN_HOLD_MS       2000

#define FOC_OC_LIMIT_A          5.5f
#define FOC_CUR_SIGN            -1
#define FOC_ENC_DIR             -1
#define FOC_PI_OFF_180          0

#define FOC_PI_KP               0.10f
#define FOC_PI_KI               240.0f
#define FOC_PI_UMAX_V           1.0f

/* 以下宏仅用于编译 FOC 代码（模式23和30不需要 I-F/电流环） */
#define FOC_OPENLOOP_FREQ_HZ    5.0f
#define FOC_OPENLOOP_VOLT_V     0.9f
#define FOC_OPENLOOP_VOLT_MAX   1.5f
#define FOC_IQ_REF_MA           3000
#define FOC_IQ_RAMP_MA_S        500
#define FOC_IF_HOLD_IQ_MA       1200
#define FOC_IF_HOLD_MAX_MS      5000
#define FOC_IF_FREQ_RAMP_HZ_S   1.0f
#define FOC_IF_SYNC_MIN_HZ      2.0f
#define FOC_IF_SYNC_WIN_CNT     2000u
#define FOC_IF_SYNC_BAND_RAD    0.30f
#define FOC_IF_SYNC_GOOD_WINS   2u
#define FOC_IF_TIMEOUT_MS       15000u
#define FOC_RUN_ANCHOR_DEG      180
#define FOC_RUN_IQ_SIGN         1
#define FOC_VMAX_V              1.0f
#define FOC_VRAMP_V_S           0.5f
#define FOC_CUR_FB_ALPHA        0.3f

#endif /* __MOTOR_CONFIG_H__ */
