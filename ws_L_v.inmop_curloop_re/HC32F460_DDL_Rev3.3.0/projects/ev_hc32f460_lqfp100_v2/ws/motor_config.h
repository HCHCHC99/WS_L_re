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
 *
 * 命名与厂商规格对齐（2026-09-23 核对）：
 *   厂商规格书写的是「最大电流 17A」「扭矩 0.2Nm」——都是**峰值**而非额定。
 *   故命名为 MAX_*，避免被误读成连续工作点。
 *   依据：17²×0.1Ω = 28.9W 铜损，对 58g 电机显然不是连续额定；
 *        且 0.2Nm/17A = 0.01176 N·m/A ≈ Kt = 1.5×pp×ψf = 0.0126，两者自洽。
 *
 * ⚠ FOC_MOTOR_FLUX_VS 与 FOC_MOTOR_KV_RPM_V 差 √3 倍：
 *   由 KV 反推 ψf = 60000/(√3·π·pp·KV) = 0.001695，比值正好 2.02 ≈ √3，
 *   属「相峰值」与「线-线 RMS」的定义差异，不是数据错误。
 *   → **算转矩/电流用 FLUX_VS；不要用 KV 反推 ψf。**
 * ==========================================================================*/
#define FOC_MOTOR_RS_OHM            0.1f      /* 相电阻（厂商，非本机辨识） */
#define FOC_MOTOR_LS_UH             42.3f     /* 相电感（厂商，相值；真值存疑见 mdl 文档） */
#define FOC_MOTOR_FLUX_VS           0.00084f  /* 磁链，相峰值 */
#define FOC_MOTOR_KV_RPM_V          650u      /* 转速常数 */
#define FOC_MOTOR_MAX_CURRENT_A     17.0f     /* 最大（峰值）电流，非额定 */
#define FOC_MOTOR_MAX_TORQUE_NM     0.2f      /* 17A 对应的峰值扭矩，非额定 */
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
#define FOC_DEADTIME_NS         500u

/* 对齐校准（comm_mode 23） */
#define FOC_ALIGN_VOLT_V        0.4f
#define FOC_ALIGN_BETA_MS       1000
#define FOC_ALIGN_STABLE_MS     300
#define FOC_ALIGN_TIMEOUT_MS    3000
#define FOC_ALIGN_HOLD_MS       2000

/* 过流阈值：必须在电流传感器量程内（±10A 传感器 → 阈值 ≤9A，留 10% 余量），
 * 否则削顶后保护失明（ADC 削顶点 = (3300-1650)/132mV/A = ±12.5A）。
 * 当前 9A = 传感器额定的 90%，标准保护值 */
#define FOC_OC_LIMIT_A          9.0f
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
