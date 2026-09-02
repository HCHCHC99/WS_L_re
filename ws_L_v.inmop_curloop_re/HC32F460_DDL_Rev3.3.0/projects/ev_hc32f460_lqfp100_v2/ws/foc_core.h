/**
 *******************************************************************************
 * @file  foc_core.h
 * @brief FOC 公共核心 — 跨模式共享的状态、观测量、保护与助手函数。
 *
 *        职责（单一）：
 *          - 定义所有跨模式共享的 volatile 观测量 / Keil Watch 调参量
 *          - FOC 内部状态机 (foc_state_t) 与对齐零点 (align offset) 的托管
 *          - 过流保护（OC 去抖）与故障停机
 *          - PWM 输出启停公共封装
 *          - 公共控制助手：GetDq(Clarke/Park)、EMA 滤波、电压包络、
 *            编码器电角度换算
 *
 *        依赖方向：各运行模式模块 (openloop/curloop/align/zizeng) -> 本模块。
 *        本模块不感知任何具体模式的业务逻辑。
 *******************************************************************************
 */

#ifndef __FOC_CORE_H__
#define __FOC_CORE_H__

#include <stdint.h>
#include "I.h"

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * FOC run modes (g_foc_mode)
 *=============================================================================*/
#define FOC_MODE_NONE      0u   /* FOC stopped */
#define FOC_MODE_OPENLOOP  1u   /* comm_mode 21: open-loop V/f */
#define FOC_MODE_CURLOOP   2u   /* comm_mode 22: encoder FOC current loop */
#define FOC_MODE_ALIGN     3u   /* comm_mode 23: standstill electrical alignment */

/*=============================================================================
 * FOC 内部状态机（模式22 I-F 启动/RUN 使用；故障与停止时回到 IDLE）
 *=============================================================================*/
typedef enum {
    FOC_STATE_IDLE     = 0,
    FOC_STATE_IF_START = 1,
    FOC_STATE_RUN      = 2,
    FOC_STATE_ALIGN    = 3,
} foc_state_t;

/*******************************************************************************
 * 跨模式共享观测量 / Keil Watch 调参量（定义见 foc_core.c）
 ******************************************************************************/

extern volatile float    g_foc_theta_rad;        /* electrical angle (rad) */
extern volatile float    g_foc_du;               /* U duty (%) */
extern volatile float    g_foc_dv;               /* V duty (%) */
extern volatile float    g_foc_dw;               /* W duty (%) */
extern volatile float    g_foc_valpha;           /* stationary alpha voltage (V) */
extern volatile float    g_foc_vbeta;            /* stationary beta voltage (V) */
extern volatile uint8_t  g_foc_active;           /* 1 = FOC output enabled */

extern volatile uint8_t  g_foc_mode;             /* FOC_MODE_xxx */
extern volatile uint8_t  g_foc_phase;            /* 0=idle 1=hold 2=ramp/sync 3=run 4=align */
extern volatile int32_t  g_foc_cur_sign;         /* current sign correction (+1/-1), Watch tunable */
extern volatile int32_t  g_foc_enc_dir;          /* encoder direction for electrical angle (+1/-1), Watch tunable */

extern volatile float    g_foc_id_ma;            /* d-axis feedback (mA) */
extern volatile float    g_foc_iq_ma;            /* q-axis feedback (mA) */
extern volatile float    g_foc_vd;               /* d-axis PI output (V) */
extern volatile float    g_foc_vq;               /* q-axis PI output (V) */

extern volatile uint8_t  g_foc_align_state;      /* 0=idle, 1=aligning, 2=running */
extern volatile uint8_t  g_foc_fault;            /* 1 = over-current fault */

/* Over-current limit (A, Keil Watch editable) and trip diagnostic (mA) */
extern volatile float    g_foc_oc_limit_a;
extern volatile float    g_foc_fault_i_ma;
extern volatile uint8_t  g_foc_fault_stage;      /* g_foc_phase at OC trip */
extern volatile int16_t  g_foc_fault_iu_ma;      /* phase currents at OC trip (mA) */
extern volatile int16_t  g_foc_fault_iv_ma;
extern volatile int16_t  g_foc_fault_iw_ma;

/* Voltage envelope / feedback filter (Keil Watch editable) */
extern volatile float    g_foc_vmax_v;           /* current-loop max |v| (V) */
extern volatile float    g_foc_vramp_v_s;        /* voltage envelope ramp (V/s) */
extern volatile float    g_foc_cur_fb_alpha;     /* EMA weight on id/iq (1.0 = off) */
extern volatile float    g_foc_vlim_v;           /* current voltage envelope (V) */

/* 开环调参量：模式21 直接使用；模式22 I-F 频率爬升以 freq 为目标 */
extern volatile float    g_foc_openloop_freq_hz; /* electrical freq (Hz), Keil Watch editable */
extern volatile float    g_foc_openloop_volt_v;  /* voltage amplitude (V), Keil Watch editable */

/* 共享角度观测量（模式22 与模式30 均写入） */
extern volatile float    g_foc_if_rotor_rad;     /* rotor electrical angle, folded [0,2PI) (rad) */
extern volatile float    g_foc_if_diff_rad;      /* encoder-elec angle - synthetic angle (rad) */

/* 对齐电零点：模式23 记录，模式22 I-F 交接沿用（编码器 electrical-zero count） */
extern volatile int32_t  g_foc_align_offset;

/*******************************************************************************
 * 公共 API
 ******************************************************************************/

/* 清除故障标志 + OC 去抖计数（各 Start 入口调用） */
void Foc_Core_ClearFault(void);

/* 过流检测（OC 去抖）。1 = 触发过流（内部已记录诊断信息） */
uint8_t Foc_Core_OverCurrent(const stc_i_data_t *pData);

/* 故障停机：置故障码、关 PWM、回 IDLE */
void Foc_Core_FaultStop(uint8_t u8Code);

/* PWM 输出公共启动：SetFocMode + 50/50/50 duty + StartOutput + g_foc_active=1 */
void Foc_Core_PwmStart(void);

/* PWM 输出公共停止：EmergencyStop + 三相 duty 观测量清零 */
void Foc_Core_PwmStop(void);

/* 电压包络复位（s_vlim = 0，g_foc_vlim_v 在下一次 Apply 时刷新） */
void Foc_Core_ResetVoltageEnvelope(void);

/* id/iq EMA 滤波器状态复位 */
void Foc_Core_ResetEma(void);

/* 电压包络限幅：s_vlim 按 vramp 爬升，|v| 超限则等比缩小 vd/vq */
void Foc_Core_ApplyVoltageEnvelope(float *vd, float *vq);

/* id/iq EMA 滤波（原位更新） */
void Foc_Core_EmaFilter(float *id, float *iq);

/* 三相电流 (mA) -> Clarke -> Park(theta) -> id/iq (A) */
void Foc_Core_GetDq(const stc_i_data_t *pData, float theta, float *id, float *iq);

/* 编码器计数 (含对齐零点/方向修正) -> 电流环电角度 (rad) */
float Foc_Core_CurLoopTheta(void);

/* 有符号取模（结果 [0, n)） */
int32_t Foc_Core_ModPos(int32_t x, int32_t n);

/* 开环电压上限钳制（每次 ISR 调用，防 Watch 误输入） */
void Foc_Core_ClampOpenLoopVolt(void);

/* 内部状态机托管 */
foc_state_t Foc_Core_GetStateMachine(void);
void Foc_Core_SetStateMachine(foc_state_t state);

/* 对齐零点托管（写入时同步刷新 g_foc_align_offset 观测量） */
int32_t Foc_Core_GetAlignOffset(void);
void Foc_Core_SetAlignOffset(int32_t offset);

/*******************************************************************************
 * 原有对外 API（实现移自 foc.c，名称不变）
 ******************************************************************************/

/* Stop FOC: clear active, disable PWM output, zero duty observables. */
void Foc_Stop(void);

/* Current-loop state (0=idle, 1=aligning, 2=running) */
uint8_t Foc_GetState(void);

/* 1 = FOC output enabled (open-loop or current-loop) */
uint8_t Foc_IsRunning(void);

/* Mechanical encoder counts -> electrical angle (rad). */
float Foc_EncoderElecAngleRad(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_CORE_H__ */
