/**
 *******************************************************************************
 * @file  foc_curloop.h
 * @brief FOC 模式22 — 编码器 FOC 电流环 (comm_mode 22, I-F 启动)。
 *
 *        Foc_StartCurrentLoop() -> IF_START 从第一个 tick 起即以合成角度
 *        做电流控制（I-F 启动）：
 *          - 频率 0 -> g_foc_openloop_freq_hz 爬升
 *          - Iq 0 -> g_foc_iq_ref_cmd_ma 爬升
 *          - 电压包络 0 -> g_foc_vmax_v 爬升
 *        编码器电角度跟踪合成角度（同步窗口）后 Foc_IfHandover() 平滑切换
 *        到编码器角度，RUN 继续编码器角度 FOC：
 *          ia/ib/ic (A) -> Clarke -> Park(theta) -> id/iq
 *          -> vd = PI_id(0, id), vq = PI_iq(iq_ref, iq)   (dev_pid)
 *          -> InvPark(vd, vq, theta) -> Svpwm -> TMR4 duty
 *
 *        ISR 约束：短小、无阻塞、无打印、无 malloc。
 *******************************************************************************
 */

#ifndef __FOC_CURLOOP_H__
#define __FOC_CURLOOP_H__

#include <stdint.h>
#include "foc_core.h"
#include "../Utils/dev_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * 模式22 观测量 / Keil Watch 调参量（定义见 foc_curloop.c）
 ******************************************************************************/
extern volatile int32_t  g_foc_pi_off_180;      /* add 180deg to control angle to flip torque direction (0/1), Watch tunable */
extern volatile float    g_foc_iq_ref_cmd_ma;   /* Iq target (mA), Keil Watch editable */
extern volatile float    g_foc_iq_ref_ma;       /* ramped Iq setpoint actually used (mA) */
extern volatile float    g_foc_iq_ramp_ma_s;    /* Iq soft-start ramp (mA/s) */
extern volatile float    g_foc_iq_pi_ma;        /* actual RUN Iq reference from speed PI (mA) */
extern volatile float    g_foc_run_target_rpm;  /* RUN speed PI target (rpm), Watch tunable */
extern volatile float    g_foc_anchor_deg;      /* extra anchor angle (deg), sweep to find RUN frame */
extern volatile float    g_foc_run_iq_sign;     /* RUN Iq sign (+-1), Watch tunable */

/* I-F start observables */
extern volatile float    g_foc_if_freq_hz;   /* current I-F electrical frequency (Hz) */
extern volatile float    g_foc_if_sweep_cHz; /* live diff sweep rate (cHz), 100ms window */
extern volatile float    g_foc_if_hold_iq_ma; /* I-F hold Iq threshold (mA), Watch tunable */
extern volatile float    g_foc_if_sync_band_rad; /* sync window band (rad), Watch tunable */
extern volatile uint32_t g_foc_if_sync_win_cnt;  /* sync window length (samples @20k), Watch tunable */
extern volatile uint32_t g_foc_if_sync_good_wins;/* consecutive good windows required, Watch tunable */
extern volatile float    g_foc_if_lock_diff_rad; /* lock offset latched while aligned in hold (rad) */
extern volatile float    g_foc_if_rel_diff_rad;  /* unwrapped diff relative to lock offset (rad) */
extern volatile float    g_foc_if_win_min_rad;   /* current sync-window min of rel diff (rad) */
extern volatile float    g_foc_if_win_max_rad;   /* current sync-window max of rel diff (rad) */
extern volatile uint32_t g_foc_if_win_cnt;       /* samples collected in current window */
extern volatile uint32_t g_foc_if_good_cnt;      /* consecutive good windows so far */
extern volatile uint8_t  g_foc_if_sync;      /* 1 = synchronized, handed over to encoder */
extern volatile uint8_t  g_foc_if_evt;       /* 1=hold done 2=handover 3=timeout */
extern volatile int32_t  g_foc_if_evt_v1;
extern volatile int32_t  g_foc_if_evt_v2;
extern volatile int32_t  g_foc_if_evt_v3;
extern volatile int32_t  g_foc_if_evt_v4;    /* diff sweep rate at timeout (cHz) */

/* Current-loop PI configs (volatile, Keil Watch can tune kp/ki live) */
extern pid_config_t g_foc_pid_id_cfg;
extern pid_config_t g_foc_pid_iq_cfg;
extern pid_config_t g_foc_pid_spd_cfg;

/*******************************************************************************
 * API
 ******************************************************************************/

/* PID 实例绑定配置（Foc_Init 调用一次） */
void Foc_CurLoop_InitPids(void);

/* Start current-loop FOC (comm_mode 22): clear fault, reset PIs, enter
 * I-F start (current-controlled, synthetic angle), hand over to encoder angle
 * when synchronized, then RUN (encoder angle + Id/Iq PI). */
void Foc_StartCurrentLoop(void);

/* 模式22 单步运算（20 kHz ISR 中由 Foc_Isr 分发调用，内部按状态机分派） */
void Foc_CurLoop_Step(const stc_i_data_t *pData);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_CURLOOP_H__ */
