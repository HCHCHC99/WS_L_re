/**
 *******************************************************************************
 * @file  foc_dci.h
 * @brief FOC 模式28 — 功角参考电流闭环 (comm_mode 28, Delta Current loop)。
 *        第 1 步（当前）：复刻 mode 27（电压开环）+ foc_calib 零偏窗。
 *        第 2 步（待做）：电压源替换为 P-only 电流环（id_ref=I·cosδ,
 *                        iq_ref=I·sinδ），之后 Watch 先扫 P 再开 I。
 *
 * ============================================================================
 * 【与 mode 27 的关系】
 *
 * 控制结构、校准序列、delta 爬坡、编码器帧、转速测量、峰峰值统计
 * 全部与 mode 27 相同。唯一新增：启动时插入 foc_calib 零矢量校准窗
 * （~210ms，丢弃 10ms + 平均 200ms），锁定三相零偏后所有 Clarke/Park
 * 使用扣除零偏的电流副本（OC 保护仍用原始采样）。
 *   目的：电流环的反馈必须零偏干净（id_ref 3~5A 级时 500mA 零偏占 10%），
 *   第 2 步换电流环之前先把观测地基打好。
 *
 * 【第 2 步预告（本步未实现，仅规划）】
 *   - 电压源 -> 电流环：vd = PID_d(id_ref, id), vq = PID_q(iq_ref, iq)
 *   - 参考生成：id_ref = I_ref·cos(δ_now), iq_ref = I_ref·sin(δ_now)，
 *     δ_now 仍由 init/targ/tr 爬坡给出；I_ref 新增 Watch 变量
 *   - 复用 dev_pid（p_valid/i_valid 独立开关，Watch 免编译先 P 后 I）
 *   - δ=90° 即 id_ref=0 的经典 FOC（本框架全覆盖）
 *
 * 工作过程（按时间顺序）：
 *   1. 进入 mode 28：清故障 -> 零矢量起 PWM -> Foc_Calib_Start() ->
 *      CALIB 态（~210ms 零矢量，锁三相零偏）。
 *   2. 校准 BETA（磁场 90°，2s）-> ALPHA（磁场 0°，2s，锁零点 offset）。
 *   3. RUN：与 mode 27 完全一致——delta 从 g_dci_dlt_init_deg 线性爬坡到
 *      g_dci_dlt_targ_deg（历时 g_dci_dlt_tr_ms，0=立即），磁场角 =
 *      转子实测电角 + delta；电压 g_dci_volt_v（Start 复位 0.6V）。
 *
 * 角度框架（与 mode 27 相同的增量累积式）：
 *   每拍 wrap-safe 差分 -> ±DCI_ENC_DELTA_MAX(32) counts 限幅 -> s_enc_pos
 *   累加；s_off_rel = ALPHA 锁零点瞬间相对计数；转子电角 =
 *   mod((s_enc_pos - s_off_rel) × 编码器方向, CPR) × 360°×极对数/CPR。
 *
 * Watch 常用变量：
 *   g_dci_dlt_init_deg / g_dci_dlt_targ_deg / g_dci_dlt_tr_ms
 *                     : 功角爬坡三参数（Start 不复位，与 mode 27 同语义）
 *   g_dci_volt_v      : 电压（=调速旋钮），Start 复位 0.6
 *   g_dci_dlt_now_deg : 当前爬坡中的 delta 指令（实时）
 *   g_dci_speed_hz    : 实测电频率（200ms 窗口，带符号）
 *   g_dci_diff_deg    : 功角实测 = 磁场角-转子角（应≈dlt_now，验证用）
 *   g_dci_id_ma / g_dci_iq_ma : 真实转子系电流（零偏校正后）
 *   g_dci_id_pp_ma / g_dci_iq_pp_ma : id/iq 峰峰值（5s 窗口刷新，抖动量化）
 *   g_dci_id_mean_ma / g_dci_iq_mean_ma : id/iq 均值（3s 窗口刷新，验零偏/
 *                     工作点核对）。近零速 ≈ I·cosδ / I·sinδ；有转速后
 *                     BEMF 吃掉 vq：iq_mean → 摩擦力矩对应小值（可≈0），
 *                     id_mean ≈ vd/R（大）。δ=90° 时 vd=0，id_mean 应≈0。
 *   g_dci_state       : 0 空闲 1 零偏校准 2 校准BETA 3 校准ALPHA 4 运行 5 过流
 *   g_calib_iu/iv/iw_off_ma : foc_calib 锁定的三相零偏 (mA)
 *
 * ISR 约束：短小、无阻塞、无打印、无 malloc。打印全部由 foc_obs 在
 * 主循环完成（事件快照 + 200ms 周期 + 5s 峰峰值）。
 *******************************************************************************
 */

#ifndef __FOC_DCI_H__
#define __FOC_DCI_H__

#include <stdint.h>
#include "foc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * Debug macros（RTT打印规范：0/1 赋值式开关，定义在 .h，.c 只调封装宏）
 *=============================================================================*/
/* 1 = RTT prints on, 0 = off */
#define FOC_DCI_DBG   1
#if FOC_DCI_DBG
    #define DCI_DBG(fmt, ...)   MAIN_D("[DCI] " fmt, ##__VA_ARGS__)
#else
    #define DCI_DBG(fmt, ...)   ((void)0)
#endif

/*=============================================================================
 * 时长/窗口（编译期常量，FOC_ISR_HZ tick 换算在 .c 内完成）
 *=============================================================================*/
#define DCI_BETA_MS       2000u  /* 校准 BETA 吸附时长 */
#define DCI_ALPHA_MS      2000u  /* 校准 ALPHA 吸附时长 */
#define DCI_SPEED_WIN_MS  200u   /* 转速测量窗口 */
#define DCI_PP_WIN_MS     5000u  /* id/iq 峰峰值统计窗口 */
#define DCI_MEAN_WIN_MS   3000u  /* id/iq 均值统计窗口 */

/*=============================================================================
 * 编码器增量限幅（与 mode 27 同：物理极限 7800rpm -> 单拍真实增量上限
 * ≈26.6 counts，超过判定为毛刺，钳位防功角框架永久污染）
 *=============================================================================*/
#define DCI_ENC_DELTA_MAX 32

/*=============================================================================
 * 状态机（g_dci_state）
 *=============================================================================*/
#define DCI_STEP_IDLE       0u  /* 未运行 */
#define DCI_STEP_CALIB      1u  /* 零矢量电流零偏校准（foc_calib，~210ms） */
#define DCI_STEP_CAL_BETA   2u  /* 校准：磁场 90°，2s */
#define DCI_STEP_CAL_ALPHA  3u  /* 校准：磁场 0°，2s，结束锁零点 */
#define DCI_STEP_RUN        4u  /* 功角闭环拖动（磁场 = 转子 + delta） */
#define DCI_STEP_FAULT_OC   5u  /* 过流停机 */

/*=============================================================================
 * 事件码（g_dci_evt，ISR 置位，Foc_Obs_Task 打印后清零）
 *=============================================================================*/
#define DCI_EVT_CALIB_DONE  1u
#define DCI_EVT_BETA_DONE   2u
#define DCI_EVT_LOCKED      3u
#define DCI_EVT_RAMP_DONE   4u
#define DCI_EVT_OC          5u

/*=============================================================================
 * Keil Watch 可调变量 / 观测量（定义见 foc_dci.c）
 *=============================================================================*/
extern volatile float    g_dci_dlt_init_deg;  /* 功角爬坡起点 (deg, 默认5, Start不复位) */
extern volatile float    g_dci_dlt_targ_deg;  /* 功角爬坡终点 (deg, 默认45, Start不复位) */
extern volatile uint32_t g_dci_dlt_tr_ms;     /* 功角爬坡过渡时间 (ms, 0=立即, Start不复位) */
extern volatile float    g_dci_volt_v;        /* 电压幅值 (V, 默认0.6, Start复位; =调速旋钮) */
extern volatile uint8_t  g_dci_running;       /* 1 = 正在运行 */
extern volatile uint8_t  g_dci_state;         /* DCI_STEP_xxx */
extern volatile uint8_t  g_dci_evt;           /* DCI_EVT_xxx */
extern volatile int32_t  g_dci_offset;        /* 校准锁零点 (hw绝对帧counts, 显示/对比用) */
extern volatile float    g_dci_dlt_now_deg;   /* 当前 delta 指令 (deg, 爬坡中实时) */
extern volatile float    g_dci_speed_hz;      /* 实测电频率 (Hz, 200ms窗口, 带符号) */
extern volatile int32_t  g_dci_field_deg;     /* 磁场电角度 (deg, 0~359) */
extern volatile int32_t  g_dci_rotor_deg;     /* 转子电角度 (deg, 0~359, 已扣零点) */
extern volatile int32_t  g_dci_diff_deg;      /* 功角实测 = field-rotor (deg, -180~180) */
extern volatile float    g_dci_id_ma;         /* 真实转子系 id (mA, 零偏校正后) */
extern volatile float    g_dci_iq_ma;         /* 真实转子系 iq (mA, 零偏校正后) */
extern volatile float    g_dci_id_pp_ma;      /* id 峰峰值 (mA, DCI_PP_WIN_MS 窗口每 5s 刷新) */
extern volatile float    g_dci_iq_pp_ma;      /* iq 峰峰值 (mA, 同上) */
extern volatile float    g_dci_id_mean_ma;    /* id 均值 (mA, DCI_MEAN_WIN_MS 窗口每 3s 刷新) */
extern volatile float    g_dci_iq_mean_ma;    /* iq 均值 (mA, 同上) */
extern volatile int32_t  g_dci_enc_pos;       /* 编码器相对计数镜像 (毛刺排查用) */
extern volatile float    g_dci_du;            /* 三相占空比观测 (%) */
extern volatile float    g_dci_dv;
extern volatile float    g_dci_dw;

/*=============================================================================
 * API
 *=============================================================================*/

/* mode 28 入口：清故障 -> 零矢量起 PWM -> foc_calib 零偏校准（~210ms）
 * -> 自动校准（BETA 2s + ALPHA 2s 锁零点）-> 功角闭环。
 * 主循环上下文调用（dev_comm_runner）。g_dci_volt_v 复位 0.6V，
 * 功角三参数不复位（便于预设后启动）。 */
void Foc_Dci_Start(void);

/* 20 kHz ISR 步进：OC 保护 -> 零偏校准/校准/锁相输出 -> 转速与电流观测。
 * 由 Foc_Isr 在 g_dci_running 时分发调用。 */
void Foc_Dci_Step(const stc_i_data_t *pData);

/* 停止（用户中途切模式时调用）：清运行标志 + 关 PWM（若在输出）。
 * 自带 g_foc_active 清零，防止 ISR 回退。 */
void Foc_Dci_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_DCI_H__ */
