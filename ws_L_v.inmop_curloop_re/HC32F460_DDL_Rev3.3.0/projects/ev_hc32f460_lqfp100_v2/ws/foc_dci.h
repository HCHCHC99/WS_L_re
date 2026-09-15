/**
 *******************************************************************************
 * @file  foc_dci.h
 * @brief FOC 模式28 — 校准 + 功角参考电流闭环 (Delta Current loop)。
 *        mode 24 和 mode 29 已分别拆入 foc_dcal24 与 foc_drun29。
 *        当前阶段：P 重调（i_valid=false）——转子捏死（BEMF=0）下调 P，
 *        判据 = 快速接近目标电流且不超过（不振铃）；合格后再开 I。
 *
 * ============================================================================
 * 【第 2 步：电流环语义（与电压版的本质差异）】
 *
 *   电压版（mode 27/28 第 1 步）：电压矢量钉在转子+δ，电流是"漂"出来的，
 *     BEMF 吃掉 vq -> iq 塌缩到摩擦平衡值，id 被 vd 顶着走（δ=45° 实测 2.4A）。
 *   电流版（本步）：电流矢量被 PI 主动钉在 δ 方向（幅值 I_ref），转矩
 *     T = 1.5·p·ψf·I_ref·sinδ 直接受控。转速 = 力矩平衡结果：参考力矩
 *     大于摩擦时持续加速，直到 PI 电压饱和（|vd/vq| ≤ UMAX）后电流 droop、
 *     力矩回落到摩擦平衡 -> 稳定在饱和限速（与电压版完全不同的平衡机理）。
 *   δ=90° 即 id_ref=0 的经典 FOC（id=0 MTPA）。
 *
 * 【P 重调工作流（mode 28 一体化）】
 *   1. comm_mode=28：自动完成零偏/BETA/ALPHA 后直接进入电流环。
 *   4. kp 扫参（Watch 改 g_dci_pid_*_cfg.kp）：0.5 → 1.0 → 2.0
 *      判据：CH5(iq) 快速接近 CH7(iq_ref) 且不超过（不振铃、eq_pp 不发散）
 *      注意 P-only 稳态差距 = R·i_ref/(kp+R) 属正常（BEMF=0、kp=0.5 → 77%）
 *   5. P 合格后：Watch 置 i_valid=1 + ki=DCI_PI_KI 收尾，eq_mean → 0
 *
 * ============================================================================
 * 【与 mode 27 的关系（第 1 步遗留说明）】
 *
 * 控制结构、校准序列、delta 爬坡、编码器帧、转速测量、峰峰值统计
 * 全部与 mode 27 相同。启动时插入 foc_calib 零矢量校准窗（~210ms），
 * 锁定三相零偏后所有 Clarke/Park 使用扣除零偏的电流副本（OC 保护仍用
 * 原始采样）。已实测验收：δ=90° 时 id_mean=69mA（理论 ωLs·iq/R≈69），
 * 零偏清除确认。
 *
 * 工作过程（按时间顺序）：
 *   1. 进入 mode 28：清故障 -> 零矢量起 PWM -> Foc_Calib_Start() ->
 *      CALIB 态（~210ms 零矢量，锁三相零偏）。
 *   2. 校准 BETA（磁场 90°，2s）-> ALPHA（磁场 0°，2s，锁零点 offset）。
 *      （校准吸附仍是电压开环，g_dci_volt_v 0.6V）
 *   3. RUN（电流环）：delta 照常爬坡（init/targ/tr），电压源替换为电流
 *      参考 id_ref = I_ref·cosδ, iq_ref = I_ref·sinδ + P-only PI
 *      （i_valid=0）-> vd/vq -> 反 Park（转子角）-> SVPWM。
 *
 * 角度框架（与 mode 27 相同的增量累积式）：
 *   每拍 wrap-safe 差分 -> ±DCI_ENC_DELTA_MAX(32) counts 限幅 -> s_enc_pos
 *   累加；s_off_rel = ALPHA 锁零点瞬间相对计数；转子电角 =
 *   mod((s_enc_pos - s_off_rel) × 编码器方向, CPR) × 360°×极对数/CPR。
 *   RUN 控制系 = 转子系（Park/反 Park 都在转子角）。
 *
 * Watch 常用变量：
 *   g_dci_dlt_init_deg / g_dci_dlt_targ_deg / g_dci_dlt_tr_ms
 *                     : 功角爬坡三参数（Start 不复位，与 mode 27 同语义）
 *   g_dci_i_ref_ma    : 电流矢量幅值参考 (mA, Start 不复位) —— 力矩旋钮
 *   g_dci_pid_id_cfg / g_dci_pid_iq_cfg 的 .kp/.ki/.i_valid
 *                     : PI 参数（Watch 实时可调；i_valid=0 为 P-only）
 *   g_dci_volt_v      : 校准吸附电压，Start 复位 0.6（RUN 不再用）
 *   g_dci_dlt_now_deg : 当前爬坡中的 delta 指令（实时）
 *   g_dci_id_ref_ma / g_dci_iq_ref_ma : 电流参考实时值 (=斜坡后幅值·cosδ/sinδ)
 *   g_dci_i_ramp_ma_s : I_ref 软启动斜率 (mA/s)，<=0 直通（阶跃），>0 每拍小目标爬坡
 *   g_dci_vsat        : 电压饱和标志（1 = 任一轴顶到 UMAX）
 *   g_dci_speed_hz    : 实测电频率（200ms 窗口，带符号）
 *   g_dci_diff_deg    : 参考功角（=dlt_now；实际跟踪误差看 e_d/e_q 统计）
 *   g_dci_id_ma / g_dci_iq_ma : 真实转子系电流（零偏校正后，= 环反馈）
 *   g_dci_id_pp_ma / g_dci_iq_pp_ma : id/iq 峰峰值（5s 窗口刷新，抖动量化）
 *   g_dci_id_mean_ma / g_dci_iq_mean_ma : id/iq 均值（3s 窗口刷新）
 *   g_dci_ed/eq_mean_ma, g_dci_ed/eq_pp_ma : 误差均值/峰峰值（5s 窗口，
 *                     P 调参主判据：mean=稳态落差，pp=振铃）
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
#include "../Utils/dev_pid.h"

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
#define DCI_ERR_WIN_MS    5000u  /* 电流环误差统计窗口 */

/*=============================================================================
 * 电流环参数（P 重调阶段：i_valid=false 纯 P。转子捏死（BEMF=0）下调 P：
 * 判据 = 快速接近目标电流且不超过（不振铃）。
 * P 合格后 Watch 置 g_dci_pid_*_cfg.i_valid=1 + ki=DCI_PI_KI 收尾）
 *=============================================================================*/
#define DCI_PI_KP         0.5f   /* PI 比例增益 (V/A)。P-only 门槛 kp≈(3~7)×R≈0.5~1；
                                    kp 过小 P 环产不出克服 R+BEMF 的电压（电机不动）；
                                    kp 过大（≈L/Ts 量级）采样延迟引发振铃 */
#define DCI_PI_KI         300.0f /* PI 积分增益（V/A/s）≈ kp×R/L（kp=0.5, L≈0.26mH 实测反推，
                                    零极点对消起点），从小往大调 */
#define DCI_PI_UMAX_V     3.5f   /* PI 输出限幅 (V)，|vd/vq| ≤ UMAX（与 mode 31 同款；
                                    3A 矢量可保持至 ~5300rpm，再高触及饱和限速） */
#define DCI_ITERM_MAX_V   3.2f   /* 积分项限幅 (V)，必须 > 最大 BEMF 3.05V @ 891Hz 天花板 */
#define DCI_I_REF_MA      500.0f /* 电流矢量幅值参考 (mA) —— 调 I 阶段默认 ≈摩擦电流；
                                    eq_mean≈0 后 Watch 逐步上调 250→500→1000→3000 */
#define DCI_I_RAMP_MA_S   0.0f   /* I_ref 软启动斜率 (mA/s)。<=0 = 直通（阶跃激励，调 P 用）；
                                    Watch 置正数（如 1000）启用软启动：参考幅值每拍向目标爬
                                    一小步（小目标），0->1000mA 约 1s 走完，避免转矩突跳 */

/*=============================================================================
 * 编码器增量限幅（与 mode 27 同：物理极限 7800rpm -> 单拍真实增量上限
 * ≈26.6 counts，超过判定为毛刺，钳位防功角框架永久污染）
 *=============================================================================*/
#define DCI_ENC_DELTA_MAX 32

/*=============================================================================
 * 状态机（g_dci_state）：mode 28 专用，校准后固定进入 RUN。
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
extern volatile uint8_t  g_dci_running;       /* 1 = mode 28 正在运行 */
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
extern volatile float    g_dci_i_ref_ma;      /* 电流矢量幅值参考 (mA, Start 不复位, Watch 可调) */
extern volatile float    g_dci_id_ref_ma;     /* d 轴电流参考 (mA, 实时 = I_ref·cosδ) */
extern volatile float    g_dci_iq_ref_ma;     /* q 轴电流参考 (mA, 实时 = 斜坡后幅值·sinδ) */
extern volatile float    g_dci_i_ramp_ma_s;   /* I_ref 软启动斜率 (mA/s, Watch 可调；<=0 = 直通阶跃) */
extern volatile uint8_t  g_dci_vsat;          /* 电压饱和标志 (1 = 任一轴顶到 UMAX，调参数据作废) */
extern volatile float    g_dci_ed_mean_ma;    /* d 轴误差均值 (mA, DCI_ERR_WIN_MS 窗口刷新) */
extern volatile float    g_dci_eq_mean_ma;    /* q 轴误差均值 (mA, 同上；P-only 稳态落差) */
extern volatile float    g_dci_ed_pp_ma;      /* d 轴误差峰峰值 (mA, 同上；增大 = 振荡) */
extern volatile float    g_dci_eq_pp_ma;      /* q 轴误差峰峰值 (mA, 同上) */
extern pid_config_t      g_dci_pid_id_cfg;    /* d 轴 PI 配置（字段 volatile，Watch 实时可调） */
extern pid_config_t      g_dci_pid_iq_cfg;    /* q 轴 PI 配置（i_valid=0 为 P-only） */
extern volatile int32_t  g_dci_enc_pos;       /* 编码器相对计数镜像 (毛刺排查用) */
extern volatile float    g_dci_du;            /* 三相占空比观测 (%) */
extern volatile float    g_dci_dv;
extern volatile float    g_dci_dw;

/*=============================================================================
 * API
 *=============================================================================*/

/* mode 28 入口：清故障 -> 零矢量起 PWM -> foc_calib 零偏校准
 * （~210ms）-> 自动校准（BETA 2s + ALPHA 2s 锁零点）-> 功角参考电流闭环。
 * 主循环上下文调用（dev_comm_runner）。g_dci_volt_v 复位 0.6V，
 * 功角三参数与 g_dci_i_ref_ma 不复位（便于预设后启动）。 */
void Foc_Dci_Start(void);

/* PI 实例绑定（Foc_Init 调用一次，与 mode 31 的 InitPids 同模式） */
void Foc_Dci_InitPids(void);

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
