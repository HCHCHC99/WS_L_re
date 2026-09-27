/**
 *******************************************************************************
 * @file  foc_24_dcal.h
 * @brief FOC mode 24 - independent calibration-only DCAL24.
 *
 * Mode 24 owns its own zero-current window, BETA alignment, ALPHA alignment,
 * over-current path, and completion event.  The finished calibration snapshot
 * is copied by value to mode 29; no run-time mode state is shared.
 *
 * ==============================================================================
 *   模式速览卡   MODE 24   仅校准（电流零偏 + 编码器零点，完事自动回 mode 0）
 *   本卡是本模式唯一事实源；代码内注释与本卡冲突时，以本卡为准
 * ==============================================================================
 *   入口   comm_mode = 24
 *   前置   无（零偏窗 + BETA 2s + ALPHA 2s 全程自动，约 4.2s）
 *   结束   自动：EVT_DONE 后自动 PwmStop 回 mode 0（转子自由）；过流立即停机回
 *          mode 0
 *   标记   [!] 易错点   (*) 主判据   [可调] Watch 可写   [只读] 仅观察
 * ------------------------------------------------------------------------------
 *   [可调] Watch 变量 -- 改值即时生效，复位后回宏默认
 * ------------------------------------------------------------------------------
 *       变量                        当前值  单位     含义
 *       --------------------------- ------- -------- ---------------------------
 *       g_dcal24_volt_v             0.6     V        校准吸附电压，0.6V 起步
 *   [!] 校准期间 0.6V 加在静止转子上（约
 *       6A，发热明显），务必摘掉机械负载再跑；过流阈值宏为 9.0A。
 * ------------------------------------------------------------------------------
 *   [只读] 关键观察变量
 * ------------------------------------------------------------------------------
 *       变量                           含义
 *       ------------------------------ -----------------------------------------
 *   (*) g_dcal24_state                 状态机：1 零偏，2 BETA，3 ALPHA，4 完成
 *       g_dcal24_running               运行标志（1 = 正在校准）
 *       g_dcal24_evt                   事件：1 零偏锁定 / 2 BETA完成 / 3 完成
 *   (*) g_dcal24_offset                锁定的编码器零点 counts，电角度 0 位
 *       g_dcal24_beta_hw               BETA 结束瞬间的 TMRA_1 原始计数
 *       g_dcal24_alpha_hw              ALPHA 结束瞬间的 TMRA_1 原始计数
 *       g_dcal24_moved                 BETA 到 ALPHA 的位移（counts，带符号）
 *       g_dcal24_zero_u_ma             U 相电流零偏 (mA，零偏窗均值)
 *       g_dcal24_zero_v_ma             V 相电流零偏 (mA，零偏窗均值)
 *       g_dcal24_zero_w_ma             W 相电流零偏 (mA，零偏窗均值)
 *       g_foc_align_offset             已生效的对齐零点（counts）
 *   (*) g_foc_elec_deg                 mode 0 下实时电角度；捏转子看它跟随
 *   (*) g_foc_fault                    故障标志（非 0 = 已跳闸停机）
 *       g_foc_fault_i_ma               过流跳闸瞬间的相电流 (mA)
 * ------------------------------------------------------------------------------
 *   VOFA 通道 -- 16ch，填充见 Foc_Dcal_VofaFill
 * ------------------------------------------------------------------------------
 *       通道  含义                   单位     备注
 *       ----- ---------------------- -------- ----------------------------------
 *       ch0   U 相电流               A        零偏窗/BETA/ALPHA 各有包络
 *       ch1   V 相电流               A
 *       ch2   W 相电流               A
 *   (*) ch3   id 反馈                A        磁场轴上的吸持电流
 *       ch4   iq 反馈                A        吸稳后应接近 0
 *       ch5   状态                   -        1 零偏 / 2 BETA / 3 ALPHA / 4 完成
 *                                             / 5 OC
 *   (*) ch6   事件                   -        1 零偏锁定 / 2 BETA完成 / 3 完成 /
 *                                             4 OC
 *       ch7   BETA 结束计数          counts   TMRA_1 原始值
 *       ch8   ALPHA 结束计数         counts   TMRA_1 原始值
 *   (*) ch9   BETA 到 ALPHA 位移     counts   应约 +/-102 = CPR/(4 x 极对数)
 *   (*) ch10  锁定的零点             counts   本模式核心产物
 *       ch11  U 相电流零偏           A        零偏窗均值
 *       ch12  V 相电流零偏           A
 *       ch13  W 相电流零偏           A
 *       ch14  已生效零点             counts   = g_foc_align_offset，应与 ch10
 *                                             相同
 *       ch15  故障标志               0/1      非 0 = 过流停机
 * ------------------------------------------------------------------------------
 *   判据与坑
 * ------------------------------------------------------------------------------
 *   (*) 主判据是 RTT 的 [DCAL24] 事件行（不是 VOFA）：zero-offset locked / BETA
 *       done hw=.. / done: beta alpha moved offset cnts。
 *   (*) done 行的 offset 换算成角度应是合理零点；随后进 mode 0
 *       捏转子，g_foc_elec_deg 平滑跟随即可。
 *   (*) 校准结果由 s_result 按值保留，mode 29/40/41/45 启动先调
 *       Foc_Dcal_GetResult() 校验，无有效校准拒绝启动。
 *   [!] 结果只在 RAM 里，掉电即失；每次上电必须先跑 mode 24，否则 mode
 *       29/40/41/45 起不来。
 *   [!] 校准完转子可随便捏：电流零偏与位置无关，编码器零点是固定基准，TMRA_1
 *       计数器 free-run 不丢计数。
 *   [!] VOFA+ 通道数必须同步配成 16（本模式自持布局），否则整帧错位。本布局只在
 *       g_dcal24_running = 1 期间出现：完成/故障后运行标志清 0，帧切回通用
 *       20ch（那时看 ch15 对齐零点）。
 * ==============================================================================
 */

#ifndef __FOC_24_DCAL_H__
#define __FOC_24_DCAL_H__

#include <stdint.h>
#include "foc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FOC24_DBG   1
#if FOC24_DBG
#define FOC24_LOG(fmt, ...)  MAIN_D("[DCAL24] " fmt, ##__VA_ARGS__)
#else
#define FOC24_LOG(fmt, ...)  ((void)0)
#endif

#define FOC24_ZERO_SKIP_SAMPLES  200u
#define FOC24_ZERO_AVG_SAMPLES   4000u
#define FOC24_BETA_MS            2000u
#define FOC24_ALPHA_MS           2000u

#define FOC24_STEP_IDLE       0u
#define FOC24_STEP_ZERO       1u
#define FOC24_STEP_BETA       2u
#define FOC24_STEP_ALPHA      3u
#define FOC24_STEP_DONE       4u
#define FOC24_STEP_FAULT_OC   5u

#define FOC24_EVT_ZERO_DONE   1u
#define FOC24_EVT_BETA_DONE   2u
#define FOC24_EVT_DONE        3u
#define FOC24_EVT_OC          4u

typedef struct {
    uint8_t valid;
    int32_t offset;
    float   zero_u_ma;
    float   zero_v_ma;
    float   zero_w_ma;
} foc_dcal24_result_t;

extern volatile uint8_t  g_dcal24_running;
extern volatile uint8_t  g_dcal24_state;
extern volatile uint8_t  g_dcal24_evt;
extern volatile uint16_t g_dcal24_beta_hw;
extern volatile uint16_t g_dcal24_alpha_hw;
extern volatile int32_t  g_dcal24_moved;
extern volatile int32_t  g_dcal24_offset;
extern volatile float    g_dcal24_zero_u_ma;
extern volatile float    g_dcal24_zero_v_ma;
extern volatile float    g_dcal24_zero_w_ma;
extern volatile float    g_dcal24_volt_v;

void Foc_Dcal_Start(void);
void Foc_Dcal_Stop(void);
void Foc_Dcal_Step(const stc_i_data_t *pData);
uint8_t Foc_Dcal_GetResult(foc_dcal24_result_t *result);

/* 模式自持 VOFA：固定 16ch，返回通道数；通道含义见本文件顶部速览卡。 */
int  Foc_Dcal_VofaFill(int32_t *cur);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_24_DCAL_H__ */
