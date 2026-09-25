/**
 *******************************************************************************
 * @file  foc_24_dcal.h
 * @brief FOC mode 24 - independent calibration-only DCAL24.
 *
 * Mode 24 owns its own zero-current window, BETA alignment, ALPHA alignment,
 * over-current path, and completion event.  The finished calibration snapshot
 * is copied by value to mode 29; no run-time mode state is shared.
 *
 * ============================ 模式速览卡（唯一事实源）========================
 * 模式：24 = **仅校准**（零偏窗 + BETA 2s + ALPHA 2s + 锁双帧零点）
 *        拆分自 mode 28，解决"捏转子时机抢不过校准"的问题
 * 入口：comm_mode = 24
 * 前置：无
 * 结束：**自动关 PWM 回 mode 0**（转子自由，此后可任意捏动转子）
 * 后续：mode 29 / 40 / 41 / 45 启动时读本模式结果，**无校准则拒绝启动**
 *
 * 【校准后转子可以随便动】（关键认知）
 *   - 电流零偏：与转子位置无关（传感器零点）
 *   - 编码器零点 offset：固定零点基准，转子移动后基准依然成立
 *   - TMRA_1 硬件计数器 free-run 不停，手转不丢计数；mode 0 下
 *     g_foc_elec_deg 实时可见转子位置
 *
 * 【Watch 可调变量】（名称 = 单位）
 *   g_dcal24_volt_v    V   校准吸附电压
 *
 * 【关键观察变量】
 *   g_dcal24_running   1 = 运行中
 *   g_dcal24_state     0 IDLE / 1 ZERO零偏窗 / 2 BETA / 3 ALPHA / 4 DONE / 5 FAULT_OC
 *   g_dcal24_evt       1 ZERO_DONE / 2 BETA_DONE / 3 DONE / 4 OC
 *   g_dcal24_beta_hw    BETA 结束时的编码器原始计数
 *   g_dcal24_alpha_hw   ALPHA 结束时的编码器原始计数
 *   g_dcal24_moved      BETA→ALPHA 位移 (counts)
 *   g_dcal24_offset     锁定的零点 (counts)
 *   g_dcal24_zero_u/v/w_ma  三相电流零偏 (mA)
 *   g_foc_elec_deg      校准后 mode 0 下实时电角度（捏转子可见跟随）
 *
 * 【给其他模式的接口】
 *   UINT8 Foc_Dcal24_GetResult(foc_dcal24_result_t *r)  ← 返回 0 = 无有效校准
 *     结构体含：valid / offset / zero_u_ma / zero_v_ma / zero_w_ma
 *     **按值拷贝，无运行时共享状态**
 *
 * 【VOFA 通道】mode 24 走通用 17ch 布局（见 main.c 顶部说明）：
 *   ch0~2 三相电流(A)，校准期间可见 BETA/ALPHA 吸附的电流包络
 *   ch3~16 通用布局其余通道 —— **本模式无专属语义**
 *   判读建议：用 RTT 的 [DCAL24] 事件（"cal-only done"）+ Watch 变量，不要依赖 VOFA。
 * ===========================================================================
 */

#ifndef __FOC_24_DCAL_H__
#define __FOC_24_DCAL_H__

#include <stdint.h>
#include "foc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DCAL24_DBG   1
#if DCAL24_DBG
#define DCAL24_LOG(fmt, ...)  MAIN_D("[DCAL24] " fmt, ##__VA_ARGS__)
#else
#define DCAL24_LOG(fmt, ...)  ((void)0)
#endif

#define DCAL24_ZERO_SKIP_SAMPLES  200u
#define DCAL24_ZERO_AVG_SAMPLES   4000u
#define DCAL24_BETA_MS            2000u
#define DCAL24_ALPHA_MS           2000u

#define DCAL24_STEP_IDLE       0u
#define DCAL24_STEP_ZERO       1u
#define DCAL24_STEP_BETA       2u
#define DCAL24_STEP_ALPHA      3u
#define DCAL24_STEP_DONE       4u
#define DCAL24_STEP_FAULT_OC   5u

#define DCAL24_EVT_ZERO_DONE   1u
#define DCAL24_EVT_BETA_DONE   2u
#define DCAL24_EVT_DONE        3u
#define DCAL24_EVT_OC          4u

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

void Foc_Dcal24_Start(void);
void Foc_Dcal24_Stop(void);
void Foc_Dcal24_Step(const stc_i_data_t *pData);
uint8_t Foc_Dcal24_GetResult(foc_dcal24_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_24_DCAL_H__ */
