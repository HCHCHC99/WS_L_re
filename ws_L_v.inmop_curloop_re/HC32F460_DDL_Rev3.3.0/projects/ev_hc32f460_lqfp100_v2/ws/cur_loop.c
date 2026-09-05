/**
 *******************************************************************************
 * @file  cur_loop.c
 * @brief Current PI loop — 精简版：电流环 PI 控制已禁用
 *        （电流采集由 I.c 继续工作，ADC 回调不再执行电流环 PI）
 *
 *        模式23（对齐校准）和模式30（自增拖动）不需要电流环 PI 控制，
 *        因此 curloop_isr 被替换为空函数，所有 PI 相关代码都被条件编译排除。
 *******************************************************************************
 */

#include "cur_loop.h"
#include "I.h"
#include "timer6_timebase.h"
#include "TickTimer.h"
#include "rtt_log.h"
#include "motor_config.h"

/* ============================================================================
 * 全局变量（Keil Watch 可调）
 * ============================================================================*/
volatile float g_i_ref_ma = 150.0f;
volatile float g_cur_ref_ext_ma = 0.0f;

volatile float g_scope_i_ref = 0.0f;
volatile float g_scope_i_fb = 0.0f;
volatile float g_scope_i_fb_raw = 0.0f;
volatile float g_scope_i_duty = 0.0f;
volatile float g_scope_i_err = 0.0f;
volatile float g_scope_i_ol = 0.0f;
volatile uint32_t g_scope_i_dt_us = 0;

volatile float g_cur_fb_alpha = 0.3f;
volatile uint32_t g_cur_dbg_ms = 20;

/* 电流环 PID 配置（保留但未使用） */
pid_config_t g_cur_pid_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = 0.1f,
    .ki           = 0.1f,
    .kd           = 0.0f,
    .output_min   = 2.0f,
    .output_max   = 98.0f,
    .integral_max = 1000.0f,
    .i_term_max   = 80.0f,
    .update_ms    = 0,
};

/* ============================================================================
 * 局部状态
 * ============================================================================*/
static uint8_t s_inited = 0;

/* ============================================================================
 * 电流环 ISR 回调 — 精简版：不执行任何 PI 控制
 * 
 * 注意：此函数仍会被 I.c 注册为回调，但只更新一些 J-Scope 变量，
 * 不执行任何 PWM 占空比控制。这样电流采集功能完全保留，
 * 同时避免引用已删除的 COMM_RUNNER_* 枚举。
 * ============================================================================*/
static void curloop_isr(const stc_i_data_t *pData)
{
    (void)pData;

    /* 电流环已禁用：不执行任何 PI 控制，只更新 J-Scope 变量为安全值 */
    g_scope_i_ref  = 0.0f;
    g_scope_i_fb   = 0.0f;
    g_scope_i_fb_raw = 0.0f;
    g_scope_i_duty = 0.0f;
    g_scope_i_err  = 0.0f;
    g_scope_i_ol   = 0.0f;
    g_scope_i_dt_us = 0;
}

/* ============================================================================
 * API 实现
 * ============================================================================*/

/**
 * @brief 初始化电流环（精简版：只注册空回调，不执行 PI）
 */
void CurLoop_Init(void)
{
    if (s_inited) {
        return;
    }

    /* 注册一个空的 ISR 回调（电流采集仍然正常工作，但电流环 PI 被禁用） */
    I_RegisterCallback(curloop_isr);

    s_inited = 1;
    CURLOOP_DBG("Init done (current-loop PI DISABLED - only mode23/30)");
}

/**
 * @brief 设置电流参考值
 */
void CurLoop_SetRef(float ma)
{
    g_i_ref_ma = ma;
}

/**
 * @brief 获取电流参考值
 */
float CurLoop_GetRef(void)
{
    return g_i_ref_ma;
}

/**
 * @brief 设置外部电流参考模式（空操作）
 */
void CurLoop_SetExternalRef(bool enable)
{
    (void)enable;
    /* 电流环已禁用，此函数为空操作 */
}