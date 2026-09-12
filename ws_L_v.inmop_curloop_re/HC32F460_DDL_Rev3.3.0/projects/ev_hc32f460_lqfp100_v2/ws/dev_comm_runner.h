#ifndef __DEV_COMM_RUNNER_H__
#define __DEV_COMM_RUNNER_H__

#include "hall_sensor_3ch.h"
#include "../Utils/dev_pid.h"
#include <stdint.h>

/*=============================================================================
 * Debug macros
 *=============================================================================*/
/* 1 = RTT prints on, 0 = off */
#define COMM_RUNNER_DBG   0
#if COMM_RUNNER_DBG
    #define RUNNER_DBG(fmt, ...)   MAIN_D("[CommRunner] " fmt, ##__VA_ARGS__)
#else
    #define RUNNER_DBG(fmt, ...)   ((void)0)
#endif

/*=============================================================================
 * CommRunner: 精简版 — 仅保留 STOP / CAL / FOC_ALIGN / ZIZENG / IQ_PI / LOCK_IQ_PI
 *=============================================================================*/

typedef enum {
    COMM_RUNNER_STOP       = 0,  /* 停止 */
    COMM_RUNNER_CAL        = 20, /* 编码器零点校准（BETA 2s + ALPHA 2s -> 锁 offset 自动回 0） */
    COMM_RUNNER_CAL_ANGLE  = 25, /* 手动角度吸附（自动校准 -> 刹车等输入 -> 吸附2s+校验500ms） */
    COMM_RUNNER_OLF        = 26, /* 开环 VF 负载角实验（校准 -> 磁场自增拖动，SW1/Watch 调频） */
    COMM_RUNNER_DCL        = 27, /* 功角闭环拖动（校准 -> 磁场=转子+delta，delta 爬坡可调） */
    COMM_RUNNER_FOC_ALIGN  = 23, /* FOC 对齐校准 */
    COMM_RUNNER_ZIZENG     = 30, /* 磁场角度自增拖动模式 */
    COMM_RUNNER_IQ_PI      = 31, /* PI 电流环模式（需先跑 mode 30 锁偏移） */
    COMM_RUNNER_LOCK_IQ_PI = 32, /* 自锁偏移 + 自动交接 mode 31（不依赖 mode 30） */
} comm_runner_mode_t;

/* 配置结构（精简版） */
typedef struct {
    uint32_t pwm_freq_hz;
    float    default_duty_pct;
    void (*on_init_done)(void);
    pid_config_t *pid_cfg;   /* 保留但不再使用 */
} comm_runner_config_t;

/*=============================================================================
 * API
 *=============================================================================*/

void CommRunner_Init(const comm_runner_config_t *cfg);
void CommRunner_SetMode(comm_runner_mode_t mode);
comm_runner_mode_t CommRunner_GetMode(void);
uint32_t CommRunner_GetPwmFreqHz(void);
void CommRunner_SetDuty(float duty_pct);
float CommRunner_GetDuty(void);
void CommRunner_Update(void);

/* 以下函数保留桩函数，供外部调用但不做实际工作 */
float CommRunner_GetRPM(void);
uint8_t CommRunner_IsRunning(void);
uint8_t CommRunner_IsStalled(void);
uint8_t CommRunner_CurLoopActive(void);
void CommRunner_SetTargetRPM(float rpm);
float CommRunner_GetTargetRPM(void);

/* 删除所有 Calibration 相关声明 */
/* 删除所有 PID/JScope 相关 extern 变量 */

#endif /* __DEV_COMM_RUNNER_H__ */
