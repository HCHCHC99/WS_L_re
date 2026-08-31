#ifndef __DEV_COMM_RUNNER_H__
#define __DEV_COMM_RUNNER_H__

#include "hall_sensor_3ch.h"
#include "../Utils/dev_pid.h"
#include <stdint.h>

/*=============================================================================
 * CommRunner: 精简版 — 仅保留 STOP / FOC_ALIGN / ZIZENG
 *=============================================================================*/

typedef enum {
    COMM_RUNNER_STOP       = 0,  /* 停止 */
    COMM_RUNNER_FOC_ALIGN  = 23, /* FOC 对齐校准 */
    COMM_RUNNER_ZIZENG     = 30, /* 磁场角度自增拖动模式 */
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
