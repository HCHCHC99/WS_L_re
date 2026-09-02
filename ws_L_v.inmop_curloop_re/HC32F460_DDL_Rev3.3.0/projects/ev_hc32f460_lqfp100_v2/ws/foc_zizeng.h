/**
 *******************************************************************************
 * @file  foc_zizeng.h
 * @brief FOC 模式30 — 磁场角度自增拖动 (ZIZENG, comm_mode 30)。
 *
 *        开环电压矢量按 g_zizeng_freq_hz 自增拖动电机，同时直接读取
 *        TIMERA_1 硬件编码器计数（不依赖主循环 Encoder_Update()），
 *        稳定运行后自动采样并锁定"编码器电角度 - 合成角度"偏移量，
 *        用于补偿 g_foc_if_diff_rad 观测量。
 *******************************************************************************
 */

#ifndef __FOC_ZIZENG_H__
#define __FOC_ZIZENG_H__

#include <stdint.h>
#include "foc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 开环加压轴选择（编译期宏）
 *   1 = q 轴加压（Vd=0, Vq=V）—— 控制系 id≈0、iq≈V/Rs，符合 FOC 直觉
 *   0 = d 轴加压（Vd=V, Vq=0）—— 控制系 id≈V/Rs、iq≈0
 * 两种方式的电机拖动行为完全相同，仅控制系电流分解的观感不同；
 * 偏移补偿会自动吸收 ±90° 的基线差。
 * ==========================================================================*/
#define ZIZENG_VOLT_ON_Q_AXIS   1

/* ============================================================================
 * Keil Watch 可调变量 / 观测量
 * ==========================================================================*/
extern volatile float   g_zizeng_theta_rad;      /* 当前自增角度 (rad) */
extern volatile float   g_zizeng_freq_hz;        /* 自增频率 (Hz)，Keil Watch 可调 */
extern volatile float   g_zizeng_volt_v;         /* 自增电压幅值 (V)，Keil Watch 可调 */
extern volatile float   g_zizeng_du;             /* U 相 duty (%) */
extern volatile float   g_zizeng_dv;             /* V 相 duty (%) */
extern volatile float   g_zizeng_dw;             /* W 相 duty (%) */
extern volatile uint8_t g_zizeng_running;        /* 1 = 正在运行 */

/* 转子系电流观测（偏移基线锁定后有效，锁定前恒为 0）：
 * 用补偿后的转子电角度做 Park 变换，iq_rotor = 真实力矩电流，
 * id_rotor = 磁链方向分量（与磁场系 g_foc_id_ma/iq_ma 坐标系不同） */
extern volatile float   g_foc_id_rotor_ma;
extern volatile float   g_foc_iq_rotor_ma;

/* 静止两相坐标系电流观测（Clarke 输出，扣除 foc_calib 残余零偏，
 * 与控制系/转子系电流同源，A 单位，瞬时值无 EMA）：
 * iab_mag = sqrt(ialpha^2 + ibeta^2) = 电流矢量幅值（与坐标系选取无关） */
extern volatile float   g_foc_ialpha;
extern volatile float   g_foc_ibeta;
extern volatile float   g_foc_iab_mag;

/* 启动/停止自增模式 */
void Foc_StartZizeng(void);
void Foc_StopZizeng(void);

/* 模式30 单步运算（20 kHz ISR 中由 Foc_Isr 分发调用） */
void Foc_Zizeng_Step(const stc_i_data_t *pData);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_ZIZENG_H__ */
