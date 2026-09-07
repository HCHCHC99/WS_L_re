/**
 *******************************************************************************
 * @file  foc_zizeng.h
 * @brief FOC 模式30 — 磁场角度自增拖动 (ZIZENG, comm_mode 30)。
 *
 * ============================================================================
 * 【傻瓜式讲解：这个模式是干什么的】
 *
 * 一句话：让磁场匀速旋转，像用一根看不见的"磁力杆"拖着电机慢慢转，
 *         一边转一边测量并记住"编码器读数"和"磁场角度"的差值。这个
 *         差值（偏移量）是 mode 31 闭环必需的"对表"结果。
 *
 * 为什么需要它？
 *   编码器装在电机轴上，只能告诉你"轴转过了多少"，但它不知道电机里面
 *   磁铁的 N 极朝哪。FOC 必须知道磁铁位置才能正确发力。所以先"对表"：
 *   把磁场转到已知角度，看编码器这时读多少，差值记下来，以后就能从
 *   编码器读数反推磁铁真实位置。
 *
 * 工作过程（按时间顺序，全程自动）：
 *   1. 启动后先"不出力"约 210ms：三相输出 50/50/50（绕组无电流），
 *      趁机测电流传感器自身的零点误差（foc_calib 的活）。
 *   2. 开始拖动：磁场角度 theta 以 g_zizeng_freq_hz（默认 3Hz）匀速
 *      前进，电压幅值 g_zizeng_volt_v（默认 0.6V）。电机被磁力平稳
 *      拖着转，转子始终略微滞后磁场一点（负载角 delta，几度，正常）。
 *   3. 启动 1 秒后开始"锁定"：连续采 2000 个"编码器角度 - 磁场角度"
 *      差值求平均，平均值即偏移量。成功后打印：
 *        "Offset LOCKED: xxx deg ... drag_dir=±1"
 *      同时记录这段时间编码器往哪边动了（drag_dir：+1/-1）。
 *   4. 锁定后继续保持拖动，直到停机。偏移量跨停止保留，mode 31
 *      启动时自动取用。
 *
 * 它的"产物"（mode 31 要用的两样东西）：
 *   - 偏移量 Foc_Zizeng_GetOffsetRad()：编码器读数换算磁铁真实角度
 *     要扣的修正值
 *   - 拖动方向 Foc_Zizeng_GetDragDir()：这次拖动编码器往哪边走。
 *     mode 31 拿它当"你应该往这边转"的参考；转反了说明角度框架差了
 *     180°，mode 31 会自动翻转修正
 *
 * Watch 常用变量：
 *   g_zizeng_freq_hz / g_zizeng_volt_v : 拖动速度 / 力道，运行中可改
 *   g_zizeng_theta_rad                 : 当前磁场角度
 *   g_zizeng_drag_dir                  : 拖动方向 +1/-1，0=没测到
 *   g_foc_if_diff_rad                  : 编码器角与磁场角之差，锁定后≈0
 *   g_foc_id_rotor_ma / g_foc_iq_rotor_ma : 转子系电流观测（锁定后有效）
 *
 * 限制（为什么又做了 mode 32）：
 *   锁定发生在"拖动中"，若转子打滑、没跟上磁场，锁出的偏移就是错的，
 *   mode 31 启动方向就会随机。mode 32 用静止吸附法锁偏移，不受打滑
 *   影响，详见 foc_lock_iq_pi.h。
 *
 * ISR 约束：短小、无阻塞、无打印、无 malloc。
 *******************************************************************************
 */

#ifndef __FOC_ZIZENG_H__
#define __FOC_ZIZENG_H__

#include <stdint.h>
#include "foc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * Debug macros
 *=============================================================================*/
/* 1 = RTT prints on, 0 = off */
#define FOC_ZIZENG_DBG   1
#if FOC_ZIZENG_DBG
    #define ZIZENG_DBG(fmt, ...)   MAIN_D("[ZIZENG] " fmt, ##__VA_ARGS__)
#else
    #define ZIZENG_DBG(fmt, ...)   ((void)0)
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
extern volatile int8_t  g_zizeng_drag_dir;       /* 拖动方向: +1/-1 = 偏移采样窗口内
                                                    编码器计数位移符号, 0 = 未测得 */

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

/* 取 ZIZENG 锁定的偏移基线 (rad)。返回 1 = 已锁定（跨 stop 保留），
 * 供 mode 31 (IQ_PI) 启动时做编码器电角度绝对化 */
uint8_t Foc_Zizeng_GetOffsetRad(float *out_rad);

/* 取偏移采样窗口内实测的拖动方向（编码器计数位移符号 +1/-1，0=未测得）。
 * mode 31 用它比对运行转向，检测 180° 框架误差 */
int8_t Foc_Zizeng_GetDragDir(void);

/* --- 注入接口（mode 32 自锁偏移用，覆盖锁定值后 mode 31 无感取用） ---
 * SetOffsetRad: 写入偏移基线并置有效标志（等价 mode 30 锁定完成的状态）
 * SetDragDir  : 写入方向基准（mode 32 用本征推导值 FOC_ENC_DIR 注入） */
void Foc_Zizeng_SetOffsetRad(float rad);
void Foc_Zizeng_SetDragDir(int8_t dir);

/* 模式30 单步运算（20 kHz ISR 中由 Foc_Isr 分发调用） */
void Foc_Zizeng_Step(const stc_i_data_t *pData);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_ZIZENG_H__ */
