/**
 *******************************************************************************
 * @file  foc_30_ramp.h
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
 *   - 偏移量 Foc_Ramp_GetOffsetRad()：编码器读数换算磁铁真实角度
 *     要扣的修正值
 *   - 拖动方向 Foc_Ramp_GetDragDir()：这次拖动编码器往哪边走。
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
 *   影响，详见 foc_32_lockiq.h。
 *
 * ISR 约束：短小、无阻塞、无打印、无 malloc。
 *
 * ==============================================================================
 *   模式速览卡   MODE 30   磁场角度自增拖动（锁定编码器-磁场偏移）
 *   本卡是本模式唯一事实源；代码内注释与本卡冲突时，以本卡为准
 * ==============================================================================
 *   入口   comm_mode = 30（其他模式下 SW1 短按也可进入）
 *   前置   无（自带 foc_calib 零矢量零偏窗约 210ms，之后直接拖动）
 *   结束   SW2 短按或 comm_mode = 0；不会自动停，ISR 内无过流判据
 *   标记   [!] 易错点   (*) 主判据   [可调] Watch 可写   [只读] 仅观察
 * ------------------------------------------------------------------------------
 *   [可调] Watch 变量 -- 改值即时生效，复位后回宏默认
 * ------------------------------------------------------------------------------
 *       变量                        当前值  单位     含义
 *       --------------------------- ------- -------- ---------------------------
 *       g_zizeng_freq_hz            0.5     Hz       拖动电频率（SW1 加 0.5Hz）
 *       g_zizeng_volt_v             0.7     V        拖动电压幅值（力道）
 *   (*) 锁定窗 100ms（FOC30_OFFSET_SAMPLES=2000 @20kHz），窗内位移要 >=
 *       FOC30_DRAGDIR_MIN_CNTS=81 counts 才判得出方向：0.5Hz 只有约 20
 *       counts，3Hz 约 123 counts（源码注释值）。
 *   [!] foc_30_ramp.h 顶部速览卡写 3.0Hz/0.6V，代码初值实为
 *       0.5Hz/0.7V（foc_30_ramp.c:32）；Start 只在值 <= 0 时才改成
 *       5.0Hz/0.6V。以代码为准。
 *   [!] 运行中改频率只改拖动速度，不会自动重锁偏移；要重锁必须退到 comm_mode = 0
 *       再进 30（Foc_Ramp_Start 会复位锁定状态）。
 * ------------------------------------------------------------------------------
 *   [只读] 关键观察变量
 * ------------------------------------------------------------------------------
 *       变量                           含义
 *       ------------------------------ -----------------------------------------
 *   (*) g_zizeng_drag_dir              拖动方向 +1/-1；0 = 锁定失败（ch13）
 *       g_foc_if_diff_rad              编码器角-磁场角 (rad)；锁定后跳到 0 附近
 *       g_zizeng_theta_rad             当前磁场电角度 (rad)，应匀速绕圈（ch7）
 *       g_zizeng_running               1 = 正在拖动（ch14），停机后清 0
 *       g_zizeng_du                    U 相占空比 (%)，随磁场角摆动（ch12）
 *       g_foc_id_rotor_ma              转子系 id (mA)；锁定前恒 0（ch9）
 *       g_foc_iq_rotor_ma              转子系 iq (mA)；锁定后即力矩电流（ch10）
 *   (*) 产物接口：Foc_Ramp_GetOffsetRad() 与 Foc_Ramp_GetDragDir()（mode 31
 *       启动时自动取用）；mode 32 可用 Foc_Ramp_SetOffsetRad() /
 *       Foc_Ramp_SetDragDir() 覆盖。
 * ------------------------------------------------------------------------------
 *   VOFA 通道 -- 15ch，填充见 Foc_Ramp_VofaFill
 * ------------------------------------------------------------------------------
 *       通道  含义                   单位     备注
 *       ----- ---------------------- -------- ----------------------------------
 *       ch0   U 相电流               A
 *       ch1   V 相电流               A
 *       ch2   W 相电流               A
 *       ch3   静止系 ialpha          A        本模块刷新，mode 40/41 复用
 *       ch4   静止系 ibeta           A
 *   (*) ch5   编码器角-磁场角        rad      锁定后跳到 0 附近；不收敛 = 打滑
 *       ch6   自增频率               Hz       跟 g_zizeng_freq_hz（SW1 可加）
 *       ch7   磁场电角度             rad
 *       ch8   电压幅值               V
 *       ch9   转子系 id              A        锁定前恒 0
 *       ch10  转子系 iq              A        锁定后 = 力矩电流
 *       ch11  静止系电流幅值         A
 *       ch12  U 相占空比             %
 *   (*) ch13  拖动方向               -        +1/-1；0 = 没测到 = 锁定失败
 *       ch14  运行标志               -
 * ------------------------------------------------------------------------------
 *   判据与坑
 * ------------------------------------------------------------------------------
 *   (*) 主判据：启动约 1.3s 后（校准窗约 210ms + FOC30_OFFSET_WAIT_MS=1000 等待
 *       + FOC30_OFFSET_SAMPLES=2000 采 100ms）RTT 打出 Offset LOCKED: .. deg,
 *       samples=2000, drag_dir=+1/-1；同时 ch13 由 0 变 +/-1，ch5 跳到 0 附近。
 *   [!] ch13 一直是 0 有两种原因：转子真没动（失步 / 电压太小），或 100ms
 *       窗内位移没到 81 counts（默认 0.5Hz 只有约 20 counts，SW1 按 4 下提到
 *       2.5Hz 再看）。
 *   [!] ch13 = 0 的连带后果：mode 31 启动时拿不到方向基准，它的 180deg
 *       框架自检整段跳过，转向错了也不会被纠正。
 *   [!] 锁定是在拖动中做的：转子打滑/失步时锁出的偏移本身就是错的，mode 31
 *       启动方向会随机（foc_30_ramp.h 已注明该缺陷）-> 优先用 mode 32
 *       静止吸附法锁偏移。
 *   [!] 本模式 ISR 内没有过流判据（其他 FOC 模式都调
 *       Foc_Core_OverCurrent），别随手加大 g_zizeng_volt_v；退出只能靠 SW2 /
 *       comm_mode = 0（Foc_Ramp_Stop）。
 *   [!] VOFA+ 通道数必须同步配成 15，否则整帧错位；改通道数改 Foc_Ramp_VofaFill
 *       末尾的 return。
 * ==============================================================================
 */

#ifndef __FOC_30_RAMP_H__
#define __FOC_30_RAMP_H__

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
    #define FOC30_DBG(fmt, ...)   MAIN_D("[ZIZENG] " fmt, ##__VA_ARGS__)
#else
    #define FOC30_DBG(fmt, ...)   ((void)0)
#endif

/* ============================================================================
 * 开环加压轴选择（编译期宏）
 *   1 = q 轴加压（Vd=0, Vq=V）—— 控制系 id≈0、iq≈V/Rs，符合 FOC 直觉
 *   0 = d 轴加压（Vd=V, Vq=0）—— 控制系 id≈V/Rs、iq≈0
 * 两种方式的电机拖动行为完全相同，仅控制系电流分解的观感不同；
 * 偏移补偿会自动吸收 ±90° 的基线差。
 * ==========================================================================*/
#define FOC30_VOLT_ON_Q_AXIS   1

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

/* ★ 静止两相系电流观测量 g_foc_ialpha / g_foc_ibeta / g_foc_iab_mag
 *   已于 2026-09-23 迁至 **foc_core.h**（它们被 mode 0/29/31/32/40/41 共用，
 *   放在本模式头文件里属归属错误）。此处不再声明 —— 需要用的模块请包含
 *   foc_core.h（几乎所有 FOC 模块都已包含）。 */

/* 启动/停止自增模式 */
void Foc_Ramp_Start(void);
void Foc_Ramp_Stop(void);

/* 取 ZIZENG 锁定的偏移基线 (rad)。返回 1 = 已锁定（跨 stop 保留），
 * 供 mode 31 (IQ_PI) 启动时做编码器电角度绝对化 */
uint8_t Foc_Ramp_GetOffsetRad(float *out_rad);

/* 取偏移采样窗口内实测的拖动方向（编码器计数位移符号 +1/-1，0=未测得）。
 * mode 31 用它比对运行转向，检测 180° 框架误差 */
int8_t Foc_Ramp_GetDragDir(void);

/* --- 注入接口（mode 32 自锁偏移用，覆盖锁定值后 mode 31 无感取用） ---
 * SetOffsetRad: 写入偏移基线并置有效标志（等价 mode 30 锁定完成的状态）
 * SetDragDir  : 写入方向基准（mode 32 用本征推导值 FOC_ENC_DIR 注入） */
void Foc_Ramp_SetOffsetRad(float rad);
void Foc_Ramp_SetDragDir(int8_t dir);

/* 模式30 单步运算（20 kHz ISR 中由 Foc_Isr 分发调用） */
void Foc_Ramp_Step(const stc_i_data_t *pData);

/* 模式自持 VOFA（15ch，通道含义见顶部速览卡） */
int  Foc_Ramp_VofaFill(int32_t *cur);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_30_RAMP_H__ */
