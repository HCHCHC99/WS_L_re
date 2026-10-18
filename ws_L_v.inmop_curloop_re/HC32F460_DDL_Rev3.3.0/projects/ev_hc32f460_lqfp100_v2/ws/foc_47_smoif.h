/**
 *******************************************************************************
 * @file  foc_47_smoif.h
 * @brief FOC mode 47 - 开环强拖 + SMO/PLL 纯程序自检（无编码器 / 无 PI）。
 *
 * 动机：mode 45/60 的 SMO/PLL 旁观链依附编码器闭环（有感拖动 + 编码器裁判），
 * 没有编码器时无法自证观测器链路健康。mode 47 用固件自合成的旋转磁场开环
 * 强拖电机（V/f 拖动，无电流环/速度环/编码器），SMO+PLL 全程旁观，
 * 以"程序自己给出的给定值"为参照完成自检：
 *   给定角度 θ_syn（Step 内积分 2π·f_cmd·Ts）、给定电角速度 ω_syn=2π·f_cmd、
 *   给定理论反电动势 ω_syn·ψf（ψf 取 SMO 模型值）。
 *
 * 四条判据（全部只依赖程序给定值，不依赖编码器）：
 *   J1 转速：g_pll_omega_out_rpm ≈ f_cmd×60/PP（±g_smo47_jdg_omega_err_pct%）
 *   J2 幅值：|e_hat| ≈ ω_syn·ψf —— SMO 内建 C1（e_ratio∈[0.85,1.08]）
 *   J3 相位：PLL 角 − θ_syn 滤波值落在已知滞后窗（comp_en=0 默认
 *            [g_smo47_jdg_theta_min_deg, g_smo47_jdg_theta_max_deg]=[-60,+5]；
 *            置 g_pll_comp_en=1 补偿后应收窄到 ≈0，可手动收紧窗）
 *   J4 SMO：g_smo_emf_ok（内建判定，参照同为 θ_syn/ω_syn，含 1000ms hold）
 * 聚合结果 g_smo47_jdg_ok / g_smo47_jdg_fail 由 foc_obs.c 主循环维护
 * （各分量自带滤波/hold，聚合层不再另加 hold）。
 *
 * 与 mode 45 的关系：45 = 有感闭环 + SMO/PLL 旁观（编码器裁判）；
 * 47 = 开环强拖 + SMO/PLL 旁观（程序给定值裁判）。观测器内核同一套
 * （foc_smo.c / foc_pll.c），模型参数（k/ψf/lpf_alpha/ω_n…）通用。
 *
 * ==============================================================================
 *   模式速览卡   MODE 47   开环强拖 + SMO/PLL 纯程序自检
 *   本卡是本模式唯一事实源；代码内注释与本卡冲突时，以本卡为准
 * ==============================================================================
 *   入口   comm_mode = 47
 *   前置   无（不依赖 mode 24 / 编码器；启动自带 Foc_Calib 零偏窗：三相
 *          零矢量 200 拍 skip + 4000 拍平均 ≈ 210ms，需电机端零电流窗口：
 *          与 mode 24/30 同款要求——测零偏期间相线里不能有电流通路差异）
 *   流程   CALIB(零矢量采零偏 ~210ms) -> ALIGN(θ_syn=0 直流吸附 800ms)
 *          -> RAMP(f 从 0 爬到 g_smo47_f_run_hz，限速 g_smo47_f_ramp_hz_s)
 *          -> RUN(匀速稳态，判定窗口打开)
 *   结束   持续运行不自动停；过流自动停并回 mode 0（state=5/evt=1）；手动停
 *          comm_mode = 0
 *   标记   [!] 易错点   (*) 主判据   [可调] Watch 可写   [只读] 仅观察
 * ------------------------------------------------------------------------------
 *   [可调] Watch 变量 -- 改值即时生效，复位后回宏默认
 * ------------------------------------------------------------------------------
 *       变量                        当前值  单位     含义
 *       --------------------------- ------- -------- ---------------------------
 *       g_smo47_f_run_hz            250     Hz       目标电频率（=1500rpm@PP=10，
 *                                                    SMO 可用窗 1300~3400rpm 中心）
 *       g_smo47_f_ramp_hz_s         50      Hz/s     爬频速率；失步就先降它
 *       g_smo47_v0_v                0.5     V        V/f 零频电压（零速吸附）
 *       g_smo47_kvf_v_khz           5.28    V/kHz    V/f 斜率 = 2π×ψf（由 SMO
 *                                                    模型 ψf=0.84mWb 推得）
 *       g_smo47_v_max_v             3.5     V        输出电压幅值上限
 *       g_smo47_jdg_omega_err_pct   15      %        J1 转速判据容差
 *       g_smo47_jdg_theta_min_deg   -60     deg      J3 相位窗下限（comp_en=0）
 *       g_smo47_jdg_theta_max_deg   +5      deg      J3 相位窗上限（comp_en=0）
 *   [!] 置 g_pll_comp_en=1 后 PLL 补偿角生效，J3 应收窄到 ≈0±锁点偏置，
 *       此时把上面两个窗改紧（如 [-20,+20]）才算严格验收。
 * ------------------------------------------------------------------------------
 *   [只读] 关键观察变量
 * ------------------------------------------------------------------------------
 *       变量                           含义
 *       ------------------------------ -----------------------------------------
 *       g_smo47_state                  0 空闲 / 1 采零偏 / 2 吸附 / 3 爬频 /
 *                                      4 稳态 / 5 过流
 *       g_smo47_evt                    1 过流 / 2 零偏锁定 / 3 达速
 *       g_smo47_f_cmd_hz               当前指令电频率（RAMP 中持续变化）
 *       g_smo47_v_drag_v               当前输出电压幅值（V/f 曲线值）
 *       g_smo47_theta_syn_deg          拖动磁场角 (deg, [0,360))
 *       g_smo47_omega_ref_rpm          给定机械转速 = f_cmd×60/PP（J1 参照）
 *       g_smo47_id_ma / iq_ma          拖动系电流（θ_syn 系，非转子系）：
 *                                      id≈磁化电流，iq≈拖动转矩电流
 *   (*) g_smo47_steady                 1=达速后已稳定 2s（J1/J3 开始评估）
 *   (*) g_smo47_jdg_ok                 聚合判定：1=四条判据全部通过
 *       g_smo47_jdg_fail               失败掩码 bit0 未稳态 / bit1 J1 转速 /
 *                                      bit2 J3 相位 / bit3 J4 SMO emf
 *       g_smo47_du/dv/dw               三相占空比观测量 (%)
 *       （SMO/PLL 诊断量沿用 foc_smo.h / foc_pll.h 全套：
 *         g_smo_diag_* / g_smo_jdg_* / g_pll_* —— 参照系已换成本模式的
 *         θ_syn/ω_syn，语义见两头文件，不再重复列表）
 * ------------------------------------------------------------------------------
 *   VOFA 通道 -- 固定 16ch，填充见 Foc_Smo47_VofaFill
 * ------------------------------------------------------------------------------
 *       通道  含义                   单位     备注
 *       ----- ---------------------- -------- ----------------------------------
 *       ch0   指令电频率             Hz       RAMP 段可见爬坡
 *       ch1   拖动电压幅值           V        V/f 曲线
 *       ch2   拖动磁场角             deg      θ_syn
 *       ch3   PLL 转子角估计         deg      g_pll_theta_rotor_deg
 *       ch4   PLL 角差原始           deg      θ_park − θ_syn（含全部已知滞后）
 *   (*) ch5   PLL 角差滤波           deg      主判据 J3：comp_en=0 应为稳定
 *                                      负值常数；comp_en=1 应≈0。缓慢漂移/摆
 *                                      动=失步前兆
 *       ch6   PLL 转速估计原始       rpm
 *       ch7   PLL 转速估计低通       rpm      J1 判据用这条
 *   (*) ch8   给定转速               rpm      程序给定值（J1 参照）
 *   (*) ch9   转速差 = ch7 − ch8     rpm      主判据 J1：稳态应≈0（±15%内）
 *       ch10  |e_hat|                V        SMO 实测反电动势幅值
 *   (*) ch11  理论幅值 ω_syn·ψf      V        主判据 J2 参照（给定频率推得）
 *       ch12  e_on_q 投影            V       稳态应为常数（无电频率纹波）
 *       ch13  e_on_d 投影            V       稳态应为常数
 *   (*) ch14  聚合判定               0/1      J1-J4 全过=1
 *       ch15  iq（拖动系）           A        拖动转矩电流；摆动加大=接近失步
 * ------------------------------------------------------------------------------
 *   判据与坑
 * ------------------------------------------------------------------------------
 *   (*) 验收流程：进 mode 47 -> 等 ch0 爬到 250Hz 且 ch14=1 -> 看 ch5/ch9
 *       平直、ch10/ch11 重合。RTT 有 5ms 节流 [SMO47] 行与判定 OK/FAIL 事件行
 *       （foc_obs.c），失败原因读 g_smo47_jdg_fail 位。
 *   [!] ALIGN 吸附电流 ≈ V0/R = 0.5V/0.1Ω ≈ 5A，默认 800ms 与 mode 24 同量级，
 *       长时间停在 ALIGN/反复重进会发热；带载会更大，测试请摘负载。
 *   [!] 失步签名：ch9 持续偏离 0、ch5 缓慢漂移或周期性摆动、ch15 摆动加大。
 *       处置：降 g_smo47_f_ramp_hz_s、或加 g_smo47_v0_v（电压富余=同步裕度）。
 *       V0 过大的反面是电流变大（V−e 全压在 R 上）。
 *   [!] 本模式读不到真实转子角——这是特性（纯程序自检，裁判就是给定值）。
 *       要编码器当裁判的版本用 mode 45。也正因此，47 不写任何编码器相关量，
 *       g_smo45_rotor_deg / encoder.h 与本模式无关。
 *   [!] SMO 固定 k 的可用验证窗 ≈ 1300~3400rpm（k/e_peak≈1.5~3.5）。默认
 *       1500rpm 在窗内；想拖更高/更低转速先看 foc_smo.h 的窗口推导，别盲改。
 *   [!] J2/J3 的 SMO 内建判定在低转速（<900rpm 给定）不评估——爬频段失败
 *       掩码 bit0/bit3 置位属预期，判定只认 g_smo47_steady=1 之后。
 *   [!] 采样率改档（MOTOR_PWM_FREQ_HZ）时：爬频/积分全部按 FOC_ISR_HZ 推导
 *       已自适应；SMO/PLL 的 lpf_alpha 须按 1/f 同步缩放（见 mode 45 卡同款警告）。
 * ==============================================================================
 */

#ifndef __FOC_47_SMOIF_H__
#define __FOC_47_SMOIF_H__

#include <stdint.h>
#include "foc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FOC47_DBG   1
#if FOC47_DBG
#define FOC47_LOG(fmt, ...)  MAIN_D("[SMO47] " fmt, ##__VA_ARGS__)
#else
#define FOC47_LOG(fmt, ...)  ((void)0)
#endif

/* 状态机 / 事件（见速览卡） */
#define FOC47_STEP_IDLE      0u
#define FOC47_STEP_CALIB     1u
#define FOC47_STEP_ALIGN     2u
#define FOC47_STEP_RAMP      3u
#define FOC47_STEP_RUN       4u
#define FOC47_STEP_FAULT_OC  5u

#define FOC47_EVT_OC         1u
#define FOC47_EVT_CALIB_DONE 2u
#define FOC47_EVT_RAMP_DONE  3u

/* V/f 强拖参数默认值（Watch 运行时可改，复位回这里） */
#define FOC47_F_RUN_HZ          250.0f  /* 目标电频率：1500rpm@PP=10 */
#define FOC47_F_RAMP_HZ_PER_S   50.0f   /* 爬频速率 */
#define FOC47_V0_V              0.5f    /* 零频电压（零速吸附） */
#define FOC47_KVF_V_PER_KHZ     5.28f   /* 2π×ψf：ψf=0.84mWb（=SMO 模型值） */
#define FOC47_V_MAX_V           3.5f    /* 电压幅值上限 */

/* 时序参数 */
#define FOC47_ALIGN_MS          800u    /* 直流吸附时长 */
#define FOC47_STEADY_MS         2000u   /* 达速后判定稳定窗 */

/* 聚合判定参数（J1/J3；J2/J4 复用 foc_smo.h 内建窗） */
#define FOC47_JDG_OMEGA_ERR_PCT 15.0f
#define FOC47_JDG_THETA_MIN_DEG (-60.0f)
#define FOC47_JDG_THETA_MAX_DEG (5.0f)

extern volatile uint8_t  g_smo47_running;
extern volatile uint8_t  g_smo47_state;
extern volatile uint8_t  g_smo47_evt;

/* Watch 可调 */
extern volatile float    g_smo47_f_run_hz;       /* 目标电频率 (Hz) */
extern volatile float    g_smo47_f_ramp_hz_s;    /* 爬频速率 (Hz/s) */
extern volatile float    g_smo47_v0_v;           /* V/f 零频电压 (V) */
extern volatile float    g_smo47_kvf_v_khz;      /* V/f 斜率 (V/kHz) */
extern volatile float    g_smo47_v_max_v;        /* 电压幅值上限 (V) */
extern volatile float    g_smo47_jdg_omega_err_pct;  /* J1 容差 (%) */
extern volatile float    g_smo47_jdg_theta_min_deg;  /* J3 窗下限 (deg) */
extern volatile float    g_smo47_jdg_theta_max_deg;  /* J3 窗上限 (deg) */

/* 只读观察量 */
extern volatile float    g_smo47_f_cmd_hz;       /* 当前指令电频率 (Hz) */
extern volatile float    g_smo47_v_drag_v;       /* 当前电压幅值 (V) */
extern volatile float    g_smo47_theta_syn_deg;  /* 拖动磁场角 (deg,[0,360)) */
extern volatile float    g_smo47_omega_ref_rpm;  /* 给定机械转速 (rpm) */
extern volatile float    g_smo47_id_ma;          /* 拖动系 id (mA) */
extern volatile float    g_smo47_iq_ma;          /* 拖动系 iq (mA) */
extern volatile float    g_smo47_du;
extern volatile float    g_smo47_dv;
extern volatile float    g_smo47_dw;
extern volatile uint8_t  g_smo47_steady;         /* 1=达速后稳定窗已过 */
extern volatile uint8_t  g_smo47_jdg_ok;         /* 聚合判定（foc_obs.c 维护） */
extern volatile uint8_t  g_smo47_jdg_fail;       /* 失败掩码（同上） */

void Foc_Smo47_Start(void);
void Foc_Smo47_Stop(void);
void Foc_Smo47_Step(const stc_i_data_t *pData);
int  Foc_Smo47_VofaFill(int32_t *cur);   /* 固定 16ch，见顶部速览卡 */

#ifdef __cplusplus
}
#endif

#endif /* __FOC_47_SMOIF_H__ */
