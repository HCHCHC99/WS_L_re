/**
 *******************************************************************************
 * @file  foc.c
 * @brief FOC 门面 — 仅保留 Foc_Init 与 Foc_Isr 模式分发。
 *
 *        ISR frequency:
 *          由 I.h:98-105 推导 —— I_ACTIVE_SAMPLE_RATE_HZ = MOTOR_PWM_FREQ_HZ
 *          （当前采样模式 = ADC2_PWM_VALLEY，仅谷点单边沿触发），
 *          故 FOC_ISR_HZ = MOTOR_PWM_FREQ_HZ = 20 kHz（2026-09-23 起）。
 *        ⚠ 旧注释曾写"INMOP 式 PEAK+VALLEY 双边沿 → 10kHz PWM 产生 20kHz"，
 *          那是 I_SAMPLE_MODE == ADC2_CONT_DMA 的老分支，当前未启用。
 *
 *        原聚焦在此文件的各运行模式与支撑逻辑已拆分：
 *          foc_core.c     共享状态/观测量、过流保护、PWM 启停、公共助手
 *          foc_openloop.c 模式21 开环 V/f
 *          foc_curloop.c  模式22 电流环（I-F 启动 + 同步交接 + RUN）
 *          foc_align.c    模式23 对齐校准
 *          foc_cal.c      模式20 编码器零点校准（BETA 2s + ALPHA 2s -> 锁 offset）
 *          foc_dcl.c      模式27 功角闭环拖动（磁场 = 转子 + delta，delta 爬坡）
 *          foc_dci.c      模式28 功角参考电流闭环（复刻27 + foc_calib 零偏窗）
 *          foc_zizeng.c   模式30 ZIZENG 自增拖动
 *          foc_iq_pi.c    模式31 PI 电流环（ZIZENG 偏移 + 编码器角度）
 *          foc_lock_iq_pi.c 模式32 自锁偏移 + 自动交接 mode 31
 *          foc_scope.c    MotorScope RTT 遥测
 *
 *        ISR constraint: short, no blocking, no prints, no malloc.
 *******************************************************************************
 */

#include "foc.h"
#include "foc_math.h"
#include "timer6_timebase.h"   /* ISR 预算自测用（Timer6_Timebase_GetTimestamp） */

static uint8_t s_bInited = 0u;

/*******************************************************************************
 * Foc_Init - math LUT + bind PIs + register ISR callback (no output)
 ******************************************************************************/
void Foc_Init(void)
{
    if (s_bInited) {
        return;
    }

    Foc_Math_Init();
    Foc_CurLoop_InitPids();
    Foc_IqPi_InitPids();
    Foc_Dci_InitPids();
    Foc_Drun29_InitPids();
    Foc_Drun41_InitPids();
    Foc_Speed40_InitPids();
    Foc_Smo45_InitPids();
    I_RegisterFocCallback(Foc_Isr);
    s_bInited = 1u;
}

/*******************************************************************************
 * Foc_Isr - FOC ISR callback (ADC EOCB, second callback slot)
 *
 *   ISR 频率 = FOC_ISR_HZ = MOTOR_PWM_FREQ_HZ（当前 20 kHz，周期 50 µs）
 *
 * 【ISR 预算自测】（2026-09-23 加，用于确认 PWM 提升到 20kHz 后是否有余量）
 *   不占用 GPIO。用 Timer6 **原始计数器**测「两次入口的时间差」：
 *     正常：恒 ≒ 目标周期（50 µs）
 *     出现 >50 µs：本拍未完成即被下一拍打断（延迟 50µs → 75µs，裕度骤降）
 *
 *   ⚠ 用 GetCounter() 而非 GetTimestamp()：后者的累加只在主循环
 *     Encoder_Update() 里由 Timer6_Timebase_UpdateTimestamp() 推进，其更新
 *     周期远大于 50 µs，直接用它测会被量化到主循环节拍上而失效。
 *     GetCounter() 是自由运行的 16 位计数（PCLK0/64 = 3.125 MHz，
 *     0.32 µs 分辨率，21 ms 回绕），回绕由 uint16 相减天然处理。
 *
 *   两条设计约束（都是踩过的坑）：
 *   ① 测量**绝不能阻断控制流程** —— 故封装成返回值的辅助函数，
 *      ISR 里只 (void) 调用，不用 return 跳过主体。
 *   ② FOC 未激活时**必须作废基准** —— 模式切换期间 g_foc_active=0，
 *      ISR 空转、计数器继续涨，而测量被跳过；若保留基准，重新激活后第一拍
 *      会把「整个间隙」当成一个 ISR 间隔，把 max 永久污染成几千 µs（假超时）。
 *      为此对样本做 ±3% 合理性检验，带外样本整拍丢弃。
 *
 *   ⚠ 验证完可整块删除（本段 + 三个 g_foc_isr_period_* 定义 + 辅助函数）。
 ******************************************************************************/
volatile uint32_t g_foc_isr_period_min_us = 0xFFFFFFFFu;  /* 实测最小入口间隔 */
volatile uint32_t g_foc_isr_period_max_us = 0u;           /* 实测最大入口间隔 */
volatile uint32_t g_foc_isr_period_last_us = 0u;          /* 最近一次入口间隔 */

/* 测量状态（需被 reset 访问，故置于文件作用域） */
static uint16_t s_isr_period_prev_cnt = 0u;
static uint8_t  s_isr_period_init     = 0u;
static uint32_t s_isr_ticks_per_us    = 1u;

/* 作废基准：下次采样重新建立起点（用于 FOC 未激活时） */
static void Foc_IsrPeriodReset(void)
{
    s_isr_period_init = 0u;
}

/* 采样一次入口间隔。返回 1 = 已计入统计，0 = 本拍跳过（基准/带外） */
static uint8_t Foc_IsrPeriodSample(void)
{
    uint16_t cnt = (uint16_t)Timer6_Timebase_GetCounter();

    if (s_isr_period_init == 0u) {
        uint32_t f = Timer6_Timebase_GetFrequency();
        if (f != 0u) {
            s_isr_ticks_per_us = f / 1000000u;
        }
        if (s_isr_ticks_per_us == 0u) {
            s_isr_ticks_per_us = 1u;
        }
        s_isr_period_init  = 1u;
        s_isr_period_prev_cnt = cnt;
        return 0u;                       /* 首拍只建立基准 */
    }

    /* 16 位回绕由无符号相减自动处理 */
    {
        uint32_t ticks = (uint32_t)(uint16_t)(cnt - s_isr_period_prev_cnt);
        uint32_t dt_us = ticks / s_isr_ticks_per_us;
        uint32_t period_us = 1000000u / (uint32_t)FOC_ISR_HZ;
        uint32_t lo = period_us - period_us / 32u;   /* 约 −3% */
        uint32_t hi = period_us + period_us / 32u;   /* 约 +3% */

        s_isr_period_prev_cnt = cnt;

        /* 合理性检验：只统计落在预期周期 ±3% 内的间隔，滤掉：
         *   (a) 模式切换/重新激活留下的长间隙（假超时）
         *   (b) 计数器回绕（>21ms 的间隙回绕后可能折成任意值）
         *   (c) 被其它 ISR 抢占造成的偏差
         * ⚠ 真正的超时（75µs 量级）在带外会被丢弃而**不记录**——有意为之：
         *   本检验回答的是"是否每一拍都在周期内被服务"。
         *   若 max/min 长时间稳定在 ≈50，即证明无漏拍。 */
        if ((dt_us < lo) || (dt_us > hi)) {
            return 0u;
        }

        g_foc_isr_period_last_us = dt_us;
        if (dt_us > g_foc_isr_period_max_us) {
            g_foc_isr_period_max_us = dt_us;
        }
        if (dt_us < g_foc_isr_period_min_us) {
            g_foc_isr_period_min_us = dt_us;
        }
        return 1u;
    }
}

void Foc_Isr(const stc_i_data_t *pData)
{
    if (!g_foc_active) {
        Foc_IsrPeriodReset();   /* 见上方约束②：不作废基准会产生假超时 */
        return;
    }

    (void)Foc_IsrPeriodSample();   /* 观测用，绝不阻断控制流程 */

    Foc_Core_ClampOpenLoopVolt();

    if (g_foc_mode == FOC_MODE_OPENLOOP) {
        Foc_OpenLoop_Step();
        return;
    }

    if (g_foc_mode == FOC_MODE_ALIGN) {
        /* mode 20 校准优先（g_cal_running 托管），mode 25 次之
         * （g_calang_running 托管），mode 26 再次（g_olf_running 托管），
         * mode 27 再次之（g_dcl_running 托管），随后 mode 24/28/29/41
         * 各自使用独立 running 标志，否则 mode 23 对齐。 */
        if (g_cal_running) {
            Foc_Cal_Step(pData);
        } else if (g_calang_running) {
            Foc_CalAngle_Step(pData);
        } else if (g_olf_running) {
            Foc_Olf_Step(pData);
        } else if (g_dcl_running) {
            Foc_Dcl_Step(pData);
        } else if (g_dcal24_running) {
            Foc_Dcal24_Step(pData);
        } else if (g_dci_running) {
            Foc_Dci_Step(pData);
        } else if (g_drun29_running) {
            Foc_Drun29_Step(pData);
        } else if (g_drun41_running) {
            Foc_Drun41_Step(pData);
        } else if (g_speed40_running) {
            Foc_Speed40_Step(pData);
        } else if (g_smo45_running) {
            Foc_Smo45_Step(pData);
        } else {
            Foc_Align_Step(pData);
        }
        return;
    }

    if (g_foc_mode == FOC_MODE_CURLOOP) {
        Foc_CurLoop_Step(pData);
        return;
    }

    /* ========== ZIZENG 模式 (mode 30) ========== */
    if (g_zizeng_running) {
        Foc_Zizeng_Step(pData);
        return;
    }

    /* ========== LOCK_IQ_PI 模式 (mode 32)：自锁偏移 -> 交接 mode 31 ========== */
    if (g_lockiq_running) {
        Foc_LockIqPi_Step(pData);
        return;
    }

    /* ========== IQ_PI 模式 (mode 31) ========== */
    if (g_iqpi_running) {
        Foc_IqPi_Step(pData);
        return;
    }
}
