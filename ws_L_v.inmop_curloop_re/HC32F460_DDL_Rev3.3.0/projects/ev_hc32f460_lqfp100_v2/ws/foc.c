/**
 *******************************************************************************
 * @file  foc.c
 * @brief FOC 门面 — 仅保留 Foc_Init 与 Foc_Isr 模式分发。
 *
 *        ISR frequency assumption:
 *          I.c uses the INMOP-style double trigger (SCMP0 @ PEAK + SCMP2 @
 *          VALLEY) -> 10 kHz PWM produces 20 kHz EOCB ISR. Foc_Isr therefore
 *          runs at FOC_ISR_HZ = 20000 (motor_config.h).
 *
 *        原聚焦在此文件的各运行模式与支撑逻辑已拆分：
 *          foc_core.c     共享状态/观测量、过流保护、PWM 启停、公共助手
 *          foc_openloop.c 模式21 开环 V/f
 *          foc_curloop.c  模式22 电流环（I-F 启动 + 同步交接 + RUN）
 *          foc_align.c    模式23 对齐校准
 *          foc_zizeng.c   模式30 ZIZENG 自增拖动
 *          foc_iq_pi.c    模式31 PI 电流环（ZIZENG 偏移 + 编码器角度）
 *          foc_scope.c    MotorScope RTT 遥测
 *
 *        ISR constraint: short, no blocking, no prints, no malloc.
 *******************************************************************************
 */

#include "foc.h"
#include "foc_math.h"

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
    I_RegisterFocCallback(Foc_Isr);
    s_bInited = 1u;
}

/*******************************************************************************
 * Foc_Isr - 20 kHz ISR callback (ADC1 EOCB, second callback slot)
 ******************************************************************************/
void Foc_Isr(const stc_i_data_t *pData)
{
    if (!g_foc_active) {
        return;
    }

    Foc_Core_ClampOpenLoopVolt();

    if (g_foc_mode == FOC_MODE_OPENLOOP) {
        Foc_OpenLoop_Step();
        return;
    }

    if (g_foc_mode == FOC_MODE_ALIGN) {
        Foc_Align_Step(pData);
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

    /* ========== IQ_PI 模式 (mode 31) ========== */
    if (g_iqpi_running) {
        Foc_IqPi_Step(pData);
        return;
    }
}
