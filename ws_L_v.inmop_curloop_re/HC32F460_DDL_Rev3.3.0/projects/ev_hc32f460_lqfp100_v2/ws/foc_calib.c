/**
 *******************************************************************************
 * @file  foc_calib.c
 * @brief FOC 相电流 DC 零偏自校准实现。
 *
 *        状态机：IDLE -> SKIP(丢弃使能瞬态) -> SAMPLING(累加均值) -> LOCKED
 *        采样对象是 I.c 已经过 boot 校准后的 i16xx_mA 值，因此本模块
 *        修正的是"温漂残余零偏"，与 I_Calibrate() 互补而非重复。
 *
 *        平均 4000 样本 @20kHz：宽带噪声按 sqrt(N) 压缩约 63 倍，
 *        0.3A rms 噪声下偏移估计误差 ~5mA rms，远小于待修正的
 *        数百 mA 温漂。
 *******************************************************************************
 */

#include "foc_calib.h"
#include "rtt_log.h"

/*******************************************************************************
 * Keil Watch 观测量（定义）
 ******************************************************************************/
volatile uint8_t g_calib_state     = 0u;    /* 0=idle 1=skip 2=sampling 3=locked */
volatile float   g_calib_iu_off_ma = 0.0f;
volatile float   g_calib_iv_off_ma = 0.0f;
volatile float   g_calib_iw_off_ma = 0.0f;

/*******************************************************************************
 * 模块内部状态
 ******************************************************************************/
static int32_t  s_sum_u = 0;
static int32_t  s_sum_v = 0;
static int32_t  s_sum_w = 0;
static uint32_t s_cnt   = 0;

/**
 * @brief 启动一轮校准
 */
void Foc_Calib_Start(void)
{
    s_sum_u = 0;
    s_sum_v = 0;
    s_sum_w = 0;
    s_cnt   = 0;

    g_calib_iu_off_ma = 0.0f;
    g_calib_iv_off_ma = 0.0f;
    g_calib_iw_off_ma = 0.0f;
    g_calib_state     = 1u;     /* SKIP */
}

/**
 * @brief 每 ISR 喂入一次采样（20kHz）
 * @return 1 = 已锁定，返回 0 = 仍在进行
 */
uint8_t Foc_Calib_Feed(const stc_i_data_t *pData)
{
    if (pData == NULL) {
        return (g_calib_state == 3u) ? 1u : 0u;
    }

    if (g_calib_state == 1u) {
        /* SKIP: 丢弃使能瞬态 */
        if (++s_cnt >= FOC_CALIB_SKIP_SAMPLES) {
            s_cnt   = 0;
            s_sum_u = 0;
            s_sum_v = 0;
            s_sum_w = 0;
            g_calib_state = 2u;     /* SAMPLING */
        }
        return 0u;
    }

    if (g_calib_state == 2u) {
        /* SAMPLING: 累加（int32 上限 4000*32767 ≈ 1.3e8，无溢出风险） */
        s_sum_u += pData->i16IU_mA;
        s_sum_v += pData->i16IV_mA;
        s_sum_w += pData->i16IW_mA;

        if (++s_cnt >= FOC_CALIB_AVG_SAMPLES) {
            g_calib_iu_off_ma = (float)s_sum_u / (float)s_cnt;
            g_calib_iv_off_ma = (float)s_sum_v / (float)s_cnt;
            g_calib_iw_off_ma = (float)s_sum_w / (float)s_cnt;
            g_calib_state     = 3u;     /* LOCKED */

            CALIB_DBG("Offset locked: U=%d mA, V=%d mA, W=%d mA",
                      (int)g_calib_iu_off_ma, (int)g_calib_iv_off_ma,
                      (int)g_calib_iw_off_ma);
        }
        return 0u;
    }

    /* IDLE / LOCKED */
    return (g_calib_state == 3u) ? 1u : 0u;
}

/**
 * @brief 1 = 偏移已锁定
 */
uint8_t Foc_Calib_IsLocked(void)
{
    return (g_calib_state == 3u) ? 1u : 0u;
}

/**
 * @brief 取三相残余零偏 (mA)，未锁定时返回 0
 */
void Foc_Calib_GetOffsetsMa(float *pU, float *pV, float *pW)
{
    if (pU != NULL) { *pU = g_calib_iu_off_ma; }
    if (pV != NULL) { *pV = g_calib_iv_off_ma; }
    if (pW != NULL) { *pW = g_calib_iw_off_ma; }
}
