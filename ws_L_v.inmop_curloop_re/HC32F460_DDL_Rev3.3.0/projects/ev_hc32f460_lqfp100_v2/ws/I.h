/**
 *******************************************************************************
 * @file  I.h
 * @brief Current sensing module �? 3-phase current via ADC1 SEQ_B interrupt mode
 *
 *        PA5 = ADC1_CH5 = IU  (current sensor U)
 *        PA6 = ADC1_CH6 = IV  (current sensor V)
 *        PA7 = ADC1_CH7 = IW  (current sensor W)
 *
 *        Sensor formula: VOUT = 1650 + IP(A) × 264 (mV)
 *          ±5A range, 3.3V / 12-bit ADC, zero = 2048 raw
 *
 *        Trigger chain:
 *          TMR4_3 SCMP0 (PWM peak) → AOS_ADC1_0 (EVT0) → ADC1_SEQ_B → EOCB ISR
 *
 *        ADC1 layout:
 *          SEQ_A (CH0-CH3): BEMF, TMR4_3 SCMP0 �? AOS_ADC1_0 �? DMA
 *          SEQ_B (CH5-CH7): Current, TMR4_3 SCMP0 → AOS_ADC1_0 (EVT0, shared with BEMF) → ISR
 *******************************************************************************
 */

#ifndef __I_H__
#define __I_H__

#include "main.h"
#include "Hardware.h"
#include "motor_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/

/* Debug switch: 1 = RTT prints on, 0 = off */
#define DEBUG_I_WS   0
#if DEBUG_I_WS
    #define I_DEBUG(fmt, ...)    MAIN_D("[I] " fmt, ##__VA_ARGS__)
#else
    #define I_DEBUG(fmt, ...)    ((void)0)
#endif

/* ============================================================================
 * Current sampling source
 *   ADC2_CONT_DMA       : legacy INMOP-style, free-running ADC2 + DMA.
 *                         The sample instant is async to PWM.
 *   ADC2_PWM_PEAK       : Timer4_3 SCMP0 at triangle peak triggers ADC2.
 *                         Effective current-loop rate = PWM frequency.
 *   ADC2_PWM_VALLEY     : Timer4_3 SCMP2 at triangle valley triggers ADC2.
 *                         Effective current-loop rate = PWM frequency.
 *   ADC2_PWM_PEAK_VALLEY: both peak and valley trigger ADC2.
 *                         Effective current-loop rate = 2 x PWM frequency.
 * ==========================================================================*/
#define I_SAMPLE_ADC2_CONT_DMA          (0U)
#define I_SAMPLE_ADC2_PWM_PEAK          (1U)
#define I_SAMPLE_ADC2_PWM_VALLEY        (2U)
#define I_SAMPLE_ADC2_PWM_PEAK_VALLEY   (3U)

/* 2026-09-16: Do NOT select ADC2_PWM_PEAK on the current board.
 * Entering mode 29 immediately reports over-current.  Extending ADC2 sample
 * time to 64 ADCLK cycles did not fix it.  VALLEY and PEAK_VALLEY have been
 * tested working; root-cause PEAK sampling/coupling before re-enabling it. */

#ifndef I_SAMPLE_MODE
#define I_SAMPLE_MODE                   (I_SAMPLE_ADC2_PWM_VALLEY)
#endif

#if ((I_SAMPLE_MODE != I_SAMPLE_ADC2_CONT_DMA) && \
     (I_SAMPLE_MODE != I_SAMPLE_ADC2_PWM_PEAK) && \
     (I_SAMPLE_MODE != I_SAMPLE_ADC2_PWM_VALLEY) && \
     (I_SAMPLE_MODE != I_SAMPLE_ADC2_PWM_PEAK_VALLEY))
#error "Invalid I_SAMPLE_MODE"
#endif

/* Longer ADC2 sample/hold window for PEAK-only tests. Units are ADCLK cycles. */
#define I_ADC2_PWM_PEAK_SAMPLE_TIME     (64U)
#define I_ADC2_PWM_VALLEY_SAMPLE_TIME   (64U)

typedef enum {
    I_SAMPLE_MODE_CONT_DMA = I_SAMPLE_ADC2_CONT_DMA,
    I_SAMPLE_MODE_PWM_PEAK = I_SAMPLE_ADC2_PWM_PEAK,
    I_SAMPLE_MODE_PWM_VALLEY = I_SAMPLE_ADC2_PWM_VALLEY,
    I_SAMPLE_MODE_PWM_PEAK_VALLEY = I_SAMPLE_ADC2_PWM_PEAK_VALLEY,
} i_sample_mode_t;

/* Compatibility flags used by the legacy INMOP read path. */
#if (I_SAMPLE_MODE == I_SAMPLE_ADC2_CONT_DMA)
#define I_INMOP_STYLE                   (1U)
#define I_ASYNC_ADC2_READ               (1U)
#else
#define I_INMOP_STYLE                   (0U)
#define I_ASYNC_ADC2_READ               (0U)
#endif

/* Keep the FOC control cadence tied to the selected current-sample cadence. */
#if ((I_SAMPLE_MODE == I_SAMPLE_ADC2_PWM_PEAK) || \
     (I_SAMPLE_MODE == I_SAMPLE_ADC2_PWM_VALLEY))
#define I_ACTIVE_SAMPLE_RATE_HZ         (MOTOR_PWM_FREQ_HZ)
#else
#define I_ACTIVE_SAMPLE_RATE_HZ         (MOTOR_PWM_FREQ_HZ * 2U)
#endif
/* The FOC timestep is tied to current sampling, not directly to PWM reload. */
#define FOC_ISR_HZ                      (I_ACTIVE_SAMPLE_RATE_HZ)
/* KCL two-sensor mode selector (derive one phase from the other two):
 *   0 = measure all three phases directly with current sensors (default)
 *   1 = U derived:  IU = -(IV + IW)
 *   2 = V derived:  IV = -(IU + IW)
 *   3 = W derived:  IW = -(IU + IV)
 * Derivation runs in the mA domain AFTER per-phase zero calibration,
 * so only physical current is summed (per-phase zero offsets do not leak
 * into the derived channel). All downstream paths (ISR globals, biquad,
 * I_GetData, I_GetCurrentMA) honor this setting. */
#define I_KCL_DERIVE_MODE                (0U)

/* ===== Active current channel definitions ===== */
#if (I_SAMPLE_MODE == I_SAMPLE_ADC2_CONT_DMA)
#define I_CH_U                          (ADC_CH5)   /* PA5/ADC1_CH5: IU */
#define I_CH_V                          (ADC_CH6)   /* PA6/ADC1_CH6: IV */
#define I_CH_W                          (ADC_CH7)   /* PA7/ADC1_CH7: IW */
#else
#define I_CH_U                          (ADC_CH1)   /* PA5/ADC2_CH1/ADC12_IN5: IU */
#define I_CH_V                          (ADC_CH2)   /* PA6/ADC2_CH2/ADC12_IN6: IV */
#define I_CH_W                          (ADC_CH3)   /* PA7/ADC2_CH3/ADC12_IN7: IW */
#endif
#define I_CHANNEL_COUNT                 (3U)

/* ===== Pin definitions ===== */
#define I_U_PORT                        (GPIO_PORT_A)
#define I_U_PIN                         (GPIO_PIN_05)
#define I_V_PORT                        (GPIO_PORT_A)
#define I_V_PIN                         (GPIO_PIN_06)
#define I_W_PORT                        (GPIO_PORT_A)
#define I_W_PIN                         (GPIO_PIN_07)

/* ===== Active ADC hardware configuration ===== */
#if (I_SAMPLE_MODE == I_SAMPLE_ADC2_CONT_DMA)
#define I_ADC_UNIT                      (CM_ADC1)
#define I_ADC_PERIPH_CLK                (FCG3_PERIPH_ADC1)
#define I_ADC_SEQ                       (ADC_SEQ_B)
#define I_ADC_SCAN_MODE                 (ADC_MD_SEQA_SEQB_SINGLESHOT)
#define I_ADC_INT_SRC                   (INT_SRC_ADC1_EOCB)
#define I_ADC_INT_TYPE                  (ADC_INT_EOCB)
#define I_ADC_INT_FLAG                  (ADC_FLAG_EOCB)
#define I_ADC_IRQn                      (INT116_IRQn)
#else
#define I_ADC_UNIT                      (CM_ADC2)
#define I_ADC_PERIPH_CLK                (FCG3_PERIPH_ADC2)
#define I_ADC_SEQ                       (ADC_SEQ_A)
#define I_ADC_SCAN_MODE                 (ADC_MD_SEQA_SINGLESHOT)
#define I_ADC_INT_SRC                   (INT_SRC_ADC2_EOCA)
#define I_ADC_INT_TYPE                  (ADC_INT_EOCA)
#define I_ADC_INT_FLAG                  (ADC_FLAG_EOCA)
#define I_ADC_IRQn                      (INT116_IRQn)
#endif

/* ===== Hardware trigger selection for the active ADC sequence ===== */
#if (I_SAMPLE_MODE == I_SAMPLE_ADC2_PWM_PEAK) || \
    (I_SAMPLE_MODE == I_SAMPLE_ADC2_PWM_VALLEY)
#define I_ADC_HARDTRIG                  (ADC_HARDTRIG_EVT0)
#elif (I_SAMPLE_MODE == I_SAMPLE_ADC2_PWM_PEAK_VALLEY)
#define I_ADC_HARDTRIG                  (ADC_HARDTRIG_EVT0_EVT1)
#else
#define I_ADC_HARDTRIG                  (ADC_HARDTRIG_EVT0_EVT1)
#endif

#define I_ADC_INT_PRIO                  (DDL_IRQ_PRIO_03)

/* ===== Current conversion constants ===== */
#define I_ADC_ZERO                      (2048)      /* ADC raw at 0A (1650mV @ 3.3V/12bit) */
#define I_MA_PER_ADC                    (1563)      /* Fixed-point slope: 3300*1000/(4095*132) ≈ 6.105 mA/count, ×256 ≈ 1563 (132mV/A, +-10A sensor) */
#define I_MA_SHIFT                      (8U)        /* Right-shift after multiply */

/* Integer conversion: I_mA = (raw - zero_ref) * 1563 >> 8 (1563 = 6.105 mA/count x 256). */
#define I_ADC_TO_MA_REF(raw, zero)  ((int16_t)(((int32_t)((int32_t)(raw) - (int32_t)(zero)) * (int32_t)I_MA_PER_ADC) >> I_MA_SHIFT))

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/**
 * @brief  Single current sample (3-phase synchronized)
 */
typedef struct {
    uint16_t u16IU;         /* IU raw ADC */
    uint16_t u16IV;         /* IV raw ADC */
    uint16_t u16IW;         /* IW raw ADC */
    int16_t  i16IU_mA;      /* IU current (mA, signed) */
    int16_t  i16IV_mA;      /* IV current (mA, signed) */
    int16_t  i16IW_mA;      /* IW current (mA, signed) */
    uint32_t u32SampleCount;
    uint8_t  u8NewData;
} stc_i_data_t;

/**
 * @brief  Current data ready callback (called from ADC2 ISR context)
 */
typedef void (*i_callback_t)(const stc_i_data_t *pData);

/*******************************************************************************
 * Global variables for JScope (extern - defined in I.c)
 ******************************************************************************/

/* Raw ADC values (0-4095) */
extern volatile uint16_t g_i_iu_raw;
extern volatile uint16_t g_i_iv_raw;
extern volatile uint16_t g_i_iw_raw;

/* Current values (mA, signed, float) */
extern volatile float    g_i_iu_ma;
extern volatile float    g_i_iv_ma;
extern volatile float    g_i_iw_ma;

/* Float mirrors (J-Scope friendly: signed, no int16 wrap-around) */
extern volatile float   g_scope_iu_ma;
extern volatile float   g_scope_iv_ma;
extern volatile float   g_scope_iw_ma;

/* Biquad-filtered current (mA, float for J-Scope) */
extern volatile float    g_i_iu_filt;
extern volatile float    g_i_iv_filt;
extern volatile float    g_i_iw_filt;

/* Display-friendly: mA + 10000 offset (always positive, J-Scope safe). Subtract 10000 for real value. */
extern volatile uint16_t g_i_iu_disp;
extern volatile uint16_t g_i_iv_disp;
extern volatile uint16_t g_i_iw_disp;

/* Sum of three phases (should be ~0 mA) */
extern volatile float    g_i_uvw_ma;
extern volatile int32_t  g_i_uvw_raw;

/* Sample count */
extern volatile uint32_t g_i_sample_cnt;

/* Module running state (0=stopped, 1=running) */
extern volatile uint8_t  g_i_running;
extern volatile int8_t    g_i_phase_order;     /* current phase remap: 0=UVW 1=UWV 2=VUW 3=VWU 4=WUV 5=WVU (Watch tunable) */


/* Calibration status (0=idle, 1=in progress, 2=done) */
extern volatile uint8_t  g_i_calib_state;

/* Calibrated zero references (per-phase ADC raw, after 500ms averaging) */
extern volatile uint16_t g_i_calib_zero_u;
extern volatile uint16_t g_i_calib_zero_v;
extern volatile uint16_t g_i_calib_zero_w;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/

/* Lifecycle */
void I_Init(void);
void I_DeInit(void);

/* Noise-diagnosis helpers: pause/resume only the sampling IRQ and hardware
 * trigger. Unlike I_DeInit(), the registered callbacks are preserved. */
void I_StopSampling(void);
void I_StartSampling(void);

/* Zero-offset calibration: blocks 500ms, samples all 3 phases, stores offsets */
void I_Calibrate(void);

/* Data access */
void     I_GetData(stc_i_data_t *pData);
uint16_t I_GetRawValue(uint8_t u8Phase);    /* 0=U, 1=V, 2=W */
int16_t  I_GetCurrentMA(uint8_t u8Phase);   /* 0=U, 1=V, 2=W, returns mA */

/* Callback (called in ISR context, keep short) */
void I_RegisterCallback(i_callback_t pfnCallback);

/* Second callback slot (e.g. FOC). Both callbacks run in the same ISR,
 * user callback first, FOC callback second. NULL unregisters. */
void I_RegisterFocCallback(i_callback_t pfnCallback);

#ifdef DEBUG
void I_PrintDebugInfo(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __I_H__ */
