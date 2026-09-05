/**
 *******************************************************************************
 * @file  I.c
 * @brief Current sensing module — ADC1 SEQ_B interrupt-mode 3-phase sampling
 *        Triggered at PWM peak via TMR4_3 SCMP0 → AOS_ADC1_0 (EVT0, shared with BEMF) → ADC1_SEQ_B
 *******************************************************************************
 */

#include "I.h"
#include "rtt_log.h"
#include "TickTimer.h"
#include "tmr4_pwm.h"
#include "motor_config.h"
#include "Usart3_Vofa.h"
#include "Aos.h"
#include "Dma.h"
#include <string.h>

/*******************************************************************************
 * Global variables for JScope monitor
 ******************************************************************************/

/* Raw ADC values (0-4095) */
volatile uint16_t g_i_iu_raw  = 0;
volatile uint16_t g_i_iv_raw  = 0;
volatile uint16_t g_i_iw_raw  = 0;

/* Current in mA (signed, float for J-Scope) */
volatile float    g_i_iu_ma   = 0.0f;
volatile float    g_i_iv_ma   = 0.0f;
volatile float    g_i_iw_ma   = 0.0f;

/* Float mirrors of the Biquad-filtered per-phase current, mA (J-Scope: signed, no int16 wrap-around) */
volatile float   g_scope_iu_ma = 0.0f;
volatile float   g_scope_iv_ma = 0.0f;
volatile float   g_scope_iw_ma = 0.0f;

/* Biquad-filtered current (mA, float for J-Scope) */
volatile float    g_i_iu_filt = 0.0f;
volatile float    g_i_iv_filt = 0.0f;
volatile float    g_i_iw_filt = 0.0f;

/* Display-friendly: mA + 10000, always positive for J-Scope */
volatile uint16_t g_i_iu_disp = 10000;
volatile uint16_t g_i_iv_disp = 10000;
volatile uint16_t g_i_iw_disp = 10000;

/* 2nd-order Butterworth IIR (fc=200Hz @ fs=50kHz design; actual sampling = PWM freq (MOTOR_PWM_FREQ_HZ); real fc = 200Hz x PWM/50k, display only)
 * Designed in MATLAB: [b,a] = butter(2, 200/25000)
 * y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2] */
#define BIQUAD_B0  0.0001551484f
#define BIQUAD_B1  0.0003102968f
#define BIQUAD_B2  0.0001551484f
#define BIQUAD_A1  (-1.9644605802f)   /* -a1 in diff eq = +1.96446*y[n-1] */
#define BIQUAD_A2  0.9650811739f       /* -a2 in diff eq = -0.96508*y[n-2] */

/* Biquad state: x[n-1], x[n-2], y[n-1], y[n-2] per phase */
static float s_fX1U = 0.0f, s_fX2U = 0.0f, s_fY1U = 0.0f, s_fY2U = 0.0f;
static float s_fX1V = 0.0f, s_fX2V = 0.0f, s_fY1V = 0.0f, s_fY2V = 0.0f;
static float s_fX1W = 0.0f, s_fX2W = 0.0f, s_fY1W = 0.0f, s_fY2W = 0.0f;
static bool  s_bBiquadInit = false;

/* Three-phase sum (should be ~0 mA / ~6144 raw) */
volatile float    g_i_uvw_ma  = 0.0f;
volatile int32_t  g_i_uvw_raw  = 0;

/* Sample count */
volatile uint32_t g_i_sample_cnt = 0;

/* Module running state */
volatile uint8_t  g_i_running = 0;
volatile int8_t    g_i_phase_order = 0;    /* current phase remap (0..5), Watch tunable */

/* Calibration state and zero references */
volatile uint8_t  g_i_calib_state  = 0;   /* 0=idle, 1=in_progress, 2=done */
volatile uint16_t g_i_calib_zero_u = 2048;
volatile uint16_t g_i_calib_zero_v = 2048;
volatile uint16_t g_i_calib_zero_w = 2048;

/*******************************************************************************
 * Local variables ('static')
 ******************************************************************************/

/* Latest current data (ISR-updated) */
static stc_i_data_t s_stcIData;

/* Initialization flag */
static bool s_bIInitialized = false;

/* User callback (slot 1, e.g. CurLoop) */
static i_callback_t s_pfnUserCallback = NULL;

/* FOC callback (slot 2, e.g. Foc_Isr) - runs after slot 1 in the same ISR */
static i_callback_t s_pfnFocCallback   = NULL;

/* Calibration accumulators (ISR writes, I_Calibrate reads after blocking) */
static volatile int32_t s_i32CalibSumU = 0;
static volatile int32_t s_i32CalibSumV = 0;
static volatile int32_t s_i32CalibSumW = 0;
static volatile int32_t s_i32CalibCnt  = 0;

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

static void I_SetPinAnalogMode(uint8_t u8Port, uint8_t u8Pin);
static void I_AdcConfig(void);
static void I_TriggerConfig(void);
static void I_IrqConfig(void);

static char I_GetPortLetter(uint8_t u8Port);
static uint8_t I_GetPinNumber(uint16_t u16Pin);

/*******************************************************************************
 * Helper functions
 ******************************************************************************/

static uint8_t I_GetPinNumber(uint16_t u16Pin)
{
    switch (u16Pin) {
        case GPIO_PIN_00: return 0;
        case GPIO_PIN_01: return 1;
        case GPIO_PIN_02: return 2;
        case GPIO_PIN_03: return 3;
        case GPIO_PIN_04: return 4;
        case GPIO_PIN_05: return 5;
        case GPIO_PIN_06: return 6;
        case GPIO_PIN_07: return 7;
        case GPIO_PIN_08: return 8;
        case GPIO_PIN_09: return 9;
        case GPIO_PIN_10: return 10;
        case GPIO_PIN_11: return 11;
        case GPIO_PIN_12: return 12;
        case GPIO_PIN_13: return 13;
        case GPIO_PIN_14: return 14;
        case GPIO_PIN_15: return 15;
        default: return 0;
    }
}

static char I_GetPortLetter(uint8_t u8Port)
{
    switch (u8Port) {
        case GPIO_PORT_A: return 'A';
        case GPIO_PORT_B: return 'B';
        case GPIO_PORT_C: return 'C';
        case GPIO_PORT_D: return 'D';
        case GPIO_PORT_E: return 'E';
        case GPIO_PORT_H: return 'H';
        default: return '?';
    }
}

/**
 * @brief  Set a single GPIO pin to analog mode
 */
static void I_SetPinAnalogMode(uint8_t u8Port, uint8_t u8Pin)
{
    stc_gpio_init_t stcGpioInit;
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    (void)GPIO_Init(u8Port, u8Pin, &stcGpioInit);
    LL_PERIPH_WP(LL_PERIPH_GPIO);
}

/*******************************************************************************
 * ADC 配置 — 独立初始化，不依赖 BEMF
 ******************************************************************************/

/**
 * @brief  Configure ADC1 SEQ_B for 3-channel current scan
 *         SEQ_B single-shot: CH5(IU), CH6(IV), CH7(IW)
 * @note   ADC1 基本模式在此函数中独立初始化，不再依赖 Bemf_Init()
 */
static void I_AdcConfig(void)
{
    stc_adc_init_t stcAdcInit;

    /* 1. Enable ADC1 peripheral clock */
    FCG_Fcg3PeriphClockCmd(I_ADC_PERIPH_CLK, ENABLE);

    /* 2. ---- 关键修复：初始化 ADC1 基本模式 ----
     *    原来这个初始化依赖于 Bemf_Init()，现在独立完成
     */
    (void)ADC_StructInit(&stcAdcInit);
    stcAdcInit.u16ScanMode = ADC_MD_SEQA_SEQB_SINGLESHOT;   /* SEQ_A + SEQ_B 单次扫描模式 */
    stcAdcInit.u16Resolution = ADC_RESOLUTION_12BIT;
    stcAdcInit.u16DataAlign = ADC_DATAALIGN_RIGHT;
    (void)ADC_Init(I_ADC_UNIT, &stcAdcInit);

    /* 3. Configure pins and enable channels in SEQ_B */
    struct {
        uint8_t u8Port;
        uint8_t u8Pin;
        uint8_t u8Channel;
        const char *pszName;
    } astcChannels[3] = {
        { I_U_PORT, I_U_PIN, I_CH_U, "IU" },
        { I_V_PORT, I_V_PIN, I_CH_V, "IV" },
        { I_W_PORT, I_W_PIN, I_CH_W, "IW" },
    };

    for (uint8_t i = 0; i < 3; i++) {
        /* Set pin to analog mode */
        I_SetPinAnalogMode(astcChannels[i].u8Port, astcChannels[i].u8Pin);

        /* Enable ADC channel in SEQ_B */
        ADC_ChCmd(I_ADC_UNIT, I_ADC_SEQ, astcChannels[i].u8Channel, ENABLE);

        I_DEBUG("%s configured: P%c%d (ADC1_CH%d, SEQ_B)",
               astcChannels[i].pszName,
               I_GetPortLetter(astcChannels[i].u8Port),
               I_GetPinNumber(astcChannels[i].u8Pin),
               astcChannels[i].u8Channel);
    }

    I_DEBUG("ADC1 SEQ_B configured for 3-channel current scan (self-init)");
}

/*******************************************************************************
 * ADC1 SEQ_B hardware trigger configuration
 ******************************************************************************/

/**
 * @brief  Configure ADC1 SEQ_B hardware trigger
 *         Shares BEMF's SCMP0 via EVT0 (AOS_ADC1_0), both sequences fire at PWM peak.
 */
static void I_TriggerConfig(void)
{
    /* SEQ_B uses EVT0, same as BEMF's SEQ_A — both triggered by SCMP0 at PWM peak */
    ADC_TriggerConfig(I_ADC_UNIT, I_ADC_SEQ, I_ADC_HARDTRIG);
    ADC_TriggerCmd(I_ADC_UNIT, I_ADC_SEQ, ENABLE);

#if I_INMOP_STYLE
    I_DEBUG("SEQ_B trigger: EVT0+EVT1 (SCMP0@PEAK + SCMP2@VALLEY, 10k PWM -> 20k sampling)");
#else
    I_DEBUG("SEQ_B trigger: EVT0 (shared with BEMF SCMP0)");
#endif
}

#if I_INMOP_STYLE && I_ASYNC_ADC2_READ
/*******************************************************************************
 * INMOP-style (legacy, async): ADC2 free-running continuous + DMA2 latest-slot
 *   Trigger : ADC2 SEQ_A continuous (software start, no hardware trigger)
 *             -> EOCA event -> AOS -> DMA2 CH1/2/3 (repeat, block=1)
 *   Read    : 20kHz ADC1 EOCB ISR (PWM peak) reads latest DMA slot
 *             (mirror of STM32 INMOP: "always sampling, read at ISR")
 *   UVW     : all three phases sampled directly, V is NOT derived
 ******************************************************************************/
#define I_ADC2_UNIT                     (CM_ADC2)
#define I_ADC2_PERIPH_CLK               (FCG3_PERIPH_ADC2)
#define I_DMA_UNIT                      (DMA_UNIT_2)
#define I_DMA_CH_IU                     (1U)   /* DMA2 CH1 <- ADC2_DR1 (IU/PA5) */
#define I_DMA_CH_IV                     (2U)   /* DMA2 CH2 <- ADC2_DR2 (IV/PA6) */
#define I_DMA_CH_IW                     (3U)   /* DMA2 CH3 <- ADC2_DR3 (IW/PA7) */
                                              /* DMA2 CH0 is used by USART3 */

static uint8_t s_au8DmaId[3] = {0xFF, 0xFF, 0xFF};

/**
 * @brief  Configure ADC2 SEQ_A continuous (free-running) on CH1/2/3 = IU/IV/IW
 * @note   PA5/6/7 are shared pins: ADC1_CH5/6/7 (original) and ADC2_CH1/2/3.
 *         Keep all three phases sampled directly; V is NOT computed from U+W.
 */
static void I_Adc2ContinuousConfig(void)
{
    stc_adc_init_t stcAdcInit;

    /* Enable ADC2 peripheral clock */
    FCG_Fcg3PeriphClockCmd(I_ADC2_PERIPH_CLK, ENABLE);

    /* ADC2 SEQ_A continuous conversion = free running (like STM32 INMOP) */
    (void)ADC_StructInit(&stcAdcInit);
    stcAdcInit.u16ScanMode   = ADC_MD_SEQA_CONT;
    stcAdcInit.u16Resolution = ADC_RESOLUTION_12BIT;
    stcAdcInit.u16DataAlign  = ADC_DATAALIGN_RIGHT;
    (void)ADC_Init(I_ADC2_UNIT, &stcAdcInit);

    /* SEQ_A channels: CH1(IU/PA5), CH2(IV/PA6), CH3(IW/PA7) */
    ADC_ChCmd(I_ADC2_UNIT, ADC_SEQ_A, ADC_CH1, ENABLE);
    ADC_ChCmd(I_ADC2_UNIT, ADC_SEQ_A, ADC_CH2, ENABLE);
    ADC_ChCmd(I_ADC2_UNIT, ADC_SEQ_A, ADC_CH3, ENABLE);

    I_DEBUG("ADC2 SEQ_A continuous configured: CH1/2/3 (IU/IV/IW)");
}

/**
 * @brief  Route ADC2_EOCA -> DMA2 CH1/2/3; each channel copies the latest
 *         16-bit sample into a single-slot repeat buffer (block size 1, so
 *         buffer[0] is always the newest value), mirroring STM32 INMOP DMA.
 * @note   No DMA interrupt needed: read cadence comes from the 20kHz EOCB ISR.
 */
static void I_DmaContinuousConfig(void)
{
    stc_dma_config_t stcDmaConfig;

    /* ADC2 end-of-conversion event -> DMA2 CH1/2/3 */
    AOS_Connect(AOS_DMA2_1, EVT_SRC_ADC2_EOCA);
    AOS_Connect(AOS_DMA2_2, EVT_SRC_ADC2_EOCA);
    AOS_Connect(AOS_DMA2_3, EVT_SRC_ADC2_EOCA);

    struct {
        uint8_t u8DmaCh;
        uint8_t u8AdcCh;
    } astcDma[3] = {
        { I_DMA_CH_IU, ADC_CH1 },
        { I_DMA_CH_IV, ADC_CH2 },
        { I_DMA_CH_IW, ADC_CH3 },
    };

    for (uint8_t i = 0; i < 3; i++) {
        memset(&stcDmaConfig, 0, sizeof(stc_dma_config_t));

        stcDmaConfig.u8DmaUnit      = I_DMA_UNIT;
        stcDmaConfig.u8Channel      = astcDma[i].u8DmaCh;
        stcDmaConfig.enDir          = DMA_DIR_PERIPH_TO_MEM;
        stcDmaConfig.enTransMode    = DMA_TRANS_MODE_REPEAT;
        stcDmaConfig.u32SrcAddr     = (uint32_t)((uint32_t)&I_ADC2_UNIT->DR0 +
                                                 (astcDma[i].u8AdcCh * 2U)); /* ADC2_DR1/2/3 */
        stcDmaConfig.u32DestAddr    = 0;   /* Dma_Init allocates and back-fills */
        stcDmaConfig.u32DataWidth   = DMA_DATAWIDTH_16BIT;
        stcDmaConfig.u16BlockSize   = 1;   /* one event -> one latest sample */
        stcDmaConfig.u16TransCount  = 0;   /* infinite transfer */
        stcDmaConfig.u32SrcAddrInc  = DMA_SRC_ADDR_FIX;
        stcDmaConfig.u32DestAddrInc = DMA_DEST_ADDR_INC;
        stcDmaConfig.u8EnableInt    = 0;   /* no DMA ISR (EOCB ISR reads data) */
        stcDmaConfig.u8IntPriority  = DDL_IRQ_PRIO_03;
        stcDmaConfig.pfnCallback    = NULL;

        s_au8DmaId[i] = Dma_Create(&stcDmaConfig);
        if (s_au8DmaId[i] != 0xFF) {
            I_DEBUG("DMA2 CH%d created (ID=%u) for ADC2_DR%d",
                    astcDma[i].u8DmaCh, s_au8DmaId[i], astcDma[i].u8AdcCh);
        } else {
            I_DEBUG("ERROR: DMA2 CH%d create failed!", astcDma[i].u8DmaCh);
        }
    }

    Dma_Init();

    for (uint8_t i = 0; i < 3; i++) {
        if (s_au8DmaId[i] != 0xFF) {
            Dma_Start(s_au8DmaId[i]);
        }
    }

    /* Software start; continuous mode runs freely (no hardware trigger) */
    (void)ADC_Start(I_ADC2_UNIT);

    I_DEBUG("INMOP-style: ADC2 continuous + DMA2 CH1/2/3 running, read via EOCB ISR");
}
#endif /* I_INMOP_STYLE && I_ASYNC_ADC2_READ */

#if I_INMOP_STYLE
/**
 * @brief  Configure TMR4_3 EVT to also fire at counter VALLEY (SCMP2).
 * @note   INMOP-style double update: SCMP0 @ PEAK (BEMF, EVT0) +
 *         SCMP2 @ VALLEY (EVT1) => 10kHz PWM generates 20kHz EOCB ISR.
 *         Uses EVT channel VH; the EVT submodule only writes SCCR/SCSR/SCMR
 *         and is independent of the PWM OC channels.
 */
static void I_Tmr4ValleyEvtConfig(void)
{
    stc_tmr4_evt_init_t stcTmr4Evt;

    (void)TMR4_EVT_StructInit(&stcTmr4Evt);
    stcTmr4Evt.u16Mode         = TMR4_EVT_MD_CMP;
    stcTmr4Evt.u16CompareValue = 0U;                    /* counter == 0 = valley */
    stcTmr4Evt.u16MatchCond    = TMR4_EVT_MATCH_CNT_VALLEY;
    stcTmr4Evt.u16OutputEvent  = TMR4_EVT_OUTPUT_EVT2;  /* -> SCMP2 */

    (void)TMR4_EVT_Init(CM_TMR4_3, TMR4_EVT_CH_VH, &stcTmr4Evt);

    I_DEBUG("TMR4_3 EVT: SCMP2 @ VALLEY configured (10k PWM -> 20k tick)");
}

#endif /* I_INMOP_STYLE */

/*******************************************************************************
 * Interrupt configuration & ISR
 ******************************************************************************/

/**
 * @brief  ADC1 EOCB interrupt callback
 *         Reads 3 current channels from DR5/DR6/DR7, converts to mA.
 */
/* Read the three ADC channels and remap to logical U/V/W per g_i_phase_order.
 * Fixes a ~120deg rotation of the measured current if the PCB/channel mapping
 * differs from the code labels. Watch tunable; run mode 23 and find the order
 * that gives id ~ +target, iq ~ 0. */
static void I_ReadRemapped(uint16_t *iu, uint16_t *iv, uint16_t *iw)
{
    uint16_t a = ADC_GetValue(I_ADC_UNIT, I_CH_U);   /* PA5 raw */
    uint16_t b = ADC_GetValue(I_ADC_UNIT, I_CH_V);   /* PA6 raw */
    uint16_t c = ADC_GetValue(I_ADC_UNIT, I_CH_W);   /* PA7 raw */

    switch (g_i_phase_order) {
    case 1:  *iu = a; *iv = c; *iw = b; break;   /* UWV */
    case 2:  *iu = b; *iv = a; *iw = c; break;   /* VUW */
    case 3:  *iu = b; *iv = c; *iw = a; break;   /* VWU */
    case 4:  *iu = c; *iv = a; *iw = b; break;   /* WUV */
    case 5:  *iu = c; *iv = b; *iw = a; break;   /* WVU */
    default: *iu = a; *iv = b; *iw = c; break;   /* UVW */
    }
}

/* Apply KCL two-sensor derivation in the mA domain (after zero calibration).
 * Mode defined by I_KCL_DERIVE_MODE: 0=three sensors, 1=U, 2=V, 3=W derived. */
static void I_ApplyKclDerive(int16_t *pU, int16_t *pV, int16_t *pW)
{
#if (I_KCL_DERIVE_MODE == 1U)
    *pU = (int16_t)(-((int32_t)*pV + (int32_t)*pW));
#elif (I_KCL_DERIVE_MODE == 2U)
    *pV = (int16_t)(-((int32_t)*pU + (int32_t)*pW));
#elif (I_KCL_DERIVE_MODE == 3U)
    *pW = (int16_t)(-((int32_t)*pU + (int32_t)*pV));
#else
    (void)pU; (void)pV; (void)pW;   /* 0: all three measured directly */
#endif
}

static void I_IrqCallback(void)
{
    /* Clear SEQ_B end-of-conversion flag */
    ADC_ClearStatus(I_ADC_UNIT, ADC_FLAG_EOCB);

    uint16_t u16IU, u16IV, u16IW;
#if I_ASYNC_ADC2_READ
    /* Legacy async: 20kHz ISR reads latest DMA values (ADC2 free-running) */
    u16IU = Dma_GetLatestValue(s_au8DmaId[0]);
    u16IV = Dma_GetLatestValue(s_au8DmaId[1]);
    u16IW = Dma_GetLatestValue(s_au8DmaId[2]);
#else
    /* PWM-synchronized: ADC1 SEQ_B hardware-triggered at PEAK+VALLEY. */
    I_ReadRemapped(&u16IU, &u16IV, &u16IW);
#endif

    /* During calibration: accumulate raw values, skip mA conversion */
    if (g_i_calib_state == 1) {
        s_i32CalibSumU += (int32_t)u16IU;
        s_i32CalibSumV += (int32_t)u16IV;
        s_i32CalibSumW += (int32_t)u16IW;
        s_i32CalibCnt++;
    }

    /* Apply calibrated zero reference, then convert to mA */
    uint16_t u16ZeroU = (g_i_calib_state == 2) ? g_i_calib_zero_u : I_ADC_ZERO;
    uint16_t u16ZeroV = (g_i_calib_state == 2) ? g_i_calib_zero_v : I_ADC_ZERO;
    uint16_t u16ZeroW = (g_i_calib_state == 2) ? g_i_calib_zero_w : I_ADC_ZERO;

    int16_t i16IU_mA = I_ADC_TO_MA_REF(u16IU, u16ZeroU);
    int16_t i16IV_mA = I_ADC_TO_MA_REF(u16IV, u16ZeroV);
    int16_t i16IW_mA = I_ADC_TO_MA_REF(u16IW, u16ZeroW);

    /* KCL two-sensor mode: derive the selected phase from the other two */
    I_ApplyKclDerive(&i16IU_mA, &i16IV_mA, &i16IW_mA);

    /* 2nd-order Butterworth IIR (fc=200Hz @ fs=50kHz design; actual sampling = PWM freq (MOTOR_PWM_FREQ_HZ); real fc = 200Hz x PWM/50k, display only) */
    float fIU, fIV, fIW;
    if (!s_bBiquadInit) {
        /* Seed states with first sample (fast settling, no ramp-up) */
        s_fX1U = s_fX2U = s_fY1U = s_fY2U = (float)i16IU_mA;
        s_fX1V = s_fX2V = s_fY1V = s_fY2V = (float)i16IV_mA;
        s_fX1W = s_fX2W = s_fY1W = s_fY2W = (float)i16IW_mA;
        s_bBiquadInit = true;
        fIU = (float)i16IU_mA;
        fIV = (float)i16IV_mA;
        fIW = (float)i16IW_mA;
    } else {
        fIU = BIQUAD_B0 * (float)i16IU_mA + BIQUAD_B1 * s_fX1U + BIQUAD_B2 * s_fX2U
              - BIQUAD_A1 * s_fY1U - BIQUAD_A2 * s_fY2U;
        fIV = BIQUAD_B0 * (float)i16IV_mA + BIQUAD_B1 * s_fX1V + BIQUAD_B2 * s_fX2V
              - BIQUAD_A1 * s_fY1V - BIQUAD_A2 * s_fY2V;
        fIW = BIQUAD_B0 * (float)i16IW_mA + BIQUAD_B1 * s_fX1W + BIQUAD_B2 * s_fX2W
              - BIQUAD_A1 * s_fY1W - BIQUAD_A2 * s_fY2W;
        /* Shift input history */
        s_fX2U = s_fX1U; s_fX1U = (float)i16IU_mA;
        s_fX2V = s_fX1V; s_fX1V = (float)i16IV_mA;
        s_fX2W = s_fX1W; s_fX1W = (float)i16IW_mA;
        /* Shift output history */
        s_fY2U = s_fY1U; s_fY1U = fIU;
        s_fY2V = s_fY1V; s_fY1V = fIV;
        s_fY2W = s_fY1W; s_fY1W = fIW;
    }
    int16_t i16IU_fmA = (int16_t)fIU;
    int16_t i16IV_fmA = (int16_t)fIV;
    int16_t i16IW_fmA = (int16_t)fIW;

    /* Update internal data structure */
    s_stcIData.u16IU     = u16IU;
    s_stcIData.u16IV     = u16IV;
    s_stcIData.u16IW     = u16IW;
    s_stcIData.i16IU_mA  = i16IU_mA;
    s_stcIData.i16IV_mA  = i16IV_mA;
    s_stcIData.i16IW_mA  = i16IW_mA;
    s_stcIData.u32SampleCount++;
    s_stcIData.u8NewData = 1U;

    /* Update JScope global variables */
    g_i_iu_raw  = u16IU;
    g_i_iv_raw  = u16IV;
    g_i_iw_raw  = u16IW;
    g_i_iu_ma   = (float)i16IU_mA;
    g_i_iv_ma   = (float)i16IV_mA;
    g_i_iw_ma   = (float)i16IW_mA;
    g_scope_iu_ma = (float)i16IU_fmA;   /* filtered mA (Biquad output) */
    g_scope_iv_ma = (float)i16IV_fmA;
    g_scope_iw_ma = (float)i16IW_fmA;
    g_i_iu_filt = (float)i16IU_fmA;   /* filtered mA, direct */
    g_i_iv_filt = (float)i16IV_fmA;
    g_i_iw_filt = (float)i16IW_fmA;
    g_i_iu_disp = (uint16_t)((int32_t)i16IU_fmA * 10 + 10000);
    g_i_iv_disp = (uint16_t)((int32_t)i16IV_fmA * 10 + 10000);
    g_i_iw_disp = (uint16_t)((int32_t)i16IW_fmA * 10 + 10000);
    g_i_uvw_raw = (int32_t)u16IU + (int32_t)u16IV + (int32_t)u16IW;
    g_i_uvw_ma  = (float)i16IU_mA + (float)i16IV_mA + (float)i16IW_mA;
    g_i_sample_cnt = s_stcIData.u32SampleCount;

    /* Invoke user callback if registered */
    if (s_pfnUserCallback != NULL) {
        s_pfnUserCallback(&s_stcIData);
    }

    /* Invoke FOC callback if registered (second slot, keep short) */
    if (s_pfnFocCallback != NULL) {
        s_pfnFocCallback(&s_stcIData);
    }
}

/**
 * @brief  Register ADC1 EOCB interrupt via INTC sign-in
 */
static void I_IrqConfig(void)
{
    stc_irq_signin_config_t stcIrq;

    stcIrq.enIntSrc    = I_ADC_INT_SRC;
    stcIrq.enIRQn      = I_ADC_IRQn;
    stcIrq.pfnCallback = &I_IrqCallback;

    if (LL_OK != INTC_IrqSignIn(&stcIrq)) {
        I_DEBUG("ERROR: INTC_IrqSignIn failed for ADC1 EOCB!");
        return;
    }

    NVIC_ClearPendingIRQ(stcIrq.enIRQn);
    NVIC_SetPriority(stcIrq.enIRQn, I_ADC_INT_PRIO);
    NVIC_EnableIRQ(stcIrq.enIRQn);

    /* Enable ADC1 EOCB interrupt */
    ADC_IntCmd(I_ADC_UNIT, ADC_INT_EOCB, ENABLE);

    I_DEBUG("ADC1 EOCB ISR registered: INT_SRC=%u, IRQn=%d, prio=%d",
           (unsigned)I_ADC_INT_SRC, (int)I_ADC_IRQn, (int)I_ADC_INT_PRIO);
}

/*******************************************************************************
 * API — Lifecycle
 ******************************************************************************/

/**
 * @brief  Initialize current sensing module
 * @note   ADC1 基本模式在此函数中独立初始化，不再依赖 Bemf_Init()
 */
void I_Init(void)
{
    I_DEBUG("Init entry");

    if (s_bIInitialized) {
        I_DEBUG("Already initialized");
        return;
    }

    /* Reset data structure */
    memset(&s_stcIData, 0, sizeof(s_stcIData));

    /* 1. Configure ADC1 SEQ_B: pins + CH5/6/7 (独立初始化 ADC1) */
    I_AdcConfig();

    /* 2. SEQ_B trigger = EVT0 + EVT1 (INMOP-style double update) */
    I_TriggerConfig();

#if I_INMOP_STYLE
    /* 2.4 INMOP-style double update: valley trigger SCMP2 -> EVT1 (20kHz ISR) */
    I_Tmr4ValleyEvtConfig();
    AOS_InitForCurrent();   /* routes TMR4_3_SCMP2 -> AOS_ADC1_1 (TRGSEL1) */
#endif

#if I_INMOP_STYLE && I_ASYNC_ADC2_READ
    /* 2.5 legacy async ADC2 free-running + DMA2 (disabled by default) */
    I_Adc2ContinuousConfig();
    I_DmaContinuousConfig();
#endif

    /* 3. Register EOCB interrupt */
    I_IrqConfig();

    s_bIInitialized = true;
    g_i_running = 1;
    I_DEBUG("Init done: ADC1 SEQ_B self-initialized, ISR=INT116_EOCB");
}

/*******************************************************************************
 * I_Calibrate — Blocking zero-offset calibration (500ms)
 ******************************************************************************/

/**
 * @brief  Blocking zero-offset calibration.
 *         Samples all 3 current channels for 500ms at PWM rate (MOTOR_PWM_FREQ_HZ), computes
 *         per-phase average as the zero reference, and stores the offsets.
 * @note   Must be called AFTER I_Init and BEFORE motor starts.
 *         Blocks for 500ms using tickTimer_DelayMs.
 */
void I_Calibrate(void)
{
    I_DEBUG("Calibration started (500ms blocking)...");

    /* 以 FOC 互补模式 + 50/50/50 零矢量采零——与运行时完全相同的
     * 开关/供电/死区条件。不用通道 OFF：静止态与开关态下传感器
     * 供电/采样条件存在差异，会捕获与运行态不符的零位
     * （实测偏差可达 ~1A 当量）。
     * 三相占空比相同 => 线电压为 0 => 绕组零电流，采零条件成立。
     * 注：SetFocMode 内部会停止计数器，必须重新 StartOutput 恢复
     *     20kHz 采样触发；上电默认通道为 OFF，也必须先切回 FOC 模式。 */
    TMR4_PWM_SetFocMode(FOC_DEADTIME_NS);
    TMR4_PWM_StartOutput();
    TMR4_PWM_SetDuty3Phase(50.0f, 50.0f, 50.0f);
    I_DEBUG("All PWM channels 50%% zero-vector (FOC mode) for calibration");

    /* Reset accumulators */
    s_i32CalibSumU = 0;
    s_i32CalibSumV = 0;
    s_i32CalibSumW = 0;
    s_i32CalibCnt  = 0;

    /* Arm calibration mode — ISR starts accumulating */
    g_i_calib_state = 1;

    /* Block 500ms. ISR fires ~10000 times during this period.
     * 期间每 20ms 向 VOFA+ 发一帧 14 通道（50Hz 刷新）：
     * CH0~2 = 校准窗口瞬时电流（U/V/W，与运行时通道位置齐平，曲线无缝衔接），
     * CH3~13 占位 0。帧长与主循环一致，VOFA+ 全程无需切换帧长。
     * 注意：校准期间 ISR 用默认零位 2048 计算 mA，因此曲线悬在
     * 原始零偏处（≈+1.2 显示当量）是正常现象，其均值即被捕获的零位。 */
    {
        uint32_t u32Ms;
        for (u32Ms = 0u; u32Ms < 500u; u32Ms++) {
            tickTimer_DelayMs(1);
        }
    }

    /* Stop calibration */
    g_i_calib_state = 2;

    /* Compute per-phase average as zero reference */
    if (s_i32CalibCnt > 0) {
        g_i_calib_zero_u = (uint16_t)(s_i32CalibSumU / s_i32CalibCnt);
        g_i_calib_zero_v = (uint16_t)(s_i32CalibSumV / s_i32CalibCnt);
        g_i_calib_zero_w = (uint16_t)(s_i32CalibSumW / s_i32CalibCnt);
    }

    I_DEBUG("Calibration done: %ld samples, zero_ref U=%u V=%u W=%u",
           s_i32CalibCnt, g_i_calib_zero_u, g_i_calib_zero_v, g_i_calib_zero_w);
}

/**
 * @brief  Deinitialize current sensing module
 */
void I_DeInit(void)
{
    s_bIInitialized = false;
    s_pfnUserCallback = NULL;
    s_pfnFocCallback   = NULL;
    g_i_running = 0;

    /* Disable ADC1 EOCB interrupt */
    ADC_IntCmd(I_ADC_UNIT, ADC_INT_EOCB, DISABLE);
    NVIC_DisableIRQ(I_ADC_IRQn);
    NVIC_ClearPendingIRQ(I_ADC_IRQn);

    /* Disable ADC1 SEQ_B trigger */
    ADC_TriggerCmd(I_ADC_UNIT, I_ADC_SEQ, DISABLE);

#if I_INMOP_STYLE && I_ASYNC_ADC2_READ
    /* Stop ADC2 free-running conversion and its DMA channels (legacy async path) */
    (void)ADC_Stop(I_ADC2_UNIT);
    for (uint8_t i = 0; i < 3; i++) {
        if (s_au8DmaId[i] != 0xFF) {
            Dma_Stop(s_au8DmaId[i]);
        }
    }
#endif

    /* Clear data */
    memset(&s_stcIData, 0, sizeof(s_stcIData));

    I_DEBUG("Deinitialized");
}

/*******************************************************************************
 * API — Data access
 ******************************************************************************/

/**
 * @brief  Get latest 3-phase current data
 * @param  pData  Output data structure
 * @note   Reads current values directly from ADC2 DR registers, not cached.
 */
void I_GetData(stc_i_data_t *pData)
{
    if (pData == NULL) {
        return;
    }

#if I_ASYNC_ADC2_READ
    /* Legacy async: read latest DMA slots */
    pData->u16IU = Dma_GetLatestValue(s_au8DmaId[0]);
    pData->u16IV = Dma_GetLatestValue(s_au8DmaId[1]);
    pData->u16IW = Dma_GetLatestValue(s_au8DmaId[2]);
#else
    /* Read current values from ADC1 data registers (PWM-synchronized) */
    I_ReadRemapped(&pData->u16IU, &pData->u16IV, &pData->u16IW);
#endif
    uint16_t u16Z;
    u16Z = (g_i_calib_state == 2) ? g_i_calib_zero_u : I_ADC_ZERO;
    pData->i16IU_mA = I_ADC_TO_MA_REF(pData->u16IU, u16Z);
    u16Z = (g_i_calib_state == 2) ? g_i_calib_zero_v : I_ADC_ZERO;
    pData->i16IV_mA = I_ADC_TO_MA_REF(pData->u16IV, u16Z);
    u16Z = (g_i_calib_state == 2) ? g_i_calib_zero_w : I_ADC_ZERO;
    pData->i16IW_mA = I_ADC_TO_MA_REF(pData->u16IW, u16Z);

    /* KCL two-sensor mode: derive the selected phase from the other two */
    I_ApplyKclDerive(&pData->i16IU_mA, &pData->i16IV_mA, &pData->i16IW_mA);

    /* Copy sample count and clear flag */
    pData->u32SampleCount = s_stcIData.u32SampleCount;
    pData->u8NewData = s_stcIData.u8NewData;
    s_stcIData.u8NewData = 0U;
}

/**
 * @brief  Get raw ADC value for a single current channel
 * @param  u8Phase  0=U, 1=V, 2=W
 * @return Raw ADC value (0-4095), 0 if invalid
 */
uint16_t I_GetRawValue(uint8_t u8Phase)
{
#if I_ASYNC_ADC2_READ
    if (u8Phase < 3u) {
        return Dma_GetLatestValue(s_au8DmaId[u8Phase]);
    }
    return 0;
#else
    uint16_t iu, iv, iw;
    I_ReadRemapped(&iu, &iv, &iw);
    switch (u8Phase) {
        case 0: return iu;
        case 1: return iv;
        case 2: return iw;
        default: return 0;
    }
#endif
}

/**
 * @brief  Get current in mA for a single phase
 * @param  u8Phase  0=U, 1=V, 2=W
 * @return Current in mA (signed), 0 if invalid
 */
int16_t I_GetCurrentMA(uint8_t u8Phase)
{
    uint16_t u16Raw = I_GetRawValue(u8Phase);
    if (u16Raw == 0) {
        return 0;
    }
    uint16_t u16Zero;
    if (g_i_calib_state == 2) {
        switch (u8Phase) {
            case 0: u16Zero = g_i_calib_zero_u; break;
            case 1: u16Zero = g_i_calib_zero_v; break;
            case 2: u16Zero = g_i_calib_zero_w; break;
            default: return 0;
        }
    } else {
        u16Zero = I_ADC_ZERO;
    }
#if (I_KCL_DERIVE_MODE == 1U)
    if (u8Phase == 0u) {
        /* KCL: U = -(V+W), from the two measured phases */
        return (int16_t)(-((int32_t)I_GetCurrentMA(1u) + (int32_t)I_GetCurrentMA(2u)));
    }
#elif (I_KCL_DERIVE_MODE == 2U)
    if (u8Phase == 1u) {
        /* KCL: V = -(U+W), from the two measured phases */
        return (int16_t)(-((int32_t)I_GetCurrentMA(0u) + (int32_t)I_GetCurrentMA(2u)));
    }
#elif (I_KCL_DERIVE_MODE == 3U)
    if (u8Phase == 2u) {
        /* KCL: W = -(U+V), from the two measured phases */
        return (int16_t)(-((int32_t)I_GetCurrentMA(0u) + (int32_t)I_GetCurrentMA(1u)));
    }
#endif
    return I_ADC_TO_MA_REF(u16Raw, u16Zero);
}

/*******************************************************************************
 * API — Callback
 ******************************************************************************/

/**
 * @brief  Register callback for new current data notification
 * @param  pfnCallback  Callback function (NULL to unregister)
 * @note   Callback runs in ADC2 ISR context — keep it short.
 */
void I_RegisterCallback(i_callback_t pfnCallback)
{
    s_pfnUserCallback = pfnCallback;
    I_DEBUG("Callback %s", (pfnCallback != NULL) ? "registered" : "unregistered");
}

/**
 * @brief  Register second callback (FOC slot) for new current data notification
 * @param  pfnCallback  Callback function (NULL to unregister)
 * @note   Runs in the same ADC1 EOCB ISR, right after the user callback.
 *         Keep it short (ISR context).
 */
void I_RegisterFocCallback(i_callback_t pfnCallback)
{
    s_pfnFocCallback = pfnCallback;
    I_DEBUG("FocCallback %s", (pfnCallback != NULL) ? "registered" : "unregistered");
}

/*******************************************************************************
 * API — Debug
 ******************************************************************************/

#ifdef DEBUG
void I_PrintDebugInfo(void)
{
    I_DEBUG("=== Current Module Debug Info ===");
    I_DEBUG("Initialized: %s", s_bIInitialized ? "Yes" : "No");
    I_DEBUG("Samples: %lu", s_stcIData.u32SampleCount);
    I_DEBUG("Latest raw:  IU=%u, IV=%u, IW=%u",
           s_stcIData.u16IU, s_stcIData.u16IV, s_stcIData.u16IW);
    I_DEBUG("Latest mA:   IU=%d, IV=%d, IW=%d",
           s_stcIData.i16IU_mA, s_stcIData.i16IV_mA, s_stcIData.i16IW_mA);
    I_DEBUG("Zero ref: %u (1650mV)", I_ADC_ZERO);
    I_DEBUG("Scale: 1 ADC count = %d.%d mA", I_MA_PER_ADC >> I_MA_SHIFT,
           (int)(((I_MA_PER_ADC & 0xFF) * 100) >> I_MA_SHIFT));
}
#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/
