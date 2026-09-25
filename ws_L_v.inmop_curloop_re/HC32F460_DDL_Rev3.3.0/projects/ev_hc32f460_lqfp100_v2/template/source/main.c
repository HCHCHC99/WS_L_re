#include "main.h"
#include "Hardware.h"
#include "rtt_log.h"
#include "timer6_timebase.h"
#include "TickTimer.h"
#include "App_Motor_Project.h"
#include "rtt_manager.h"
#include "hc32_ll_utility.h"
#include "tmr4_pwm.h"
#include "dev_comm_runner.h"
#include "I.h"
#include "motor_config.h"
#include "foc.h"
#include "foc_obs.h"
#include "encoder.h"
#include "Usart3_Vofa.h"
#include "../Utils/dev_pid.h"
#include "Gpio_io.h"
#include "Timer0_Unit1.h"
#include "Timer0_Unit2.h"
#include <math.h>   /* sqrtf */
#include <stdbool.h>

/* Hall sensor raw pins (from hall_sensor_3ch.c ISR, updated in real-time) */
extern volatile uint8_t g_scope_ha;
extern volatile uint8_t g_scope_hb;
extern volatile uint8_t g_scope_hc;
extern volatile uint8_t  g_scope_step;

/*=============================================================================
 * Debug switch: 1 = RTT prints on, 0 = off
 *=============================================================================*/
#define DEBUG_MAIN   1
#if DEBUG_MAIN
    #define MAIN_DBG(fmt, ...)    MAIN_D(fmt, ##__VA_ARGS__)
#else
    #define MAIN_DBG(fmt, ...)    ((void)0)
#endif

/*=============================================================================
 * Keil Watch 可调变量
 *=============================================================================*/
volatile int   comm_mode        = 0;     /* 0=Stop, 23=FOC_Align, 30=ZIZENG, 31=IQ_PI, 32=LOCK_IQ_PI */
volatile float g_comm_duty_pct  = 80.0f;

/*=============================================================================
 * 电流传感器 VCC 3.3V 周期抖动排查开关（Keil Watch 运行时可改）
 *   0 = 完整固件
 *   1 = 关闭电流 ADC 采样触发/EOC 中断；保留 TMR4_3 计数与 PWM
 *   2 = 关闭 6 路 PWM 输出；保留 TMR4_3 计数（VALLEY 触发仍存在）
 *   3 = 停止全部 Timer0 周期中断
 *   4 = 关闭 SysTick（HC32 侧当前 Handler 为空）
 *   5 = 停止 Timer6 us 时基
 *   6 = 关闭 USART3/VOFA
 *   7 = 停止主循环 Encoder_Update（编码器硬件计数不关）
 *   8 = 停止主循环 Foc_Obs_Task / RTT 观察输出
 * 任意非 0 模式会先强制 comm_mode=0；改回 0 时逐项恢复。
 *=============================================================================*/
#define NOISY_MODE_FULL        (0)
#define NOISY_MODE_ADC         (1)
#define NOISY_MODE_PWM         (2)
#define NOISY_MODE_TIMER0      (3)
#define NOISY_MODE_SYSTICK     (4)
#define NOISY_MODE_TIMER6      (5)
#define NOISY_MODE_VOFA        (6)
#define NOISY_MODE_ENCODER     (7)
#define NOISY_MODE_FOC_OBS     (8)

volatile int check_noisy_mode = NOISY_MODE_FULL;

/* FOC 对齐参数（Keil Watch 可调） */
extern volatile float g_foc_align_volt_v;

/* ZIZENG 参数（Keil Watch 可调） */
extern volatile float g_zizeng_freq_hz;
extern volatile float g_zizeng_volt_v;

/* mode 26 开环 VF 参数（Keil Watch 可调） */
extern volatile float g_olf_freq_hz;      /* 当前磁场转速 Hz（斜坡实时输出，只读） */
extern volatile float g_olf_freq_targ_hz; /* 斜坡目标频率 Hz（SW1 每按 +0.5） */
extern volatile int   g_olf_step_010;    /* 自增步长 ×0.1°/步（1≈连续旋转） */
extern volatile int   g_olf_dir;         /* 自增方向 +1=加 / -1=减 */

/*=============================================================================
 * 最小系统测试宏（电流采样抖动排查）
 *   0 = 正常固件（全部功能）
 *   1 = 最小系统：只保留 电流采样 + VOFA+，PWM = 50/50/50 零矢量开关；
 *       跳过 Foc_Init/编码器/按键/事件总线/模式逻辑/RTT 打印
 *   2 = 在 1 的基础上再把三相输出完全关闭（无任何开关动作，计数器
 *       仍运行以维持 20kHz 采样触发）——用于区分"开关耦合"与"传感器本底"
 *   判读：
 *     2 的抖动 << 1 的抖动 -> 开关动作耦合（dV/dt/门极驱动/布局），硬件问题
 *     2 ≈ 1               -> 传感器固有噪声，与开关无关
 *     1 ≈ 正常固件         -> 固件活动（ISR/打印）无额外影响
 *   注意：模式 2 下传感器偏置与开关态不同，DC 零点会整体偏移
 *         （即旧"静止态 zero_ref=2240"现象的逆过程），本模式只看
 *         抖动幅度，不看 DC 中心。
 *=============================================================================*/
#define APP_MINIMAL_CURRENT_TEST  0

/*=============================================================================
 * 噪声排查模块状态
 *=============================================================================*/
static int     s_noisy_applied              = NOISY_MODE_FULL;
static bool    s_noisy_adc_disabled         = false;
static bool    s_noisy_pwm_disabled         = false;
static bool    s_noisy_timer0_disabled      = false;
static bool    s_noisy_systick_disabled     = false;
static bool    s_noisy_timer6_disabled      = false;
static bool    s_noisy_vofa_disabled        = false;
static volatile uint8_t s_noisy_encoder_enabled = 1U;
static volatile uint8_t s_noisy_foc_obs_enabled = 1U;

static void Noisy_Apply(int requested, int *prev_comm_mode);

/*=============================================================================
 * Foc_Common_VofaFill — 通用 / mode 0 专属 VOFA 布局（20 通道）
 *
 * 【通道表】（单位换算：传"毫单位"，SendScaled 内部 ×0.001）
 *   ch0~2  三相电流 (mA → A)                     g_i_iu/iv/iw_ma
 *   ch3~5  三相零偏，已校准值 (kcounts)           g_i_calib_zero_u/v/w ÷1000
 *          ⚠ 原始量是 ADC counts(0~4095)，不 ÷1000 会被 0.001 缩放显示成
 *            0.002 量级而丢精度；此处折算为 kcounts（约 2.0）
 *   ch6    静止系 ialpha (mA → A)                 g_foc_ialpha×1000
 *   ch7    静止系 ibeta  (mA → A)                 g_foc_ibeta×1000
 *   ch8    控制系 id 反馈 (mA → A)                g_foc_id_ma
 *   ch9    控制系 iq 反馈 (mA → A)                g_foc_iq_ma
 *   ch10   控制系 vd 输出 (mV → V)                g_foc_vd×1000
 *   ch11   控制系 vq 输出 (mV → V)                g_foc_vq×1000
 *   ch12   静止系电流幅值 (mA → A)                g_foc_iab_mag×1000
 *   ch13   转子电角度 (deg，0~360×极对数)         g_foc_elec_deg
 *   ch14   转子机械角度 (deg，0~360)              g_foc_mech_deg
 *   ch15   对齐零点 (counts)                      g_foc_align_offset
 *   ch16   故障标志 (0/1)                         g_foc_fault
 *   ch17   **ZIZENG 自增电压幅值 (mV → V)**        g_zizeng_volt_v×1000
 *          ⚠ 本通道为 mode 30 服务的兼容保留项（30/31 迁出通用布局前需要它）
 *   ch18   **ZIZENG 磁场角 (mrad → rad)**         g_zizeng_theta_rad×1000
 *          ⚠ 同上，兼容保留
 *   ch19   OC 阈值 (mA → A)                       g_foc_oc_limit_a×1000
 *
 * 【本布局的设计意图】mode 0 是"静止观测态"，主要用来查**电流测量质量**：
 *   - ch0~2 + ch12：看零漂/温漂/噪声（幅值 ch12 是总噪声的单一指标）
 *   - ch3~5：直接给出零偏基准，与 ch0~2 对比可判断零偏是否漂了
 *   - ch13/14：捏转子可实时看角度跟随（验证 mode 24 校准是否仍有效）
 *   - ch17/18：为尚未迁出本布局的 mode 30/31 保留；迁移完成后可换成
 *              其它诊断量（如 vmax/vramp）
 *
 * ⚠ 修改通道数：只改末尾 return 值即可（当前 20）。
 *   硬上限 USART3_VOFA_MAX_CHANNELS(24)，见 Adp/Usart3_Vofa.h。
 * =============================================================================*/
static int Foc_Common_VofaFill(int32_t *cur)
{
    cur[0]  = (int32_t)(g_i_iu_ma);                   /* ch0 U 相电流 */
    cur[1]  = (int32_t)(g_i_iv_ma);                   /* ch1 V 相电流 */
    cur[2]  = (int32_t)(g_i_iw_ma);                   /* ch2 W 相电流 */
    cur[3]  = (int32_t)(g_i_calib_zero_u / 1000u);    /* ch3 U 相零偏 (kcounts) */
    cur[4]  = (int32_t)(g_i_calib_zero_v / 1000u);    /* ch4 V 相零偏 */
    cur[5]  = (int32_t)(g_i_calib_zero_w / 1000u);    /* ch5 W 相零偏 */
    cur[6]  = (int32_t)(g_foc_ialpha  * 1000.0f);     /* ch6 静止系 ialpha */
    cur[7]  = (int32_t)(g_foc_ibeta   * 1000.0f);     /* ch7 静止系 ibeta */
    cur[8]  = (int32_t)(g_foc_id_ma);                 /* ch8 id 反馈 */
    cur[9]  = (int32_t)(g_foc_iq_ma);                 /* ch9 iq 反馈 */
    cur[10] = (int32_t)(g_foc_vd * 1000.0f);          /* ch10 vd 输出 */
    cur[11] = (int32_t)(g_foc_vq * 1000.0f);          /* ch11 vq 输出 */
    cur[12] = (int32_t)(g_foc_iab_mag * 1000.0f);     /* ch12 静止系电流幅值 */
    cur[13] = (int32_t)(g_foc_elec_deg);              /* ch13 电角度 */
    cur[14] = (int32_t)(g_foc_mech_deg);              /* ch14 机械角度 */
    cur[15] = (int32_t)(g_foc_align_offset);          /* ch15 对齐零点 */
    cur[16] = (int32_t)(g_foc_fault);                 /* ch16 故障标志 */
    cur[17] = (int32_t)(g_zizeng_volt_v * 1000.0f);   /* ch17 ZIZENG 电压幅值（兼容保留） */
    cur[18] = (int32_t)(g_zizeng_theta_rad * 1000.0f);/* ch18 ZIZENG 磁场角（兼容保留） */
    cur[19] = (int32_t)(g_foc_oc_limit_a * 1000.0f);  /* ch19 OC 阈值 */
    return 20;
}

/*=============================================================================
 * main
 *=============================================================================*/
int main(void)
	{
    Hardware_Init();
    MAIN_DBG("System started (MINIMAL: mode23/30 only)");

    /* ---- USART3 + VOFA+ ---- */
    {
        Usart3_HW_Config_t cfg = USART3_HW_CONFIG_DEFAULT;
        cfg.baudrate = 115200;   /* SMO 波形观察：15ch 帧 ~1440fps / 5ch 窄帧 ~3840fps
                                    （115200 时仅 ~180fps，433Hz 电频率必然混叠）。
                                    VOFA+ 串口设置需同步改为 921600 */
        Usart3_Vofa_Init(&cfg);
    }

    tickTimer_DelayMs(5);

    /* ---- CommRunner 初始化（精简配置；同时初始化 TMR4 PWM/计数器） ---- */
    static const comm_runner_config_t runner_cfg = {
        .pwm_freq_hz      = MOTOR_PWM_FREQ_HZ,
        .default_duty_pct = 80.0f,
        .on_init_done     = NULL,
        .pid_cfg          = NULL,
    };
    CommRunner_Init(&runner_cfg);

    /* ---- 电流采样（模式23和30需要电流监视） ---- */
    I_Init();
    tickTimer_DelayMs(2000);    /* 等传感器基准/VDDA 冷启动暂态稳定后再校零 */
    I_Calibrate();              /* 内部已切 FOC 模式 + 50/50/50 零矢量并保持 */

#if APP_MINIMAL_CURRENT_TEST == 2
    /* 附加实验：完全关闭三相输出（HIN/LIN 全低，无任何开关动作）。
     * 只关输出通道、不停计数器——20kHz 采样触发依赖 TMR4_3 比较事件。 */
    TMR4_PWM_SetChannelMode(TMR4_CHANNEL_U, TMR4_MODE_OFF, 0.0f);
    TMR4_PWM_SetChannelMode(TMR4_CHANNEL_V, TMR4_MODE_OFF, 0.0f);
    TMR4_PWM_SetChannelMode(TMR4_CHANNEL_W, TMR4_MODE_OFF, 0.0f);
#endif

    /* ---- FOC 初始化 / 编码器 / 按键（最小系统模式下跳过） ---- */
#if !APP_MINIMAL_CURRENT_TEST
#if MOTOR_FOC_ENABLE
    Foc_Init();
#endif

    /* ---- ABZ 编码器 ---- */
    Encoder_Init();

    /* ---- 按键（PB0=SW3, PB1=SW1, PB2=SW2, 上拉输入） ---- */
    Key_GPIO_Init();

    EventBus_Enable();
#endif /* !APP_MINIMAL_CURRENT_TEST */

    /* ---- 主循环 ---- */
    static int s_prev_mode = -1;
    s_noisy_applied = NOISY_MODE_FULL;

    while (1) {
#if !APP_MINIMAL_CURRENT_TEST
        /* Keil Watch 噪声排查开关：必须先处理，避免在异常模块关闭期间
         * 继续执行 FOC 模式。 */
        if (check_noisy_mode != s_noisy_applied) {
            Noisy_Apply(check_noisy_mode, &s_prev_mode);
        }
        if (check_noisy_mode != NOISY_MODE_FULL) {
            if (comm_mode != COMM_RUNNER_STOP) {
                comm_mode = COMM_RUNNER_STOP;
                s_prev_mode = COMM_RUNNER_STOP;
                CommRunner_SetMode((comm_runner_mode_t)COMM_RUNNER_STOP);
            }
        }

        /* ---- 按键扫描 + 短按处理（必须先于模式检查：末尾的"模式状态同步"
         *      会把 comm_mode 回写成实际模式，若放在其后会同一轮被覆盖丢失） ---- */
        Key_Scan();
        if (Key_GetShortPress(KEY_ID_SW1)) {    /* SW1 短按：进拖动模式；拖动中每按提频 +0.5Hz */
            if (comm_mode == 30) {
                g_zizeng_freq_hz += 0.5f;       /* mode 30：电频率 +0.5Hz（机械转速 = freq×6 rpm） */
                MAIN_DBG("SW1: zizeng freq -> %d mHz", (int)(g_zizeng_freq_hz * 1000.0f));
            } else if (comm_mode == 26) {
                g_olf_freq_targ_hz += 0.5f;     /* mode 26：抬目标频率，斜坡自动跟随 */
                MAIN_DBG("SW1: olf targ -> %d mHz", (int)(g_olf_freq_targ_hz * 1000.0f));
            } else {
                comm_mode = 30;                 /* 其他模式 → ZIZENG */
            }
        }
        if (Key_GetShortPress(KEY_ID_SW2)) {    /* SW2 短按：停转（mode 0），mode 30 的按键出口 */
            comm_mode = 0;
        }
        if (Key_GetShortPress(KEY_ID_SW3)) {    /* SW3 短按： */
            /* 在此填写动作 */
			if(comm_mode != 31)
            {
                comm_mode = 31;
            }
			if(comm_mode == 31)
            {
                comm_mode = 0;
            }
        }

        /* Keil Watch 模式切换 */
        if (comm_mode != s_prev_mode) {
            s_prev_mode = comm_mode;
            CommRunner_SetMode((comm_runner_mode_t)comm_mode);
        }

        CommRunner_Update();
        if (s_noisy_encoder_enabled) {
            Encoder_Update();
        }

        // Foc_RttSend(Timer6_Timebase_GetTimestamp());  /* 已注释：MotorScope RTT 打印 */

        /* 模式状态同步 */
        {
            int actual = (int)CommRunner_GetMode();
            if (actual != comm_mode) {
                comm_mode = actual;
                s_prev_mode = actual;
            }
        }

        /* ---- 观察模块（foc_obs）：RTT 运行监视/事件打印 + mode32->31 交接 ----
         * 原先这里的六段代码（FOC 故障打印 / ALIGN 事件 / ZIZENG_DBG /
         * IQPI_MON / IQPI_FLIP / LOCKIQ 事件+交接）已整体迁移到 foc_obs.c，
         * 变量名与打印格式未变，模块说明见 foc_obs.h。 */
#if MOTOR_FOC_ENABLE
        if (s_noisy_foc_obs_enabled) {
            Foc_Obs_Task();
        }
#endif /* MOTOR_FOC_ENABLE */
#endif /* !APP_MINIMAL_CURRENT_TEST — 最小系统模式下主循环只跑 VOFA */

        /* ---- VOFA+ USART3 数据发送（每模式自持布局） ----
         * 接口约定：SendScaled 内部 ×0.001，即"传毫单位、显示基本单位"。
         *   电流传 mA → 显示 A；电压传 mV → 显示 V；
         *   角度传 mdeg → 显示 deg；占空比传 m% → 显示 %。
         *
         * 【架构】每个模式实现自己的 Foc_Xxx_VofaFill(cur)，返回**通道数**；
         *   返回 0 表示该模式无专属布局（回落到本文件的通用布局）。
         *   **通道含义的唯一事实源 = 各模式 .h 顶部的【模式速览卡】**：
         *     mode 20/23/24/25 → 无专属布局（用通用）
         *     mode 26/27/28/29/30/31/32/41 → 暂用通用（阶段 2 逐步迁移）
         *     mode 40 → foc_40_speed.h（18ch）
         *     mode 45 → foc_45_smo.h（16ch，g_smo45_wave_mode 三种布局）
         *     通用/mode 0 → 本文件下方 Foc_Common_VofaFill（20ch，见其注释）
         *
         * ⚠ 各模式通道数**可以不同**（A 方案）：切模式时 VOFA+ 需同步改通道数。
         * ⚠ 通道数硬上限 = USART3_VOFA_MAX_CHANNELS(24)，见 Adp/Usart3_Vofa.h。
         *   本函数内缓冲区按 32 开，留余量；每个 Fill 的返回值都必须 ≤ 32。
         */
#if 1
        if (!Usart3_Vofa_IsTxBusy()) {
            int32_t cur[32];
            int     n = 0;

            /* ---- 模式自持 VOFA：每个模式一行，语义彻底独立 ----
             * ⚠ 顺序有意义：g_dci_running 同时标记 mode 24/28/29 系，
             *   必须先把已拆出的独立模块（24/29/41）判掉，最后才轮到 28。 */
            if      (g_smo45_running)     { n = Foc_Smo45_VofaFill(cur); }
            else if (g_speed40_running)   { n = Foc_Speed40_VofaFill(cur); }
            else if (g_dcal24_running)    { n = 0; }   /* mode24 待迁移 */
            else if (g_drun29_running)    { n = Foc_Drun29_VofaFill(cur); }  /* 19ch */
            else if (g_drun41_running)    { n = Foc_Drun41_VofaFill(cur); }  /* 21ch */
            else if (g_dci_running)       { n = 0; }   /* mode28 待迁移 */
            else if (g_dcl_running)       { n = 0; }   /* mode27 待迁移 */
            else if (g_olf_running)       { n = 0; }   /* mode26 待迁移 */
            else if (g_calang_running)    { n = 0; }   /* mode25 待迁移 */
            else if (g_cal_running)       { n = 0; }   /* mode20 待迁移 */
            else if (g_zizeng_running)    { n = 0; }   /* mode30 待迁移 */
            else if (g_lockiq_running)    { n = 0; }   /* mode32 待迁移 */
            else if (g_iqpi_running)      { n = 0; }   /* mode31 待迁移 */
            else                          { n = 0; }   /* mode 0 及空闲态 */

            if (n <= 0) {
                n = Foc_Common_VofaFill(cur);    /* 通用/mode 0 布局，20ch */
            }

            /* 越界护栏：Fill 返回值必须能装进 cur[] 且不超上层上限 */
            if (n > 0 && n <= 32 && n <= (int)USART3_VOFA_MAX_CHANNELS) {
                Usart3_Vofa_SendScaled(cur, (uint8_t)n, USART3_VOFA_SCALE_MILLI);
            }
        }
#endif
    }
}

/*=============================================================================
 * 噪声排查：恢复全部被暂停模块
 *=============================================================================*/
static void Noisy_RestoreAll(void)
{
    if (s_noisy_adc_disabled) {
        I_StartSampling();
        s_noisy_adc_disabled = false;
    }

    if (s_noisy_pwm_disabled) {
        /* I_Calibrate 之后的 mode0 基线是互补 PWM 50/50/50 零矢量。
         * 这里恢复到同一状态；SetFocMode 内部会重启计数器。 */
        TMR4_PWM_SetFocMode(FOC_DEADTIME_NS);
        TMR4_PWM_StartOutput();
        TMR4_PWM_SetDuty3Phase(50.0f, 50.0f, 50.0f);
        s_noisy_pwm_disabled = false;
    }

    if (s_noisy_timer0_disabled) {
        TMR0_Unit2_Start(TMR0_CHANNEL_A_2);
        TMR0_Unit2_Start(TMR0_CHANNEL_B_2);
        TMR0_Unit1_Start(TMR0_CHANNEL_B_1);
        TMR0_Unit1_Start(TMR0_CHANNEL_A_1);
        s_noisy_timer0_disabled = false;
    }

    if (s_noisy_systick_disabled) {
        uint32_t ctrl = SysTick->CTRL;
        ctrl |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
        SysTick->VAL = 0UL;
        SysTick->CTRL = ctrl;
        s_noisy_systick_disabled = false;
    }

    if (s_noisy_timer6_disabled) {
        Timer6_Timebase_Start();
        s_noisy_timer6_disabled = false;
    }

    if (s_noisy_vofa_disabled) {
        Usart3_Vofa_Init(NULL);    /* NULL 使用默认 115200 配置 */
        s_noisy_vofa_disabled = false;
    }

    s_noisy_encoder_enabled = 1U;
    s_noisy_foc_obs_enabled = 1U;
}

/*=============================================================================
 * 噪声排查：Watch 修改 check_noisy_mode 后逐项隔离可疑模块
 *=============================================================================*/
static void Noisy_Apply(int requested, int *prev_comm_mode)
{
    if ((requested < NOISY_MODE_FULL) || (requested > NOISY_MODE_FOC_OBS)) {
        requested = NOISY_MODE_FULL;
        check_noisy_mode = NOISY_MODE_FULL;
    }

    /* 每次切换都先回到安全 mode0，再做恢复/隔离，避免恢复 ADC 时仍处在
     * mode24/29/40 等电流控制模式。 */
    comm_mode = COMM_RUNNER_STOP;
    *prev_comm_mode = COMM_RUNNER_STOP;
    CommRunner_SetMode((comm_runner_mode_t)COMM_RUNNER_STOP);
    Noisy_RestoreAll();

    switch (requested) {
    case NOISY_MODE_ADC:
        I_StopSampling();
        s_noisy_adc_disabled = true;
        break;

    case NOISY_MODE_PWM:
        /* 只禁 OC 输出，不停 TMR4_3；因此 ADC 的 VALLEY 比较事件仍在。 */
        TMR4_PWM_SetChannelMode(TMR4_CHANNEL_U, TMR4_MODE_OFF, 0.0f);
        TMR4_PWM_SetChannelMode(TMR4_CHANNEL_V, TMR4_MODE_OFF, 0.0f);
        TMR4_PWM_SetChannelMode(TMR4_CHANNEL_W, TMR4_MODE_OFF, 0.0f);
        s_noisy_pwm_disabled = true;
        break;

    case NOISY_MODE_TIMER0:
        TMR0_Unit2_Stop(TMR0_CHANNEL_B_2);
        TMR0_Unit2_Stop(TMR0_CHANNEL_A_2);
        TMR0_Unit1_Stop(TMR0_CHANNEL_B_1);
        TMR0_Unit1_Stop(TMR0_CHANNEL_A_1);
        s_noisy_timer0_disabled = true;
        break;

    case NOISY_MODE_SYSTICK:
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        s_noisy_systick_disabled = true;
        break;

    case NOISY_MODE_TIMER6:
        Timer6_Timebase_Stop();
        s_noisy_timer6_disabled = true;
        break;

    case NOISY_MODE_VOFA:
        Usart3_Vofa_DeInit();
        s_noisy_vofa_disabled = true;
        break;

    case NOISY_MODE_ENCODER:
        s_noisy_encoder_enabled = 0U;
        break;

    case NOISY_MODE_FOC_OBS:
        s_noisy_foc_obs_enabled = 0U;
        break;

    case NOISY_MODE_FULL:
    default:
        break;
    }

    s_noisy_applied = requested;
    MAIN_DBG("[NOISY] mode=%d applied", requested);
}
