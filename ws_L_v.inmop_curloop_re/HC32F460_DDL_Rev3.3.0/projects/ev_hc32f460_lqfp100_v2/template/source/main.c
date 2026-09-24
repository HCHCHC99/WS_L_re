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
 * main
 *=============================================================================*/
int main(void)
	{
    Hardware_Init();
    MAIN_DBG("System started (MINIMAL: mode23/30 only)");

    /* ---- USART3 + VOFA+ ---- */
    {
        Usart3_HW_Config_t cfg = USART3_HW_CONFIG_DEFAULT;
        cfg.baudrate = 912600;   /* SMO 波形观察：15ch 帧 ~1440fps / 5ch 窄帧 ~3840fps
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

        /* ---- VOFA+ USART3 数据发送（模式自持 + 通用布局） ----
         * 接口约定：SendScaled 内部 ×0.001，即"传毫单位、显示基本单位"。
         * 电流通道传整数 mA -> 显示 A（1mA 分辨率，µA 精度已舍弃）；
         * 电压通道传 mV -> 显示 V；角度通道传 mrad/mdeg -> 显示 rad/deg。
         * 模式自持：mode45 / mode40 各自实现 Foc_Xxx_VofaFill 填充自己的
         * 通道布局，通道含义速览卡 = 各模式 .h 顶部注释（唯一事实源）：
         *   mode45（g_smo45_wave_mode=0/1/2 三种布局）见 foc_smo45.h
         *   mode40（固定 18ch，ch17 = PI 反馈真实转速）见 foc_speed40.h
         * 其余模式走下方通用 17ch 布局。
         * 【通用布局（mode 0/26/28/29/30/31/41，17 通道）】
         * CH0~2  : 三相原始电流（含上电校准残余零偏，mode 0 下用于观察温漂/噪声）
         * CH3    : 静止系 ialpha（瞬时值，无 EMA）
         * CH4    : 静止系 ibeta；mode28 时 = id 反馈 (A)
         * CH5    : 控制系 iq（mode28 = iq 反馈）
         * CH6    : 控制系 id；mode28/29/41 时 = id 参考 (A)
         * CH7    : 自增电压幅值 g_zizeng_volt_v；mode28/29/41 时 = iq 参考 (A)
         * CH8    : 控制系总电流幅值 = sqrt(iq^2+id^2)；mode28/29/41 时 = PI 输出 vd (V)
         * CH9    : 控制系角度（ZIZENG 磁场角）；mode28/29/41 时 = PI 输出 vq (V)
         * CH10   : 静止系电流幅值 sqrt(ialpha^2+ibeta^2)；mode29/41/28 时 = iq 3s均值 (A)
         * CH11   : 母线直流电流估算 = 1.5*(vd*id+vq*iq)/Vbus（功率守恒估算）；
         *          mode29/41/28 时 = id 3s均值 (A)
         * CH12   : mode31 iq 参考（斜坡后）
         * CH13   : mode26 负载角 delta（deg，其他模式下恒 0）
         * CH14~15: 预留 0（mode40 目标/显示转速已迁至自持布局，见 foc_speed40.h）
         * CH16   : mode29/41 显示用滤波 iq（EMA；PI 仍使用 CH5 的原始 iq） */
#if 1
        if (!Usart3_Vofa_IsTxBusy()) {
            /* 缓冲按上层通道上限开（Usart3_Vofa_SendScaled 拒收
             * count > USART3_VOFA_MAX_CHANNELS=24）。模式自持布局会随调试
             * 需求加通道，此数组若按旧值写死，Fill 函数就会越界写栈
             * （mode 40 已 17ch -> 18ch）。 */
            int32_t cur[USART3_VOFA_MAX_CHANNELS];
            int     n;

            /* 模式自持 VOFA：先问各模式要不要自己的布局 */
            if (g_smo45_running) {
                n = Foc_Smo45_VofaFill(cur);    /* foc_smo45.h 速览卡 */
            } else if (g_speed40_running) {
                n = Foc_Speed40_VofaFill(cur);  /* foc_speed40.h 速览卡 */
            } else {
                n = 0;
            }

            if (n <= 0) {
            /*===== 通用布局（mode 0/26/28/29/30/31/41，17 通道）=====*/
            cur[0] = (int32_t)(g_i_iu_ma);            /* U 相电流 (mA -> A) */
            cur[1] = (int32_t)(g_i_iv_ma);            /* V 相电流 (mA -> A) */
            cur[2] = (int32_t)(g_i_iw_ma);            /* W 相电流 (mA -> A) */
            cur[3] = (int32_t)(g_foc_ialpha * 1000.0f); /* 静止系 ialpha (mA -> A) */
            cur[4] = (g_drun29_running || g_drun41_running || g_dci_running)
                                   ? (int32_t)(g_foc_id_ma)            /* mode28/29/41: id 反馈 (mA -> A) */
                                   : (int32_t)(g_foc_ibeta * 1000.0f); /* 静止系 ibeta (mA -> A) */
            cur[5] = (int32_t)(g_foc_iq_ma);           /* 控制系 iq / mode28: iq 反馈 (mA -> A) */
            cur[6] = g_drun29_running ? (int32_t)(g_drun29_id_ref_ma)  /* mode29: id 参考 (mA -> A) */
                    : g_drun41_running ? (int32_t)(g_drun41_id_ref_ma)  /* mode41: id 参考 (mA -> A) */
                    : g_dci_running    ? (int32_t)(g_dci_id_ref_ma)    /* mode28: id 参考 (mA -> A) */
                                       : (int32_t)(g_foc_id_ma);       /* 控制系 id (mA -> A) */

            cur[7] = g_drun29_running ? (int32_t)(g_drun29_iq_ref_ma)  /* mode29: iq 参考 (mA -> A) */
                    : g_drun41_running ? (int32_t)(g_drun41_iq_ref_ma)  /* mode41: iq 参考 (mA -> A) */
                    : g_dci_running    ? (int32_t)(g_dci_iq_ref_ma)    /* mode28: iq 参考 (mA -> A) */
                                       : (int32_t)(g_zizeng_volt_v * 1000.0f); /* mode30: 电压幅值 (mV -> V) */
            cur[8] = (g_drun29_running || g_drun41_running || g_dci_running)
                                   ? (int32_t)(g_foc_vd * 1000.0f)     /* mode28/29/41: 输出 vd (V) */
                                   : (int32_t)sqrtf(g_foc_iq_ma * g_foc_iq_ma
                                      + g_foc_id_ma * g_foc_id_ma);    /* 控制系合成 (mA -> A) */
            cur[9] = (g_drun29_running || g_drun41_running || g_dci_running)
                                   ? (int32_t)(g_foc_vq * 1000.0f)     /* mode28/29/41: 输出 vq (V) */
                                   : (int32_t)(g_zizeng_theta_rad * 1000.0f); /* mode30: 控制系角度 (mrad -> rad) */

            cur[10] = g_drun29_running ? (int32_t)(g_drun29_iq_mean_ma) /* mode29: iq 3s均值 (mA -> A) */
                     : g_drun41_running ? (int32_t)(g_drun41_iq_mean_ma) /* mode41: iq 3s均值 (mA -> A) */
                     : g_dci_running    ? (int32_t)(g_dci_iq_mean_ma)   /* mode28: iq 3s均值 (mA -> A) */
                                        : (int32_t)(g_foc_iab_mag * 1000.0f); /* 静止系幅值 (mA -> A) */
#if ZIZENG_VOLT_ON_Q_AXIS
            /* P = 1.5*vq*iq，iq_ma 已是 mA，结果直接为 mA -> 显示 A */
            cur[11] = (int32_t)(1.5f * g_zizeng_volt_v
                                * (float)g_foc_iq_ma / FOC_VBUS_V);
#else
            /* P = 1.5*vd*id */
            cur[11] = (int32_t)(1.5f * g_zizeng_volt_v
                                * (float)g_foc_id_ma / FOC_VBUS_V);
#endif
            if (g_drun29_running) {
                cur[11] = (int32_t)(g_drun29_id_mean_ma); /* mode29: id 3s均值 (mA -> A) */
            } else if (g_drun41_running) {
                cur[11] = (int32_t)(g_drun41_id_mean_ma); /* mode41: id 3s均值 (mA -> A) */
            } else if (g_dci_running) {
                cur[11] = (int32_t)(g_dci_id_mean_ma); /* mode28: id 3s均值 (mA -> A) */
            }
            cur[12] = (int32_t)(g_iqpi_iq_ref_ramp_ma); /* mode31 iq 参考(斜坡后), mA -> A */
            cur[13] = g_olf_diff_deg * 1000;          /* mode26 负载角 delta (mdeg -> deg) */
            cur[14] = 0;  /* 预留（mode40 目标转速已迁自持布局，见 foc_speed40.h） */
            cur[15] = 0;  /* 预留（mode40 实际转速已迁自持布局） */
            cur[16] = g_drun41_running
                            ? (int32_t)(g_drun41_iq_filt_ma)           /* mode41: filtered iq (mA -> A) */
                            : g_drun29_running
                            ? (int32_t)(g_drun29_iq_filt_ma)           /* mode29: filtered iq (mA -> A) */
                            : 0;
            n = 17;
            }

            Usart3_Vofa_SendScaled(cur, n, USART3_VOFA_SCALE_MILLI);
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
