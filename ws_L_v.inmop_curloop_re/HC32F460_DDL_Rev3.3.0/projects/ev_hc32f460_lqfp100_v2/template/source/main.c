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
#include <math.h>   /* sqrtf */

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

/* FOC 对齐参数（Keil Watch 可调） */
extern volatile float g_foc_align_volt_v;

/* ZIZENG 参数（Keil Watch 可调） */
extern volatile float g_zizeng_freq_hz;
extern volatile float g_zizeng_volt_v;

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
 * main
 *=============================================================================*/
int main(void)
{
    Hardware_Init();
    MAIN_DBG("System started (MINIMAL: mode23/30 only)");

    /* ---- USART3 + VOFA+ ---- */
    {
        Usart3_HW_Config_t cfg = USART3_HW_CONFIG_DEFAULT;
        cfg.baudrate = 115200;
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

    while (1) {
#if !APP_MINIMAL_CURRENT_TEST
        /* ---- 按键扫描 + 短按处理（必须先于模式检查：末尾的"模式状态同步"
         *      会把 comm_mode 回写成实际模式，若放在其后会同一轮被覆盖丢失） ---- */
        Key_Scan();
        if (Key_GetShortPress(KEY_ID_SW1)) {    /* SW1 短按：ZIZENG 启停切换（mode 0 ↔ 30） */
            if (comm_mode == 30) {
                comm_mode = 0;                  /* mode 30 → 停转 */
            } else {
                comm_mode = 30;                 /* 其他模式 → ZIZENG */
            }
        }
        if (Key_GetShortPress(KEY_ID_SW2)) {    /* SW2 短按：停转（mode 0） */

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
        Encoder_Update();

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
        Foc_Obs_Task();
#endif /* MOTOR_FOC_ENABLE */
#endif /* !APP_MINIMAL_CURRENT_TEST — 最小系统模式下主循环只跑 VOFA */

        /* ---- VOFA+ USART3 数据发送（电流观测通道，12 通道定长） ----
         * 接口约定：SendScaled 内部 ×0.001，即"传毫单位、显示基本单位"。
         * 电流通道传整数 mA -> 显示 A（1mA 分辨率，µA 精度已舍弃）；
         * 电压通道传 mV -> 显示 V；角度通道传 mrad -> 显示 rad。
         * CH0~2  : 三相原始电流（含上电校准残余零偏，mode 0 下用于观察温漂/噪声）
         * CH3~4  : 静止系 ialpha / ibeta（瞬时值，无 EMA）
         * CH5~6  : 控制系 iq / id（mode 30 内已经过 foc_calib 零偏校正）
         * CH7    : 自增电压幅值 g_zizeng_volt_v
         * CH8    : 控制系总电流幅值 = sqrt(iq^2+id^2)
         * CH9    : 控制系角度（ZIZENG 磁场角）
         * CH10   : 静止系电流幅值 sqrt(ialpha^2+ibeta^2)（应≈CH8）
         * CH11   : 母线直流电流估算 = 1.5*(vd*id+vq*iq)/Vbus（无母线采样，
         *          由功率守恒估算，含铜损前的电功率；mode 0 下为 0） */
#if 1
        if (!Usart3_Vofa_IsTxBusy()) {
            int32_t cur[13];

            cur[0] = (int32_t)(g_i_iu_ma);            /* U 相电流 (mA -> A) */
            cur[1] = (int32_t)(g_i_iv_ma);            /* V 相电流 (mA -> A) */
            cur[2] = (int32_t)(g_i_iw_ma);            /* W 相电流 (mA -> A) */
            cur[3] = (int32_t)(g_foc_ialpha * 1000.0f); /* 静止系 ialpha (mA -> A) */
            cur[4] = (int32_t)(g_foc_ibeta * 1000.0f);  /* 静止系 ibeta (mA -> A) */
            cur[5] = (int32_t)(g_foc_iq_ma);          /* 控制系 iq (mA -> A) */
            cur[6] = (int32_t)(g_foc_id_ma);          /* 控制系 id (mA -> A) */

            cur[7] = (int32_t)(g_zizeng_volt_v * 1000.0f); /* 电压幅值 (mV -> V) */
            cur[8] = (int32_t)sqrtf(g_foc_iq_ma * g_foc_iq_ma
                              + g_foc_id_ma * g_foc_id_ma);   /* 控制系合成 (mA -> A) */
            cur[9] = (int32_t)(g_zizeng_theta_rad * 1000.0f); /* 控制系角度 (mrad -> rad) */

            cur[10] = (int32_t)(g_foc_iab_mag * 1000.0f); /* 静止系幅值 (mA -> A) */
#if ZIZENG_VOLT_ON_Q_AXIS
            /* P = 1.5*vq*iq，iq_ma 已是 mA，结果直接为 mA -> 显示 A */
            cur[11] = (int32_t)(1.5f * g_zizeng_volt_v
                                * (float)g_foc_iq_ma / FOC_VBUS_V);
#else
            /* P = 1.5*vd*id */
            cur[11] = (int32_t)(1.5f * g_zizeng_volt_v
                                * (float)g_foc_id_ma / FOC_VBUS_V);
#endif
            cur[12] = (int32_t)(g_iqpi_iq_ref_ramp_ma); /* mode31 iq 参考(斜坡后), mA -> A */
            Usart3_Vofa_SendScaled(cur, 13, USART3_VOFA_SCALE_MILLI);
        }
#endif
    }
}
