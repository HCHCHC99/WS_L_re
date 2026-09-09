/**
 *******************************************************************************
 * @file  foc_olf.c
 * @brief FOC 模式26 — 开环 VF 负载角实验实现。
 *
 *        流程：自动校准 BETA(2s, 90°) -> ALPHA(2s, 0°, 锁零点 offset)
 *              -> 拖动：磁场角 theta 以 g_olf_freq_hz 转速步长量化自增
 *                 （每攒满 g_olf_step_010×0.1° 跳一步，方向 g_olf_dir ±1，
 *                 默认步长 1 ≈ 连续旋转），
 *                 Vd=V 约定（valpha=V·cos(theta), vbeta=V·sin(theta)，
 *                 磁场角 = theta 本身，与转子 N 极对齐时 delta=0），
 *              -> 用编码器真实转子角（扣校准零点）做 Park 变换得到
 *                 真实转子系 id/iq（实验核心观测量），
 *              -> 角度观测 field/rotor/diff(delta) 整型化供打印。
 *
 *        角度框架（与 mode 25 / mode 20 / mode 0 观测一致）：
 *          offset = mod(ALPHA 结束时 TMRA_1 原始计数 × 编码器方向, CPR)，
 *          转子电角度 = mod(原始计数 × dir - offset, CPR) × 360° × 极对数 / CPR
 *
 *        负载角折叠：diff = field - rotor 折叠到 (-180, 180]，
 *        拖动时磁场领先转子 -> diff > 0；逼近 +90° 即牵出点（失步边界）。
 *
 *        电流观测用原始 pData（与 mode 25 同策略：吸附式校准无 DC
 *        零偏窗口，传感器残余零偏相对 5A 拖动电流可忽略）。
 *
 *        ISR 内禁止打印：事件经 g_olf_evt、周期数据经 Watch 观测量
 *        由 foc_obs 在主循环完成。
 *******************************************************************************
 */

#include "foc_olf.h"
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "hc32_ll_tmra.h"       /* TMRA_GetCountValue(CM_TMRA_1) */

/* 阶段时长 -> ISR tick 数 */
#define OLF_BETA_TICKS    ((uint32_t)OLF_BETA_MS    * (uint32_t)FOC_ISR_HZ / 1000u)
#define OLF_ALPHA_TICKS   ((uint32_t)OLF_ALPHA_MS   * (uint32_t)FOC_ISR_HZ / 1000u)

/* 电角度 rad -> deg 换算（观测显示用，取整） */
#define OLF_RAD2DEG   57.2958f

/*******************************************************************************
 * Watch 观测量 / 可调变量
 ******************************************************************************/
volatile float    g_olf_freq_hz   = 3.0f;   /* 磁场角自增频率，与 mode 30 默认相同 */
volatile int32_t  g_olf_step_010  = 1;      /* 自增步长 ×0.1°/步（1≈连续旋转，同旧模型） */
volatile int32_t  g_olf_dir       = 1;      /* 自增方向：+1=角度加，-1=角度减 */
volatile float    g_olf_volt_v    = 0.6f;   /* 拖动电压，与 mode 30 默认相同 (≈5A) */
volatile uint8_t  g_olf_running   = 0u;
volatile uint8_t  g_olf_state     = OLF_STEP_IDLE;
volatile uint8_t  g_olf_evt       = 0u;
volatile int32_t  g_olf_offset    = 0;
volatile int32_t  g_olf_field_deg = 0;
volatile int32_t  g_olf_rotor_deg = 0;
volatile int32_t  g_olf_diff_deg  = 0;
volatile float    g_olf_id_ma     = 0.0f;
volatile float    g_olf_iq_ma     = 0.0f;
volatile float    g_olf_theta_rad = 0.0f;
volatile float    g_olf_du        = 50.0f;
volatile float    g_olf_dv        = 50.0f;
volatile float    g_olf_dw        = 50.0f;

/* 内部状态 */
static uint32_t s_phase_tick = 0u;    /* 当前阶段计时（tick） */
static int32_t  s_acc_mdeg   = 0;     /* 步长累积器（mdeg，攒满一个步长跳一步） */

/*******************************************************************************
 * 内部助手：输出指定电角度的固定磁场 + 刷新观测量（ISR 内调用）
 *   Vd=V 约定：valpha=V·cos(theta), vbeta=V·sin(theta)，磁场角 = theta
 ******************************************************************************/
static void Olf_OutputField(const stc_i_data_t *pData, float theta)
{
    float valpha, vbeta, du, dv, dw, id, iq;

    valpha = g_olf_volt_v * Foc_Math_Cos(theta);
    vbeta  = g_olf_volt_v * Foc_Math_Sin(theta);
    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);

    g_foc_valpha    = valpha;
    g_foc_vbeta     = vbeta;
    g_foc_vd        = valpha;
    g_foc_vq        = vbeta;
    g_foc_theta_rad = theta;
    g_foc_du        = du;
    g_foc_dv        = dv;
    g_foc_dw        = dw;
    g_olf_du        = du;
    g_olf_dv        = dv;
    g_olf_dw        = dw;

    /* 控制系（磁场角框架）电流观察 */
    Foc_Core_GetDq(pData, theta, &id, &iq);
    g_foc_id_ma = id * 1000.0f;
    g_foc_iq_ma = iq * 1000.0f;
}

/*******************************************************************************
 * Foc_Olf_Start - mode 26 入口（主循环上下文，允许打印）
 *   自动先跑 mode 25 式校准锁零点，随后直接进入拖动。
 ******************************************************************************/
void Foc_Olf_Start(void)
{
    Foc_Core_ClearFault();
    g_foc_mode        = FOC_MODE_ALIGN;   /* 复用 ALIGN 分发路径（按 g_olf_running 区分） */
    g_foc_phase       = 4u;
    g_foc_theta_rad   = FOC_MATH_HALF_PI;
    g_foc_id_ma       = 0.0f;
    g_foc_iq_ma       = 0.0f;
    g_foc_vd          = 0.0f;
    g_foc_vq          = 0.0f;

    Foc_Core_SetStateMachine(FOC_STATE_ALIGN);
    Foc_Core_ResetVoltageEnvelope();
    Foc_Core_ResetEma();
    g_foc_align_state = 1u;

    g_olf_running   = 1u;
    g_olf_state     = OLF_STEP_CAL_BETA;
    g_olf_evt       = 0u;
    s_phase_tick    = 0u;
    g_olf_theta_rad = 0.0f;
    g_olf_field_deg = 0;
    g_olf_rotor_deg = 0;
    g_olf_diff_deg  = 0;
    g_olf_id_ma     = 0.0f;
    g_olf_iq_ma     = 0.0f;
    /* 频率/电压复位为默认（清除上次实验残留，防止启动即高速失步；
     * 拖动中用 SW1/Watch 调频，重新进入则重新从 3Hz 开始） */
    g_olf_freq_hz   = 3.0f;
    g_olf_volt_v    = 0.6f;
    /* 步长/方向同步复位（防上次实验残留的 dir=-1 或大步长） */
    g_olf_step_010  = 1;
    g_olf_dir       = 1;
    s_acc_mdeg      = 0;
    /* g_olf_offset 保留上次锁定值（校准完成后覆盖刷新） */

    Foc_Core_PwmStart();   /* 零矢量起 PWM（g_foc_active=1），下一拍开始吸附 */

    OLF_DBG("start calib BETA 90deg f=%d mHz volt=%d mV step=%d010deg dir=%d",
            (int)(g_olf_freq_hz * 1000.0f), (int)(g_olf_volt_v * 1000.0f),
            (int)g_olf_step_010, (int)g_olf_dir);
}

/*******************************************************************************
 * Foc_Olf_Step - 步进（20 kHz ISR 中调用，禁止打印）
 ******************************************************************************/
void Foc_Olf_Step(const stc_i_data_t *pData)
{
    float theta, rot_rad, id, iq, frq;
    uint16_t hw;
    int32_t diff, off, fld_deg, rot_deg, dfd, step_mdeg, jumps;

    /* ===== OC 保护（原始 pData，去抖在 Foc_Core_OverCurrent 内） ===== */
    if (Foc_Core_OverCurrent(pData)) {
        g_olf_state   = OLF_STEP_FAULT_OC;
        g_olf_evt     = OLF_EVT_OC;
        g_olf_running = 0u;
        Foc_Core_FaultStop(1u);   /* 置故障 + 关 PWM + active=0 + IDLE */
        return;
    }

    switch (g_olf_state) {
    /* ===== 校准 BETA：磁场定 90°，2s ===== */
    case OLF_STEP_CAL_BETA:
        Olf_OutputField(pData, FOC_MATH_HALF_PI);
        if (++s_phase_tick >= OLF_BETA_TICKS) {
            s_phase_tick = 0u;
            g_olf_state  = OLF_STEP_CAL_ALPHA;
            g_olf_evt    = OLF_EVT_BETA_DONE;
        }
        break;

    /* ===== 校准 ALPHA：磁场定 0°，2s，结束锁零点 -> 拖动 ===== */
    case OLF_STEP_CAL_ALPHA:
        Olf_OutputField(pData, 0.0f);
        if (++s_phase_tick >= OLF_ALPHA_TICKS) {
            /* 零点：ALPHA 结束时转子 d 轴在静止系 0°（与 mode 20/25 同框架） */
            hw  = TMRA_GetCountValue(CM_TMRA_1);
            off = Foc_Core_ModPos((int32_t)hw * (int32_t)g_foc_enc_dir,
                                  (int32_t)ENCODER_CPR);
            Foc_Core_SetAlignOffset(off);
            g_olf_offset = off;

            g_olf_theta_rad = 0.0f;   /* 拖动从磁场 0° 起步，与 ALPHA 末角度无缝衔接 */
            s_phase_tick    = 0u;
            g_olf_state     = OLF_STEP_DRAG;
            g_olf_evt       = OLF_EVT_LOCKED;
        }
        break;

    /* ===== 拖动：磁场角自增 + 真实转子系观测（实验主体） ===== */
    case OLF_STEP_DRAG:
        /* 1. 磁场角步长量化自增：
         *    目标转速 = 360°×freq /s -> 每拍累积 mdeg，攒满一个步长
         *    (g_olf_step_010×0.1°) 沿 g_olf_dir 方向跳一步。
         *    等效自增节拍 = 360×freq/step 次/秒；step=1 时≈连续旋转。
         *    阶跃改 freq 立即生效，大改动会失步。 */
        theta = g_olf_theta_rad;
        frq = g_olf_freq_hz;
        if (frq < 0.0f) {
            frq = -frq;   /* 转速取绝对值，方向只由 g_olf_dir 决定 */
        }
        s_acc_mdeg += (int32_t)(360000.0f * frq / (float)FOC_ISR_HZ + 0.5f);
        step_mdeg = (g_olf_step_010 >= 1) ? (g_olf_step_010 * 100) : 100;
        if (s_acc_mdeg >= step_mdeg) {
            jumps = s_acc_mdeg / step_mdeg;
            if (jumps > 16) {
                jumps = 16;         /* 防极端参数长循环，多余累积丢弃 */
                s_acc_mdeg = 0;
            } else {
                s_acc_mdeg -= jumps * step_mdeg;
            }
            /* 0.0017453 rad = 0.1° */
            theta += (float)(g_olf_dir * jumps * g_olf_step_010) * 0.0017453f;
            theta -= (float)((int32_t)(theta * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
            if (theta < 0.0f) {
                theta += FOC_MATH_2PI;
            }
            g_olf_theta_rad = theta;
        }

        /* 2. Vd=V 约定输出磁场 */
        Olf_OutputField(pData, theta);

        /* 3. 转子真实电角度（原始计数 - 校准零点，机械圈 -> 电角度） */
        hw   = TMRA_GetCountValue(CM_TMRA_1);
        diff = Foc_Core_ModPos((int32_t)hw * (int32_t)g_foc_enc_dir
                               - (int32_t)g_olf_offset,
                               (int32_t)ENCODER_CPR);
        rot_rad  = (float)diff * (FOC_MATH_2PI * (float)FOC_POLE_PAIRS
                                  / (float)ENCODER_CPR);
        rot_rad -= (float)((int32_t)(rot_rad * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
        if (rot_rad < 0.0f) {
            rot_rad += FOC_MATH_2PI;
        }

        /* 4. 真实转子系 id/iq（实验核心观测量：iq=力矩, id=磁链分量） */
        Foc_Core_GetDq(pData, rot_rad, &id, &iq);
        g_olf_id_ma = id * 1000.0f;
        g_olf_iq_ma = iq * 1000.0f;

        /* 5. 角度观测（整型电角度 deg）+ 负载角折叠 (-180,180] */
        fld_deg = (int32_t)(theta * OLF_RAD2DEG);
        rot_deg = (int32_t)(rot_rad * OLF_RAD2DEG);
        dfd     = fld_deg - rot_deg;
        dfd     = 180 - Foc_Core_ModPos(180 - dfd, 360);

        g_olf_field_deg = fld_deg;
        g_olf_rotor_deg = rot_deg;
        g_olf_diff_deg  = dfd;
        break;

    case OLF_STEP_FAULT_OC:
    default:
        /* 故障/未知状态：不发波，等待主循环切模式 */
        break;
    }
}

/*******************************************************************************
 * Foc_Olf_Stop - 停止（用户中途切模式，主循环上下文）
 *   自带 g_foc_active 清零，保证任何退出路径 ISR 都不会再进入本模块步进。
 ******************************************************************************/
void Foc_Olf_Stop(void)
{
    if (g_olf_running) {
        g_olf_running     = 0u;
        g_foc_align_state = 0u;
        if (g_foc_active) {
            g_foc_active = 0u;
            Foc_Core_PwmStop();
            Foc_Core_SetStateMachine(FOC_STATE_IDLE);
        }
        OLF_DBG("stopped");
    }
}
