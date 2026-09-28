/**
 *******************************************************************************
 * @file  foc_51_rs.c
 * @brief FOC mode 51 - 定子电阻辨识（静止直流注入 + 多电平最小二乘）。
 *
 * 原理（转子静止 ⇒ 无 BEMF、稳态无 di/dt ⇒ 电压方程退化为纯电阻）：
 *     |v| = R_eff · |i| + V_dead
 *   斜率 = R_eff（环路真正看到的电阻：相阻 + 2×Rdson + 采样电阻 + 走线）
 *   截距 = V_dead（死区 + 管压降的等效电压，理论 ≈ Vbus·Td/Ts = 0.12 V）
 * 单点测量会被 V_dead 淹掉，所以必须多电平线性拟合。
 *
 * 用途定位：**验证辨识思路本身**，因此
 *   - 只输出观测量（Watch / VOFA），不写 Flash、不改任何配置宏；
 *   - 同时给出 R_eff / 手册值 的比值，便于直接与商家参数对比；
 *   - 数据质量指标（R²、位移、零偏）与结果同等重要。
 *
 * 前置：**无**（不需要 mode 24 校准、不需要编码器零点）。
 *   电压矢量固定在静止 α 轴（vα = V, vβ = 0），与转子位置无关。
 *   注入会产生力矩趋势，故用编码器监视位移；能夹紧转子时数据最好。
 *******************************************************************************
 */

#include "foc_51_rs.h"
#include "I.h"                 /* g_i_*（VOFA 用） */
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "hc32_ll_tmra.h"      /* TMRA_GetCountValue(CM_TMRA_1) 位移监视 */
#include "timer6_timebase.h"   /* Timer6 计数器：0.32us 分辨率时基，实测 ISR 频率 */
#include <math.h>              /* sqrtf / fabsf */

/*=============================================================================
 * 时长换算（FOC_ISR_HZ tick）
 *=============================================================================*/
#define RS51_MS_TO_TICKS(ms)   ((uint32_t)(ms) * FOC_ISR_HZ / 1000u)

/*=============================================================================
 * Keil Watch 可调参数
 *=============================================================================*/
volatile float  g_rs51_i_max_a       = 3.0f;    /* 最大注入电流 (A)，硬上限见 .h */
volatile float  g_rs51_i_min_a       = 1.0f;    /* 档位规划的最低目标电流 (A) */
volatile float  g_rs51_vdead_prior_v = 0.15f;   /* 死区电压先验 (V)：探测扣偏置 + 档位规划 */
volatile float  g_rs51_v_hard_max_v  = 1.5f;    /* 单档电压硬上限 (V) */
volatile uint32_t g_rs51_points      = 6u;      /* 档位数 (3~10) */
volatile uint32_t g_rs51_settle_ms   = 100u;    /* 每档稳定等待 (ms) */
volatile uint32_t g_rs51_avg_ms      = 100u;    /* 每档平均窗口 (ms) */
volatile uint32_t g_rs51_precond_ms  = 300u;    /* 预置注入时长 (ms) */

/*=============================================================================
 * 观测量（Watch / VOFA）
 *=============================================================================*/
volatile uint8_t  g_rs51_running     = 0u;
volatile uint8_t  g_rs51_state       = FOC51_STEP_IDLE;
volatile uint8_t  g_rs51_evt         = 0u;

volatile float    g_rs51_meas_ohm    = 0.0f;    /* 拟合出的 R_eff */
volatile float    g_rs51_vdead_v     = 0.0f;    /* 拟合截距（死区等效电压） */
volatile float    g_rs51_r2          = 0.0f;    /* 拟合优度 */
volatile float    g_rs51_ratio       = 0.0f;    /* R_eff / FOC_MOTOR_RS_OHM */
volatile float    g_rs51_v_now_v     = 0.0f;    /* 当前注入电压 */
volatile float    g_rs51_i_now_a     = 0.0f;    /* 当前电流矢量幅值 */
volatile int32_t  g_rs51_moved_cnts  = 0;       /* 全程编码器位移 (counts) */
volatile uint32_t g_rs51_elapsed_ms  = 0u;      /* 累计注入时间 */
volatile uint32_t g_rs51_fs_meas_hz  = 0u;      /* 实测 ISR 频率（零偏窗用 Timer6 计数测得） */
volatile uint32_t g_rs51_pts_done    = 0u;      /* 已完成档位数（供 foc_obs 打印进度） */
volatile float    g_rs51_v_scale     = 1.0f;    /* 运行时电压缩放（电流超限自动下调） */
volatile uint32_t g_rs51_scale_hits  = 0u;      /* 电流护栏触发次数 */

volatile float    g_rs51_pts_v[FOC51_MAX_POINTS];  /* 各档实测电压 (V) */
volatile float    g_rs51_pts_i[FOC51_MAX_POINTS];  /* 各档实测电流 (A) */

/*=============================================================================
 * 内部状态
 *=============================================================================*/
static uint32_t s_tick;                 /* 当前相位计时（tick，相位切换时清零） */
static uint32_t s_ms_tick;              /* 1ms 计时用（与 s_tick 分开，否则相位计时永远到不了） */
static uint32_t s_pt;                   /* 当前档位序号 */
static uint32_t s_npts;                 /* 本轮的档位数 */
static uint32_t s_acc_cnt;              /* 平均窗口采样计数 */
static float    s_acc_i;                /* 平均窗口电流累加 */
static float    s_zero_u, s_zero_v, s_zero_w;   /* 零偏窗测得的零偏 (mA) */
static int32_t  s_acc_zero_u, s_acc_zero_v, s_acc_zero_w;
static uint32_t s_zero_cnt;
static uint32_t s_tb_ticks;             /* 零偏窗内累计的 Timer6 计数差（回绕已处理） */
static uint16_t s_tb_prev;              /* 上一拍的 Timer6 计数 */
static float    s_v_probe;              /* 探测档电压 */
static float    s_r_guess;              /* 探测得到的 R 粗估 */
static int32_t  s_cnt_entry;            /* 进模式时的编码器计数 */
static float    s_i_guard;              /* 护栏用的滤波电流（EMA），防纹波误触发 */

/*=============================================================================
 * 内部助手
 *=============================================================================*/
static void Rs51_ClearResults(void)
{
    uint32_t k;

    g_rs51_meas_ohm   = 0.0f;
    g_rs51_vdead_v    = 0.0f;
    g_rs51_r2         = 0.0f;
    g_rs51_ratio      = 0.0f;
    g_rs51_v_now_v    = 0.0f;
    g_rs51_i_now_a    = 0.0f;
    g_rs51_moved_cnts = 0;
    g_rs51_elapsed_ms = 0u;
    for (k = 0u; k < FOC51_MAX_POINTS; k++) {
        g_rs51_pts_v[k] = 0.0f;
        g_rs51_pts_i[k] = 0.0f;
    }
}

/* 输出固定电压矢量到静止 α 轴（vα = v, vβ = 0） */
static void Rs51_OutputAlpha(float v)
{
    float du, dv, dw;

    Foc_Svpwm(v, 0.0f, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);
    g_foc_valpha = v;
    g_foc_vbeta  = 0.0f;
    g_foc_vd     = v;      /* 静止系下 vd/vq 无意义，仅保持一致 */
    g_foc_vq     = 0.0f;
    g_foc_du     = du;
    g_foc_dv     = dv;
    g_foc_dw     = dw;
}

/* 三相电流 → 静止系（扣本轮零偏窗测得的零偏） */
static void Rs51_Clarke(const stc_i_data_t *pData, float *ia, float *ib, float *ic,
                        float *ialpha, float *ibeta)
{
    float sign = (float)g_foc_cur_sign;

    *ia = ((float)pData->i16IU_mA - s_zero_u) * 0.001f * sign;
    *ib = ((float)pData->i16IV_mA - s_zero_v) * 0.001f * sign;
    *ic = ((float)pData->i16IW_mA - s_zero_w) * 0.001f * sign;
    Foc_Clarke(*ia, *ib, *ic, ialpha, ibeta);
}

static float Rs51_CurrentMag(const stc_i_data_t *pData)
{
    float ia, ib, ic, ialpha, ibeta;

    Rs51_Clarke(pData, &ia, &ib, &ic, &ialpha, &ibeta);
    return sqrtf(ialpha * ialpha + ibeta * ibeta);
}

/* 用累计点做最小二乘：v = R·i + Vdead，并算 R² */
static void Rs51_Fit(void)
{
    float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f, syy = 0.0f;
    float n, den, r, vdead, ss_tot, ss_res, e;
    uint32_t j;

    if (s_npts < 2u) {
        return;
    }
    for (j = 0u; j < s_npts; j++) {
        sx  += g_rs51_pts_i[j];
        sy  += g_rs51_pts_v[j];
        sxx += g_rs51_pts_i[j] * g_rs51_pts_i[j];
        sxy += g_rs51_pts_i[j] * g_rs51_pts_v[j];
        syy += g_rs51_pts_v[j] * g_rs51_pts_v[j];
    }
    n   = (float)s_npts;
    den = n * sxx - sx * sx;
    if (fabsf(den) < 1.0e-9f) {
        return;                          /* 电流几乎没变化 → 拟合无意义 */
    }
    r     = (n * sxy - sx * sy) / den;   /* 斜率 = R_eff */
    vdead = (sy - r * sx) / n;           /* 截距 = V_dead */

    /* R² = 1 − SS_res/SS_tot */
    ss_tot = syy - sy * sy / n;
    ss_res = 0.0f;
    for (j = 0u; j < s_npts; j++) {
        e = g_rs51_pts_v[j] - (r * g_rs51_pts_i[j] + vdead);
        ss_res += e * e;
    }
    g_rs51_meas_ohm = r;
    g_rs51_vdead_v  = vdead;
    g_rs51_r2       = (ss_tot > 1.0e-9f) ? (1.0f - ss_res / ss_tot) : 0.0f;
    g_rs51_ratio    = (FOC_MOTOR_RS_OHM > 0.0f) ? (r / FOC_MOTOR_RS_OHM) : 0.0f;
}

/*=============================================================================
 * Foc_RsId_Start - mode 51 入口（主循环上下文，允许打印）
 *=============================================================================*/
void Foc_RsId_Start(void)
{
    float i_max = g_rs51_i_max_a;

    Foc_Core_ClearFault();
    g_foc_mode        = FOC_MODE_ALIGN;   /* 复用 ALIGN 分发路径（按 g_rs51_running 区分） */
    g_foc_phase       = 4u;
    g_foc_theta_rad   = 0.0f;
    g_foc_id_ma       = 0.0f;
    g_foc_iq_ma       = 0.0f;
    g_foc_vd          = 0.0f;
    g_foc_vq          = 0.0f;

    Foc_Core_SetStateMachine(FOC_STATE_ALIGN);
    Foc_Core_ResetVoltageEnvelope();
    Foc_Core_ResetEma();
    g_foc_align_state = 1u;

    /* 参数护栏（Watch 改坏也不至于打太高） */
    if (i_max > FOC51_I_HARD_MAX_A) {
        i_max = FOC51_I_HARD_MAX_A;
        g_rs51_i_max_a = i_max;
    }
    if (i_max < 0.5f) {
        i_max = 0.5f;
        g_rs51_i_max_a = i_max;
    }
    if (g_rs51_points < 3u)  { g_rs51_points = 3u; }
    if (g_rs51_points > FOC51_MAX_POINTS) { g_rs51_points = FOC51_MAX_POINTS; }
    if (g_rs51_settle_ms < 20u) { g_rs51_settle_ms = 20u; }
    if (g_rs51_i_min_a < 0.1f) { g_rs51_i_min_a = 0.1f; }
    if (g_rs51_i_min_a > i_max) { g_rs51_i_min_a = i_max * 0.5f; }
    if (g_rs51_vdead_prior_v < 0.0f) { g_rs51_vdead_prior_v = 0.0f; }
    if (g_rs51_vdead_prior_v > 0.6f) { g_rs51_vdead_prior_v = 0.6f; }

    Rs51_ClearResults();
    s_tick        = 0u;
    s_ms_tick     = 0u;
    s_pt          = 0u;
    s_npts        = g_rs51_points;
    s_acc_cnt     = 0u;
    s_acc_i       = 0.0f;
    s_acc_zero_u  = 0;
    s_acc_zero_v  = 0;
    s_acc_zero_w  = 0;
    s_zero_cnt    = 0u;
    s_zero_u      = 0.0f;
    s_zero_v      = 0.0f;
    s_zero_w      = 0.0f;
    s_v_probe     = FOC51_PROBE_V;
    s_r_guess     = 0.0f;
    s_i_guard     = 0.0f;
    s_cnt_entry   = (int32_t)TMRA_GetCountValue(CM_TMRA_1);
    g_rs51_v_scale    = 1.0f;
    g_rs51_scale_hits = 0u;

    g_rs51_running = 1u;
    g_rs51_state   = FOC51_STEP_ZERO;
    g_rs51_evt     = 0u;
    g_rs51_pts_done = 0u;

    Foc_Core_PwmStart();   /* 零矢量起 PWM（g_foc_active=1） */

    RS51_DBG("start: pts=%u i=%d..%d mA vdead_prior=%d mV vmax=%d mV",
             (unsigned)s_npts,
             (int)(g_rs51_i_min_a * 1000.0f), (int)(i_max * 1000.0f),
             (int)(g_rs51_vdead_prior_v * 1000.0f),
             (int)(g_rs51_v_hard_max_v * 1000.0f));
}

/*=============================================================================
 * Foc_RsId_Stop - 停止（用户中途切模式时调用）
 *=============================================================================*/
void Foc_RsId_Stop(void)
{
    if (g_rs51_running) {
        g_rs51_running    = 0u;
        g_foc_align_state = 0u;
        if (g_foc_active) {
            g_foc_active = 0u;
            Foc_Core_PwmStop();
            Foc_Core_SetStateMachine(FOC_STATE_IDLE);
        }
        RS51_DBG("stopped");
    }
}

/*=============================================================================
 * Foc_RsId_Step - 步进（20 kHz ISR 中调用，禁止打印）
 *=============================================================================*/
void Foc_RsId_Step(const stc_i_data_t *pData)
{
    float i_now;
    float v_step;

    /* ===== 过流保护（原始 pData，去抖在 Foc_Core_OverCurrent 内） ===== */
    if (Foc_Core_OverCurrent(pData)) {
        g_rs51_state = FOC51_STEP_FAULT_OC;
        g_rs51_evt   = FOC51_EVT_OC;
        /* 保留 running=1：VOFA 停在本模式布局上，便于看清故障时的电流；
         * PWM 已由 FaultStop 关断，ISR 因 g_foc_active=0 不再进来。 */
        Foc_Core_FaultStop(1u);
        return;
    }

    s_tick++;
    s_ms_tick++;
    if (s_ms_tick >= FOC_ISR_HZ / 1000u) {      /* 每 1 ms 更新一次计时观测 */
        s_ms_tick = 0u;
        g_rs51_elapsed_ms++;
    }

    switch (g_rs51_state) {
    /* ---------- 零偏窗：零矢量采平均，扣掉残余零偏 ---------- */
    case FOC51_STEP_ZERO:
        Rs51_OutputAlpha(0.0f);
        if (s_zero_cnt == 0u) {
            s_tb_prev  = (uint16_t)Timer6_Timebase_GetCounter();   /* 零偏窗起点（us 时基） */
            s_tb_ticks = 0u;
        } else {
            uint16_t now_cnt = (uint16_t)Timer6_Timebase_GetCounter();
            s_tb_ticks += (uint32_t)(uint16_t)(now_cnt - s_tb_prev);  /* 16 位相减天然处理回绕 */
            s_tb_prev = now_cnt;
        }
        s_acc_zero_u += (int32_t)pData->i16IU_mA;
        s_acc_zero_v += (int32_t)pData->i16IV_mA;
        s_acc_zero_w += (int32_t)pData->i16IW_mA;
        s_zero_cnt++;
        if (s_zero_cnt >= FOC51_ZERO_SAMPLES) {
            uint32_t freq = Timer6_Timebase_GetFrequency();   /* PCLK0/64 = 3.125MHz -> 0.32us/tick */
            s_zero_u = (float)s_acc_zero_u / (float)s_zero_cnt;
            s_zero_v = (float)s_acc_zero_v / (float)s_zero_cnt;
            s_zero_w = (float)s_acc_zero_w / (float)s_zero_cnt;
            /* 实测 ISR 频率：FOC51_ZERO_SAMPLES 拍共耗多少 Timer6 计数（真实时基，不是推断值）。
             * 本模式所有时长预算都按 FOC_ISR_HZ 换算，此值一旦偏离即说明预算整体失真。
             * 用 GetCounter() 而非 GetTimestamp()：后者的累加只在主循环推进，会被量化到主循环节拍。 */
            if ((freq != 0u) && (s_tb_ticks != 0u)) {
                g_rs51_fs_meas_hz = (uint32_t)((float)FOC51_ZERO_SAMPLES * (float)freq / (float)s_tb_ticks);
            }
            g_rs51_state = FOC51_STEP_PRECOND;   /* 先预置，再探测 */
            s_tick    = 0u;
            s_acc_cnt = 0u;
            s_acc_i   = 0.0f;
            g_rs51_v_now_v = s_v_probe;
        }
        break;

    /* ---------- 预置注入：先给一个电流让转子/夹具稳定 ----------
     * 实测教训：探测档读数曾偏低 15%（1.345A vs 反推 1.58A）、扫描第 0 档
     * 电流偏高 2.5 倍 —— 两者都指向"开机头 0.5s 转子/夹具还在蠕动"。
     * 这里先保持一个固定电流 300ms，把机械状态稳下来再开始测量。 */
    case FOC51_STEP_PRECOND:
        Rs51_OutputAlpha(s_v_probe);
        g_rs51_v_now_v = s_v_probe;
        i_now = Rs51_CurrentMag(pData);
        g_rs51_i_now_a = i_now;
        s_i_guard += FOC51_GUARD_EMA * (i_now - s_i_guard);
        if (s_i_guard > (g_rs51_i_max_a * FOC51_OC_SCALE)) {  /* 滤波后超限 → 降档 */
            s_v_probe *= 0.9f;
            if (s_v_probe < 0.05f) { s_v_probe = 0.05f; }
        }
        if (s_tick >= RS51_MS_TO_TICKS(g_rs51_precond_ms)) {
            g_rs51_state = FOC51_STEP_PROBE;
            s_tick    = 0u;
            s_acc_cnt = 0u;
            s_acc_i   = 0.0f;
        }
        break;

    /* ---------- 探测档：估 R 量级，决定正式档位 ---------- */
    case FOC51_STEP_PROBE:
        Rs51_OutputAlpha(s_v_probe);
        if (s_tick >= RS51_MS_TO_TICKS(g_rs51_settle_ms)) {
            s_acc_i += Rs51_CurrentMag(pData);
            s_acc_cnt++;
            if (s_acc_cnt >= RS51_MS_TO_TICKS(g_rs51_avg_ms)) {
                i_now = s_acc_i / (float)s_acc_cnt;
                g_rs51_i_now_a = i_now;
                if (i_now < FOC51_PROBE_MIN_A) {
                    /* 电流太小 → 升一档再探一次（最多一次） */
                    if (s_v_probe < FOC51_PROBE_V * 2.0f) {
                        s_v_probe *= 2.0f;
                        s_acc_cnt = 0u;
                        s_acc_i   = 0.0f;
                        g_rs51_v_now_v = s_v_probe;
                        break;
                    }
                    /* 仍然太小：可能开路或电流采样异常 */
                    g_rs51_evt   = FOC51_EVT_NO_CURRENT;
                    g_rs51_state = FOC51_STEP_DONE;
                    Rs51_OutputAlpha(0.0f);
                    g_foc_active = 0u;
                    Foc_Core_PwmStop();
                    break;
                }
                /* 关键修正：扣掉死区先验再算 R。
                 * 旧写法 v_probe/i_probe 把死区偏置算进了电阻，
                 * 0.3V/1.345A 得 0.222Ω（真值约 0.1Ω）→ 档位电压算高 2 倍
                 * → 实际电流冲到 5.4A。 */
                s_r_guess = (s_v_probe - g_rs51_vdead_prior_v) / i_now;
                if (s_r_guess < 0.02f) { s_r_guess = 0.02f; }
                if (s_r_guess > 2.0f)  { s_r_guess = 2.0f;  }
                g_rs51_state = FOC51_STEP_SCAN;
                s_pt      = 0u;
                s_tick    = 0u;
                s_acc_cnt = 0u;
                s_acc_i   = 0.0f;
            }
        }
        break;

    /* ---------- 扫描：按目标电流规划档位、逐步注入并平均 ----------
     * 档位目标电流 i_min..i_max 线性分布，电压 = i·R_guess + 死区先验。
     * 这样每一档的"有效驱动电压"（v − Vdead）都 ≥ Vdead，不会出现
     * "87% 电压被死区吃掉"的病态点（旧版最低档只有 21mV 有效驱动）。 */
    case FOC51_STEP_SCAN: {
        float i_target;

        if (s_pt >= s_npts) {
            Rs51_OutputAlpha(0.0f);
            g_rs51_state = FOC51_STEP_FIT;
            break;
        }

        i_target = g_rs51_i_min_a
                 + (g_rs51_i_max_a - g_rs51_i_min_a)
                   * (float)s_pt / (float)(s_npts - 1u);
        v_step = (i_target * s_r_guess + g_rs51_vdead_prior_v) * g_rs51_v_scale;
        if (v_step > g_rs51_v_hard_max_v) { v_step = g_rs51_v_hard_max_v; }
        if (v_step < 0.0f) { v_step = 0.0f; }

        Rs51_OutputAlpha(v_step);
        g_rs51_v_now_v = v_step;
        i_now = Rs51_CurrentMag(pData);
        /* 护栏必须用"滤波后"的电流：瞬时值含开关纹波与采样噪声（本工况下
         * 均值 3.09A 时纹波尖峰就能顶到 3.45A 阈值），实测曾误触发 7 次、
         * 把后两档电压砍掉 8~13%。EMA α=0.05 → τ≈1ms，20kHz 纹波衰减百倍
         * 以上，而真正的持续过流仍在几个 ms 内判出。 */
        s_i_guard += FOC51_GUARD_EMA * (i_now - s_i_guard);

        if (s_tick < RS51_MS_TO_TICKS(g_rs51_settle_ms)) {
            /* 稳定期内的电流护栏：R_guess 万一还不准，也不许电流失控。
             * 超限 → 按比例下调缩放系数，本档重测（s_tick 清零）。 */
            if ((s_tick > RS51_MS_TO_TICKS(20u))
                && (s_i_guard > (g_rs51_i_max_a * FOC51_OC_SCALE))) {
                g_rs51_v_scale *= (g_rs51_i_max_a * FOC51_OC_SCALE) / s_i_guard;
                if (g_rs51_v_scale < 0.05f) { g_rs51_v_scale = 0.05f; }
                g_rs51_scale_hits++;
                s_i_guard = 0.0f;      /* 重测本档，滤波值一并清零 */
                s_tick    = 0u;
                s_acc_cnt = 0u;
                s_acc_i   = 0.0f;
            }
            break;                                  /* 等稳定 */
        }
        s_acc_i += i_now;
        s_acc_cnt++;
        if (s_acc_cnt >= RS51_MS_TO_TICKS(g_rs51_avg_ms)) {
            g_rs51_pts_v[s_pt] = v_step;
            g_rs51_pts_i[s_pt] = s_acc_i / (float)s_acc_cnt;
            g_rs51_i_now_a     = g_rs51_pts_i[s_pt];
            s_pt++;
            g_rs51_pts_done = s_pt;      /* 供 foc_obs 打印逐档进度 */
            s_tick    = 0u;
            s_acc_cnt = 0u;
            s_acc_i   = 0.0f;
        }
        break;
    }

    /* ---------- 拟合 + 收尾 ---------- */
    case FOC51_STEP_FIT:
        Rs51_Fit();
        g_rs51_moved_cnts = (int32_t)TMRA_GetCountValue(CM_TMRA_1) - s_cnt_entry;
        g_rs51_evt        = (g_rs51_r2 >= 0.99f) ? FOC51_EVT_FIT_OK : FOC51_EVT_FIT_POOR;
        g_rs51_state      = FOC51_STEP_DONE;
        /* 注意：**不清 g_rs51_running** —— 让 VOFA 停在本模式的 12ch 布局上，
         * 便于回看 v-i 直线与拟合结果；切模式时 CommRunner_StopFocModes()
         * 会调 Foc_RsId_Stop() 收尾。 */
        g_foc_active      = 0u;
        Foc_Core_PwmStop();                         /* 不再保持注入，避免发热 */
        Foc_Core_SetStateMachine(FOC_STATE_IDLE);
        /* 注：ISR 内不打印（项目规范）。结果全部在 Watch / VOFA 里读。 */
        break;

    case FOC51_STEP_DONE:
    case FOC51_STEP_FAULT_OC:
    case FOC51_STEP_IDLE:
    default:
        Rs51_OutputAlpha(0.0f);
        break;
    }
}

/*===========================================================================
 * 模式自持 VOFA：固定 12ch 布局，通道含义见 foc_51_rs.h 顶部速览卡
 *（唯一事实源）。单位换算：传"毫单位"，SendScaled 内部 ×0.001。
 *===========================================================================*/
int Foc_RsId_VofaFill(int32_t *cur)
{
    cur[0]  = (int32_t)(g_rs51_state);                 /* ch0 状态码 */
    cur[1]  = (int32_t)(g_rs51_v_now_v * 1000.0f);     /* ch1 注入电压 (mV -> V) */
    cur[2]  = (int32_t)(g_rs51_i_now_a * 1000.0f);     /* ch2 电流矢量幅值 (mA -> A) */
    cur[3]  = (int32_t)(g_rs51_meas_ohm * 1000.0f);    /* ch3 R_eff 估计 (mohm -> ohm) */
    cur[4]  = (int32_t)(g_rs51_vdead_v * 1000.0f);     /* ch4 V_dead 估计 (mV -> V) */
    cur[5]  = (int32_t)(g_rs51_r2 * 1000.0f);          /* ch5 R2 (x1000) */
    cur[6]  = (int32_t)(s_pt);                         /* ch6 已完成档位数 */
    cur[7]  = (int32_t)(g_i_iu_ma);                    /* ch7 U 相电流 (mA -> A) */
    cur[8]  = (int32_t)(g_i_iv_ma);                    /* ch8 V 相电流 */
    cur[9]  = (int32_t)(g_i_iw_ma);                    /* ch9 W 相电流 */
    cur[10] = (int32_t)(g_rs51_moved_cnts);            /* ch10 编码器位移 (counts) */
    cur[11] = (int32_t)(g_rs51_elapsed_ms);            /* ch11 累计注入时间 (ms) */
    return 12;
}
