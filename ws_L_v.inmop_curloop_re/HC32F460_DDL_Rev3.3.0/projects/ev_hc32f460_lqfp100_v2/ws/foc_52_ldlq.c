/**
 *******************************************************************************
 * @file  foc_52_ldlq.c
 * @brief FOC mode 52 - d/q 轴电感辨识（直流偏置方波注入 + 单极性电流纹波）。
 *
 * 原理与判据见 foc_52_ldlq.h 顶部速览卡（唯一事实源）。
 * 关键实现点：
 *   1) 命令 v_axis = V_bias·bias_sign + Vsq·sgn，其中 V_bias = R·I_bias + V_dead先验。
 *      被测轴电流因此**始终同号**（|I_bias| > 纹波半幅），死区等效电压在两极性
 *      平台上同号 => 在平台内采样差里与 R·i 项一起抵消。
 *   2) 半周期 = FOC52_HALF_ISRS = **2 个采样周期**（平台 100 µs）。不能取 1 拍：
 *      ADC 谷点采样 + 占空比峰点装载相差 25 µs，1 拍时采样点恰好落在平台正中
 *      （电流三角波中点），采样差退化成约 0。
 *   3) 一个平台内采 2 点（ISR 序号 0 与 1），第 2 点与第 1 点相减得到 di；
 *      |di| = Vsq·(1/fs)/L（仿真残差 <1%，见 _cardgen/sim_mode52_uni.mjs）。
 *   4) 偏置每 FOC52_BIAS_CYCLES 个周期翻号（净冲量抵消，防转子被推走），
 *      翻号后跳过 FOC52_BIAS_SKIP 个平台的统计（等电流建立）。
 *   5) 电流用扣过零偏的副本送 Foc_Core_GetDq（不能直接用 pData）。
 *******************************************************************************
 */

#include "foc_52_ldlq.h"
#include "foc_24_dcal.h"       /* Foc_Dcal_GetResult */
#include "I.h"                 /* FOC_ISR_HZ */
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "hc32_ll_tmra.h"
#include "TickTimer.h"         /* tickTimer_GetCount：独立时基，实测 ISR 频率 */
#include <math.h>

/*=============================================================================
 * 时长换算
 *=============================================================================*/
#define LDLQ52_MS_TO_TICKS(ms)   ((uint32_t)(ms) * FOC_ISR_HZ / 1000u)

/*=============================================================================
 * Keil Watch 可调参数
 *=============================================================================*/
volatile float    g_ldlq52_di_target_a = 1.0f;     /* 目标纹波峰峰值 (A) */
volatile float    g_ldlq52_bias_a      = 0.7f;     /* 直流偏置电流 (A) */
volatile float    g_ldlq52_v_min_v     = 0.3f;     /* 自适应方波幅值下限 (V) */
volatile float    g_ldlq52_v_max_v     = 3.0f;     /* 自适应方波幅值上限 (V) */
volatile uint32_t g_ldlq52_cycles      = 32u;      /* 正式测量周期数 */
volatile uint32_t g_ldlq52_do_q        = 1u;       /* 是否测 q 轴 */
volatile uint32_t g_ldlq52_wave_en     = 0u;       /* 1 = 抓取并打印原始波形（排查用） */
volatile float    g_ldlq52_r_used_ohm  = 0.098f;   /* 反推 V_dead 用的 R（mode 51 实测） */

/*=============================================================================
 * 观测量（Watch / VOFA）
 *=============================================================================*/
volatile uint8_t  g_ldlq52_running    = 0u;
volatile uint8_t  g_ldlq52_state      = FOC52_STEP_IDLE;
volatile uint8_t  g_ldlq52_evt        = 0u;
volatile uint8_t  g_ldlq52_axis       = 0u;
volatile float    g_ldlq52_ld_uh      = 0.0f;
volatile float    g_ldlq52_lq_uh      = 0.0f;
volatile float    g_ldlq52_ratio      = 0.0f;
volatile float    g_ldlq52_ratio_vs_vendor = 0.0f;
volatile float    g_ldlq52_di_pp_a    = 0.0f;
volatile float    g_ldlq52_v_inj_v    = 0.0f;
volatile float    g_ldlq52_i_bias_ma  = 0.0f;
volatile float    g_ldlq52_vdead_v    = 0.0f;
volatile float    g_ldlq52_d_scatter_pct = 0.0f;
volatile uint32_t g_ldlq52_dcnt       = 0u;
volatile float    g_ldlq52_cons_pct   = 0.0f;
volatile float    g_ldlq52_phase_ratio = 0.0f;
volatile uint8_t  g_ldlq52_probe_seq  = 0u;
volatile uint8_t  g_ldlq52_ld_done_seq = 0u;   /* d 轴测量完成次数：do_q=0/1 都据此打 d done 行 */
volatile float    g_ldlq52_vbias_v    = 0.0f;
volatile float    g_ldlq52_l_probe_uh = 0.0f;
volatile float    g_ldlq52_l_env_uh   = 0.0f;   /* 平台均值法反推的电感 (uH) —— 最终结果用这个 */
volatile float    g_ldlq52_l_pair_uh  = 0.0f;   /* 单点差分法反推的电感 (uH) —— 诊断对照用 */
volatile float    g_ldlq52_de_ma      = 0.0f;   /* 相邻平台均值之差 (mA) */
volatile float    g_ldlq52_dp_ma      = 0.0f;   /* 平台内两点之差 (mA) */
volatile float    g_ldlq52_de_cons_pct = 0.0f;  /* 平台均值差的符号一致性 (%) */
volatile uint32_t g_ldlq52_fs_meas_hz = 0u;
volatile uint32_t g_ldlq52_path_cnts  = 0u;
/* 波形抓取缓冲（排查采样结构用；只在 ISR 里写，主循环里读/打印） */
volatile int16_t  g_ldlq52_wave[FOC52_WAVE_N];
volatile int8_t   g_ldlq52_wave_sgn[FOC52_WAVE_N];
volatile uint8_t  g_ldlq52_wave_n   = 0u;
volatile uint8_t  g_ldlq52_wave_arm = 0u;
volatile uint8_t  g_ldlq52_q_skipped = 0u;
volatile uint16_t g_ldlq52_v_probe_mv = 0u;   /* 本次探测真正用过的幅值 (mV) */
volatile uint8_t  g_ldlq52_wave_seq = 0u;
volatile int32_t  g_ldlq52_moved_d_cnts = 0;
volatile int32_t  g_ldlq52_moved_q_cnts = 0;
volatile int32_t  g_ldlq52_moved_cnts = 0;
volatile uint32_t g_ldlq52_elapsed_ms = 0u;

/*=============================================================================
 * 内部状态
 *=============================================================================*/
static uint32_t s_tick;
static uint32_t s_ms_tick;
static uint32_t s_hn;                 /* 本相位已过的半周期（平台）数 */
static uint32_t s_cyc;                /* 本相位已过的整周期数 */
static uint32_t s_htick;              /* 平台内已过的采样周期数（到 FOC52_HALF_ISRS 翻转） */
static int8_t   s_sign;               /* 当前平台的方波符号 */
static int8_t   s_bias_sign;          /* 当前直流偏置符号 */
static uint32_t s_skip;               /* 偏置翻号后剩余跳过的平台数 */
static float    s_v_amp;              /* 当前方波幅值 Vsq */
static float    s_v_bias;             /* 自适应后的偏置电压幅值 */
static float    s_i_prev;             /* 上一个采样 */
static uint8_t  s_have_prev;          /* s_i_prev 是否有效 */
static uint8_t  s_have_pair;          /* 是否已有可用的上一对（首对只用来播种） */
/* 平台 = 2 个采样周期 => 每个平台内有**两对**相邻采样。翻转是按 ISR 次数计的，
 * 一旦某次写入错过峰点影子装载，采样对与平台边界的相位就永久错开一拍：此时两对里
 * 必有一对**跨过三角波顶点**，它的 |di| 被抵消掉一部分（实测均值只剩 0.72 倍、
 * 离散度 52%）。而干净的那一对必然是两者中**较大**的那个 —— 所以每个平台取
 * max(|di_k|, |di_k-1|) 即可，与相位无关（10ns 步长仿真：任意相位下误差 0.00%、
 * 离散度 0.00%；对照"线性闸门"方案会把顶点两侧的对全误杀，有效样本归零）。
 * 平台内 max/min 之比另作相位诊断：100 = 相位锁定，越大说明错开越多。 */
static float    s_dsum, s_dsq, s_isum, s_minsum;
static uint32_t s_dcnt, s_icnt, s_mincnt, s_cons;
static float    s_prev_mag;           /* 上一对的 |di| */
static float    s_prev_diff;          /* 上一对的带符号差值 */
static int8_t   s_prev_sign;          /* 上一拍采样时的平台极性 */
static int8_t   s_prev_sign2;         /* 上上拍采样时的平台极性 */
/* 平台均值法（最终结果用）：平台长度 = FOC52_HALF_ISRS 个采样周期，平台内取两点平均，
 * 相邻平台均值之差 |dE| 与 Vsq 成正比：L = Vsq·(平台时长)/|dE|。
 * 为什么不用单点差分：2026-09-27 的抓取波形显示，采样序列里混着约 ±0.5A 的快速
 * 交替干扰（10 kHz 量级，靠近奈奎斯特），单点差分被它污染、且"跨平台边界那一跳"
 * 比平台内斜坡大 ~2 倍，导致 L 被系统性压掉一半（实测 20uH vs 平台均值法 34~37uH）。
 * 取两点平均正好把交替干扰抵消（相邻两点符号相反），因此平台均值法结果稳定。 */
static float    s_desum, s_dpsum;     /* 平台均值差 / 平台内两点差 的累加 */
static float    s_gmean_prev;         /* 上一个平台的均值 */
static float    s_gsum;               /* 当前平台正在累加的和 */
static uint8_t  s_gn;                 /* 当前平台已累加的点数 */
static uint8_t  s_have_gmean;
static uint32_t s_decnt, s_dpcnt, s_decons;
static int8_t   s_gmean_sign;         /* 上一平台极性 */
static int8_t   s_gsum_sign;
static int32_t  s_enc_prev;           /* 上一拍编码器计数（算路径） */
static int32_t  s_enc_axis;           /* 本轴正式测量开始时的编码器计数 */
static uint32_t s_path;               /* 本轴测量窗内路径累计 |dcount| */
static float    s_ld, s_lq;           /* 已测得电感 (H) */
static float    s_zero_u, s_zero_v, s_zero_w;
static int32_t  s_acc_zero_u, s_acc_zero_v, s_acc_zero_w;
static uint32_t s_zero_cnt;
static int32_t  s_cnt_entry;
static uint64_t s_tick0;              /* 零偏窗起点的独立时基读数 (ms)，用于实测 ISR 频率 */
static int32_t  s_offset;             /* mode 24 锁定的编码器零点 */

/*=============================================================================
 * 内部助手
 *=============================================================================*/
static void LdLq52_ClearPhaseStats(void)
{
    s_dsum    = 0.0f;
    s_dsq     = 0.0f;
    s_isum    = 0.0f;
    s_minsum  = 0.0f;
    s_dcnt    = 0u;
    s_icnt    = 0u;
    s_mincnt  = 0u;
    s_cons    = 0u;
    s_prev_mag  = 0.0f;
    s_prev_sign = 1;
    s_prev_sign2 = 1;
    s_have_pair = 0u;
    s_gmean_prev = 0.0f;
    s_gsum       = 0.0f;
    s_gn         = 0u;
    s_have_gmean = 0u;
    s_gmean_sign = 1;
    s_gsum_sign  = 1;
    s_desum      = 0.0f;
    s_dpsum      = 0.0f;
    s_decnt      = 0u;
    s_dpcnt      = 0u;
    s_decons     = 0u;
    s_path     = 0u;
    s_have_prev = 0u;
}

static void LdLq52_ClearResults(void)
{
    g_ldlq52_ld_uh    = 0.0f;
    g_ldlq52_lq_uh    = 0.0f;
    g_ldlq52_ratio    = 0.0f;
    g_ldlq52_ratio_vs_vendor = 0.0f;
    g_ldlq52_di_pp_a  = 0.0f;
    g_ldlq52_v_inj_v  = 0.0f;
    g_ldlq52_i_bias_ma = 0.0f;
    g_ldlq52_vdead_v  = 0.0f;
    g_ldlq52_d_scatter_pct = 0.0f;
    g_ldlq52_dcnt     = 0u;
    g_ldlq52_cons_pct = 0.0f;
    g_ldlq52_phase_ratio = 0.0f;
    g_ldlq52_l_probe_uh = 0.0f;
    g_ldlq52_wave_n   = 0u;
    g_ldlq52_wave_seq = 0u;
    g_ldlq52_q_skipped = 0u;
    g_ldlq52_path_cnts = 0u;
    g_ldlq52_moved_d_cnts = 0;
    g_ldlq52_moved_q_cnts = 0;
    g_ldlq52_moved_cnts = 0;
    g_ldlq52_elapsed_ms = 0u;
}

/* 本拍转子电角度（编码器计数 - mode 24 零点） */
static float LdLq52_RotorAngle(void)
{
    uint16_t hw = TMRA_GetCountValue(CM_TMRA_1);
    int32_t cnt = Foc_Core_ModPos((int32_t)hw * (int32_t)g_foc_enc_dir - s_offset,
                                  (int32_t)ENCODER_CPR);

    return (float)cnt * (FOC_MATH_2PI / (float)ENCODER_CPR) * (float)FOC_POLE_PAIRS;
}

/* 按转子坐标系 (vd, vq) 输出 */
static void LdLq52_OutputDq(float vd, float vq, float theta)
{
    float c = Foc_Math_Cos(theta);
    float s = Foc_Math_Sin(theta);
    float valpha = vd * c - vq * s;
    float vbeta  = vd * s + vq * c;
    float du, dv, dw;

    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);
    g_foc_valpha    = valpha;
    g_foc_vbeta     = vbeta;
    g_foc_vd        = vd;
    g_foc_vq        = vq;
    g_foc_du        = du;
    g_foc_dv        = dv;
    g_foc_dw        = dw;
    g_foc_theta_rad = theta;
}

/* 本拍被测轴电流：用扣过零偏的副本送 GetDq（关键：不能直接用 pData） */
static float LdLq52_AxisCurrent(const stc_i_data_t *pData, float theta)
{
    stc_i_data_t d = *pData;
    float id, iq;

    d.i16IU_mA = (int16_t)((float)pData->i16IU_mA - s_zero_u);
    d.i16IV_mA = (int16_t)((float)pData->i16IV_mA - s_zero_v);
    d.i16IW_mA = (int16_t)((float)pData->i16IW_mA - s_zero_w);
    Foc_Core_GetDq(&d, theta, &id, &iq);
    return (g_ldlq52_axis == 0u) ? id : iq;
}

/* 输出当前平台的 (偏置 + 方波) 电压 */
static void LdLq52_OutputAxis(float theta)
{
    float v = s_v_bias * (float)s_bias_sign + s_v_amp * (float)s_sign;

    if (g_ldlq52_axis == 0u) {
        LdLq52_OutputDq(v, 0.0f, theta);
    } else {
        LdLq52_OutputDq(0.0f, v, theta);
    }
}

/* 一个 ISR 拍：读电流 -> （平台第 2 点）统计 |di| -> 翻转/换偏置 -> 输出。
 * 返回 1 = 本相位已完成。
 *
 * 采样与装载的相位关系（本函数成立的前提）：
 *   谷点采样 -> 本拍读到的电流对应"上一次峰点装载"的那段平台；
 *   峰点装载 -> 本拍写的符号要到 25 µs 后才生效。
 * 因此：读到的电流属于当前 s_sign/s_bias_sign 的平台，统计要在翻转之前做。
 */
static uint8_t LdLq52_SquareStep(const stc_i_data_t *pData, uint32_t cycles)
{
    float theta  = LdLq52_RotorAngle();
    float i_axis = LdLq52_AxisCurrent(pData, theta);
    uint8_t in_meas = (s_cyc >= FOC52_SETTLE_CYCLES) ? 1u : 0u;
    int32_t cnt_now = (int32_t)TMRA_GetCountValue(CM_TMRA_1);

    /* 波形抓取（诊断用）：抓一窗连续采样，主循环打印出来即可看清采样结构 */
    if (g_ldlq52_wave_arm != 0u) {
        if (in_meas != 0u) {
            if (g_ldlq52_wave_n < (uint8_t)FOC52_WAVE_N) {
                g_ldlq52_wave[g_ldlq52_wave_n]     = (int16_t)(i_axis * 1000.0f);
                g_ldlq52_wave_sgn[g_ldlq52_wave_n] = s_sign;
                g_ldlq52_wave_n++;
                if (g_ldlq52_wave_n >= (uint8_t)FOC52_WAVE_N) {
                    g_ldlq52_wave_arm = 0u;        /* 记满自动解除请求 */
                    g_ldlq52_wave_seq++;           /* 序号 +1：foc_obs 据此打印一次 */
                }
            }
        }
    }

    /* 编码器路径：净位移会回绕（65330 实际是 -206），路径累计才是"转子是否在动"的判据 */
    if (in_meas != 0u) {
        int32_t dc = cnt_now - s_enc_prev;
        s_path += (uint32_t)((dc >= 0) ? dc : -dc);
    }
    s_enc_prev = cnt_now;

    if (in_meas != 0u) {
        if (s_skip != 0u) {
            s_skip--;                             /* 偏置翻号后等电流建立 */
            s_gn = 0u; s_gsum = 0.0f; s_have_gmean = 0u;
        } else {
            uint8_t is_second = (s_gn == 1u) ? 1u : 0u;   /* 本组第 2 点 */

            /* ---- 平台均值法（结果用）---- */
            if (s_gn == 0u) { s_gsum_sign = s_sign; }
            s_gsum += i_axis;
            s_gn++;
            if (s_gn >= (uint8_t)FOC52_HALF_ISRS) {
                float gmean = s_gsum / (float)s_gn;

                if (s_have_gmean != 0u) {
                    float de = gmean - s_gmean_prev;

                    s_desum += fabsf(de);
                    s_decnt++;
                    if ((de * (float)s_gmean_sign) < 0.0f) { s_decons++; }
                }
                s_gmean_prev = gmean;
                s_gmean_sign = s_gsum_sign;
                s_have_gmean = 1u;
                s_gn = 0u; s_gsum = 0.0f;
            }

            /* ---- 单点差分法（诊断对照）---- */
            if (s_have_prev != 0u) {
                float d = i_axis - s_i_prev;
                float m = fabsf(d);

                if (is_second != 0u) {            /* 同一平台两点之差 */
                    s_dpsum += m;
                    s_dpcnt++;
                    s_isum  += (float)s_bias_sign * i_axis;
                    s_icnt++;
                }
                if (s_have_pair != 0u) {          /* 相邻两对取 max（旧算法，仅对照） */
                    float use = (m > s_prev_mag) ? m : s_prev_mag;
                    float low = (m > s_prev_mag) ? s_prev_mag : m;
                    float du  = (m > s_prev_mag) ? d : s_prev_diff;
                    int8_t sgn = (m > s_prev_mag) ? s_prev_sign : s_prev_sign2;

                    s_dsum   += use;
                    s_dsq    += use * use;
                    s_minsum += low;
                    s_dcnt++;
                    s_mincnt++;
                    if ((du * (float)sgn) > 0.0f) { s_cons++; }
                }
                s_prev_diff  = d;
                s_prev_mag   = m;
                s_prev_sign2 = s_prev_sign;
                s_prev_sign  = s_sign;
                s_have_pair  = 1u;
            }
        }
    }
    s_i_prev = i_axis;
    if (s_have_prev == 0u) { s_have_prev = 1u; }

    /* 半周期翻转（每 FOC52_HALF_ISRS 拍一次），并在整周期边界考虑偏置翻号 */
    s_htick++;
    if (s_htick >= FOC52_HALF_ISRS) {
        s_htick = 0u;
        s_sign  = (int8_t)(-s_sign);
        s_hn++;
        if ((s_hn & 1u) == 0u) {                 /* 一个整周期结束 */
            s_cyc++;
            if (s_cyc >= FOC52_SETTLE_CYCLES) {
                uint32_t m = s_cyc - FOC52_SETTLE_CYCLES;
                if ((m % FOC52_BIAS_CYCLES) == 0u) {
                    if (m != 0u) {
                        s_bias_sign = (int8_t)(-s_bias_sign);
                    }
                    s_skip = FOC52_BIAS_SKIP;    /* 等电流建立/过零 */
                }
            }
        }
    }

    LdLq52_OutputAxis(theta);

    return (s_cyc >= (FOC52_SETTLE_CYCLES + cycles)) ? 1u : 0u;
}

/* 相位开始：清计数与统计（偏置方向与符号都回到 +） */
static void LdLq52_PhaseReset(void)
{
    s_hn        = 0u;
    s_cyc       = 0u;
    s_htick     = 0u;
    s_sign      = 1;
    s_bias_sign = 1;
    s_skip      = 0u;
    s_i_prev    = 0.0f;
    LdLq52_ClearPhaseStats();
}

/* 汇总本相位统计：
 *   d_mean  = 平台均值法给出的"相邻平台均值之差"（V）→ 结果用它算 L
 *   i_bias  = 实测偏置电流
 *   cons    = 旧算法的一致性（诊断）
 * 同时把两种算法反推的电感都算出来落全局，RTT 直接打印。 */
static uint8_t LdLq52_Stats(float *d_mean_out, float *i_bias_out, float *cons_out)
{
    const float t_s   = 1.0f / (float)FOC_ISR_HZ;              /* 采样周期 (s) */
    const float t_plt = t_s * (float)FOC52_HALF_ISRS;          /* 平台时长 (s) */
    float de, dp;

    if ((s_decnt < 4u) || (s_dpcnt < 4u)) { return 0u; }

    de = s_desum / (float)s_decnt;
    dp = s_dpsum / (float)s_dpcnt;

    *d_mean_out = de;
    *i_bias_out = (s_icnt != 0u) ? (s_isum / (float)s_icnt) : 0.0f;
    *cons_out   = 100.0f * (float)s_cons / (float)((s_dcnt != 0u) ? s_dcnt : 1u);

    g_ldlq52_de_ma      = de * 1000.0f;
    g_ldlq52_dp_ma      = dp * 1000.0f;
    g_ldlq52_de_cons_pct = 100.0f * (float)s_decons / (float)((s_decnt != 0u) ? s_decnt : 1u);
    /* 结果：L = Vsq · 平台时长 / 平台均值差 */
    g_ldlq52_l_env_uh   = (de > 1.0e-6f)
                        ? (s_v_amp * t_plt / de) * 1.0e6f : 0.0f;
    /* 诊断：单点差分法 L = Vsq · 采样周期 / 平台内两点差 */
    g_ldlq52_l_pair_uh  = (dp > 1.0e-6f)
                        ? (s_v_amp * t_s / dp) * 1.0e6f : 0.0f;
    g_ldlq52_cons_pct   = *cons_out;
    if (s_mincnt != 0u) {
        float lo = s_minsum / (float)s_mincnt;
        g_ldlq52_phase_ratio = (lo > 1.0e-6f) ? (100.0f * (s_dsum / (float)s_dcnt) / lo) : 9999.0f;
    } else {
        g_ldlq52_phase_ratio = 0.0f;
    }
    return 1u;
}

/* 一次测量收尾：汇总统计 -> 算 L、V_dead；返回 1 = 数据可用 */
static uint8_t LdLq52_FinishMeasure(void)
{
    float d_mean = 0.0f, i_bias = 0.0f, var, l_h;
    float cons_pct = 0.0f;
    uint8_t ok;

    ok = LdLq52_Stats(&d_mean, &i_bias, &cons_pct);
    if (ok == 0u) {
        g_ldlq52_evt = FOC52_EVT_NO_RIPPLE;
        return 0u;
    }

    /* 先记过程量（无论最终是否判无效，都能在 Watch/RTT 里看到） */
    g_ldlq52_dcnt        = s_decnt;
    g_ldlq52_i_bias_ma   = fabsf(i_bias) * 1000.0f;
    g_ldlq52_path_cnts   = s_path;
    g_ldlq52_di_pp_a     = 2.0f * d_mean;
    {   /* 净位移按 16 位计数器解回绕（65330 实际是 -206）；路径另有累计，不怕超一圈 */
        int32_t net = (int32_t)(int16_t)((int32_t)TMRA_GetCountValue(CM_TMRA_1) - s_enc_axis);

        if (g_ldlq52_axis == 0u) { g_ldlq52_moved_d_cnts = net; }
        else                     { g_ldlq52_moved_q_cnts = net; }
    }

    if (d_mean < 1.0e-5f) {
        g_ldlq52_evt = FOC52_EVT_NO_RIPPLE;
        return 0u;
    }
    /* 有效性 1 已合并进 Stats：dE 与 dp 都可算即视为有效。
     *（原来这里有一道 de_cons 闸门，区间写反、永不触发，已删除；de_cons 只作为诊断打印。） */
    /* 有效性 2：单极性 —— 偏置电流必须大于纹波半幅（= dE/2），否则电流过零、死区误差不再抵消 */
    if (fabsf(i_bias) <= (d_mean * 0.5f)) {
        g_ldlq52_evt = FOC52_EVT_BIAS_FAIL;
        return 0u;
    }
    /* 有效性 3：测量窗内转子必须基本不动（动了就有 BEMF/交叉耦合污染） */
    if (s_path > FOC52_PATH_BAD_CNTS) {
        g_ldlq52_evt = FOC52_EVT_MOTION;
        return 0u;
    }

    /* 核心公式（平台均值法）：|dE| = Vsq·平台时长/L */
    l_h = s_v_amp * (1.0f / (float)FOC_ISR_HZ) * (float)FOC52_HALF_ISRS / d_mean;

    /* 质量指标：各平台 |dE| 的相对标准差（用旧累加器的平方和近似由 dp 提供不了，
     * 这里以平台内两点差的离散度作参考量） */
    var = 0.0f;
    (void)var;
    g_ldlq52_d_scatter_pct = (g_ldlq52_dp_ma > 1.0f)
                           ? (100.0f * g_ldlq52_de_ma / g_ldlq52_dp_ma - 100.0f) : 0.0f;
    if (g_ldlq52_d_scatter_pct < 0.0f) { g_ldlq52_d_scatter_pct = 0.0f; }

    /* 稳态下 v_axis 的平均值 = V_bias - V_dead（死区顶住电流），
     * 且该平均电压 = R·I_bias ⇒ V_dead = V_bias - R·I_bias（取幅值） */
    g_ldlq52_vdead_v    = s_v_bias - g_ldlq52_r_used_ohm * fabsf(i_bias);
    if (g_ldlq52_vdead_v < 0.0f) { g_ldlq52_vdead_v = 0.0f; }

    if (g_ldlq52_axis == 0u) {
        s_ld = l_h;
        g_ldlq52_ld_uh = l_h * 1.0e6f;
        if (FOC_MOTOR_LS_UH > 0.0f) {
            g_ldlq52_ratio_vs_vendor = g_ldlq52_ld_uh / FOC_MOTOR_LS_UH;
        }
    } else {
        s_lq = l_h;
        g_ldlq52_lq_uh = l_h * 1.0e6f;
        if (s_ld > 1.0e-9f) {
            g_ldlq52_ratio = s_lq / s_ld;
        }
    }
    return 1u;
}

/* 收尾：记录位移、补完成事件、关 PWM、回到 IDLE（保留 running=1 便于 VOFA 回看，
 * 切模式时由 Foc_LdLqId_Stop 清掉）。
 * 调用上下文：由 Foc_LdLqId_Step（20 kHz ISR）在测量结束/探测失败时调用 ——
 * 与 mode 51 的收尾同一套路（那边已在硬件上验证）。ISR 安全：只写标志位与
 * PWM/状态机寄存器，不打印、不阻塞、不做浮点除法。 */
static void LdLq52_Finish(void)
{
    LdLq52_OutputDq(0.0f, 0.0f, LdLq52_RotorAngle());
    g_ldlq52_moved_cnts = (int32_t)(int16_t)((int32_t)TMRA_GetCountValue(CM_TMRA_1) - s_cnt_entry);
    if (g_ldlq52_evt == 0u) { g_ldlq52_evt = FOC52_EVT_DONE; }
    g_ldlq52_state = FOC52_STEP_IDLE;
    g_foc_active   = 0u;
    Foc_Core_PwmStop();
    Foc_Core_SetStateMachine(FOC_STATE_IDLE);
}

/*=============================================================================
 * Foc_LdLqId_Start
 *=============================================================================*/
void Foc_LdLqId_Start(void)
{
    foc_dcal24_result_t cal;
    float d_target, bias_min;

    Foc_Core_ClearFault();
    g_foc_mode        = FOC_MODE_ALIGN;   /* 复用 ALIGN 分发路径（按 g_ldlq52_running 区分） */
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

    /* ---- 前置：必须有 mode 24 的零点 ---- */
    if (Foc_Dcal_GetResult(&cal) == 0u) {
        g_ldlq52_running = 0u;
        g_ldlq52_state   = FOC52_STEP_IDLE;
        LDLQ52_DBG("ERROR: no mode 24 calibration; run mode 24 first");
        return;
    }
    s_offset = cal.offset;

    /* ---- 参数护栏 ---- */
    if (g_ldlq52_di_target_a < 0.1f) { g_ldlq52_di_target_a = 0.1f; }
    if (g_ldlq52_di_target_a > FOC52_DI_HARD_MAX_A) { g_ldlq52_di_target_a = FOC52_DI_HARD_MAX_A; }
    d_target = 0.5f * g_ldlq52_di_target_a;            /* |di| 目标 = 纹波半幅 */
    bias_min = d_target * FOC52_BIAS_MARGIN + 0.05f;
    if (g_ldlq52_bias_a < bias_min) { g_ldlq52_bias_a = bias_min; }
    if (g_ldlq52_bias_a > FOC52_DI_HARD_MAX_A) { g_ldlq52_bias_a = FOC52_DI_HARD_MAX_A; }
    if (g_ldlq52_r_used_ohm < 0.001f) { g_ldlq52_r_used_ohm = 0.001f; }
    if (g_ldlq52_v_min_v < 0.05f) { g_ldlq52_v_min_v = 0.05f; }
    if (g_ldlq52_v_max_v > FOC52_V_HARD_MAX_V) { g_ldlq52_v_max_v = FOC52_V_HARD_MAX_V; }
    if (g_ldlq52_v_max_v < g_ldlq52_v_min_v) { g_ldlq52_v_max_v = g_ldlq52_v_min_v; }
    /* 周期数取整到偏置周期的整数倍，保证 ± 偏置两个子窗等长 */
    if (g_ldlq52_cycles < FOC52_BIAS_CYCLES) { g_ldlq52_cycles = FOC52_BIAS_CYCLES; }
    if (g_ldlq52_cycles > (16u * FOC52_BIAS_CYCLES)) { g_ldlq52_cycles = 16u * FOC52_BIAS_CYCLES; }
    g_ldlq52_cycles = ((g_ldlq52_cycles + FOC52_BIAS_CYCLES / 2u) / FOC52_BIAS_CYCLES)
                      * FOC52_BIAS_CYCLES;

    LdLq52_ClearResults();
    s_tick       = 0u;
    s_ms_tick    = 0u;
    s_zero_u     = 0.0f;
    s_zero_v     = 0.0f;
    s_zero_w     = 0.0f;
    s_acc_zero_u = 0;
    s_acc_zero_v = 0;
    s_acc_zero_w = 0;
    s_zero_cnt   = 0u;
    s_ld         = 0.0f;
    s_lq         = 0.0f;
    s_v_amp      = FOC52_PROBE_V;
    s_v_bias     = g_ldlq52_r_used_ohm * g_ldlq52_bias_a + FOC52_VDEAD_PRIOR_V;
    if (s_v_bias > FOC52_VBIAS_HARD_MAX_V) { s_v_bias = FOC52_VBIAS_HARD_MAX_V; }
    s_cnt_entry  = (int32_t)TMRA_GetCountValue(CM_TMRA_1);
    LdLq52_PhaseReset();

    g_ldlq52_running = 1u;
    g_ldlq52_state   = FOC52_STEP_ZERO;
    g_ldlq52_evt     = 0u;
    g_ldlq52_axis    = 0u;

    Foc_Core_PwmStart();

    LDLQ52_DBG("start: do_q=%u pp=%d mA bias=%d mA cyc=%u v=%d mV vbias=%d mV",
               (unsigned)g_ldlq52_do_q,
               (int)(g_ldlq52_di_target_a * 1000.0f), (int)(g_ldlq52_bias_a * 1000.0f),
               (unsigned)g_ldlq52_cycles, (int)(s_v_amp * 1000.0f), (int)(s_v_bias * 1000.0f));
}

/*=============================================================================
 * Foc_LdLqId_Stop
 *=============================================================================*/
void Foc_LdLqId_Stop(void)
{
    if (g_ldlq52_running) {
        g_ldlq52_running  = 0u;
        g_foc_align_state = 0u;
        g_ldlq52_state    = FOC52_STEP_IDLE;    /* 中途停机也把状态摆回 IDLE，便于下次判断 */
        if (g_foc_active) {
            g_foc_active = 0u;
            Foc_Core_PwmStop();
            Foc_Core_SetStateMachine(FOC_STATE_IDLE);
        }
        LDLQ52_DBG("stopped");
    }
}

/*=============================================================================
 * Foc_LdLqId_Step（20 kHz ISR，禁止打印）
 *=============================================================================*/
void Foc_LdLqId_Step(const stc_i_data_t *pData)
{
    /* ===== 过流保护 ===== */
    if (Foc_Core_OverCurrent(pData)) {
        g_ldlq52_state = FOC52_STEP_FAULT_OC;
        g_ldlq52_evt   = FOC52_EVT_OC;
        Foc_Core_FaultStop(1u);
        return;
    }

    s_ms_tick++;
    if (s_ms_tick >= FOC_ISR_HZ / 1000u) {
        s_ms_tick = 0u;
        g_ldlq52_elapsed_ms++;
    }

    switch (g_ldlq52_state) {
    /* ---------- 零偏窗 ---------- */
    case FOC52_STEP_ZERO:
        LdLq52_OutputDq(0.0f, 0.0f, LdLq52_RotorAngle());
        if (s_zero_cnt == 0u) {
            s_tick0 = tickTimer_GetCount();     /* 零偏窗起点：用独立时基实测 ISR 频率 */
        }
        s_acc_zero_u += (int32_t)pData->i16IU_mA;
        s_acc_zero_v += (int32_t)pData->i16IV_mA;
        s_acc_zero_w += (int32_t)pData->i16IW_mA;
        s_zero_cnt++;
        if (s_zero_cnt >= FOC52_ZERO_SAMPLES) {
            uint32_t dt = (uint32_t)(tickTimer_GetCount() - s_tick0);   /* ms */
            s_zero_u = (float)s_acc_zero_u / (float)s_zero_cnt;
            s_zero_v = (float)s_acc_zero_v / (float)s_zero_cnt;
            s_zero_w = (float)s_acc_zero_w / (float)s_zero_cnt;
            /* 实测 ISR 频率：FOC52_ZERO_SAMPLES 拍用了多少 ms（独立时基，不是推断值）。
             * 2026-09-27 波形分析提示真实采样率可能只有宏值的一半，必须实测。 */
            if (dt > 0u) {
                g_ldlq52_fs_meas_hz = (uint32_t)((float)FOC52_ZERO_SAMPLES * 1000.0f / (float)dt);
            }
            s_v_amp  = FOC52_PROBE_V;
            g_ldlq52_v_inj_v = FOC52_PROBE_V;
            LdLq52_PhaseReset();
            g_ldlq52_wave_n   = 0u;
            g_ldlq52_wave_arm = (g_ldlq52_wave_en != 0u) ? 1u : 0u;   /* 默认不抓，wave_en=1 才抓 */
            g_ldlq52_state = FOC52_STEP_PROBE;
        }
        break;

    /* ---------- 探测：定方波幅值 + 把偏置电流调到真正够用 ---------- */
    case FOC52_STEP_PROBE:
        if (LdLq52_SquareStep(pData, FOC52_PROBE_CYCLES) != 0u) {
            float d_mean = 0.0f, i_bias = 0.0f, cons = 0.0f;
            uint8_t ok  = LdLq52_Stats(&d_mean, &i_bias, &cons);
            float v_cmd;
            uint8_t bias_low = 0u;

            /* 过程量先落全局：不管这轮成不成，Watch/RTT 都能看到真实数字
             *（上一版这里没记，导致失败打印出来全是 0，误判成"纹波也是 0"） */
            g_ldlq52_dcnt       = s_decnt;
            g_ldlq52_di_pp_a    = 2.0f * d_mean;
            g_ldlq52_i_bias_ma  = fabsf(i_bias) * 1000.0f;
            g_ldlq52_v_inj_v    = s_v_amp;      /* 本次探测用的幅值（后面会被规划值覆盖） */
            g_ldlq52_v_probe_mv = (uint16_t)(s_v_amp * 1000.0f);   /* 这个不会被覆盖 */
            g_ldlq52_vbias_v    = s_v_bias;
            g_ldlq52_probe_seq++;

            if (ok != 0u) {
                /* 合理性闸门：纹波反推的电感必须落在合理区间。
                 * 2026-09-27 实测教训：波形异常时"纹波半幅"被高估到 1.4A，偏置自适应
                 * 就一路把偏置电压抬到上限、d 轴电流冲到 3A —— 必须有这道闸门兜住。 */
                if (d_mean > 1.0e-6f) {
                    /* 平台均值法：L = Vsq·平台时长/|dE| */
                    float l_guess_uh = s_v_amp * ((float)FOC52_HALF_ISRS / (float)FOC_ISR_HZ)
                                     / d_mean * 1.0e6f;

                    g_ldlq52_l_probe_uh = l_guess_uh;
                    if ((l_guess_uh < FOC52_L_MIN_UH) || (l_guess_uh > FOC52_L_MAX_UH)) {
                        g_ldlq52_evt = FOC52_EVT_IMPLAUSIBLE;
                        LdLq52_Finish();
                        break;
                    }
                }
                /* 实测偏置必须明显大于纹波半幅（= 平台均值差的一半，dE 是全摆幅），
                 * 否则电流会过零、死区误差在平台内变号 */
                bias_low = (fabsf(i_bias) < (d_mean * 0.5f * FOC52_BIAS_NEED_RATIO)) ? 1u : 0u;
            }

            if ((ok == 0u) || (cons < FOC52_CONS_MIN_PCT) ||
                (d_mean < 1.0e-4f) || (bias_low != 0u)) {
                /* 偏置不够：抬偏置电压再探一次（不依赖 R/V_dead 先验是否准确） */
                /* 偏置电流有硬上限：实测已超 FOC52_BIAS_HARD_MAX_A 就停手，
                 * 免得像 2026-09-27 那样被虚高的纹波估计一路推到 3A。 */
                if ((bias_low != 0u) && (s_v_bias < FOC52_VBIAS_HARD_MAX_V) &&
                    (fabsf(i_bias) < FOC52_BIAS_HARD_MAX_A)) {
                    s_v_bias += FOC52_VBIAS_STEP_V;
                    if (s_v_bias > FOC52_VBIAS_HARD_MAX_V) { s_v_bias = FOC52_VBIAS_HARD_MAX_V; }
                    LdLq52_PhaseReset();
                    break;
                }
                /* 方波太小：升到上限再探一次 */
                if ((ok != 0u) && (bias_low == 0u) && (d_mean < 1.0e-4f) &&
                    (s_v_amp < g_ldlq52_v_max_v)) {
                    s_v_amp = g_ldlq52_v_max_v;
                    LdLq52_PhaseReset();
                    break;
                }
                if (ok == 0u)                            { g_ldlq52_evt = FOC52_EVT_NO_RIPPLE; }
                else if (cons < FOC52_CONS_MIN_PCT)      { g_ldlq52_evt = FOC52_EVT_PAIR_FAIL; }
                else if (d_mean < 1.0e-4f)               { g_ldlq52_evt = FOC52_EVT_NO_RIPPLE; }
                else                                     { g_ldlq52_evt = FOC52_EVT_BIAS_FAIL; }
                LdLq52_Finish();
                break;
            }
            /* L_guess = s_v_amp·t_s/d_mean；平台内 |di| 与 Vsq 成正比 => 按目标半幅重定幅值 */
            v_cmd = s_v_amp * (0.5f * g_ldlq52_di_target_a) / d_mean;
            if (v_cmd < g_ldlq52_v_min_v) { v_cmd = g_ldlq52_v_min_v; }
            if (v_cmd > g_ldlq52_v_max_v) { v_cmd = g_ldlq52_v_max_v; }
            s_v_amp = v_cmd;
            g_ldlq52_v_inj_v = v_cmd;
            LdLq52_PhaseReset();
            s_enc_axis = (int32_t)TMRA_GetCountValue(CM_TMRA_1);   /* 位移/路径从正式测量起算 */
            g_ldlq52_wave_n   = 0u;
            g_ldlq52_wave_arm = (g_ldlq52_wave_en != 0u) ? 1u : 0u;
            g_ldlq52_state = FOC52_STEP_MEAS;
        }
        break;

    /* ---------- 正式测量 ---------- */
    case FOC52_STEP_MEAS:
        if (LdLq52_SquareStep(pData, g_ldlq52_cycles) != 0u) {
            (void)LdLq52_FinishMeasure();
            if (g_ldlq52_axis == 0u) {
                g_ldlq52_ld_done_seq++;     /* d 轴测完就记一次：do_q=0/1 都会打 d done 行 */
            }
            if ((g_ldlq52_axis == 0u) && (g_ldlq52_do_q != 0u)) {
                g_ldlq52_axis  = 1u;
                s_tick         = 0u;
                g_ldlq52_state = FOC52_STEP_GAP;
            } else {
                /* do_q = 0 时只测 d 轴：显式记一个事件，免得日志上看不出 q 被跳过 */
                if ((g_ldlq52_axis == 0u) && (g_ldlq52_do_q == 0u)) {
                    g_ldlq52_q_skipped = 1u;
                }
                LdLq52_Finish();
            }
        }
        break;

    /* ---------- 间隔：零矢量让电流归零、转子停稳 ---------- */
    case FOC52_STEP_GAP:
        LdLq52_OutputDq(0.0f, 0.0f, LdLq52_RotorAngle());
        if (s_tick >= LDLQ52_MS_TO_TICKS(FOC52_GAP_MS)) {
            s_v_amp = FOC52_PROBE_V;
            g_ldlq52_v_inj_v = FOC52_PROBE_V;
            LdLq52_PhaseReset();
            g_ldlq52_wave_n   = 0u;
            g_ldlq52_wave_arm = (g_ldlq52_wave_en != 0u) ? 1u : 0u;
            g_ldlq52_state = FOC52_STEP_PROBE;
        }
        break;

    case FOC52_STEP_FAULT_OC:
    case FOC52_STEP_IDLE:
    default:
        LdLq52_OutputDq(0.0f, 0.0f, 0.0f);
        break;
    }

    s_tick++;
}

/*===========================================================================
 * 模式自持 VOFA：固定 12ch，单元换算"毫单位"（SendScaled 内部 ×0.001）
 *===========================================================================*/
int Foc_LdLqId_VofaFill(int32_t *cur)
{
    cur[0]  = (int32_t)(g_ldlq52_state);               /* ch0 状态码 */
    cur[1]  = (int32_t)(g_ldlq52_axis);                /* ch1 当前轴 0=d 1=q */
    cur[2]  = (int32_t)(g_ldlq52_v_inj_v * 1000.0f);   /* ch2 方波幅值 (V) */
    cur[3]  = (int32_t)(g_ldlq52_di_pp_a * 1000.0f);   /* ch3 纹波峰峰值 (A) */
    cur[4]  = (int32_t)(g_ldlq52_ld_uh * 1000.0f);     /* ch4 Ld (uH) */
    cur[5]  = (int32_t)(g_ldlq52_lq_uh * 1000.0f);     /* ch5 Lq (uH) */
    cur[6]  = (int32_t)(g_ldlq52_ratio * 1000.0f);     /* ch6 Lq/Ld (x1000) */
    cur[7]  = (int32_t)(g_ldlq52_i_bias_ma);           /* ch7 实测偏置电流 (A) */
    cur[8]  = (int32_t)(g_ldlq52_d_scatter_pct * 10.0f); /* ch8 离散度 (%) */
    cur[9]  = (int32_t)(g_ldlq52_cons_pct * 10.0f);    /* ch9 配对符号一致性 (%) */
    cur[10] = (int32_t)(g_ldlq52_moved_cnts);          /* ch10 编码器位移 (counts) */
    cur[11] = (int32_t)(g_ldlq52_elapsed_ms);          /* ch11 耗时 (ms) */
    return 12;
}
