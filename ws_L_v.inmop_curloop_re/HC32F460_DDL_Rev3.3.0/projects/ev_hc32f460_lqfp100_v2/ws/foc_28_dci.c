/**
 *******************************************************************************
 * @file  foc_28_dci.c
 * @brief FOC 模式28 — 功角参考电流闭环实现（第 2 步：P-only 电流环；
 *        第 1 步复刻 mode 27 + foc_calib 零偏窗已实测验收）。
 *
 *        流程：零矢量零偏校准（foc_calib，丢弃 10ms + 平均 200ms，锁三相
 *              零偏）-> 校准 BETA(2s, 90°) -> ALPHA(2s, 0°, 锁零点 offset)
 *              （校准吸附仍是电压开环，Dci_OutputField）
 *              -> RUN（电流环）：delta 照常爬坡（init/targ/tr），到点置
 *                 RAMP_DONE 一次；电流参考 id_ref = I_ref·cosδ,
 *                 iq_ref = I_ref·sinδ（δ = 参考电流矢量方向，δ=90° 即
 *                 id_ref=0 经典 FOC）；vd/vq = PID_UpdateUs 输出
 *                 （i_valid=false 仅 P，限幅 ±FOC28_PI_UMAX_V），反 Park 在
 *                 转子角 -> SVPWM。转速 = 力矩平衡结果（参考力矩 > 摩擦时
 *                 加速至 PI 电压饱和后电流 droop、力矩回落平衡）。
 *
 *        电流路径：
 *          锁定零偏后每拍构造 dataCal 副本（三相减 g_calib_*_off_ma），
 *          所有 Clarke/Park 用 dataCal；OC 保护仍用原始 pData。
 *          RUN 控制系 = 转子系：Park（反馈）与反 Park（输出）都在转子角。
 *
 *        输出约定：
 *          BETA/ALPHA（电压开环）：磁场角 fa 直接作为控制变量
 *          valpha=V·cos(fa), vbeta=V·sin(fa)，theta = fa - 90°。
 *          RUN（电流环）：vd/vq 为 PI 输出（g_foc_vd/vq 首次成为真实
 *          d/q 轴电压），g_foc_theta_rad = 转子角，g_foc_id/iq_ma 与
 *          g_dci_id/iq_ma 同源（= 环反馈）。
 *
 *        角度框架（增量累积式，与 mode 27 相同）：
 *          每拍 wrap-safe 差分 -> ±FOC28_ENC_DELTA_MAX(32) counts 限幅
 *          -> s_enc_pos 累加 -> 转子电角 =
 *             mod((s_enc_pos - s_off_rel) × 编码器方向, CPR) × 360°×极对数/CPR
 *          s_off_rel = ALPHA 锁零点瞬间的 s_enc_pos（锁相瞬间转子角=0）。
 *
 *        ISR 内禁止打印：事件经 g_dci_evt、周期数据经 Watch 观测量
 *        由 foc_obs 在主循环完成。
 *******************************************************************************
 */

#include "foc_28_dci.h"
#include "foc_calib.h"
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "hc32_ll_tmra.h"       /* TMRA_GetCountValue(CM_TMRA_1) */
#include "I.h"                  /* g_i_iu/iv/iw_ma（VOFA 三相电流通道） */

#define FOC_ISR_DT_US  (1000000u / FOC_ISR_HZ)   /* PI 积分离散步长 (us) */

/* 阶段时长/窗口 -> ISR tick 数 */
#define FOC28_BETA_TICKS    ((uint32_t)FOC28_BETA_MS    * (uint32_t)FOC_ISR_HZ / 1000u)
#define FOC28_ALPHA_TICKS   ((uint32_t)FOC28_ALPHA_MS   * (uint32_t)FOC_ISR_HZ / 1000u)
#define FOC28_SPEED_WIN_TICKS ((uint32_t)FOC28_SPEED_WIN_MS * (uint32_t)FOC_ISR_HZ / 1000u)
#define FOC28_PP_WIN_TICKS  ((uint32_t)FOC28_PP_WIN_MS  * (uint32_t)FOC_ISR_HZ / 1000u)
#define FOC28_MEAN_WIN_TICKS ((uint32_t)FOC28_MEAN_WIN_MS * (uint32_t)FOC_ISR_HZ / 1000u)
#define FOC28_ERR_WIN_TICKS ((uint32_t)FOC28_ERR_WIN_MS  * (uint32_t)FOC_ISR_HZ / 1000u)

/* 角度换算（观测/控制用） */
#define FOC28_RAD2DEG   57.2958f
#define FOC28_DEG2RAD   0.0174533f

/*******************************************************************************
 * Watch 观测量 / 可调变量
 ******************************************************************************/
volatile float    g_dci_dlt_init_deg = 90.0f;   /* 功角爬坡起点（软吸附起步） */
volatile float    g_dci_dlt_targ_deg = 90.0f;  /* 功角爬坡终点 */
volatile uint32_t g_dci_dlt_tr_ms   = 0u;   /* 功角爬坡过渡时间 */
volatile float    g_dci_volt_v      = 0.8f;    /* 电压（=调速旋钮），Start 复位 */
volatile uint8_t  g_dci_running     = 0u;
volatile uint8_t  g_dci_state       = FOC28_STEP_IDLE;
volatile uint8_t  g_dci_evt         = 0u;
volatile int32_t  g_dci_offset      = 0;
volatile float    g_dci_dlt_now_deg = 0.0f;
volatile float    g_dci_speed_hz    = 0.0f;
volatile int32_t  g_dci_field_deg   = 0;
volatile int32_t  g_dci_rotor_deg   = 0;
volatile int32_t  g_dci_diff_deg    = 0;
volatile float    g_dci_id_ma       = 0.0f;
volatile float    g_dci_iq_ma       = 0.0f;
volatile float    g_dci_id_pp_ma    = 0.0f;  /* id 峰峰值 (mA, 每 FOC28_PP_WIN_MS 刷新) */
volatile float    g_dci_iq_pp_ma    = 0.0f;  /* iq 峰峰值 (mA, 同上) */
volatile float    g_dci_id_mean_ma  = 0.0f;  /* id 均值 (mA, 每 FOC28_MEAN_WIN_MS 刷新) */
volatile float    g_dci_iq_mean_ma  = 0.0f;  /* iq 均值 (mA, 同上) */
volatile float    g_dci_i_ref_ma    = FOC28_I_REF_MA;  /* 电流矢量幅值参考 (Start 不复位) */
volatile float    g_dci_id_ref_ma   = 0.0f;  /* d 轴电流参考 (mA, 实时) */
volatile float    g_dci_iq_ref_ma   = 0.0f;  /* q 轴电流参考 (mA, 实时) */
volatile float    g_dci_i_ramp_ma_s = (float)FOC28_I_RAMP_MA_S;  /* I_ref 软启动斜率 (mA/s)，<=0 直通 */
static float      s_i_ref_ramp      = 0.0f;   /* 斜坡后的幅值实际值 (mA)，Start 复位 0 */
volatile uint8_t  g_dci_vsat        = 0u;    /* 电压饱和标志 (1 = 任一轴顶到 UMAX) */
volatile float    g_dci_ed_mean_ma  = 0.0f;  /* d 轴误差均值 (mA, 每 FOC28_ERR_WIN_MS 刷新) */
volatile float    g_dci_eq_mean_ma  = 0.0f;  /* q 轴误差均值 (mA, 同上) */
volatile float    g_dci_ed_pp_ma    = 0.0f;  /* d 轴误差峰峰值 (mA, 同上) */
volatile float    g_dci_eq_pp_ma    = 0.0f;  /* q 轴误差峰峰值 (mA, 同上) */
volatile int32_t  g_dci_enc_pos     = 0;
volatile float    g_dci_du          = 50.0f;
volatile float    g_dci_dv          = 50.0f;
volatile float    g_dci_dw          = 50.0f;

/* 内部状态 */
static uint32_t s_phase_tick  = 0u;    /* 校准阶段计时（tick） */
static uint32_t s_run_tick    = 0u;    /* RUN 态计时（tick），爬坡时基 */
static uint8_t  s_ramp_done   = 0u;    /* 爬坡完成事件已置位（每次运行只置一次） */
static uint16_t s_enc_prev_hw = 0u;    /* 上拍编码器硬件计数 */
static int32_t  s_enc_pos     = 0;     /* 相对计数累积（限幅后） */
static uint8_t  s_enc_init    = 0u;    /* 首拍基准初始化标志 */
static int32_t  s_off_rel     = 0;     /* 锁零点瞬间的相对计数（控制帧零点） */
static uint32_t s_speed_tick  = 0u;    /* 转速窗口计时（tick） */
static int32_t  s_speed_acc   = 0;     /* 转速窗口内计数累积 */

/* id/iq 峰峰值统计（仅 RUN 态喂数，窗口无缝衔接） */
static uint32_t s_pp_tick = 0u;        /* 窗口计时（tick） */
static uint8_t  s_pp_init = 0u;        /* 首样本初始化标志 */
static float    s_id_min  = 0.0f;
static float    s_id_max  = 0.0f;
static float    s_iq_min  = 0.0f;
static float    s_iq_max  = 0.0f;

/* id/iq 均值统计（仅 RUN 态喂数，mA 整数累加避免浮点累积误差：
 * 60000 样本 × ±10000mA 上限 = 6e8，int32 范围内） */
static uint32_t s_mean_tick  = 0u;     /* 窗口计时（tick） */
static int32_t  s_id_sum_ma  = 0;      /* 窗口内 id 累加 (mA) */
static int32_t  s_iq_sum_ma  = 0;      /* 窗口内 iq 累加 (mA) */

/* 误差统计（e = ref - fb，仅 RUN 态喂数，5s 窗口无缝衔接）：
 * mean = P-only 稳态落差（应随 kp 加倍约减半）；pp = 振铃/振荡指标 */
static uint32_t s_err_tick   = 0u;     /* 窗口计时（tick） */
static uint8_t  s_err_init   = 0u;     /* 首样本初始化标志 */
static int32_t  s_ed_sum_ma  = 0;      /* 窗口内 ed 累加 (mA) */
static int32_t  s_eq_sum_ma  = 0;      /* 窗口内 eq 累加 (mA) */
static float    s_ed_min     = 0.0f;
static float    s_ed_max     = 0.0f;
static float    s_eq_min     = 0.0f;
static float    s_eq_max     = 0.0f;

/*******************************************************************************
 * d/q 轴 PI 配置（P 重调阶段：i_valid=false 纯 P，ki=0。
 * 转子捏死工况下调 P：快速接近目标且不振铃即为合格。
 * P 合格后 Watch 置 i_valid=1 + ki=FOC28_PI_KI；字段 volatile 实时生效）
 ******************************************************************************/
pid_config_t g_dci_pid_id_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = false,          /* P 重调阶段；P 合格后 Watch 置 1 开 I */
    .d_valid      = false,
    .kp           = FOC28_PI_KP,
    .ki           = 0.0f,           /* 开 I 时 Watch 改为 FOC28_PI_KI（≈kp×R/L） */
    .kd           = 0.0f,
    .output_min   = -FOC28_PI_UMAX_V,
    .output_max   =  FOC28_PI_UMAX_V,
    .integral_max =  FOC28_ITERM_MAX_V,
    .i_term_max   =  FOC28_ITERM_MAX_V,
    .update_ms    = 0,
};

pid_config_t g_dci_pid_iq_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = false,          /* P 重调阶段；P 合格后 Watch 置 1 开 I */
    .d_valid      = false,
    .kp           = FOC28_PI_KP,
    .ki           = 0.0f,
    .kd           = 0.0f,
    .output_min   = -FOC28_PI_UMAX_V,
    .output_max   =  FOC28_PI_UMAX_V,
    .integral_max =  FOC28_ITERM_MAX_V,
    .i_term_max   =  FOC28_ITERM_MAX_V,
    .update_ms    = 0,
};

/* PI runtime states */
static pid_state_t s_pid_id;
static pid_state_t s_pid_iq;

/*******************************************************************************
 * 内部助手：峰峰值喂数（ISR 内调用）
 *   每拍更新窗口内 min/max，满 FOC28_PP_WIN_TICKS 时锁存峰峰值并以下一拍
 *   样本为新窗口起点（窗口无缝衔接，无重叠无遗漏）。入参单位 A。
 ******************************************************************************/
static void Dci_PpFeed(float id, float iq)
{
    if (s_pp_init == 0u) {
        s_id_min = s_id_max = id;
        s_iq_min = s_iq_max = iq;
        s_pp_init = 1u;
    } else {
        if (id < s_id_min) { s_id_min = id; }
        if (id > s_id_max) { s_id_max = id; }
        if (iq < s_iq_min) { s_iq_min = iq; }
        if (iq > s_iq_max) { s_iq_max = iq; }
    }
    if (++s_pp_tick >= FOC28_PP_WIN_TICKS) {
        s_pp_tick = 0u;
        g_dci_id_pp_ma = (s_id_max - s_id_min) * 1000.0f;
        g_dci_iq_pp_ma = (s_iq_max - s_iq_min) * 1000.0f;
        s_id_min = s_id_max = id;   /* 新窗口从当前样本重新起步 */
        s_iq_min = s_iq_max = iq;
    }
}

/*******************************************************************************
 * 内部助手：输出指定磁场电角度 + 刷新观测量（ISR 内调用）
 *   入参 fa = 磁场角（本模式的主控制变量 = 转子角 + delta）。
 *   valpha=V·cos(fa), vbeta=V·sin(fa)；
 *   等效 d 轴角 theta = fa - 90°（g_foc_theta_rad 语义与 mode 26/27 统一），
 *   锁相稳态时控制系电流 g_foc_id_ma≈0, g_foc_iq_ma≈I。
 *   电流入参 p 为零偏校正后的 dataCal 副本。
 ******************************************************************************/
static void Dci_OutputField(const stc_i_data_t *p, float fa)
{
    float valpha, vbeta, du, dv, dw, id, iq, th;

    valpha = g_dci_volt_v * Foc_Math_Cos(fa);
    vbeta  = g_dci_volt_v * Foc_Math_Sin(fa);
    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);

    th = fa - FOC_MATH_HALF_PI;
    th -= (float)((int32_t)(th * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
    if (th < 0.0f) {
        th += FOC_MATH_2PI;
    }

    g_foc_valpha    = valpha;
    g_foc_vbeta     = vbeta;
    g_foc_vd        = valpha;
    g_foc_vq        = vbeta;
    g_foc_theta_rad = th;
    g_foc_du        = du;
    g_foc_dv        = dv;
    g_foc_dw        = dw;
    g_dci_du        = du;
    g_dci_dv        = dv;
    g_dci_dw        = dw;

    /* 控制系（theta d 轴框架）电流观察：锁相稳态 id≈0, iq≈I */
    Foc_Core_GetDq(p, th, &id, &iq);
    g_foc_id_ma = id * 1000.0f;
    g_foc_iq_ma = iq * 1000.0f;
}

/*******************************************************************************
 * Foc_Dci_InitPids - 绑定 PI 实例与配置（Foc_Init 调用一次）
 ******************************************************************************/
void Foc_Dci_InitPids(void)
{
    PID_Init(&s_pid_id, &g_dci_pid_id_cfg);
    PID_Init(&s_pid_iq, &g_dci_pid_iq_cfg);
}

/*******************************************************************************
 * Foc_Dci_Start - mode 28 入口：校准 + 闭环 一体化（主循环上下文，允许打印）
 ******************************************************************************/
void Foc_Dci_Start(void)
{
    Foc_Core_ClearFault();
    g_foc_mode        = FOC_MODE_ALIGN;   /* 复用 ALIGN 分发路径（按 g_dci_running 区分） */
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

    g_dci_running     = 1u;
    g_dci_state       = FOC28_STEP_CALIB;
    g_dci_evt         = 0u;
    s_phase_tick      = 0u;
    s_run_tick        = 0u;
    s_ramp_done       = 0u;
    s_i_ref_ramp      = 0.0f;
    s_enc_init        = 0u;
    s_enc_pos         = 0;
    s_off_rel         = 0;
    s_speed_tick      = 0u;
    s_speed_acc       = 0;
    g_dci_dlt_now_deg = g_dci_dlt_init_deg;
    g_dci_speed_hz    = 0.0f;
    g_dci_field_deg   = 0;
    g_dci_rotor_deg   = 0;
    g_dci_diff_deg    = 0;
    g_dci_id_ma       = 0.0f;
    g_dci_iq_ma       = 0.0f;
    g_dci_id_pp_ma    = 0.0f;
    g_dci_iq_pp_ma    = 0.0f;
    g_dci_id_mean_ma  = 0.0f;
    g_dci_iq_mean_ma  = 0.0f;
    g_dci_id_ref_ma   = 0.0f;
    g_dci_iq_ref_ma   = 0.0f;
    g_dci_vsat        = 0u;
    g_dci_ed_mean_ma  = 0.0f;
    g_dci_eq_mean_ma  = 0.0f;
    g_dci_ed_pp_ma    = 0.0f;
    g_dci_eq_pp_ma    = 0.0f;
    s_pp_tick         = 0u;
    s_pp_init         = 0u;
    s_mean_tick       = 0u;
    s_id_sum_ma       = 0;
    s_iq_sum_ma       = 0;
    s_err_tick        = 0u;
    s_err_init        = 0u;
    s_ed_sum_ma       = 0;
    s_eq_sum_ma       = 0;
    /* g_dci_i_ref_ma 不复位（力矩旋钮，预设后启动） */
    PID_Reset(&s_pid_id);   /* 清 PI 累计（积分/首拍标志） */
    PID_Reset(&s_pid_iq);
    g_dci_enc_pos     = 0;
    g_dci_volt_v      = 0.6f;
    /* g_dci_offset 保留上次锁定值（校准完成后覆盖刷新） */

    Foc_Calib_Start();     /* 零偏校准状态机复位，CALIB 态逐拍喂样 */
    Foc_Core_PwmStart();   /* 零矢量起 PWM（g_foc_active=1），下一拍开始校准 */

    FOC28_DBG("start calib zero-offset -> run dlt %d->%d deg tr=%d ms Iref=%d mA kp=%d m iValid=%d",
            (int)g_dci_dlt_init_deg, (int)g_dci_dlt_targ_deg,
            (int)g_dci_dlt_tr_ms, (int)g_dci_i_ref_ma,
            (int)(g_dci_pid_iq_cfg.kp * 1000.0f),
            (int)g_dci_pid_iq_cfg.i_valid);
}

/*******************************************************************************
 * Foc_Dci_Step - 步进（20 kHz ISR 中调用，禁止打印）
 ******************************************************************************/
void Foc_Dci_Step(const stc_i_data_t *pData)
{
    float rot_rad, id, iq, w, dlt, dlt_rad, ramp_step;
    float id_ref, iq_ref, vd, vq, valpha, vbeta, du, dv, dw, cos_r, sin_r;
    uint16_t hw, hw0;
    uint32_t tr_ticks;
    int32_t delta, rel, rot_deg, fld_deg, dfd, dir;
    stc_i_data_t dataCal;   /* 零偏校正后的电流采样副本（OC 保护仍用原始 pData） */

    /* ===== OC 保护（原始 pData，去抖在 Foc_Core_OverCurrent 内） ===== */
    if (Foc_Core_OverCurrent(pData)) {
        g_dci_state   = FOC28_STEP_FAULT_OC;
        g_dci_evt     = FOC28_EVT_OC;
        g_dci_running = 0u;
        Foc_Core_FaultStop(1u);   /* 置故障 + 关 PWM + active=0 + IDLE */
        return;
    }

    /* ===== 零偏校正副本（foc_calib 未锁定时全 0 偏移 = 原始值） ===== */
    dataCal = *pData;
    if (Foc_Calib_IsLocked()) {
        float off_u, off_v, off_w;
        Foc_Calib_GetOffsetsMa(&off_u, &off_v, &off_w);
        dataCal.i16IU_mA = (int16_t)((float)pData->i16IU_mA - off_u);
        dataCal.i16IV_mA = (int16_t)((float)pData->i16IV_mA - off_v);
        dataCal.i16IW_mA = (int16_t)((float)pData->i16IW_mA - off_w);
    }

    switch (g_dci_state) {
    /* ===== 零矢量零偏校准：三相 50/50/50（零电流），~210ms ===== */
    case FOC28_STEP_CALIB:
        (void)Foc_Calib_Feed(pData);
        TMR4_PWM_SetDuty3Phase(50.0f, 50.0f, 50.0f);
        g_foc_du = 50.0f;
        g_foc_dv = 50.0f;
        g_foc_dw = 50.0f;
        g_dci_du = 50.0f;
        g_dci_dv = 50.0f;
        g_dci_dw = 50.0f;
        g_foc_id_ma = 0.0f;
        g_foc_iq_ma = 0.0f;
        g_foc_vd = 0.0f;
        g_foc_vq = 0.0f;
        if (Foc_Calib_IsLocked()) {
            s_phase_tick = 0u;
            g_dci_state  = FOC28_STEP_CAL_BETA;
            g_dci_evt    = FOC28_EVT_CALIB_DONE;
        }
        break;

    /* ===== 校准 BETA：磁场定 90°，2s ===== */
    case FOC28_STEP_CAL_BETA:
        Dci_OutputField(&dataCal, FOC_MATH_HALF_PI);
        if (++s_phase_tick >= FOC28_BETA_TICKS) {
            s_phase_tick = 0u;
            g_dci_state  = FOC28_STEP_CAL_ALPHA;
            g_dci_evt    = FOC28_EVT_BETA_DONE;
        }
        break;

    /* ===== 校准 ALPHA：磁场定 0°，2s，结束锁零点 -> 功角闭环 ===== */
    case FOC28_STEP_CAL_ALPHA:
        Dci_OutputField(&dataCal, 0.0f);
        if (++s_phase_tick >= FOC28_ALPHA_TICKS) {
            /* 锁零点（双帧）：
             *  - 控制帧：s_off_rel = 此刻相对计数，RUN 态转子电角 =
             *    mod((s_enc_pos - s_off_rel) × dir, CPR)，锁相瞬间 = 0
             *  - 显示帧：g_dci_offset = hw 绝对帧（与 mode 20/25/26/27 同框架，
             *    便于跨模式/跨次对比） */
            hw0 = TMRA_GetCountValue(CM_TMRA_1);
            dir = (int32_t)g_foc_enc_dir;
            g_dci_offset = Foc_Core_ModPos((int32_t)hw0 * dir,
                                           (int32_t)ENCODER_CPR);
            Foc_Core_SetAlignOffset(g_dci_offset);   /* core 共享角度观测同帧（与 mode 27 一致） */
            s_off_rel     = s_enc_pos;
            s_enc_prev_hw = hw0;      /* 编码器基准锚定在锁零点瞬间 */
            s_enc_init    = 1u;

            s_phase_tick = 0u;
            s_run_tick   = 0u;
            s_ramp_done  = 0u;
            g_dci_state  = FOC28_STEP_RUN;
            g_dci_evt    = FOC28_EVT_LOCKED;
        }
        break;

    /* ===== RUN：磁场 = 转子 + delta（功角闭环核心），与 mode 27 逐拍等价 ===== */
    case FOC28_STEP_RUN:
        /* 1. delta 爬坡：value = init + (targ-init)×w，w = elapsed/tr 线性，
         *    init/targ/tr 每拍实时读 Watch（运行中改 = 按新值重算轨迹）。
         *    tr=0 立即到目标；到点置 RAMP_DONE 一次。 */
        tr_ticks = g_dci_dlt_tr_ms * (FOC_ISR_HZ / 1000u);
        if (tr_ticks == 0u) {
            w = 1.0f;
        } else {
            w = (float)s_run_tick / (float)tr_ticks;
            if (w > 1.0f) {
                w = 1.0f;
            }
        }
        dlt = g_dci_dlt_init_deg
            + (g_dci_dlt_targ_deg - g_dci_dlt_init_deg) * w;
        g_dci_dlt_now_deg = dlt;
        if ((s_ramp_done == 0u) && (s_run_tick >= tr_ticks)) {
            s_ramp_done = 1u;
            g_dci_evt   = FOC28_EVT_RAMP_DONE;
        }
        s_run_tick++;

        /* 2. 编码器增量读取：wrap-safe 差分 + ±FOC28_ENC_DELTA_MAX 限幅
         *    （毛刺单拍污染封顶 ±32 counts ≈ ±2.8° 电角） */
        hw = TMRA_GetCountValue(CM_TMRA_1);
        if (!s_enc_init) {
            s_enc_prev_hw = hw;
            s_enc_init    = 1u;
        }
        delta = (int32_t)(int16_t)((uint16_t)hw - s_enc_prev_hw);
        s_enc_prev_hw = hw;
        if (delta > FOC28_ENC_DELTA_MAX) {
            delta = FOC28_ENC_DELTA_MAX;
        }
        if (delta < -FOC28_ENC_DELTA_MAX) {
            delta = -FOC28_ENC_DELTA_MAX;
        }
        s_enc_pos += delta;
        g_dci_enc_pos = s_enc_pos;

        /* 3. 转子真实电角度（相对帧 - 锁零点，机械圈 -> 电角度） */
        rel = Foc_Core_ModPos((s_enc_pos - s_off_rel) * (int32_t)g_foc_enc_dir,
                              (int32_t)ENCODER_CPR);
        rot_rad = (float)rel * (FOC_MATH_2PI * (float)FOC_POLE_PAIRS
                                / (float)ENCODER_CPR);
        rot_rad -= (float)((int32_t)(rot_rad * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
        if (rot_rad < 0.0f) {
            rot_rad += FOC_MATH_2PI;
        }

        /* 4. 电流环反馈：真实转子系 id/iq（零偏校正后；RUN 控制系 = 转子系） */
        Foc_Core_GetDq(&dataCal, rot_rad, &id, &iq);
        g_dci_id_ma = id * 1000.0f;
        g_dci_iq_ma = iq * 1000.0f;
        g_foc_id_ma = g_dci_id_ma;
        g_foc_iq_ma = g_dci_iq_ma;
        Dci_PpFeed(id, iq);   /* 峰峰值统计（5s 窗口刷新） */

        /* 5. 均值统计（3s 窗口刷新，mA 整数累加无浮点误差） */
        s_id_sum_ma += (int32_t)(id * 1000.0f);
        s_iq_sum_ma += (int32_t)(iq * 1000.0f);
        if (++s_mean_tick >= FOC28_MEAN_WIN_TICKS) {
            s_mean_tick = 0u;
            g_dci_id_mean_ma = (float)s_id_sum_ma / (float)FOC28_MEAN_WIN_TICKS;
            g_dci_iq_mean_ma = (float)s_iq_sum_ma / (float)FOC28_MEAN_WIN_TICKS;
            s_id_sum_ma = 0;
            s_iq_sum_ma = 0;
        }

        /* 6. 电流参考生成：先 I_ref 斜坡（每拍小目标逼近最终目标），再
         *    id_ref = I·cosδ, iq_ref = I·sinδ（δ=90° 即 id_ref=0 经典 FOC）。
         *    斜率 g_dci_i_ramp_ma_s (mA/s) Watch 可调，<=0 直通（阶跃激励，
         *    调 P 用）。g_dci_i_ref_ma 恒为最终目标，s_i_ref_ramp 为生效幅值，
         *    Start 复位 0 —— 启动即从 0 爬坡，改目标即时跟随 */
        if (g_dci_i_ramp_ma_s > 0.0f) {
            ramp_step = g_dci_i_ramp_ma_s / (float)FOC_ISR_HZ;
            if (s_i_ref_ramp < g_dci_i_ref_ma) {
                s_i_ref_ramp += ramp_step;
                if (s_i_ref_ramp > g_dci_i_ref_ma) {
                    s_i_ref_ramp = g_dci_i_ref_ma;
                }
            } else if (s_i_ref_ramp > g_dci_i_ref_ma) {
                s_i_ref_ramp -= ramp_step;
                if (s_i_ref_ramp < g_dci_i_ref_ma) {
                    s_i_ref_ramp = g_dci_i_ref_ma;
                }
            }
        } else {
            s_i_ref_ramp = g_dci_i_ref_ma;
        }
        dlt_rad = dlt * FOC28_DEG2RAD;
        id_ref = (s_i_ref_ramp * 0.001f) * Foc_Math_Cos(dlt_rad);
        iq_ref = (s_i_ref_ramp * 0.001f) * Foc_Math_Sin(dlt_rad);
        g_dci_id_ref_ma = id_ref * 1000.0f;
        g_dci_iq_ref_ma = iq_ref * 1000.0f;

        /* 7. PI 电流环（I 已启用）：vd/vq = PID(ref, fb)，限幅 ±FOC28_PI_UMAX_V。
         *    ki≈kp×R/L 零极点对消；调 P 时 Watch 置 g_dci_pid_*_cfg.i_valid=0 */
        vd = PID_UpdateUs(&s_pid_id, id_ref, id, FOC_ISR_DT_US);
        vq = PID_UpdateUs(&s_pid_iq, iq_ref, iq, FOC_ISR_DT_US);
        g_foc_vd = vd;
        g_foc_vq = vq;
        g_dci_vsat = ((vd <= -FOC28_PI_UMAX_V + 0.01f) || (vd >= FOC28_PI_UMAX_V - 0.01f) ||
                      (vq <= -FOC28_PI_UMAX_V + 0.01f) || (vq >= FOC28_PI_UMAX_V - 0.01f)) ? 1u : 0u;

        /* 8. 反 Park（转子角）-> SVPWM 输出 */
        cos_r = Foc_Math_Cos(rot_rad);
        sin_r = Foc_Math_Sin(rot_rad);
        valpha = vd * cos_r - vq * sin_r;
        vbeta  = vd * sin_r + vq * cos_r;
        Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
        TMR4_PWM_SetDuty3Phase(du, dv, dw);

        g_foc_valpha    = valpha;
        g_foc_vbeta     = vbeta;
        g_foc_theta_rad = rot_rad;   /* RUN 控制系 = 转子系 */
        g_foc_du = du;
        g_foc_dv = dv;
        g_foc_dw = dw;
        g_dci_du = du;
        g_dci_dv = dv;
        g_dci_dw = dw;

        /* 9. 误差统计（e = ref - fb，5s 窗口无缝衔接）：
         *    mean = P-only 稳态落差；pp = 振铃/振荡指标 */
        {
            int32_t e_d_ma = (int32_t)((id_ref - id) * 1000.0f);
            int32_t e_q_ma = (int32_t)((iq_ref - iq) * 1000.0f);

            if (s_err_init == 0u) {
                s_ed_min = s_ed_max = (float)e_d_ma;
                s_eq_min = s_eq_max = (float)e_q_ma;
                s_err_init = 1u;
            } else {
                if (e_d_ma < s_ed_min) { s_ed_min = (float)e_d_ma; }
                if (e_d_ma > s_ed_max) { s_ed_max = (float)e_d_ma; }
                if (e_q_ma < s_eq_min) { s_eq_min = (float)e_q_ma; }
                if (e_q_ma > s_eq_max) { s_eq_max = (float)e_q_ma; }
            }
            s_ed_sum_ma += e_d_ma;
            s_eq_sum_ma += e_q_ma;
            if (++s_err_tick >= FOC28_ERR_WIN_TICKS) {
                s_err_tick = 0u;
                g_dci_ed_mean_ma = (float)s_ed_sum_ma / (float)FOC28_ERR_WIN_TICKS;
                g_dci_eq_mean_ma = (float)s_eq_sum_ma / (float)FOC28_ERR_WIN_TICKS;
                g_dci_ed_pp_ma = s_ed_max - s_ed_min;
                g_dci_eq_pp_ma = s_eq_max - s_eq_min;
                s_ed_sum_ma = 0;
                s_eq_sum_ma = 0;
                s_ed_min = s_ed_max = (float)e_d_ma;   /* 新窗口从当前样本起步 */
                s_eq_min = s_eq_max = (float)e_q_ma;
            }
        }

        /* 10. 转速测量（200ms 窗口，带符号）：Hz = counts×极对数×窗口率/CPR */
        s_speed_acc += delta;
        if (++s_speed_tick >= FOC28_SPEED_WIN_TICKS) {
            g_dci_speed_hz = (float)s_speed_acc * (float)FOC_POLE_PAIRS
                             * (1000.0f / (float)FOC28_SPEED_WIN_MS)
                             / (float)ENCODER_CPR;
            s_speed_tick = 0u;
            s_speed_acc  = 0;
        }

        /* 11. 角度观测（整型电角度 deg）：fld = 参考电流矢量角 = rot + δ，
         *     diff = δ（参考；实际电流矢量跟踪误差看 e_d/e_q 统计） */
        fld_deg = (int32_t)((rot_rad + dlt_rad) * FOC28_RAD2DEG);
        if (fld_deg >= 360) {
            fld_deg -= 360;
        }
        rot_deg = (int32_t)(rot_rad * FOC28_RAD2DEG);
        dfd     = fld_deg - rot_deg;
        dfd     = 180 - Foc_Core_ModPos(180 - dfd, 360);

        g_dci_field_deg = fld_deg;
        g_dci_rotor_deg = rot_deg;
        g_dci_diff_deg  = dfd;
        break;

    case FOC28_STEP_FAULT_OC:
    default:
        /* 故障/未知状态：不发波，等待主循环切模式 */
        break;
    }
}

/*******************************************************************************
 * Foc_Dci_Stop - 停止（用户中途切模式，主循环上下文）
 *   自带 g_foc_active 清零，保证任何退出路径 ISR 都不会再进入本模块步进。
 ******************************************************************************/
void Foc_Dci_Stop(void)
{
    if (g_dci_running) {
        g_dci_running     = 0u;
        g_foc_align_state = 0u;
        if (g_foc_active) {
            g_foc_active = 0u;
            Foc_Core_PwmStop();
            Foc_Core_SetStateMachine(FOC_STATE_IDLE);
        }
        FOC28_DBG("stopped");
    }
}

/*===========================================================================
 * mode 28 自持 VOFA：固定 17ch 布局
 * 通道含义速览卡 = foc_28_dci.h 顶部【模式速览卡】（唯一事实源）
 *
 * 本模式是"校准+功角电流闭环一体化"，P 重调主判据在 ch11/ch12（误差均值）
 * 与 ch15/ch16（误差峰峰值）：**均值=稳态落差（P-only 属正常），峰峰值增大=振铃**。
 * ⚠ i_valid=0（P-only）时稳态落差 = R·i_ref/(kp+R) 是理论必然，不是故障。
 * 单位换算：传"毫单位"，SendScaled 内部 ×0.001
 *===========================================================================*/
int Foc_Dci_VofaFill(int32_t *cur)
{
    cur[0]  = (int32_t)(g_i_iu_ma);                 /* ch0  U 相电流 (mA -> A) */
    cur[1]  = (int32_t)(g_i_iv_ma);                 /* ch1  V 相电流 */
    cur[2]  = (int32_t)(g_i_iw_ma);                 /* ch2  W 相电流 */
    cur[3]  = (int32_t)(g_dci_speed_hz * 1000.0f);  /* ch3  实测电频率 (mHz -> Hz) */
    cur[4]  = (int32_t)(g_dci_dlt_now_deg * 1000.0f);/* ch4 δ 指令 (mdeg -> deg) */
    cur[5]  = (int32_t)(g_dci_diff_deg * 1000);     /* ch5  功角实测 (mdeg -> deg) */
    cur[6]  = (int32_t)(g_dci_iq_ma);               /* ch6  iq 反馈 */
    cur[7]  = (int32_t)(g_dci_id_ma);               /* ch7  id 反馈 */
    cur[8]  = (int32_t)(g_dci_iq_ref_ma);           /* ch8  iq 参考 */
    cur[9]  = (int32_t)(g_dci_id_ref_ma);           /* ch9  id 参考（= I_ref·cosδ） */
    cur[10] = (int32_t)(g_dci_i_ref_ma);            /* ch10 电流矢量**幅值**参考 */
    cur[11] = (int32_t)(g_dci_eq_mean_ma);          /* ch11 **q 误差均值 ← P 调参主判据** */
    cur[12] = (int32_t)(g_dci_ed_mean_ma);          /* ch12 d 误差均值 */
    cur[13] = (int32_t)(g_dci_iq_mean_ma);          /* ch13 iq 3s 均值 */
    cur[14] = (int32_t)(g_dci_id_mean_ma);          /* ch14 id 3s 均值 */
    cur[15] = (int32_t)(g_dci_eq_pp_ma);            /* ch15 **q 误差峰峰值（振铃判据）** */
    cur[16] = (int32_t)(g_dci_state);               /* ch16 状态 0IDLE/1零偏/2BETA/3ALPHA/4RUN/5OC */
    return 17;
}
