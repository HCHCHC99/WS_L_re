/**
 *******************************************************************************
 * @file  foc_26_olf.c
 * @brief FOC 模式26 — 开环 VF 负载角实验实现。
 *
 *        流程：自动校准 BETA(2s, 90°) -> ALPHA(2s, 0°, 锁零点 offset)
 *              -> 拖动：自增频率从 g_olf_freq_init_hz 线性爬坡到
 *                 g_olf_freq_targ_hz（历时 g_olf_freq_tr_ms, 0=立即, 到点置
 *                 RAMP_DONE 一次），当前频率实时写入 g_olf_freq_hz；
 *                 磁场角以当前频率步长量化自增
 *                 （每攒满 g_olf_step_010×0.1° 跳一步，方向 g_olf_dir ±1，
 *                 默认步长 1 ≈ 连续旋转），
 *                 q 轴电压约定（与 mode 30 一致）：g_olf_theta_rad 为控制系
 *                 d 轴角，电压矢量在 theta+90°（q 轴），磁场角 = theta+90°，
 *                 与转子 N 极对齐时 delta=0；锁定时控制系 g_foc_id_ma≈0,
 *                 g_foc_iq_ma≈I（真实转子系 g_olf_id/iq_ma 仍按物理分布），
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

#include "foc_26_olf.h"
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "hc32_ll_tmra.h"       /* TMRA_GetCountValue(CM_TMRA_1) */
#include "I.h"                  /* g_i_iu/iv/iw_ma（VOFA 三相电流通道） */

/* 阶段时长 -> ISR tick 数 */
#define OLF_BETA_TICKS    ((uint32_t)OLF_BETA_MS    * (uint32_t)FOC_ISR_HZ / 1000u)
#define OLF_ALPHA_TICKS   ((uint32_t)OLF_ALPHA_MS   * (uint32_t)FOC_ISR_HZ / 1000u)
#define OLF_PP_WIN_TICKS  ((uint32_t)OLF_PP_WIN_MS  * (uint32_t)FOC_ISR_HZ / 1000u)

/* 电角度 rad -> deg 换算（观测显示用，取整） */
#define OLF_RAD2DEG   57.2958f

/*******************************************************************************
 * Watch 观测量 / 可调变量
 ******************************************************************************/
volatile float    g_olf_freq_hz      = 0.2f;  /* 当前自增频率（斜坡实时输出，只读观察） */
volatile float    g_olf_freq_init_hz = 0.2f;  /* 斜坡起点频率（低速起步，防大功角） */
volatile float    g_olf_freq_targ_hz = 150.0f;  /* 斜坡目标频率（扫频实验改此值，如 25.0） */
volatile uint32_t g_olf_freq_tr_ms   = 20000u;    /* 斜坡过渡时间（0=立即；扫频如 50000=50s 爬到目标） */
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
volatile float    g_olf_id_pp_ma  = 0.0f;   /* id 峰峰值 (mA, 每 OLF_PP_WIN_MS 刷新) */
volatile float    g_olf_iq_pp_ma  = 0.0f;   /* iq 峰峰值 (mA, 同上) */
volatile float    g_olf_theta_rad = 0.0f;
volatile float    g_olf_du        = 50.0f;
volatile float    g_olf_dv        = 50.0f;
volatile float    g_olf_dw        = 50.0f;

/* 内部状态 */
static uint32_t s_phase_tick = 0u;    /* 当前阶段计时（tick） */
static uint32_t s_run_tick   = 0u;    /* 拖动阶段计时（tick），频率斜坡时基 */
static uint8_t  s_ramp_done  = 0u;    /* 频率斜坡完成事件已置位（每次运行只置一次） */
static int32_t  s_acc_mdeg   = 0;     /* 步长累积器（mdeg，攒满一个步长跳一步） */

/* id/iq 峰峰值统计（仅拖动态喂数，窗口无缝衔接） */
static uint32_t s_pp_tick = 0u;       /* 窗口计时（tick） */
static uint8_t  s_pp_init = 0u;       /* 首样本初始化标志 */
static float    s_id_min  = 0.0f;
static float    s_id_max  = 0.0f;
static float    s_iq_min  = 0.0f;
static float    s_iq_max  = 0.0f;

/*******************************************************************************
 * 内部助手：峰峰值喂数（ISR 内调用）
 *   每拍更新窗口内 min/max，满 OLF_PP_WIN_TICKS 时锁存峰峰值并以下一拍
 *   样本为新窗口起点（窗口无缝衔接，无重叠无遗漏）。入参单位 A。
 ******************************************************************************/
static void Olf_PpFeed(float id, float iq)
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
    if (++s_pp_tick >= OLF_PP_WIN_TICKS) {
        s_pp_tick = 0u;
        g_olf_id_pp_ma = (s_id_max - s_id_min) * 1000.0f;
        g_olf_iq_pp_ma = (s_iq_max - s_iq_min) * 1000.0f;
        s_id_min = s_id_max = id;   /* 新窗口从当前样本重新起步 */
        s_iq_min = s_iq_max = iq;
    }
}

/*******************************************************************************
 * 内部助手：输出指定电角度的固定磁场 + 刷新观测量（ISR 内调用）
 *   q 轴电压约定（与 mode 30 一致）：入参 theta 为控制系 d 轴角，
 *   电压矢量在 theta+90°（q 轴）：valpha=-V·sin(theta), vbeta=V·cos(theta)，
 *   磁场角 = theta+90°。锁定时控制系电流 g_foc_id_ma≈0, g_foc_iq_ma≈I。
 ******************************************************************************/
static void Olf_OutputField(const stc_i_data_t *pData, float theta)
{
    float fa, valpha, vbeta, du, dv, dw, id, iq;

    fa = theta + FOC_MATH_HALF_PI;          /* 磁场角 = d 轴角 + 90° */
    valpha = g_olf_volt_v * Foc_Math_Cos(fa);
    vbeta  = g_olf_volt_v * Foc_Math_Sin(fa);
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

    /* 控制系（theta d 轴框架）电流观察：Vd=0/Vq=V -> 锁定时 id≈0, iq≈I */
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
    s_run_tick      = 0u;
    s_ramp_done     = 0u;
    g_olf_theta_rad = 0.0f;
    g_olf_field_deg = 0;
    g_olf_rotor_deg = 0;
    g_olf_diff_deg  = 0;
    g_olf_id_ma     = 0.0f;
    g_olf_iq_ma     = 0.0f;
    g_olf_id_pp_ma  = 0.0f;
    g_olf_iq_pp_ma  = 0.0f;
    s_pp_tick       = 0u;
    s_pp_init       = 0u;
    /* 电压复位为默认；当前频率复位为斜坡起点。
     * 频率斜坡三参数 init/targ/tr 不复位（与 mode 27 一致，便于预设后启动），
     * 拖动中 SW1/Watch 改 targ/tr 即按新参数重算轨迹 */
    g_olf_freq_hz   = g_olf_freq_init_hz;
    g_olf_volt_v    = 0.6f;
    /* 步长/方向同步复位（防上次实验残留的 dir=-1 或大步长） */
    g_olf_step_010  = 1;
    g_olf_dir       = 1;
    s_acc_mdeg      = 0;
    /* g_olf_offset 保留上次锁定值（校准完成后覆盖刷新） */

    Foc_Core_PwmStart();   /* 零矢量起 PWM（g_foc_active=1），下一拍开始吸附 */

    OLF_DBG("start calib BETA 90deg f %d->%d mHz tr=%d ms volt=%d mV step=%d010deg dir=%d",
            (int)(g_olf_freq_init_hz * 1000.0f), (int)(g_olf_freq_targ_hz * 1000.0f),
            (int)g_olf_freq_tr_ms, (int)(g_olf_volt_v * 1000.0f),
            (int)g_olf_step_010, (int)g_olf_dir);
}

/*******************************************************************************
 * Foc_Olf_Step - 步进（20 kHz ISR 中调用，禁止打印）
 ******************************************************************************/
void Foc_Olf_Step(const stc_i_data_t *pData)
{
    float theta, rot_rad, id, iq, frq, w;
    uint16_t hw;
    uint32_t tr_ticks;
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
    /* ===== 校准 BETA：磁场定 90°（theta=0, 场=theta+90°），2s ===== */
    case OLF_STEP_CAL_BETA:
        Olf_OutputField(pData, 0.0f);
        if (++s_phase_tick >= OLF_BETA_TICKS) {
            s_phase_tick = 0u;
            g_olf_theta_rad = -FOC_MATH_HALF_PI;   /* ALPHA 场 0° -> theta=-90° */
            g_olf_state  = OLF_STEP_CAL_ALPHA;
            g_olf_evt    = OLF_EVT_BETA_DONE;
        }
        break;

    /* ===== 校准 ALPHA：磁场定 0°（theta=-90°），2s，结束锁零点 -> 拖动 ===== */
    case OLF_STEP_CAL_ALPHA:
        Olf_OutputField(pData, -FOC_MATH_HALF_PI);
        if (++s_phase_tick >= OLF_ALPHA_TICKS) {
            /* 零点：ALPHA 结束时转子 d 轴在静止系 0°（与 mode 20/25 同框架） */
            hw  = TMRA_GetCountValue(CM_TMRA_1);
            off = Foc_Core_ModPos((int32_t)hw * (int32_t)g_foc_enc_dir,
                                  (int32_t)ENCODER_CPR);
            Foc_Core_SetAlignOffset(off);
            g_olf_offset = off;

            g_olf_theta_rad = -FOC_MATH_HALF_PI;   /* 拖动起步场 0°（theta=-90°），与 ALPHA 末角度无缝衔接 */
            s_phase_tick    = 0u;
            s_run_tick      = 0u;
            s_ramp_done     = 0u;
            g_olf_state     = OLF_STEP_DRAG;
            g_olf_evt       = OLF_EVT_LOCKED;
        }
        break;

    /* ===== 拖动：频率斜坡 + 磁场角自增 + 真实转子系观测（实验主体） ===== */
    case OLF_STEP_DRAG:
        /* 0. 频率斜坡：f = init + (targ-init)×w，w = elapsed/tr 线性，
         *    init/targ/tr 每拍实时读 Watch（运行中改 = 按新值重算轨迹）。
         *    tr=0 立即到目标；到点置 RAMP_DONE 一次（与 mode 27 同语义）。 */
        tr_ticks = g_olf_freq_tr_ms * (FOC_ISR_HZ / 1000u);
        if (tr_ticks == 0u) {
            w = 1.0f;
        } else {
            w = (float)s_run_tick / (float)tr_ticks;
            if (w > 1.0f) {
                w = 1.0f;
            }
        }
        frq = g_olf_freq_init_hz
            + (g_olf_freq_targ_hz - g_olf_freq_init_hz) * w;
        g_olf_freq_hz = frq;
        if ((s_ramp_done == 0u) && (s_run_tick >= tr_ticks)) {
            s_ramp_done = 1u;
            g_olf_evt   = OLF_EVT_RAMP_DONE;
        }
        s_run_tick++;

        /* 1. 磁场角步长量化自增：
         *    目标转速 = 360°×freq /s -> 每拍累积 mdeg，攒满一个步长
         *    (g_olf_step_010×0.1°) 沿 g_olf_dir 方向跳一步。
         *    等效自增节拍 = 360×freq/step 次/秒；step=1 时≈连续旋转。
         *    tr=0 或斜坡已完成时改 targ 阶跃生效，大改动会失步。 */
        theta = g_olf_theta_rad;
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

        /* 2. q 轴电压约定输出磁场（场 = theta+90°，函数内处理） */
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
        Olf_PpFeed(id, iq);   /* 峰峰值统计（5s 窗口刷新） */

        /* 5. 角度观测（整型电角度 deg）+ 负载角折叠 (-180,180]
         *    磁场角 = theta+90°（q 轴约定），theta∈[0,2π) -> fld∈[90,449]，
         *    超 360 减一圈 */
        fld_deg = (int32_t)((theta + FOC_MATH_HALF_PI) * OLF_RAD2DEG);
        if (fld_deg >= 360) {
            fld_deg -= 360;
        }
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

/*===========================================================================
 * mode 26 自持 VOFA：固定 16ch 布局
 * 通道含义速览卡 = foc_26_olf.h 顶部【模式速览卡】（唯一事实源）
 *
 * 本模式是"开环 VF 负载角实验"，**核心观测量是 ch7 的 delta**——
 * 它随频率的走势就是实验结论（摩擦锥 → 缓升 → 逼近 90° 牵出）。
 * ch3/ch4 给出参考转速与实际转速，ch12/ch13 是相位裕度检查量。
 * 单位换算：传"毫单位"，SendScaled 内部 ×0.001
 *===========================================================================*/
int Foc_Olf_VofaFill(int32_t *cur)
{
    cur[0]  = (int32_t)(g_i_iu_ma);                 /* ch0  U 相电流 (mA -> A) */
    cur[1]  = (int32_t)(g_i_iv_ma);                 /* ch1  V 相电流 */
    cur[2]  = (int32_t)(g_i_iw_ma);                 /* ch2  W 相电流 */
    cur[3]  = (int32_t)(g_olf_freq_hz * 1000.0f);   /* ch3  磁场自增频率 (mHz -> Hz) */
    cur[4]  = (int32_t)(g_olf_freq_targ_hz * 1000.0f);/* ch4 目标频率（SW1 可调） */
    cur[5]  = (int32_t)(g_olf_iq_ma);               /* ch5  iq（力矩分量） */
    cur[6]  = (int32_t)(g_olf_id_ma);               /* ch6  id（磁链分量，高频转负） */
    cur[7]  = (int32_t)(g_olf_diff_deg * 1000);     /* ch7  **负载角 delta (mdeg -> deg) ← 实验主曲线** */
    cur[8]  = (int32_t)(g_olf_field_deg * 1000);    /* ch8  磁场电角度 (mdeg -> deg) */
    cur[9]  = (int32_t)(g_olf_rotor_deg * 1000);    /* ch9  转子电角度 */
    cur[10] = (int32_t)(g_olf_volt_v * 1000.0f);    /* ch10 拖动电压幅值 (mV -> V) */
    cur[11] = (int32_t)(g_olf_iq_pp_ma);            /* ch11 iq 峰峰值（失步后剧烈振荡） */
    cur[12] = (int32_t)(g_olf_id_pp_ma);            /* ch12 id 峰峰值 */
    cur[13] = (int32_t)(g_olf_theta_rad * 1000.0f); /* ch13 控制系 d 轴角 (mrad -> rad) */
    cur[14] = (int32_t)(g_olf_dir);                 /* ch14 自增方向 +1/-1 */
    cur[15] = (int32_t)(g_olf_state);               /* ch15 状态 0IDLE/1BETA/2ALPHA/3DRAG/4OC */
    return 16;
}
