/**
 *******************************************************************************
 * @file  foc_lock_iq_pi.c
 * @brief FOC 模式32 — 自锁偏移 + 自动进入 mode 31 实现。
 *
 *        偏移注入值推导（关键）：
 *
 *          mode 31 每次启动清零自身相对编码器计数（s_enc_pos = 0），其
 *          角度管线为：
 *              theta_used = wrap( wrap(Θ - Θ_s31) - off + 90° )
 *          其中 Θ_s31 = mode 31 启动瞬间的转子真实电角度。要求
 *          theta_used = Θ（iq 电流矢量落在转子 d 轴前方 90°），必须：
 *              off = 90° - Θ_s31
 *
 *          ALPHA 直流对齐把转子 d 轴吸到静止系 0°——这是不依赖编码器的
 *          物理真值；锁定后至 mode 31 闭环起动前全程零矢量，转子保持在
 *          对齐位，故 Θ_s31 ≈ 0（残余仅为几度静摩擦滞后角）：
 *              off_inject = 90° = FOC_MATH_HALF_PI（恒定，与停位无关）
 *
 *          警告：不可用 ElecFromPos(ALPHA 位置) 推 off。mode 32 的相对
 *          计数零点在 mode 32 启动时刻，与 mode 31 的零点不同，任何由
 *          本模式计数推出的 off 都携带随机停位项——这正是此前 ±90° 两
 *          个版本都 OC 的根因。mode 30 锁定值编码的是上电位置，同理仅
 *          在转子恰好停在特定位置时对 mode 31 有效。
 *
 *          使用约束：off=90° 仅在 mode 32 -> 自动交接 mode 31 链路上
 *          成立（转子必须仍停在对齐位）；交接后转子被动过再单独进
 *          mode 31 是无效的。
 *
 *        方向基准推导：iq>0 时转子向 +Θ 加速，编码器计数增量符号 =
 *          sign(g_foc_enc_dir)（由 enc_elec = pos·enc_dir 映射关系直接
 *          得出），故注入 ref_dir = FOC_ENC_DIR。实测佐证：FOC_ENC_DIR=-1
 *          与 mode 30 五次拖动 drag_dir=-1 一致。
 *
 *        相位时序（沿用 mode 23 的两段式单侧逼近，破坏摩擦迟滞）：
 *          BETA(磁场 90°, 盲等) -> ALPHA(磁场 0°, 等静止, 取锁定位置)
 *          -> VERIFY(磁场 90°, 等静止, 校验位移 ≈ +90°电角度)
 *          -> 锁定注入 -> 零矢量等 main.c 交接。
 *
 *        注入通道：foc_zizeng 的 SetOffsetRad/SetDragDir（刻意复用
 *        mode 31 现有取值路径，foc_iq_pi.c 保持零改动）。
 *******************************************************************************
 */

#include "foc_lock_iq_pi.h"
#include "foc_obs.h"
#include "foc_math.h"
#include "foc_zizeng.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "rtt_log.h"
#include "hc32_ll_tmra.h"   /* 直接读取 TIMERA_1 计数器 */

/*******************************************************************************
 * Keil Watch 可调变量 / 观测量
 ******************************************************************************/
volatile uint8_t g_lockiq_running  = 0;
volatile lockiq_step_t g_lockiq_step = LOCKIQ_STEP_IDLE;
volatile float   g_lockiq_align_volt_v = FOC_ALIGN_VOLT_V;
volatile int32_t g_lockiq_off_mrad     = 0;

/* 注：窗口诊断量（g_lockiq_win_moved/win_evals/track_err_cnts）与
 * 锁定/失败事件快照（g_lockiq_evt_*）已集中迁移到 foc_obs.c/.h
 * 观察模块，变量名未变，Keil Watch 用法不变。 */

/*******************************************************************************
 * 内部状态
 ******************************************************************************/
static uint32_t s_phase_tick = 0u;    /* 当前相位已运行 ISR tick（超时用） */

/* 编码器硬件计数跟踪（与 foc_zizeng/foc_iq_pi 相同的回绕处理，独立状态） */
static int32_t s_prev_hw_cnt = 0;
static int32_t s_enc_pos = 0;
static uint8_t s_enc_initialized = 0;

/* 稳定判据窗口（ALPHA/VERIFY 共用机制） */
static float    s_win_sum = 0.0f;     /* 窗口内位置采样累加 */
static uint32_t s_win_cnt = 0u;
static float    s_win_ref = 0.0f;     /* 上一窗口平均位置 */
static uint8_t  s_quiet = 0u;         /* 连续静止窗口计数 */

/* ALPHA 锁定快照：静止窗口平均位置（counts）——仅用于 VERIFY 位移校验 */
static float    s_alpha_avg = 0.0f;

/*******************************************************************************
 * LockIqPi_OutputVolt - 对齐加压输出（磁场定 theta，模式23同款路径）
 ******************************************************************************/
static void LockIqPi_OutputVolt(const stc_i_data_t *pData, float theta)
{
    float valpha, vbeta, du, dv, dw;
    float id, iq;

    valpha = g_lockiq_align_volt_v * Foc_Math_Cos(theta);
    vbeta  = g_lockiq_align_volt_v * Foc_Math_Sin(theta);
    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);

    g_foc_valpha = valpha;
    g_foc_vbeta  = vbeta;
    g_foc_vd = valpha;
    g_foc_vq = vbeta;
    g_foc_du = du;
    g_foc_dv = dv;
    g_foc_dw = dw;
    g_foc_theta_rad = theta;

    /* 对齐阶段电流仅作 Watch 观测（未校零偏，含残余零偏属正常） */
    Foc_Core_GetDq(pData, theta, &id, &iq);
    g_foc_id_ma = id * 1000.0f;
    g_foc_iq_ma = iq * 1000.0f;
}

/*******************************************************************************
 * LockIqPi_ResetWindow - 复位稳定判据窗口（进入新相位时调用）
 ******************************************************************************/
static void LockIqPi_ResetWindow(void)
{
    s_win_sum = 0.0f;
    s_win_cnt = 0u;
    s_quiet   = 0u;
    s_win_ref = (float)s_enc_pos;
}

/*******************************************************************************
 * LockIqPi_LockAndSignal - 锁定注入 + 置事件，进入交接等待
 ******************************************************************************/
static void LockIqPi_LockAndSignal(void)
{
    /* off 恒为 90°：mode 31 清零自身计数后要求 off = 90° - Θ_s31，
     * 对齐已把 Θ_s31 定义为 ≈0（静止系真值，见文件头推导），
     * 与 mode 32 启动时的转子停位无关 */
    float off_inject = FOC_MATH_HALF_PI;

    /* 注入 mode 31 取值路径（foc_iq_pi.c 零改动） */
    Foc_Zizeng_SetOffsetRad(off_inject);
    /* 本征方向基准：iq>0 => 计数增量符号 = sign(ENC_DIR)（见文件头推导） */
    Foc_Zizeng_SetDragDir((int8_t)FOC_ENC_DIR);

    g_lockiq_off_mrad = (int32_t)(off_inject * 1000.0f);

    g_lockiq_evt_code     = LOCKIQ_EVT_LOCKED;
    g_lockiq_evt_off_mrad = g_lockiq_off_mrad;
    g_lockiq_evt_flag     = 1u;

    g_lockiq_step = LOCKIQ_STEP_LOCKED_WAIT_HANDOFF;
}

/*******************************************************************************
 * Foc_LockIqPi_Start - mode 32 启动（主循环上下文调用）
 ******************************************************************************/
void Foc_LockIqPi_Start(void)
{
    if (g_lockiq_align_volt_v <= 0.0f) {
        g_lockiq_align_volt_v = FOC_ALIGN_VOLT_V;   /* 防止 Watch 改坏后卡死 */
    }

    Foc_Core_ClearFault();
    g_lockiq_running = 1;
    g_lockiq_step    = LOCKIQ_STEP_ALIGN_BETA;

    g_lockiq_off_mrad       = 0;
    g_lockiq_win_moved      = 0;
    g_lockiq_win_evals      = 0;
    g_lockiq_track_err_cnts = 0;
    g_lockiq_evt_flag       = 0;
    g_lockiq_evt_code       = 0;
    g_lockiq_evt_off_mrad   = 0;

    s_phase_tick = 0u;
    s_enc_initialized = 0;
    s_enc_pos = 0;
    LockIqPi_ResetWindow();
    s_alpha_avg = 0.0f;

    TMR4_PWM_SetFocMode(FOC_DEADTIME_NS);
    g_foc_du = 50.0f;
    g_foc_dv = 50.0f;
    g_foc_dw = 50.0f;
    TMR4_PWM_SetDuty3Phase(50.0f, 50.0f, 50.0f);
    TMR4_PWM_StartOutput();
    g_foc_active = 1u;
    g_foc_mode   = FOC_MODE_NONE;

    g_foc_id_ma = 0.0f;
    g_foc_iq_ma = 0.0f;
    g_foc_vd    = 0.0f;
    g_foc_vq    = 0.0f;
    Foc_Core_ResetEma();

    LOCKIQ_DBG("Started: volt=%d mV, beta=%d ms, win=%d ms, tol=%d",
               (int)(g_lockiq_align_volt_v * 1000.0f + 0.5f),
               (int)LOCKIQ_BETA_MS, (int)LOCKIQ_WIN_MS,
               (int)LOCKIQ_WIN_TOL_CNTS);
}

/*******************************************************************************
 * Foc_LockIqPi_Stop - mode 32 停止
 ******************************************************************************/
void Foc_LockIqPi_Stop(void)
{
    /* 停止不清状态：g_lockiq_step 保持最后状态（如 LOCK_FAIL/FAULT_OC），
     * Watch 可查停机原因；下次成功启动时复位 */

    g_lockiq_running = 0;
    g_foc_active     = 0u;
    TMR4_PWM_EmergencyStop();
    g_foc_du = 0.0f;
    g_foc_dv = 0.0f;
    g_foc_dw = 0.0f;
    LOCKIQ_DBG("Stopped");
}

/*******************************************************************************
 * Foc_LockIqPi_Step - 模式32 单步运算（20 kHz ISR 中调用）
 ******************************************************************************/
void Foc_LockIqPi_Step(const stc_i_data_t *pData)
{
    uint32_t hw_cnt;
    int16_t delta;

    if (!g_lockiq_running) {
        return;
    }

    /* 过流保护（原始 pData） */
    if (Foc_Core_OverCurrent(pData)) {
        Foc_Core_FaultStop(1u);
        g_lockiq_running = 0;
        g_lockiq_step = LOCKIQ_STEP_FAULT_OC;   /* 停机后保持，Watch 可查 */
        return;
    }

    /* ===== 编码器硬件计数（与 ZIZENG/IQPI 相同的回绕处理） ===== */
    hw_cnt = TMRA_GetCountValue(CM_TMRA_1);
    if (!s_enc_initialized) {
        s_prev_hw_cnt = (int32_t)hw_cnt;
        s_enc_pos = 0;
        s_enc_initialized = 1;
    }
    delta = (int16_t)((uint16_t)hw_cnt - (uint16_t)s_prev_hw_cnt);
    s_prev_hw_cnt = (int32_t)hw_cnt;
    s_enc_pos += (int32_t)delta;
    g_enc_count = s_enc_pos;
    g_enc_count_f = (float)s_enc_pos;

    /* ===== 锁定/失败保持态：零矢量等待 main.c 交接或停机 ===== */
    if ((g_lockiq_step == LOCKIQ_STEP_LOCKED_WAIT_HANDOFF) ||
        (g_lockiq_step == LOCKIQ_STEP_LOCK_FAIL)) {
        TMR4_PWM_SetDuty3Phase(50.0f, 50.0f, 50.0f);
        g_foc_du = 50.0f;
        g_foc_dv = 50.0f;
        g_foc_dw = 50.0f;
        g_foc_id_ma = 0.0f;
        g_foc_iq_ma = 0.0f;
        g_foc_vd = 0.0f;
        g_foc_vq = 0.0f;
        return;
    }

    /* ===== 对齐加压（BETA: 90° / ALPHA: 0° / VERIFY: 90°） ===== */
    if (g_lockiq_step == LOCKIQ_STEP_ALIGN_BETA) {
        LockIqPi_OutputVolt(pData, FOC_MATH_HALF_PI);
    } else if (g_lockiq_step == LOCKIQ_STEP_ALIGN_ALPHA) {
        LockIqPi_OutputVolt(pData, 0.0f);
    } else if (g_lockiq_step == LOCKIQ_STEP_ALIGN_VERIFY) {
        LockIqPi_OutputVolt(pData, FOC_MATH_HALF_PI);
    } else {
        return;
    }

    s_phase_tick++;

    /* ===== BETA: 盲等吸附完成 -> ALPHA ===== */
    if (g_lockiq_step == LOCKIQ_STEP_ALIGN_BETA) {
        if (s_phase_tick >= LOCKIQ_BETA_CNT) {
            g_lockiq_step = LOCKIQ_STEP_ALIGN_ALPHA;
            s_phase_tick = 0u;
            LockIqPi_ResetWindow();
        }
        return;
    }

    /* ===== ALPHA/VERIFY: 窗口静止判据（共用机制） ===== */
    {
        s_win_sum += (float)s_enc_pos;
        s_win_cnt++;

        if (s_win_cnt >= LOCKIQ_WIN_CNT) {
            float avg = s_win_sum / (float)s_win_cnt;
            float moved = avg - s_win_ref;
            float moved_abs = (moved < 0.0f) ? -moved : moved;

            s_win_sum = 0.0f;
            s_win_cnt = 0u;

            g_lockiq_win_moved = (int32_t)moved;
            g_lockiq_win_evals++;

            if (moved_abs <= (float)LOCKIQ_WIN_TOL_CNTS) {
                s_quiet++;
            } else {
                s_quiet = 0u;
            }
            s_win_ref = avg;

            if (s_quiet >= LOCKIQ_QUIET_NEED) {
                if (g_lockiq_step == LOCKIQ_STEP_ALIGN_ALPHA) {
                    /* ALPHA 稳定 -> 快照锁定位置，进入 VERIFY 复测 */
                    s_alpha_avg = avg;
                    g_lockiq_step = LOCKIQ_STEP_ALIGN_VERIFY;
                    s_phase_tick = 0u;
                    LockIqPi_ResetWindow();
                    return;
                }

                /* VERIFY 稳定 -> 校验位移后锁定注入 */
                {
                    float moved_v = avg - s_alpha_avg;
                    float expect = (float)FOC_ENC_DIR
                                 * (float)ENCODER_CPR
                                 / (4.0f * (float)FOC_POLE_PAIRS);
                    float err = moved_v - expect;
                    if (err > (float)(ENCODER_CPR / 2)) {
                        err -= (float)ENCODER_CPR;
                    }
                    if (err < -(float)(ENCODER_CPR / 2)) {
                        err += (float)ENCODER_CPR;
                    }
                    g_lockiq_track_err_cnts = (int32_t)err;

                    if ((err >= -(float)LOCKIQ_TRACK_TOL_CNTS) &&
                        (err <=  (float)LOCKIQ_TRACK_TOL_CNTS)) {
                        LockIqPi_LockAndSignal();
                    } else {
                        /* 转子跟踪失败（卡死/堵转/编码器异常）：
                         * 拒绝锁定，宁可不起动 */
                        g_lockiq_evt_code = LOCKIQ_EVT_FAIL_TRACK;
                        g_lockiq_evt_off_mrad = 0;
                        g_lockiq_evt_flag = 1u;
                        g_lockiq_step = LOCKIQ_STEP_LOCK_FAIL;
                    }
                }
                return;
            }
        }

        /* 相位超时（ALPHA/VERIFY 各自独立计时） */
        if (s_phase_tick >= LOCKIQ_TIMEOUT_CNT) {
            g_lockiq_evt_code = LOCKIQ_EVT_FAIL_TIMEOUT;
            g_lockiq_evt_off_mrad = 0;
            g_lockiq_evt_flag = 1u;
            g_lockiq_step = LOCKIQ_STEP_LOCK_FAIL;
        }
    }
}
