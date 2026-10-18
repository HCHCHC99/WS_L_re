# HC32F460 BLDC 六步换相 / 3霍尔 / BEMF 精读报告（美的面试材料：压缩机·风机驱动，无感FOC方向）

> 工程根：`HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/`
> 硬件平台：HC32F460（Cortex-M4）+ 6288T-MNS 三相门驱动 + 3×霍尔120°安装 + TMR4 Unit3 中心对齐 PWM（`MOTOR_PWM_FREQ_HZ 20000u`，`ws/motor_config.h:30`）
> 本文所有结论带 `文件:行号` 锚点；数值参数原样引用。

---

## 0. 先讲一个"演化事实"（面试开场就要说清，否则会被追问打穿）

**当前工作树的 `dev_comm_runner.c` 已经重构为 FOC 模式分发器，六步换相 FSM 已不在活代码中。**

- 现行模式枚举（`ws/dev_comm_runner.h:23-42`）：`STOP=0, DCAL=24, CAL=20, CAL_ANGLE=25, OLF=26, DCL=27, DCI=28, DRUN=29, ZIZENG=30, IQ_PI=31, LOCK_IQ_PI=32, SPEED_FOC=40, DRUN41=41, SMO45=45, RS51=51, LDLQ52=52, FLX53=53, LUEN60=60` —— **没有** OPEN_FW/CLOSED_FW/CURLOOP_FW/CASCADE_FW（模式0-11 的六步时代编号）。
- git 考古（仓库根 `D:\WS_L_re\ws_L_v.inmop_curloop_re`，initial commit `ca8ece6`）：全历史 `git grep` 证实 `s_hall2step_cw` / `CALIB_FAIL_TIMEOUT` / `OPEN_FW` 这些六步符号**只存在于文档**（`CLAUDE.md`、`md_record/六步方波.md`、`md_record/闭环校准说明.md`、`md_record/角度30度90度分析.md`、`step解释.html`），**从未进入任何已提交的 .c**。initial commit 的 runner 就已是 FOC 精简版。⇒ 六步 FSM 版 runner 早于 git 建仓，代码级唯一来源是 CLAUDE.md + md_record 文档。
- 但六步的**底层仍然活着**：`dev_commutation.c`（六步表+懒更新PWM）与 `hall_sensor_3ch.c`（3霍尔驱动）仍在树中；只是 `hall_3ch_create` 在全工程**无调用者**（仅 `hall_sensor_3ch.h:59` 声明 + `hall_sensor_3ch.c:296` 定义）——孤儿模块，保留供六步实验。佐证：`MOTOR_HALL_ENABLE 0`（`ws/motor_config.h:33`，PA8/9/10 已让给编码器 ABZ）。
- 项目现在的重心恰是面试方向：`ws/` 下有完整 FOC 栈——`foc_core.c`、`foc_math.c`、`foc_curloop.c`、`foc_pll.c`、`foc_smo.c`（SMO 滑模观测器）、`foc_obs.c`、`foc_luen.c`（Luenberger）、`foc_45_smo.c`（mode45 无感）、`foc_53_flx.c`（磁链观测）、`cur_loop.c`、`speed_loop.c`、`I.c`（三电阻采样）。**"从方波六步+霍尔 → 无感FOC"的完整演化路径就是本项目最大亮点。**

---

## 1. dev_commutation.c / dev_commutation.h —— 六步换相表与懒更新PWM（仍在活代码）

### 1.1 职责与数据流
纯查表执行器：`Commutation_Step(step, duty)` → 查 `s_states[6][6]` 得三相各自的 半桥模式+占空比类别 → 经懒更新缓存比对 → 只把**真正变化**的寄存器写入 TMR4。上层（runner/开环斜坡/霍尔ISR回调）只给它"第几步+占空比"。

### 1.2 关键函数（`ws/dev_commutation.c`）
| 签名 | 锚点 |
|---|---|
| `void Commutation_Init(uint32_t freq_hz)` | c:68-78（缓存强制 0xFF 防陈旧命中） |
| `void Commutation_Step(uint8_t step, float duty_pct)` | c:86-148 |
| `uint8_t Commutation_GetPwmChannel(uint8_t step)` | c:149-160（返回 M_HIGH 所在通道，否则 0xFF） |
| `void Commutation_SetActiveDuty(float duty_pct)` | c:162-184（只改活跃相占空比，同步缓存 c:183） |
| `void Commutation_Stop(void)` | c:190-197（三相全 M_OFF） |

### 1.3 六步换相表（必须能默写）
步宏（`ws/dev_commutation.h:35-40`）：`0=UH_VL, 1=UH_WL, 2=VH_WL, 3=VH_UL, 4=WH_UL, 5=WH_VL`。
通道模式枚举 `M_OFF=0/M_HIGH=1/M_LOW=2`、占空比类别 `D_PWM=0/D_MIN=1/D_OFF=2`（c:15-16）。

```c
static const uint8_t s_states[6][6] = {      /* U_mode U_duty V_mode V_duty W_mode W_duty */
    /* UH_VL */ { M_HIGH, D_PWM, M_LOW, D_MIN, M_OFF, D_OFF },   /* 磁场角330° */
    /* UH_WL */ { M_HIGH, D_PWM, M_OFF, D_OFF, M_LOW, D_MIN },   /*  30° */
    /* VH_WL */ { M_OFF, D_OFF, M_HIGH, D_PWM, M_LOW, D_MIN },   /*  90° */
    /* VH_UL */ { M_LOW, D_MIN, M_HIGH, D_PWM, M_OFF, D_OFF },   /* 150° */
    /* WH_UL */ { M_LOW, D_MIN, M_OFF, D_OFF, M_HIGH, D_PWM },   /* 210° */
    /* WH_VL */ { M_OFF, D_OFF, M_LOW, D_MIN, M_HIGH, D_PWM },   /* 270° */
};                                               /* c:18-26 */
```
磁场角元数据 `s_step_meta`（c:33-40）：330/30/90/150/210/270，每步跨 60°电角度，构成完整电周期。要点：**导通相高侧 D_PWM（调制）、低侧 D_MIN（2% 常通防续流异常）、悬空相 M_OFF**。

### 1.4 6288T-MNS 真值表与 THROUGH/HIGH_SIDE/LOW_SIDE（`ws/dev_commutation.h:18-26`）
| HIN | LIN | HO | LO | 状态 |
|---|---|---|---|---|
| L | L | L | L | **双关** |
| L | H | L | H | 低侧导通 |
| H | L | H | L | 高侧导通 |
| H | H | — | — | **双关（H=L 关断）** |

- **6288T-MNS：H≠L 才导通，H=L 关断——与 SDH21263（同相为通/另一逻辑族）相反**。这是换门驱动芯片时最容易翻车的点。
- PWM 抽象三模式（`Adp/tmr4_pwm.c/h`）：`THROUGH`（直通/使能输出）、`HIGH_SIDE`（H 调制+L 关）、`LOW_SIDE`（H 关+L 常通）。全桥 PWM 统一走 THROUGH 模式 + 影子寄存器 PEAK 同步传输 → **换相瞬间无毛刺**。
- 占空比硬限制：`COMM_DUTY_MIN_F 2.0f / COMM_DUTY_MAX_F 98.0f / COMM_DUTY_OFF_F 0.0f`（`dev_commutation.h:30-32`），`Commutation_Step` 入口即钳位（c:94-100）——保住电荷泵自举的最小关断时间。

### 1.5 懒更新（Lazy Update）缓存——面试亮点
缓存 `s_last_freq + s_last_ch_mode[3] + s_last_ch_duty[3]`（c:61-63）：
1. 频率变 → `TMR4_PWM_SetFrequency()` + 缓存全失效（c:103-110）；
2. 某相模式变 → `TMR4_PWM_SetChannelMode()` 全通道重初始化（c:126-138）；
3. **模式不变仅占空比变 → `TMR4_PWM_SetDutyFloat()` 只写 OCCR 比较寄存器**（c:139-143）；
4. 全都相同 → 直接跳过（c:144）。
收益：霍尔 ISR 里每 60° 电角度才换一次模式，两次换相之间的占空比调节（调压调速）退化为一次 OCCR 写，且经影子寄存器在周期边界生效，杜绝毛刺。

---

## 2. dev_comm_runner.c / .h —— 顶层模式状态机（现行FOC版 + 文档版六步FSM）

### 2.1 现行代码（FOC 分发器）
- `CommRunner_Init()` c:22-59：`TMR4_PWM_Config` 三相 `output_type=TMR4_OUTPUT_SYNC`、`dead_time_ns=0`（死区由 6288T 硬件负责）、`active_high=true`（c:31-38）；Timer6 µs 时基；三通道置 `TMR4_MODE_OFF` 待机（c:47-49）。
- `CommRunner_SetMode()` c:126-263：**开头 `if (mode == s_mode) return;` 同模式早退（c:129）**；每个 case = `CommRunner_StopFocModes()`（c:65-121，按 `g_*_running` 标志逐个停 17 个 FOC 子模块）+ `Commutation_Stop()` + 对应 `Foc_*_Start()`；STOP 分支**保留 `TMR4_PWM_StartOutput`**（c:137）——PWM 停转但不停触发，维持 VALLEY/PEAK ADC 采样链（电流/BEMF 观测不掉线）。
- `CommRunner_ReleaseMode()` c:287-290（声明 h:95-98）：**只把 `s_mode` 放回 STOP、不清 FOC 模块**——历史 bug 根因：配合 c:129 的同模式早退，会"吃掉"重测请求（mode52 LDLQ 重测问题）。
- `CommRunner_SetDuty()` 钳位 2.0~98.0%（c:295-300）；`CommRunner_Update()` 已退化为空操作（c:313-317）；`SetTargetRPM→Foc_Speed_SetTargetRPM`（c:326-329）、`GetTargetRPM→g_speed40_speed_target_rpm`（c:331-334）；`GetRPM/IsRunning/IsStalled/CurLoopActive` 均为桩（c:322-325）。
- VOFA 通道数表 h:61-68：mode0 通用 20 通道，硬上限 `USART3_VOFA_MAX_CHANNELS(24)`，`main.c` 接收缓冲 `cur[32]`。
- `main.c` 侧：Keil Watch 变量 `comm_mode` 分发到 `CommRunner_SetMode()`，`main.c:319-340` 对 mode30/26/31 有特殊映射；`check_noisy_mode` 非 0 先强制 `comm_mode=0`。

### 2.2 文档版六步 FSM（CLAUDE.md + md_record，面试按此讲架构思想）
模式流：`STOP(0) → OPEN_FW(1)/OPEN_RV(2)`（定时开环斜坡）`→ CLOSED_FW(3)/CLOSED_RV(4)`（开环斜坡→flying start→霍尔接管）`→ CALIB(5)`（开环+霍尔边沿采样→0°偏移表）`→ CALIB_CW(6)/CALIB_CCW(7)`（载入派生表闭环）`→ CURLOOP_FW(10)/CASCADE_FW(11)`。
旧文档时期（md_record/六步方波.md）：STOP = 三相全 HIGH_SIDE + 98% 占空比 = **三相短接至 DC+ 刹车**；PWM 当时 50kHz，现 20kHz。

---

## 3. hall_sensor_3ch.c / .h —— 3霍尔ISR、M法测速、噪声防御（孤儿模块但逻辑完整）

### 3.1 职责与FSM
3通道霍尔（U=PA10/EXTINT_CH10、V=PA9/CH09、W=PA8/CH08，`hall_sensor_3ch.h:32-52` 配置结构含 `hall_to_step[8]`（0xFF=非法）、`on_step/on_fault` 回调、`align_step/align_duty_pct/align_duration_ms`、`stall_timeout_ms`）。FSM：`STATE_IDLE/ALIGNING/RUNNING/FAULT`（c:32-37）。J-Scope 全局 `g_scope_ha/hb/hc/step/rpm`（h:80-85）。

### 3.2 关键函数/ISR
| 签名 | 锚点 |
|---|---|
| `static void register_hall_irq(const hall_3ch_config_t*, uint8_t ch)` | c:127-166 |
| `void INT010/009/008_IRQHandler(void)` → `hall_u/v/w_isr` → `hall_common_handler(ch)` | c:169-171 |
| `static uint8_t read_hall_state_raw(void)`：`state = HA<<2 | HB<<1 | HC` | c:174-181 |
| `static void hall_common_handler(uint8_t ch)` | c:185-278 |
| `hall_3ch_start_flying(inst, table, dir)` | c:364-413 |
| `void hall_3ch_update(inst)`（1ms 主循环调用） | c:452-573 |
| `hall_3ch_update_rpm_filter`（6点滑动平均） | c:281-293 |

### 3.3 ISR 噪声防御：现存 3 层 + 已删 2 层（高频考点）
**现存：**
1. **硬件滤波**：`EXTINT_TRIG_BOTH + EXTINT_FILTER_ON + EXTINT_FCLK_DIV64`（c:142-144）。滤波时钟≈PCLK1/64≈1.56MHz，可滤 <约10µs 毛刺——PWM 开关耦合噪声（沿宽 ns~µs 级）在硬件层直接吃掉。
2. **非法编码拒绝**：`hall_to_step[state]==0xFF`（000/111）→ `g_dbg_isr_fault++`；RUNNING 中则转 `STATE_FAULT + on_fault`（c:205-214）。
3. **步进邻接检查**：`diff=(step-old+6)%6`，≠1 且 ≠5 → `g_dbg_isr_baddiff++`，**但仅诊断计数、不拦截**（c:239-248，注释"diagnostic only... always accept its reading"）。
**已删（化石仍在）**：`g_dbg_isr_unchanged` / `g_dbg_isr_noise` 计数器已定义但永不递增（c:101-111）；`MIN_PULSE_INTERVAL_US 50u` 常量已无 ISR 引用（c:12-17）。被删的两层是：①"状态未变早退"②"<50µs 间隔拒绝"。删除原因：三路独立 EXTI 会**被同一次 PWM 开关噪声同时触发**，噪声沿各路到达时间差小于 50µs 时这层反而把合法边沿拒掉；实测每次噪声爆发产生 3 条 BAD，`baddiff/valid≈13%`——BAD 的存在恰恰证明查表在正确工作（拒的是非法跳变），不是表错。遗留 todo：ISR 重入锁（首个 ISR 设忙标志，100µs 内后续直接返回）。

**ISR 主体流程**（c:185-278）：清 EXTI 标志 → 读三相 → 查表 → 非法码处理 → `g_scope_step` 同步（c:217）→ Timer6 计数差（**含 65536 回绕处理**，c:220-227）→ 非 RUNNING 只记录返回（c:230-237）→ 邻接诊断（c:239-248）→ 更新跟踪态 → `display_step±1`（c:260-264）→ `on_step(step)` 回调（内部再走 `Commutation_Step` → TMR4 影子寄存器），回调耗时测量 `g_dbg_isr_time_us/max`（c:267-277）。

### 3.4 M法测速（M-method）
- 常量（c:12-17）：`RPM_WINDOW_SIZE 6`、`RPM_UPDATE_MIN_US 20000u`、`RPM_UPDATE_MIN_PULSES 6u`、`RPM_TIMEOUT_US 500000u`。
- 公式（c:462-473）：**主循环窗口期 `elapsed≥20000µs 且脉冲增量≥6` 才更新：`rpm = pulse_delta * 60000000 / (elapsed_us * pole_pairs * 6)`**，封顶 100000。分母的 `pole_pairs*6` = 每机械转脉冲数（3霍尔×2边沿×极对数个电周期）；极对数3 → 18 脉冲/转。
- 超时归零：`elapsed>500000µs 且 delta=0` → RPM=0（c:474-481）。
- 6点滑动平均 `update_rpm_filter`（c:281-293）。
- **分辨率账**（面试常问）：3000rpm、18脉冲/转 → 900脉冲/s，20ms 窗口≈18 脉冲 → 量化步进 ≈3000/18≈167rpm（±8%）；2000rpm 时窗口仅 12 脉冲，分辨率恶化到 ~333rpm/步。低速下 M 法精度不足 → 改 T 法（测周法）或 M/T 混合。**旧模块 Motor_hall 就是 T 法**（见 §5）——两种方法在同一工程里都有实例，对比着讲非常加分。
- 堵转检测坑（c:513-529）：曾用 64 位 Timer6 微秒计数做 `since_pulse`，**主循环读 64 位量无原子性 → 高低字节撕裂 → 假堵转（age=14天）**；已改用 1ms tickTimer 的 32 位毫秒计数（单读原子），且 `since_pulse_ms≥0x80000000` 视为时钟抖动忽略。
- ALIGNING→RUNNING 转换（c:486-510）：对齐时间到 → 置 RUNNING + `kick_step=align_step±1` 强制第一步换相。

### 3.5 飞行启动 `hall_3ch_start_flying`（c:364-413，判据精确版）
1. 读霍尔 GPIO → **100 次 NOP 延时** → 再读（c:381-383）；
2. 两次值不同 ⇒ 转子在动 ⇒ **取后一次（更新）读数**（c:385-388）；
3. 查 `hall_to_step`，非法码 0xFF → **兜底 step 0**（c:390-393）；
4. **直接 `state=STATE_RUNNING`，跳过 ALIGNING**（c:395）；
5. 同步 `g_scope_*`（c:400-403）→ **立即回调 `on_step(step)` 执行第一次闭环换相**（c:407-409）。
设计逻辑：开环斜坡已把电机拖转，首拍即使错 1 步，下一个霍尔沿（≤60°电角度后）即自纠。

### 3.6 霍尔校准状态机（文档版，md_record/闭环校准说明.md 299行）
- 流程：开环 5000µs/步恒速 → **500ms 稳定期** → 主循环 ~1kHz 轮询 `hall_3ch_read_raw()` 边沿检测，记录 `pending_map[hall]=当前step` → **6 态齐后双校验（双射 + 与参考周期一致）→ 首个有效周期锁参考表 → 连续 12 个一致周期 → SUCCESS**，电机自动停。
- 状态码 0-8：`IDLE/RUNNING/SUCCESS/TIMEOUT(3000ms)/STALL/MISSING/DUPLICATE/INVALID(0x00或0x07反复出现)/AMBIGUOUS(legacy)`。
- 失败诊断：TIMEOUT 时查 `g_calib_hall_seen[1..6]`，`g_calib_error.hall_state` = 未见态 bitmask，**0x7E=全未见 → 优先查霍尔接线/供电**。
- 产物：`g_calib_table[1..6]`（0偏移映射），槽 0/7=0xFF。
- **派生表偏移——两处文档冲突，如实并列**：
  - `闭环校准说明.md`：`CW表 = (g_calib_table+5)%6`、`CCW表 = (g_calib_table+2)%6`；与手工表 `s_hall2step_cw={-,5,3,4,1,0,2,-}`、`s_hall2step_ccw={-,2,0,1,4,3,5,-}` 及示例 `g_calib_table={-,0,4,5,2,1,3,-}` 算术自洽（(0+5)%6=5✓、(0+2)%6=2✓；若 +4 则 (0+4)%6=4≠5 不自洽）。**+5/+2 为可信版本**。
  - `CLAUDE.md` 记载 `+4/+2`——疑似笔误/旧值。面试口径：先给 +5/+2（有算术闭环验证），注明 CLAUDE.md 存在 +4 的旧记法。
- **为什么 CW/CCW 表"互换"**（表名与物理方向反直觉，`六步方波.md` + `角度30度90度分析.md`）：本电机手拧 CW 时霍尔序列 0x02→0x06→0x04→0x05→0x01→0x03 = 扇区角 240→180→120→60→0→300 **电角度递减**（标准电机 CW 递增）。要产生正扭矩，定子磁场须**超前**转子 90°（sin90°=1 最大）；沿递减方向走表，"后退 1 步"（-1 mod 6 = **+5**）才等价于物理上超前 90°；CCW 沿递增方向，前进 1 步（**+2** 亦即超前 120°，落在梯形波转矩平坦区 80°~130°）。**偏移由控制板硬件决定（6288T 真值表 + TMR4 接线 + 六步通电表），与电机无关：换电机不用重调，换板必须重测**（方法：mode5 校准 → 改 `calib_build_derived_tables()` 偏移 → mode6/7 试跑取转速最高最平稳的一组）。
- 超前角实验数据（加分项）：x=5,y=1（60°/60°超前）两向转速都低；x=4,y=2（120°/120°）两向都高；x=3,y=3（180°）也高（梯形波在 180° 附近有上升沿 + 电感电流滞后，有效超前角落回平坦区）。CW(+5=60°超前，斜坡区) 慢于 CCW(+2=120°超前，平坦区) 是实测现象。

---

## 4. Bemf.c / Bemf.h —— 反电动势观测（仅观测，无换相）

### 4.1 职责与数据流（面试画这张链路图）
```
TMR4_3 PWM中心对齐(20kHz) → EVT CH_UH(SCMP0, compare=period, match=PEAK)
  → AOS: EVT_SRC_TMR4_3_SCMP0 → AOS_ADC1_0 → ADC1 SEQ_A 硬件触发单次扫描 CH0→CH3
  → ADC1 EOCA → AOS: EVT_SRC_ADC1_EOCA → AOS_DMA1_0/1/2/3 → DMA1 CH0-3 并行REPEAT搬运
  → BTC中断(DMA1_CH0) → Bemf_DataCallback → 8点滑动平均 → J-Scope 全局
```
通道（`Bemf.h:45-59`）：**PA0/ADC1_CH0 = M（中性点），PA1/CH1=U，PA2/CH2=V，PA3/CH3=W**；`BEMF_SEQA_HARDTRIG=ADC_HARDTRIG_EVT0`（h:62-64）；DMA1 CH0-3、每通道 buffer 8、16bit、中断优先级 `DDL_IRQ_PRIO_03`（h:71-78）。

### 4.2 关键函数（`ws/Bemf.c`）
| 签名 | 锚点 |
|---|---|
| `void Bemf_DataCallback(uint8_t ch)`（DMA BTC ISR 上下文） | c:205-262 |
| `static void Bemf_DmaConfig(void)`：src=`&DR0+ch*2`、REPEAT 无限、block 8、仅 CH0 注册回调 | c:271-321（c:308） |
| `static void Bemf_Tmr4EvtConfig(void)`：TMR4_EVT CH_UH、CMP 模式、compare=period、match=PEAK | c:334-357 |
| `Bemf_TriggerConfig()`：`#if BEMF_SEQA_TRIGGER_ENABLE` → ENABLE 否则 DISABLE | c:367-378 |
| `void Bemf_Init(void)`：AOS_InitForBemf → AdcConfig → DmaConfig → Dma_Init → Tmr4Evt → Trigger → Dma_StartAll | c:398-436 |
| `uint8_t Bemf_GetFloatingChannel(uint8_t step)`：map `{3,2,1,3,2,1}` | c:530-541 |
| `Bemf_GetFloatingPhaseRaw / Bemf_GetFloatingPhaseBemf / Bemf_UpdateWaveData` | c:548-555 / c:562-574 / c:583-600 |

### 4.3 必须精确引用的事实
- **触发源 = PWM 计数器峰值**（三角波顶点，SCMP0=period）：20kHz PWM → 每 50µs 触发一次 4 通道扫描，8 点 buffer 搬满 → **BTC 中断每 400µs（2.5kHz）**；首个零点落在 fs/8，天然抑制 PWM 开关噪声（c:205-262）。
- **相电压差分测量**：`i16BemfX = 相 - M（中性点）`，`mV = diff * 3300 / 4096`（c:239-242）——第四通道 M 是点睛之笔：端电压含共模分量，相对中性点相减才是真反电动势（无感方波的经典做法是重构中性点，这里直接硬件测）。
- **浮空相自动选择**：`Bemf_GetFloatingChannel` 映射 `{3,2,1,3,2,1}`（c:530-541）：step0(UH+VL)→W、step1(UH+WL)→V、step2(VH+WL)→U、step3(VH+UL)→W、step4(WH+UL)→V、step5(WH+VL)→U；DMA BTC ISR 里按 `g_scope_step` 把 `g_bemf_wave_data` 自动切到浮空相（c:247-256）。**只有浮空相的端电压才等于反电动势**——导通相读到的是被驱动 PWM 电压（0~3300mV 方波）。
- **为何只观测不换相**：①有意分阶段——先验证信号质量（bemf.md 已知问题②"无零交叉检测=仅观测，有意为之待信号质量验证"）②全文件确无零交叉比较/换相逻辑；③后续换相会走 FOC 路线（SMO/磁链观测器已在 ws/ 落地），六步无感只是过渡方案。
- **`BEMF_SEQA_TRIGGER_ENABLE 0`（Bemf.h:66-68）当前为 DISABLE**：实验态——SEQ_A 触发关闭、仅保留电流采样（SEQ_B 共用同一 SCMP0 触发）。读代码时 BEMF 数据实际不更新，别在面试里说"BEMF 在跑"。
- **AOS 约束（CLAUDE.md 关键）**：`AOS_InitForBemf()` 覆盖 `AOS_ADC1_0` 上 legacy 的 Timer0_CMP_B→ADC1 连接——**init 顺序必须 Hardware_Init→…→CommRunner_Init→Bemf_Init→I_Init**，谁再启用 PA6 的 ADC 路由就会打断 BEMF 时序。
- **已知问题（主动交代=加分）**：①mV 换算未含分压比，`raw*3300/4096` 只是 ADC 引脚电压，10:1 分压则实际相电压约 10 倍（c:139-142 附近 `CalcVoltage`）；②PWM 峰值采样对零交叉未必最优——理想采样窗在 PWM 关断期（浮空相完全悬浮稳定后），可调 SCMP 比较值或加采样窗；③PA2 与 USART2 复用冲突风险；④50kHz 时代算过 6000rpm@4对极每电步仅 ~2.6 次 BTC，高转速零交叉点检测可能欠采样（buffer 减到 4/2 可加密；现 20kHz 已缓解为 400µs/BTC）。

---

## 5. Motor_hall.c / .h —— 旧 2 霍尔 T 法测速（对比素材；GBK 编码，App 层仍在注册）

- 仍在注册：`App_Motor_Project.c:88-103` `motor_hall_config_t{HA=PA10/EXTINT_CH10, HB=PA9/EXTINT_CH09, irq_priority=2, pole_pairs=3, hall_count=2}`，`update_interval_ms=1`。**注意与 ws/ 的 hall_3ch 是两套独立模块，当前注册的是这套旧的。**
- T 法（测周）：`rpm = 60000000 / (interval_us * pulses_per_rev)`，`pulses_per_rev = pole_pairs*2*active_halls` =3×2×2=12（c:318-339），封顶 100000；6 个间隔平均（c:290-316）+ 6 点窗口滤波（c:273-288）。
- 方向判别（c:179-205）：B 下降沿读 A 电平，A=1 正转否则反转，连续 3 次确认（`DIRECTION_CONFIRM_COUNT 3`）。
- 常量（c:15-22）：`MIN_PULSE_INTERVAL_US 50u / MAX_PULSE_INTERVAL_US 200000u / STALL_DETECTION_MS 500u / STOP_DETECTION_MS 50u / RPM_WINDOW_SIZE 6 / HALL_CHECK_INTERVAL_MS 1000 / HALL_WARNING_THRESHOLD 10`。
- 中断配置**无硬件滤波**：`EXTINT_FILTER_OFF + FCLK_DIV1`（c:242-243）——与 hall_3ch 的 `FILTER_ON+DIV64` 形成直接对比：hall_3ch 是为 PWM 噪声环境补的课。
- 健康检查：每 1000ms 对比两霍尔计数，差 >10% → `HALL_STATUS_ERROR`（c:366-406）。
- M/T 对比一句话：**T 法高速溢出、低速精准；M 法低速精度差、高速稳；工程折中是 M/T 混合**。本工程 3 霍尔用 M 法（18 脉冲/转比 2 霍尔 12 脉冲/转分辨率高 50%），就是"用 M 法但靠 3 霍尔补分辨率"的实例。

---

## 6. 面试官追问预测（10 问）+ 答案要点

**Q1 六步表为什么每行必有一相 M_OFF？悬空相占空比为什么是 D_OFF 而不是低侧常通？**
→ 悬空相是反电动势观测窗口（方波无感的物理基础）；D_OFF 双关保证悬空相端电压完全悬浮；若低侧常通会经电机绕组形成泄放路径，污染 BEMF。三相对称性：每步"一相调制（高侧 D_PWM）+ 一相低侧 D_MIN + 一相悬空"，6 步磁场角 330/30/90/150/210/270 等差 60°（`dev_commutation.c:18-40`）。

**Q2 6288T-MNS 和别的驱动芯片（SDH21263）换的时候要注意什么？**
→ 逻辑极性相反：6288T H≠L 导通、H=L 关断；换芯片只需改 `Adp/tmr4_pwm.c/h` 的模式枚举实现，上层 `dev_commutation/runner` 通过模式枚举解耦不动（CLAUDE.md 设计模式节）。另外本板死区交给驱动芯片（`dead_time_ns=0`，`dev_comm_runner.c:31-38`），换芯片要重核死区。

**Q3 CW/CCW 查表为什么看起来是"反"的？+5/+2 是什么？**
→ 本电机 CW=电角度递减（霍尔序列实测 0x02→0x06→0x04→0x05→0x01→0x03）；表名跟随物理方向，查表方向跟随电角度方向，故 CW 用"后退一步"=+5、CCW 用"前进一步"=+2；本质都是让磁场超前转子 90°~120°（梯形波平坦区 80°~130°）。偏移是控制板属性（门驱真值表+TMR4 接线），换电机不变、换板重测。注：CLAUDE.md 有 +4/+2 旧记法，闭环校准文档 +5/+2 与手工表算术自洽，以 +5/+2 为准。

**Q4 方波（六步）vs FOC 怎么选？压缩机/风机为什么行业在转 FOC？**
→ 方波：控制简单（一张表+一个占空比）、霍尔 3 根线成本、60° 换相瞬间电流跳变→转矩脉动→**噪声与振动**（压缩机对噪声极敏感）、低速转矩脉动大、无法精确控torque。FOC：Id/Iq 解耦、转矩脉动小、噪声低、效率高（可做 MTPA/弱磁）、无感化路径成熟（SMO/磁链+PLL），代价是算力（本项目 20kHz FOC ISR @Cortex-M4M的 CLA 级优化）与参数敏感（R/L/ψ：`FOC_MOTOR_RS_OHM 0.1f, LS 42.3uH, FLUX_VS 0.00084f, KV 650, MAX_CURRENT 17A, MAX_SPEED 7800`，`motor_config.h:73-79`）。本项目完整走了这条路，foc_smo/foc_obs/foc_luen/foc_pll 都在 ws/ 里。

**Q5 霍尔 ISR 噪声防御为什么删掉两层软件防御只留硬件滤波？**
→ 根因：三路 EXTI 独立，一次 PWM 开关噪声可同时打三路，"<50µs 拒绝"把合法沿也拒了；同态振铃早退则掩盖真实沿。硬件 DIV64 滤波（≈1.56MHz 滤时钟，滤 <10µs）在源头解决且零 CPU；非法码拒绝（000/111）无副作用保留；邻接检查降级为诊断计数（`g_dbg_isr_baddiff`，baddiff/valid≈13% 恰证明表在工作）。教训：**防御层次要放在与噪声同源的物理层**。

**Q6 飞行启动为什么要双采样？失败模式是什么？**
→ 双采样（中间 100 次 NOP）是廉价的"转子在动"判据：两次不同→取新值。失败模式：①两次都读不到合法码→兜底 step0，首拍最多错 1 步，60° 电角度内由霍尔 ISR 自纠；②开环拖转方向与预期相反（实测开环 FW 斜坡在本机实际驱动 CCW）——飞行启动不判方向，只跟位置，所以仍能接管；③静止误触发：两采样都同值（静止），此时查表值即真实位置，无误。

**Q7 校准为什么要求 12 个连续一致周期？**
→ 1 个周期只证明"6 态各出现过一次"；连续 12 周期与参考表一致才排除：偶发噪声沿（会制造 DUPLICATE/错位）、低速爬行抖动、机械回差。配合前置校验（双射性+与首周期一致）与 3000ms 总超时、失败原因码（`g_calib_error.hall_state` bitmask，0x7E=全未见→查接线）构成可诊断闭环。

**Q8 M 法测速 3000rpm 分辨率多少？低速怎么办？**
→ 3 极对 3 霍尔=18 脉冲/转；20ms 窗口 delta≥6 才更新：3000rpm→900p/s→窗口 18 脉冲→量化 ±167rpm（±5.6%）；2000rpm→12 脉冲→±333rpm。低速改 T 法（旧 Motor_hall 模块即 T 法：`rpm=6e7/(interval_us*12)`），工业标准是 M/T 混合（分界约几百 rpm）。

**Q9 BEMF 无感换相，采样点选 PWM 峰值还是关断期？为什么你只观测不换相？**
→ 峰值采样优点：触发链免费（SCMP0=period）、噪声大但定时确定；缺点：导通相正在开关、浮空相端电压仍含振铃。关断期采样（PWM off 区间末端）浮空相真正悬浮，BEMF 最干净，但要改 SCMP 比较值+处理低占空比时窗口不足。本工程先只观测是刻意的分阶段验证（bemf.md 问题②），且无感最终路线选了 FOC 观测器（SMO/磁链+PLL，ws/foc_45_smo.c、foc_53_flx.c 已落地），六步无感零交叉只是备选。

**Q10 你的 BEMF 第四通道 M 是干嘛的？没有中性点引出的电机怎么办？**
→ M=中性点实测，端电压减 M 得相 BEMF（消共模）。无中性点引出时软件重构：`M_recon=(VU+VV+VW)/3`（三相端电压均值≈中性点，导通相 PWM 会污染，需在采样窗内取值或滤波）——这正是 PWM 关断期采样重要的另一个原因。

**Q11（若被问项目真实性）你说六步 FSM 不在代码里？**
→ 如实答：六步 FSM 是早期版本（git 建仓前），现行树保留其全部底层（换相表/霍尔驱动/BEMF 观测）并已演进到多模式 FOC（mode20/24/25/26/27/28/29/30/31/32/40/41/45/51/52/53/60，`dev_comm_runner.h:23-42`）；md_record 目录有当时完整调试文档（换相表推导、噪声分析、校准流程），可以白板复现 s_hall2step_cw={0xFF,5,3,4,1,0,2,0xFF} 的推导过程。**诚实+能推导 > 背假代码。**

---

## 7. 附：一句话速记卡
- 换相表：`0=UH_VL…5=WH_VL`，高侧 D_PWM / 低侧 D_MIN(2%) / 悬空 D_OFF，磁场角 330 起步进 60°。
- 6288T：H≠L 通、H=L 断；占空比 2~98%。
- 懒更新：模式变才写 CMR，占空比变只写 OCCR，影子寄存器 PEAK 生效。
- 霍尔防御：DIV64 硬件滤波 / 000·111 拒绝 / 邻接仅诊断；已删同态早退与 50µs 拒绝（三路 EXTI 同爆是根因）。
- M 法：`rpm=Δp·6e7/(t·pp·6)`，20ms/6 脉冲门限，6 点滑动平均，500ms 超时归零。
- 飞行启动：双采样取新 → 查表（0xFF→step0）→ 直入 RUNNING → 立即 on_step。
- 校准：500ms 稳定 → 6 态齐双校验 → 锁参考 → **12 连续一致周期** → SUCCESS；派生表 **CW=+5、CCW=+2**（CLAUDE.md +4 系旧值）。
- BEMF：TMR4 峰值触发 SCMP0 → SEQ_A CH0-3(M,U,V,W) → DMA1 REPEAT → BTC 400µs；差分减 M；浮空相 map{3,2,1,3,2,1}；当前 `BEMF_SEQA_TRIGGER_ENABLE=0` 仅观测。
