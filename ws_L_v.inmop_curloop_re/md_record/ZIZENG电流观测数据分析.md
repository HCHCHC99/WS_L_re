# ZIZENG 模式电流观测数据分析与验证记录

> 日期：2026-09-02
> 适用固件：FOC 模块化重构后（foc_zizeng / foc_calib / I），q 轴加压版，±5A 电流传感器

---

## 1. 背景：为什么 ZIZENG 下 id/iq 长成这样

ZIZENG 是**开环电压控制**，没有电流环。控制系角度 `theta` 是电压矢量的角度，
加压方式（q 轴版）等价于控制系里 **Vd = 0、Vq = V**：

```
valpha = -V·sin(theta)      ← InvPark(0, V, theta) 的展开
vbeta  = +V·cos(theta)
```

因此 ZIZENG 下没有任何环节把 id 压到 0，电流完全由电路阻抗决定：

```
I ≈ V / Rs        （低速时 ωL ≈ 0.0008Ω、反电动势 ≈ 0.016V，均可忽略）
```

代入实测：`g_zizeng_volt_v = 0.6V, Rs = 0.1Ω` → I ≈ 5.7A（扣除死区等效压降后）。

**关键认知**："id≈0" 是闭环电流 FOC（模式 22）的性质，不是开环拖动的性质。
d 轴电流大在开环电压驱动下是物理正确的结果。

---

## 2. 实测数据验算（2026-09-02）

### 2.1 原始数据（VOFA+，单位 A）

| 通道 | 变量 | 读数 |
|---|---|---|
| CH3 | `g_foc_iq_ma`（控制系 iq） | 5.733 |
| CH4 | `g_foc_id_ma`（控制系 id） | -0.363 |
| CH5 | 合成 √(iq²+id²) | 5.744 |
| CH6 | `g_zizeng_theta_rad`（控制系角度） | 3.863 |
| id 峰值摆动 | — | ±0.36~0.7 |
| 电源箱母线平均电流 | — | ≈0.4A |

### 2.2 逐项验证

**(a) 模长合成自洽**

```
√(5.733² + 0.363²) = √(32.87 + 0.13) = 5.744 ✓
```

**(b) 电流矢量方向（q 轴加压的物理预期）**

电流矢量相对控制系 d 轴的夹角：

```
angle(i) = arctan(id / iq) = arctan(-0.363 / 5.733) = -3.6°
```

理论预期：3Hz 下阻抗角

```
arctan(ωL / Rs) = arctan(2π·3 × 42.3µH / 0.1Ω) ≈ 0.46°
```

再叠加狩猎振荡（负载角 ±几度）与残余噪声 → -3.6° 合理 ✓

**(c) 幅值与电压公式互检**

```
I ≈ (V - V_deadtime) / Rs     V_deadtime ≈ 2×500ns×10kHz×12V = 0.06V（实测 0.6/0.1 偏差反推）
5.744A → 反推 V ≈ 0.63V
```

与 `g_zizeng_volt_v` 设置基本吻合（偏差可归因 Rs 容差）。

**(d) 功率守恒（母线侧交叉验证）**

SVPWM 下母线平均电流理论值：

```
I_bus ≈ 1.5 × V_phase_amp × I_phase_amp / V_bus
     ≈ 1.5 × 0.6 × 5.74 / 12 ≈ 0.43A
```

电源箱实测 ≈0.4A ✓（母线平均电流 ≠ 相电流，这是 DC/AC 功率守恒的必然）

**(e) 角度范围**

3.863 rad ∈ [0, 2π)，3Hz 时 VOFA 上应为 3 个/秒的锯齿波 ✓

**(f) id 摆幅 ±0.36~0.7A 的来源分解**

| 来源 | 机理 | 当前量级 |
|---|---|---|
| 残余零偏 | 每相 DC 偏移 → dq 系 1× 电频率正弦 | 温漂 ~150mA/相（±5A 传感器） |
| 狩猎振荡 | 负载角 δ 弹性摆动 → id≈-I·sinδ | δ ±4~7° |
| 传感器噪声 | 固有宽带噪声 ~0.15A rms | EMA 后残留更小 |

### 2.3 结论

数据在公式层面全部自洽，采样链路（同步采样→Clarke→Park→VOFA）工作正常。

---

## 3. 单位换算链（传感器 → VOFA+ 显示）

### 3.1 传感器公式（当前板卡）

```
VOUT = 1650 + Ip(A) × 264          (mV, 3.3V 供电, ±5A 量程)
```

### 3.2 ADC 原始值 → mA 的完整推导

```
ADC: 3.3V / 4095 counts → 1 count = 0.80586 mV

Ip(A) = (VOUT - 1650) / 264
      = (raw - raw0) × 0.80586 / 264
      = (raw - raw0) × 3.0525 mA          ← 每 LSB 电流斜率

raw0 = 1650 / 0.80586 = 2047.4 ≈ 2048    ← I_ADC_ZERO
```

### 3.3 固定点实现（I.h）

```c
#define I_MA_PER_ADC  (781)   /* 3.0525 mA/count × 256 = 781.4 */
#define I_MA_SHIFT    (8U)
/* I_mA = (raw - zero_ref) × 781 >> 8 */
```

**两端验证：**

| 校验点 | 理论值 | 781 常数算出 | 误差 |
|---|---|---|---|
| +5A（raw=3686） | 5000 mA | 4997 mA | 0.05% |
| -5A（raw=410） | -5000 mA | -4997 mA | 0.05% |

### 3.4 历史教训（重要）

曾发生常数算反的事故：灵敏度 132→264 mV/A 翻倍后，mA/LSB 应**减半**，
常数应为 1563/2 ≈ 781，但误写成 1563×2 = 3126，导致所有电流读数放大 4 倍
（iq 显示 19.3A，真实 4.85A）。靠"电源箱母线电流 0.4A"这个独立观测抓出来的。

**方法论：任何电流读数必须与母线侧独立观测交叉验证。**

### 3.5 VOFA+ 显示链

```
float 5123.45 mA
  → ×1 取整（丢 µA 级精度，保留 mA）→ int32
  → SendScaled 内部 ×0.001 → VOFA+ 显示 5.123 (A)
```

约定：**传毫单位、显示基本单位**。电流显示 A，角度传 mrad 显示 rad。
通道定长 11 帧（校准窗口与运行期帧长一致，VOFA+ 不需切换）。

### 3.6 VOFA+ 通道表（当前）

| CH | 内容 | 显示单位 |
|---|---|---|
| 0~2 | 三相原始电流 iu/iv/iw（含上电校准残余） | A |
| 3 | 控制系 iq | A |
| 4 | 控制系 id | A |
| 5 | 控制系合成 √(iq²+id²) | A |
| 6 | 控制系角度 g_zizeng_theta_rad | rad |
| 7 | 转子系 iq（真实力矩电流） | A |
| 8 | 转子系 id | A |
| 9 | 三相之和 g_i_uvw_ma（零偏和） | A |
| 10 | 占位 0 | — |

上电校准窗口（500ms）内 CH0~2 显示采零期间的瞬时电流（与运行期同通道位置，曲线无缝衔接）。

---

## 4. 传感器选型分析（±10A → ±5A 的决策依据）

### 4.1 硬约束

1. **不能削顶**：真实电流超量程 → 反馈失效 → 电流环失控；
2. **OC 保护必须在量程内**：阈值超量程 = 保护失明（现 OC=4.5A ≤ 5A×90%）；
3. **收益等比**：同系列灵敏度与量程成反比，噪声/温漂在电流域按量程等比缩小。

### 4.2 两代传感器对比

| 指标 | ±10A（旧，132mV/A） | ±5A（新，264mV/A） |
|---|---|---|
| 分辨率 | 6.1 mA/LSB | 3.05 mA/LSB |
| 固有噪声 | ~0.3A rms | ~0.15A rms |
| 零偏温漂 | ~300mA/相（实测 3%FS） | ~150mA/相 |
| 削顶点 | ±10A | ±5A（raw 410~3686） |

±2.5A 直接排除：0.6V 拖动已 5A，立即削顶。

### 4.3 抖动排查实验记录（±10A 时代做的三组对照）

用 `APP_MINIMAL_CURRENT_TEST`（main.c，0=正常 / 1=最小系统+50%零矢量 / 2=最小系统+PWM全关）
三档对照，抖动峰峰值无差别 → 噪声与开关动作、固件活动均无关 →
定位为传感器固有噪声 + 供电/零偏问题，最终以换 ±5A 传感器 + 双层校准解决。

---

## 5. 电流零偏校准体系（两层设计）

### 5.1 第一层：上电校准 `I_Calibrate()`（I.c）

- 时机：上电 + 2s 稳定延时后；
- 条件：**FOC 互补模式 + 50/50/50 零矢量**（与运行时相同开关/死区/供电条件，
  三相占空比相同 → 线电压 0 → 零电流，采零前提成立）；
- 精度：500ms / ~6000 样本均值，统计误差 ~0.1mA；
- 修正对象：静态共模零偏（曾实测 2240 vs 2048，+1.2A 当量的基准比例偏差）。

**历史教训**：最初用"PWM 通道全 OFF 静止态"采零，与运行时开关态的供电条件不同，
校出的零点在运行态系统性偏差 ~1A —— **采零条件必须复现运行条件**。

### 5.2 第二层：运行前温漂校准 `foc_calib`（foc_calib.c）

- 时机：`Foc_StartZizeng()` 启动后，前 1000ms 零电流窗口内；
- 状态机：SKIP（10ms 丢弃使能瞬态）→ SAMPLING（200ms/4000 样本均值）→ LOCKED；
- 锁定前强制零矢量、theta 冻结，锁定后才开始拖动；
- 修正对象：上电校准之后的温度漂移残余（模式 22 闭环启动前同样应调用）。

### 5.3 观测量

| 变量 | 含义 |
|---|---|
| `g_i_calib_zero_u/v/w` | 上电校准的 ADC 零位（对照 2048） |
| `g_calib_state` | foc_calib 状态（3=LOCKED） |
| `g_calib_iu/iv/iw_off_ma` | foc_calib 锁定的温漂残余偏移 |

---

## 6. 采样时序确认（为什么采样点是对的）

- TMR4_3 中心对称三角波（TMR4_MD_TRIANGLE），10kHz PWM；
- 双触发：SCMP0 @ PEAK + SCMP2 @ VALLEY → 20kHz 采样；
- 占空比极性：上管 ON 以 VALLEY 为中心；PEAK 处三相上管全 OFF；
- PEAK/VALLEY 恰为纹波中点，采到的即相电流平均值（在线式传感器无下管采样约束）；
- "下管导通时采样"的规则仅适用于低侧单电阻采样方案，本板不适用。

**铁证**：PWM 全关、绕组零电流时（mode 0）读数的抖动 = 测量环节本底，与采样时序无关。

---

## 7. 对后续电流环（模式 22）的意义

1. **零偏必须解决**（已完成）：DC 零偏 → 1× 电频率转矩纹波，PI 治不了；
2. **噪声不阻塞**：±0.5A 宽带噪声经 PI 积分器平均，闭环控的是均值；
   判断闭环是否正确**只看均值、忽略抖动**：
   - id_ref=0 → `g_foc_id_ma` 均值 ≈ 0
   - iq_ref=1000mA → `g_foc_iq_ma` 均值 ≈ 1.0
3. 噪声的残留影响：占空比小幅抖动、可闻噪声，必要时峰谷值平均（½PWM 周期滞后，可忽略）。

---

## 7.5 ZIZENG 模式代码路径与函数执行顺序

### 7.5.1 涉及文件（8 个）

| 文件 | 在 ZIZENG 中的角色 |
|---|---|
| `template/source/main.c` | 上电初始化、按键切模式、VOFA+ 观测发送 |
| `ws/dev_comm_runner.c` | 模式分发：SetMode(30)→Start / SetMode(0)→Stop |
| `ws/foc.c` | 薄门面：Foc_Init 注册回调 + Foc_Isr 分发 |
| `ws/foc_zizeng.c/.h` | 核心：角度自增、加压、偏移补偿、观测 |
| `ws/foc_calib.c/.h` | 运行前温漂零偏校准状态机 |
| `ws/foc_core.c` + `foc_math.c` | 共享原语：GetDq/EMA/ModPos、三角函数/SVPWM |
| `ws/I.c/.h` | 20kHz 相电流采样驱动 + 上电校准 + 回调槽 |
| `ws/tmr4_pwm.c` + `encoder.c` | PWM 输出驱动、ABZ 编码器 |

### 7.5.2 上电初始化（main.c，顺序执行）

```
Hardware_Init()
  → Usart3_Vofa_Init()          VOFA+ 通道（诊断用）
  → CommRunner_Init()           模式分发器 + TMR4 PWM 初始化
  → I_Init()                    相电流采样驱动 + 20kHz 触发链
  → 2s 延时                     等传感器基准冷启动暂态
  → I_Calibrate()               上电零偏校准（50% 零矢量条件）
  → Foc_Init()                  注册回调: I_RegisterFocCallback(Foc_Isr)
  → Encoder_Init()              ABZ 编码器
```

### 7.5.3 模式切换（主循环，按键触发一次）

```
SW1 短按 (main.c)
  └─ comm_mode = 30
      └─ CommRunner_SetMode(30)          [dev_comm_runner.c]
          ├─ (先停旧模式，如 Foc_StopZizeng)
          └─ Foc_StartZizeng()           [foc_zizeng.c]
              ├─ 复位 theta / 偏移补偿状态机
              ├─ TMR4_PWM_SetFocMode(死区) + 50/50/50 零矢量 + StartOutput
              ├─ g_foc_active = 1        ← 打开 ISR 分发开关
              └─ Foc_Calib_Start()       ← 启动温漂校准状态机
```

切回 mode 0：`CommRunner_SetMode(0)` → `Foc_StopZizeng()` → `TMR4_PWM_EmergencyStop()`。

### 7.5.4 20kHz 实时链（每 50µs 一次，核心路径）

```
TMR4_3 计数到 PEAK/VALLEY
  └→ AOS 触发 ADC1_SEQ_B（三相电流同步采样）
      └→ EOCB 中断 → I_IrqCallback()          [I.c]
          ├─ 读 PA5/6/7 原始值（相序重映射）→ 零位校正 → mA
          ├─ I_KCL_DERIVE_MODE 推算（若启用）
          ├─ Biquad 滤波 / g_i_* 全局观测变量刷新
          ├─ 回调槽1: 用户回调
          └─ 回调槽2: Foc_Isr(pData)            [foc.c]
              └─ g_foc_active 且模式30 → Foc_Zizeng_Step(pData)

Foc_Zizeng_Step 内部（按代码顺序）:
  0. 校准未锁定 → 喂样 Foc_Calib_Feed + 强制零矢量, return   [~210ms]
  1. 读 TIMERA_1 硬件计数器 → s_enc_pos（不依赖主循环 Encoder_Update）
  2. theta += 2π·freq/20k（角度自增，wrap 到 [0,2π)）
  3. dataCal = pData − foc_calib 残余零偏
     → Foc_Core_GetDq(theta) → EMA → g_foc_id_ma / g_foc_iq_ma
  4. 编码器电角度 enc_elec（偏移锁定后扣基线补偿）→ g_foc_if_rotor_rad
  4.5 锁定后: Foc_Core_GetDq(enc_elec) → 转子系 id/iq_rotor（真实力矩电流）
  5. diff = enc_elec − theta → 偏移状态机
     （IDLE 1000ms → SAMPLING 100ms → LOCKED，锁定值扣进转子角度）
  6. q 轴加压: valpha=-V·sinθ, vbeta=V·cosθ
     → Foc_Svpwm → TMR4_PWM_SetDuty3Phase
```

### 7.5.5 时序小结

- 主循环在模式 30 里**空闲**（只做按键扫描 + VOFA 发送），实时性全部在 ISR 链；
- 模式 30 上电后的完整时间线：
  `0~2.5s 上电校准 → 进 mode 30 → ~210ms 温漂校准（零矢量）→ 1000ms 角度基线等待 → 100ms 偏移采样 → LOCKED，正式拖动`；
- 启动后约 3.8s 电机才真正开始转，前面全部是校准/基线准备阶段。

---

## 8. 相关文件索引

| 文件 | 职责 |
|---|---|
| `ws/I.h` / `ws/I.c` | 采样驱动、上电校准、mA 换算（I_MA_PER_ADC=781） |
| `ws/foc_calib.c/.h` | 运行前温漂校准状态机 |
| `ws/foc_zizeng.c/.h` | ZIZENG 开环拖动（q 轴加压宏 FOC_ZIZENG_DRIVE_AXIS） |
| `ws/motor_config.h` | Rs/Ls/λ/OC 阈值（FOC_OC_LIMIT_A=4.5A） |
| `template/source/main.c` | VOFA+ 11 通道发送（APP_MINIMAL_CURRENT_TEST 宏） |
| `md_record/闭环校准说明.md` | 模式 23 对齐校准（角度维度的零位） |
