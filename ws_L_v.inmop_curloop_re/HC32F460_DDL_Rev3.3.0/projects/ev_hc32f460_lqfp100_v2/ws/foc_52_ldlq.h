/**
 *******************************************************************************
 * @file  foc_52_ldlq.h
 * @brief FOC mode 52 - d/q 轴电感辨识（直流偏置方波注入 + 单极性电流纹波）。
 *
 * 方法（细节与推导见下方速览卡，唯一事实源）：
 *   把电压矢量锁在 d 轴（theta = 转子电角度）或 q 轴（+90 度），命令
 *       v(t) = V_bias + sgn(t)·Vsq          V_bias = R·I_bias + V_dead先验
 *   即"直流偏置 + 方波"叠加，使被测轴电流**始终同号**（|I_bias| > 纹波半幅）。
 *   每 2 个 ISR 周期翻转一次 sgn（电压平台 100 µs = 5 kHz 方波），
 *   每个平台内采到 2 点（平台内 25 µs / 75 µs），两点的差即
 *       |di| = Vsq·(1/fs)/L        ->        L = Vsq·(1/fs)/|di|
 *
 * 为什么必须单极性（不能用 ±Vsq 的无偏置方波）：
 *   无偏置时电流每周期两次过零，死区等效电压在采样窗内**变号**，采样差里
 *   混进与死区有关的几何误差（仿真：R=0.098/Vdead=0.16 时 L 系统性偏小
 *   8%~13%，且随幅值非单调）。加直流偏置后电流不过零 => 死区在两极性平台上
 *   同号，V_bias 里的 (R·I_bias + V_dead) 与平台内 R·i 项一起精确抵消，
 *   仿真残差 < 1%（L = 20~260 µH 全范围）。
 *
 * 为什么半周期必须取 **2 个采样周期**（不是 1 个）：
 *   ADC 在三角波谷点采样、占空比影子寄存器在峰点装载（TMR4_OC_BUF_COND_PEAK），
 *   相差 25 µs。半周期取 1 拍时采样点落在平台正中 = 电流三角波中点，采样差退化
 *   成约 0，会误报"没有纹波"。取 2 拍后每平台内采到 2 点，差恰为半纹波。
 *
 * 机械要求：d 轴测量不产生力矩（id 不产生转矩）；q 轴测量期间有一个与 I_bias
 *   成正比的恒定力矩（约 kt·I_bias ≈ 0.009 N·m @0.7 A），**建议夹紧转子**。
 *   偏置每 FOC52_BIAS_CYCLES 个周期翻一次号，使净冲量抵消，未夹紧时也不至于
 *   被推着转起来；用 g_ldlq52_moved_cnts 判断数据是否可信。
 *
 * 前置：必须先跑 mode 24（要编码器电气零点定位 d/q 轴）。
 * 结束：自动（d 轴 -> 间隔 -> q 轴 -> 关 PWM）。
 *
 * 用途定位：只输出观测量（Watch / VOFA），零写入；同时给出 L/手册 比值与
 *   由偏置电流反推的 V_dead（可与 mode 51 的 160 mV 互证）。
 *
 * ==============================================================================
 *   模式速览卡   MODE 52   d/q 轴电感辨识（直流偏置方波 + 单极性电流纹波）
 *   本卡是本模式唯一事实源；代码内注释与本卡冲突时，以本卡为准
 * ==============================================================================
 *   入口   comm_mode = 52（Watch 直接写）
 *   前置   必须先跑 mode 24（要编码器电气零点才能把矢量锁在 d/q 轴上）；d
 *          轴测量不产生力矩，q 轴测量期间有一个与偏置电流成正比的恒定力矩（约
 *          0.009 N.m @0.7A）
 *   结束   自动：d 轴 -> 200ms 零矢量 -> q 轴 -> 关 PWM、state 回 IDLE；不写
 *          Flash、不改任何配置宏
 *   标记   [!] 易错点   (*) 主判据   [可调] Watch 可写   [只读] 仅观察
 * ------------------------------------------------------------------------------
 *   [可调] Watch 变量 -- 改值即时生效，复位后回宏默认
 * ------------------------------------------------------------------------------
 *       变量                        当前值  单位     含义
 *       --------------------------- ------- -------- ---------------------------
 *       g_ldlq52_di_target_a        1.0     A        目标纹波峰峰值（硬上限
 *                                                    2.0；自适应方波幅值按它定）
 *   (*) g_ldlq52_bias_a             0.7     A        直流偏置电流，**必须大于纹
 *                                                    波半幅**，否则电流过零、方
 *                                                    法失效
 *       g_ldlq52_v_min_v            0.3     V        自适应方波幅值下限
 *       g_ldlq52_v_max_v            3.0     V        自适应方波幅值上限（探测一
 *                                                    次不够就升到它再探）
 *       g_ldlq52_cycles             32      -        正式测量周期数（1 周期 =
 *                                                    200us；会取整到偏置周期的整
 *                                                    数倍）
 *       g_ldlq52_do_q               1       -        1 = 测 d 再测 q；0 = 只测
 *                                                    d（省 200ms 间隔）
 *       g_ldlq52_r_used_ohm         0.098   ohm      反推 V_dead 用的 R，填 mode
 *                                                    51 实测值
 *       g_ldlq52_wave_en            0       -        1 = 抓取并打印原始波形（9
 *                                                    行，排查用）；默认
 *                                                    0，日志保持精简
 *   (*) 上七行为本轮可调项；全程约 0.5s（零偏窗 200ms + 每轴约 20ms + 间隔
 *       200ms）。
 *   [!] bias
 *       是这套方法成立的前提：电流必须**始终同号**，才让死区等效电压在两极性平台
 *       上同号、从而在平台内采样差里精确抵消。启动护栏会自动把 bias 抬到 1.3
 *       倍纹波半幅 + 0.05A，所以调小 di_target 时 bias 也会跟着变小。
 *   [!] 注入幅值不要随手加大：3V 加在 42uH 上、100us 平台内会摆 7A。默认 0.5V
 *       探测后自适应到约 0.42V（对应纹波峰峰值 1A、峰值电流 0.5A）；若实测 L 是
 *       260uH，自适应会升到约 2.6V。
 *   [!] 必须先跑 mode 24。offset 不对 => 锁的不是 d/q
 *       轴，两个数都是混轴结果（表现：Ld/Lq 差别很大且每次不一样）。
 * ------------------------------------------------------------------------------
 *   [只读] 关键观察变量
 * ------------------------------------------------------------------------------
 *       变量                           含义
 *       ------------------------------ -----------------------------------------
 *   (*) g_ldlq52_ld_uh                 Ld ---- 主结果 (uH)
 *   (*) g_ldlq52_lq_uh                 Lq ---- 主结果 (uH)
 *   (*) g_ldlq52_ratio                 Lq/Ld ---- 约 1 = 表贴(SPM)，> 1.15 =
 *                                      凸极(IPM)，MTPA 才有意义
 *   (*) g_ldlq52_ratio_vs_vendor       Ld / FOC_MOTOR_LS_UH ---- 直接判定手册
 *                                      42.3uH 还是此前推的 260uH
 *   (*) g_ldlq52_i_bias_ma             实测偏置电流 (mA) ----
 *                                      必须大于纹波半幅，否则数据无效
 *   (*) g_ldlq52_vdead_v               由 V_bias - R x |I_bias| 反推的死区电压
 *                                      (V) ---- 与 mode 51 的 160mV 互证
 *       g_ldlq52_vbias_v               当前指令偏置电压 (V) ----
 *                                      偏置不够时模式自己每次抬 0.1V 再探
 *       g_ldlq52_probe_seq             探测完成次数（每次变化 foc_obs 打一行
 *                                      probe，含两套一致性）
 *       g_ldlq52_l_probe_uh            探测档纹波反推的电感 (uH) ---- 落在 [3,
 *                                      3000] 之外判 evt=7 并停止抬偏置
 *   (*) g_ldlq52_l_env_uh              平台均值法反推的电感 (uH) ----
 *                                      最终结果用这个（每平台取两点平均，相邻平
 *                                      台均值求差）
 *       g_ldlq52_l_pair_uh             单点差分法反推的电感 (uH) ----
 *                                      对照用；易被快速交替干扰污染（实测会偏小
 *                                      约一半）
 *   (*) g_ldlq52_de_ma                 相邻平台均值之差 (mA) ----
 *                                      结果的原始量；L = Vsq x 平台时长 / 它
 *       g_ldlq52_dp_ma                 平台内两点之差 (mA) ----
 *                                      单点差分法的原始量（对照）
 *   (*) g_ldlq52_de_cons_pct           平台均值差的符号一致性 (%) ----
 *                                      平台均值法的有效性判据
 *       g_ldlq52_cons_pct              单点差分法的一致性 (%) ---- 仅供对照
 *       g_ldlq52_dcnt                  参与统计的平台数
 *       g_ldlq52_wave                  波形抓取缓冲 (mA, 64
 *                                      点)：正式测量窗开头自动抓一窗，RTT 打印 +
 *                                      Keil Watch 可看
 *       g_ldlq52_wave_sgn              抓取窗内每拍的平台极性
 *                                      (+1/-1)，用于对齐电压与电流
 *       g_ldlq52_wave_arm              写 1 请求抓取（记满自动清
 *                                      0）；一般不用手写，正式测量会自动抓
 *   (*) g_ldlq52_path_cnts             测量窗内编码器路径累计 (counts) ----
 *                                      转子是否在动的真正判据
 *       g_ldlq52_v_inj_v               自适应后的方波幅值 Vsq (V)
 *       g_ldlq52_state                 0 IDLE / 1 零偏窗 / 2 探测 / 3 正式测量 /
 *                                      4 间隔 / 5 OC
 *   [!] g_ldlq52_evt                   1 完成 / 2 过流 / 3 纹波太小 / 4
 *                                      偏置没建立 / 5 转子在动 / 6
 *                                      采样配对异常（4/5/6 都不更新结果）
 *       g_ldlq52_axis                  0 = d 轴，1 = q 轴（当前/最后测的轴）
 *       g_ldlq52_running               1 = 本轮有效（DONE 后仍为
 *                                      1，便于回看；切模式时 Stop）
 *   [!] g_ldlq52_moved_d_cnts          d 轴测量期间的净位移 (counts，已解回绕)
 *   [!] g_ldlq52_moved_q_cnts          q 轴测量期间的净位移 (counts，已解回绕)
 *                                      ---- d 轴就不该动
 *       g_ldlq52_moved_cnts            全程净位移 (counts，已解回绕)
 *       g_ldlq52_elapsed_ms            累计耗时 (ms)
 * ------------------------------------------------------------------------------
 *   VOFA 通道 -- 12ch，填充见 Foc_LdLqId_VofaFill
 * ------------------------------------------------------------------------------
 *       通道  含义                   单位     备注
 *       ----- ---------------------- -------- ----------------------------------
 *       ch0   状态码                 -        见状态机
 *       ch1   当前轴                 -        0=d 1=q
 *   (*) ch2   方波幅值               V        自适应结果
 *   (*) ch3   纹波峰峰值             A        应贴住 di_target
 *   (*) ch4   Ld                     uH       主结果
 *   (*) ch5   Lq                     uH       主结果
 *   (*) ch6   Lq/Ld                  x1000    约 1000 = 表贴
 *   (*) ch7   偏置电流               A        须大于 ch3 的一半
 *   (*) ch8   离散度                 %        < 5 合格
 *   (*) ch9   配对一致性             %        必须约 100
 *   [!] ch10  编码器位移             counts   > 8 则数据可疑
 *       ch11  耗时                   ms       全程约 500
 * ------------------------------------------------------------------------------
 *   判据与坑
 * ------------------------------------------------------------------------------
 *   (*) 原理：把电压矢量锁在 d 轴（theta = 转子电角度）或 q 轴（+90 度）上，命令
 *       v = V_bias + sgn x Vsq，其中 V_bias = R x I_bias + V_dead先验
 *       让被测轴有一个直流偏置电流 I_bias，方波幅值只要小于 I_bias x R
 *       就不会把电流推过零。每 2 个采样周期翻转一次 sgn（电压平台 100us = 5kHz
 *       方波），每个平台内采 2 点（平台内 25us / 75us），两点之差就是 |di| = Vsq
 *       x (1/fs) / L。
 *   (*) 为什么必须做成单极性（本模式的核心）：若用无偏置的
 *       +/-Vsq，电流每周期两次过零，死区等效电压在采样窗内**变号**，采样差里就混
 *       进与死区有关的几何误差。用 10ns
 *       步长的波形仿真（_cardgen/sim_mode52.mjs、sim_mode52_est.mjs）量化：
 *       R=0.098、Vdead=0.16、L=42.3uH 时无偏置方案系统性偏小
 *       8~13%，且随幅值非单调、无法用简单修正项补掉。改成单极性后，V_bias 里的
 *       (R x I_bias + V_dead) 与平台内的 R x i 项一起精确抵消，仿真残差 < 1%（L
 *       = 20~260uH 全范围、Vsq = 0.3~2.6V）。
 *   [!] 半周期必须取 2 个采样周期，不能取 1：本板 ADC
 *       在三角波谷点触发采样，而占空比影子寄存器在峰点装载，两者相差
 *       25us。若半周期取 1 拍（平台 50us），采样点正好落在每个平台正中间 =
 *       电流三角波的中点，采样差退化成约 0，会误报『没有纹波』。取 2
 *       拍后每平台内采到 2 点（25us / 75us 处），差才是有效信号。
 *   [!] 偏置电流必须大于纹波半幅（|I_bias| >
 *       di_target/2）：这就是单极性的定义。启动时护栏会把 g_ldlq52_bias_a 抬到
 *       1.3 x 半幅 + 0.05A；运行中若实测 |I_bias| 仍不大于半幅，会置 evt = 4
 *       (BIAS_FAIL) 并且**不更新结果**，RTT 打印 bias not
 *       established。看到它就小幅抬高 g_ldlq52_bias_a 重跑。
 *   [!] 机械：d 轴注入不产生转矩（id 不产生转矩、且 iq = 0）；q
 *       轴注入期间有一个与偏置电流成正比的恒定力矩（约 kt x 0.7A =
 *       0.009N.m），**建议夹紧转子**。偏置每 16
 *       个周期翻一次号，使净冲量抵消、未夹紧时也不至于被推着连续转；用
 *       moved_cnts 判断（> 8 counts 判可疑）。
 *   (*) 判据：ratio_vs_vendor 约 1 => 手册 42.3uH 可信，且与 mode 40
 *       电流环整定自洽（kp=0.1 / ki=300 => ki/kp = 3000 => L = R x kp/ki =
 *       0.098/3000 = 33uH）；若约 6 => 此前由 R=0.15ohm 反推的 260uH
 *       成立，电流环需要重新整定。d_scatter_pct < 5% 且 moved_cnts 在 +/-8
 *       以内才认这两个数。
 *   (*) 副产物 V_dead：稳态下 v 的平均值 = V_bias - V_dead，且等于 R x
 *       I_bias，故 V_dead = V_bias - R x |I_bias|。这条与 mode 51 拟合出的 160mV
 *       互相印证：两者接近说明 R 与死区模型都对上了（本式误差主要取决于 R
 *       的准确度）。
 *   [!] 本模式只输出观测量：不写 Flash、不改
 *       motor_config.h。用途是与商家参数对比、验证辨识思路本身。测到 260uH
 *       也不会自动改环路，需要人工决定。
 *   (*) RTT 输出（开关 FOC_LDLQ52_DBG）：每次成功跑完共 6~9 行 -- start（含
 *       do_q，确认本轮会不会测 q）；每个轴的 probe 两行（配置/偏置 +
 *       cons/path/env/pair）；d done 一行（do_q=0/1 都会打，d
 *       轴结果位置固定）；done 一行（Ld/Lq/Lq/Ld/vs_vendor/vdead）；last axis
 *       一行；moved 一行。失败时只多打一条原因行（OC / bias not established /
 *       rotor moving / pair mismatch / implausible / no
 *       ripple）。原始波形默认不打印，要看就把 g_ldlq52_wave_en 写 1。
 *   [!] 相位免疫的关键规则：平台长度正好是 2 个采样周期 =>
 *       每个平台内有两对相邻采样；翻转是按 ISR
 *       次数计的，一旦某次写入错过峰点影子装载，采样相位就永久错开一拍，此时两对
 *       里必有一对跨过三角波顶点、其 |di| 被抵消一部分（实测均值只剩 0.72
 *       倍、离散度 52%）。干净的那一对必然是两者中较大的那个，所以每个平台取
 *       max(|di|) 即可，与相位无关（10ns 步长仿真：任意相位下误差 0.00%、离散度
 *       0.00%；对照方案"相邻两对斜率差做闸门"会把顶点两侧的对全部误杀、有效样本
 *       归零，已弃用）。phase_ratio 顺带给出相位诊断。
 *   [!] 三条有效性判据（任一不过就不更新结果，只留过程量供排查）：1) cons_pct >=
 *       85（采样配对正确）；2) |I_bias| > 纹波半幅（单极性成立）；3)
 *       测量窗内编码器路径 path_cnts <= 64（转子基本不动）。另 path_cnts > 8
 *       时追一行 warn。注意净位移 moved_cnts 会随 16 位计数器回绕（曾把 -206
 *       打成 65330），所以判"转子是否在动"要看路径累计而不是净位移。
 *   [!] 偏置电流是自适应建立的，不依赖 R/V_dead 先验的准确性：每次探测后检查实测
 *       |I_bias| 是否 >= 1.5 x 纹波半幅，不够就把偏置电压抬
 *       FOC52_VBIAS_STEP_V(0.1V) 重探，直到够用或到 1.5V 上限。2026-09-27
 *       两次上机就是因为实测偏置小于指令值（先验偏差）而导致电流过零、cons 只有
 *       67.8% -- 这条环路就是为它加的。日志里每次探测都会打一行 probe#，直接看
 *       bias / ripple/2 / cons / other 四个数。
 *   [!] evt=7 (IMPLAUSIBLE)：探测档纹波反推的电感不在 [3, 3000] uH
 *       就立刻停手、不再抬偏置电压。2026-09-27
 *       实测教训：波形异常时"纹波半幅"被高估到 1.4A（应为
 *       0.59A），偏置自适应就一路把偏置电压抬到上限、d 轴电流冲到 3A。看到 evt=7
 *       请先看打印出来的波形串（下面那条），确认采样结构再谈标定。
 *   (*) 波形抓取（排查采样结构用）：正式测量窗开头自动抓 64
 *       拍被测轴电流与平台极性，RTT 打出 8 个数一行 + 一行 +/-
 *       极性串。重复采样（相邻数相同）、相位错拍（极性串与电流拐点错位）、开关纹
 *       波、直流瞬态在这两行上一眼可见 -- 比反复推断"波形应该是什么样"可靠得多。
 *   (*) 结果算法 = 平台均值法：平台长度正好是 2
 *       个采样周期，把平台内两点取平均（这一步把采样序列里约 +/-0.5A
 *       的快速交替干扰抵消掉 -- 相邻两点符号相反），再取相邻平台均值之差
 *       |dE|，则 L = Vsq x 平台时长 / |dE|。2026-09-27
 *       的抓取波形显示：单点差分法里"跨平台边界那一跳"比平台内斜坡大 ~2
 *       倍（边界瞬态），取 max 的规则专门挑到它，导致 L 被系统性压掉一半（报
 *       20uH，而平台均值法给出 34~37uH、与手册 42.3uH
 *       同量级）。两种算法的结果都在 RTT 里直接打印（env / pair），方便对照。
 *   [!] RTT 打印的 probe 行含三行：第一行是偏置/纹波/实测 ISR
 *       频率，第二行是一致性与偏置需求，第三行直接给出两种算法各自的电感：probe
 *       L: env=..uH (dE=..mA) pair=..uH (dp=..mA) fs=..Hz。结束时再打
 *       done（Ld/Lq/比值/手册倍数）与过程量行。无需手工推算。
 *   [!] 重测：跑完后固件调用 CommRunner_ReleaseMode() 把模式号放回 STOP（Keil 里
 *       comm_mode 会自动回 0），因此**直接再写一次 52 就能重测**，不必手动先写
 *       0。原因：CommRunner_SetMode() 开头有"同模式早退"，若模式号停在 52，再写
 *       52 会被吃掉 -- 这就是以前"有时能重测、有时不能"的根因。
 * ==============================================================================
 *******************************************************************************
 */

#ifndef __FOC_52_LDLQ_H__
#define __FOC_52_LDLQ_H__

#include <stdint.h>
#include "foc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * Debug macros（RTT打印规范：0/1 赋值式开关，定义在 .h，.c 只调封装宏；
 * 打印点落在 foc_obs 事件段，ISR 内不打印；禁止 %f 与中文）
 *=============================================================================*/
/* Debug switch: 1 = RTT prints on, 0 = off */
#define FOC_LDLQ52_DBG   1
#if FOC_LDLQ52_DBG
    #define LDLQ52_DBG(fmt, ...)   MAIN_D("[LDLQ52] " fmt, ##__VA_ARGS__)
#else
    #define LDLQ52_DBG(fmt, ...)   ((void)0)
#endif

/*=============================================================================
 * 编译期常量
 *=============================================================================*/
#define FOC52_ZERO_SAMPLES     4000u   /* 零偏窗采样数（4000 @20kHz = 200ms） */
#define FOC52_SETTLE_CYCLES    45u     /* 每个相位开始前跳过的周期数（9ms，等直流分量稳；260uH 时 3.4 tau） */
#define FOC52_HALF_ISRS        2u      /* 半周期 = 几个采样周期（**必须 2，见文件头说明**） */
#define FOC52_PROBE_V          0.5f    /* 探测方波幅值 (V) */
#define FOC52_PROBE_CYCLES     10u     /* 探测周期数 */
#define FOC52_GAP_MS           200u    /* d/q 之间的零矢量间隔 */
#define FOC52_BIAS_CYCLES      16u     /* 直流偏置每多少个周期翻一次号（防转子被推走） */
#define FOC52_BIAS_SKIP        8u      /* 偏置翻号后跳过的平台数（等电流建立） */
#define FOC52_BIAS_MARGIN      1.5f    /* 指令偏置下限 = 纹波半幅 x 它 + 0.05A */
#define FOC52_BIAS_HARD_MAX_A  2.0f    /* 偏置电流硬上限：超过就不再抬电压（防把 d 轴电流推大） */
#define FOC52_BIAS_NEED_RATIO  1.5f    /* 实测偏置必须 >= 纹波半幅 x 它，否则抬偏置电压再探 */
#define FOC52_VBIAS_STEP_V     0.10f   /* 每次抬偏置电压的步长 (V) */
#define FOC52_VDEAD_PRIOR_V    0.16f   /* V_dead 先验（mode 51 实测 160mV），只影响偏置落点 */
#define FOC52_DI_HARD_MAX_A    2.0f    /* 纹波峰峰值硬上限 (A) */
#define FOC52_V_HARD_MAX_V     3.0f    /* 注入方波幅值硬上限 (V) */
#define FOC52_VBIAS_HARD_MAX_V 1.5f    /* 直流偏置电压硬上限 (V) */
#define FOC52_CONS_MIN_PCT     85.0f   /* 采样对符号一致性合格线 (%)，低于它说明那对没落在驱动方向上 */
#define FOC52_PATH_WARN_CNTS   8u      /* 测量窗内编码器路径超过它 -> warn */
#define FOC52_PATH_BAD_CNTS    64u     /* 超过它 -> 判转子在动，结果作废 (evt=MOTION) */
#define FOC52_WAVE_N           64u     /* 波形抓取长度（ISR 逐拍记录被测轴电流与平台极性） */
#define FOC52_L_MIN_UH         3.0f    /* 纹波反推的电感合理下限 (uH)：低于它判数据异常、停止抬偏置 */
#define FOC52_L_MAX_UH         3000.0f /* 合理上限 (uH)，高于它同样判异常 */

/*=============================================================================
 * 状态机（g_ldlq52_state）
 *=============================================================================*/
#define FOC52_STEP_IDLE       0u  /* 未运行 / 已完成 */
#define FOC52_STEP_ZERO       1u  /* 零偏窗 */
#define FOC52_STEP_PROBE      2u  /* 探测：估 L 量级以定正式幅值 */
#define FOC52_STEP_MEAS       3u  /* 正式测量 */
#define FOC52_STEP_GAP        4u  /* d/q 之间的零矢量间隔 */
#define FOC52_STEP_FAULT_OC   5u  /* 过流停机 */

/*=============================================================================
 * 事件码（g_ldlq52_evt）
 *=============================================================================*/
#define FOC52_EVT_DONE        1u  /* 测量完成 */
#define FOC52_EVT_OC          2u  /* 过流停机 */
#define FOC52_EVT_NO_RIPPLE   3u  /* 纹波太小（电流几乎不动）→ 查接线/幅值 */
#define FOC52_EVT_BIAS_FAIL   4u  /* 偏置电流没建立（电流过零）→ 数据不可信 */
#define FOC52_EVT_MOTION      5u  /* 测量窗内转子在动 → 数据不可信，夹紧后重测 */
#define FOC52_EVT_PAIR_FAIL   6u  /* 两种采样配对都没有符号一致性 → 采样相位异常 */
#define FOC52_EVT_IMPLAUSIBLE 7u  /* 纹波反推的电感不在合理范围 → 采样/波形异常，停止抬偏置 */

/*=============================================================================
 * Keil Watch 可调参数（定义见 foc_52_ldlq.c）
 *=============================================================================*/
extern volatile float    g_ldlq52_di_target_a;   /* 目标纹波峰峰值 (A, 默认 1.0，硬上限 2.0) */
extern volatile float    g_ldlq52_bias_a;        /* 直流偏置电流 (A, 默认 0.7，须大于纹波半幅) */
extern volatile float    g_ldlq52_v_min_v;       /* 自适应方波幅值下限 (V, 默认 0.3) */
extern volatile float    g_ldlq52_v_max_v;       /* 自适应方波幅值上限 (V, 默认 3.0) */
extern volatile uint32_t g_ldlq52_cycles;        /* 正式测量周期数 (默认 32，会取整到偏置周期的整数倍) */
extern volatile uint32_t g_ldlq52_do_q;          /* 是否测 q 轴 (1=测, 0=只测 d) */
extern volatile uint32_t g_ldlq52_wave_en;       /* 1 = 抓取并打印原始波形（排查用，默认 0 关） */
extern volatile float    g_ldlq52_r_used_ohm;    /* 反推 V_dead 用的 R (默认 0.098, 填 mode 51 实测值) */

/*=============================================================================
 * 观测量（Watch / VOFA）
 *=============================================================================*/
extern volatile uint8_t  g_ldlq52_running;
extern volatile uint8_t  g_ldlq52_state;
extern volatile uint8_t  g_ldlq52_evt;
extern volatile uint8_t  g_ldlq52_axis;          /* 0 = d 轴, 1 = q 轴 */
extern volatile float    g_ldlq52_ld_uh;         /* 主结果：Ld (µH) */
extern volatile float    g_ldlq52_lq_uh;         /* 主结果：Lq (µH) */
extern volatile float    g_ldlq52_ratio;         /* Lq/Ld（约 1 => SPM；>1.15 => IPM） */
extern volatile float    g_ldlq52_ratio_vs_vendor; /* Ld / FOC_MOTOR_LS_UH */
extern volatile float    g_ldlq52_di_pp_a;       /* 实测纹波峰峰值 (A) = 2 x 采样差均值 */
extern volatile float    g_ldlq52_v_inj_v;       /* 自适应后的方波幅值 Vsq (V) */
extern volatile float    g_ldlq52_i_bias_ma;     /* 实测偏置电流 (mA，取 |值|） */
extern volatile float    g_ldlq52_vdead_v;       /* 由偏置电流反推的 V_dead (V) */
extern volatile float    g_ldlq52_d_scatter_pct; /* 各平台 |di| 的离散度 (%，越小越好) */
extern volatile uint32_t g_ldlq52_dcnt;          /* 参与统计的平台数 */
extern volatile float    g_ldlq52_cons_pct;      /* 采用样本的符号一致性 (%，应 ~100) */
extern volatile float    g_ldlq52_phase_ratio;   /* 平台内 max/min 之比 (x100)：100 = 相位锁定，>200 = 错开 */
extern volatile uint8_t  g_ldlq52_probe_seq;     /* 探测完成次数（变化时 foc_obs 打一行 probe） */
extern volatile uint8_t  g_ldlq52_ld_done_seq;   /* d 轴测量完成次数（do_q=0/1 都会打 d done 行） */
extern volatile float    g_ldlq52_vbias_v;       /* 当前指令偏置电压 (V) */
extern volatile uint32_t g_ldlq52_fs_meas_hz;
extern volatile float    g_ldlq52_l_env_uh;      /* 平台均值法反推电感 (uH)：最终结果用这个 */
extern volatile float    g_ldlq52_l_pair_uh;     /* 单点差分法反推电感 (uH)：诊断对照用 */
extern volatile float    g_ldlq52_de_ma;         /* 相邻平台均值之差 (mA) */
extern volatile float    g_ldlq52_dp_ma;         /* 平台内两点之差 (mA) */
extern volatile float    g_ldlq52_de_cons_pct;   /* 平台均值差的符号一致性 (%) */
extern volatile uint16_t g_ldlq52_v_probe_mv;    /* 本次探测真正用过的方波幅值 (mV) */
extern volatile float    g_ldlq52_l_probe_uh;    /* 探测档纹波反推的电感 (uH)：超出合理区间判 evt=7 */
extern volatile uint32_t g_ldlq52_path_cnts;     /* 测量窗内编码器路径累计（判转子在动） */
/* 波形抓取：排查"到底采到什么样的波形"用（重复采样/相位/开关纹波一眼可见）。
 * wave[i] = 该拍被测轴电流 (mA)，wave_sgn[i] = 该拍平台极性 (+1/-1)，wave_n = 有效点数。
 * wave_arm 写 1 请求抓取：下一拍起记录一窗 FOC52_WAVE_N 点，记满自动清 0。 */
extern volatile int16_t  g_ldlq52_wave[FOC52_WAVE_N];
extern volatile int8_t   g_ldlq52_wave_sgn[FOC52_WAVE_N];
extern volatile uint8_t  g_ldlq52_wave_n;
extern volatile uint8_t  g_ldlq52_wave_arm;
extern volatile uint8_t  g_ldlq52_q_skipped;    /* 1 = 本轮因 do_q=0 只测了 d 轴 */
extern volatile uint8_t  g_ldlq52_wave_seq;   /* 每抓满一窗 +1（foc_obs 据此打印一次） */
extern volatile int32_t  g_ldlq52_moved_d_cnts;  /* d 轴测量期间的净位移 (counts, 已解回绕) */
extern volatile int32_t  g_ldlq52_moved_q_cnts;  /* q 轴测量期间的净位移 (counts, 已解回绕) */
extern volatile int32_t  g_ldlq52_moved_cnts;    /* 全程编码器净位移 (counts, 已解回绕) */
extern volatile uint32_t g_ldlq52_elapsed_ms;

/*=============================================================================
 * API
 *=============================================================================*/

/* mode 52 入口：校验 mode 24 结果 -> 零偏窗 -> d 轴探测/测量 -> 间隔 -> q 轴 -> 收尾。
 * 主循环上下文调用（dev_comm_runner）。 */
void Foc_LdLqId_Start(void);

/* 20 kHz ISR 步进：OC 保护 -> 输出 (偏置 + 方波) 并采集平台内电流差。 */
void Foc_LdLqId_Step(const stc_i_data_t *pData);

/* 停止（用户中途切模式时调用）：清运行标志 + 关 PWM。 */
void Foc_LdLqId_Stop(void);

/* 模式自持 VOFA：固定 12ch，返回通道数；通道含义见本文件顶部速览卡。 */
int  Foc_LdLqId_VofaFill(int32_t *cur);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_52_LDLQ_H__ */
