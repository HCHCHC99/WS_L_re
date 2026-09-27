// 清理"mode 23 已删除"之后仍然描述它为现存模式的注释/日志文案
// 每条替换都要求：全文命中次数 == 期望值，否则报错不写（防误伤）
import fs from 'node:fs';

const WS = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/';
const SRC = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/source/';

const EDITS = [
  [WS + 'foc_core.h', 1,
    ' * @brief 电机控制参数主配置',  // 占位，实际改下面这条
    ' * @brief 电机控制参数主配置'],
  [WS + 'foc_core.h', 1,
    '#define FOC_MODE_ALIGN     3u   /* comm_mode 23: standstill electrical alignment */',
    '#define FOC_MODE_ALIGN     3u   /* comm_mode 20/24/25/26/27/28/29/40/41/45 共用的对齐分发路径 */'],
  [WS + 'foc_core.h', 1,
    '/* 对齐电零点：模式23 记录，模式22 I-F 交接沿用（编码器 electrical-zero count） */',
    '/* 对齐电零点：模式20/24 锁定时写入（编码器 electrical-zero count） */'],
  [WS + 'foc_core.c', 1,
    '/* 对齐零点（模式23 记录 / 模式22 I-F 交接沿用） */',
    '/* 对齐零点（模式20/24 锁定时写入） */'],
  [SRC + 'main.c', 1,
    'MAIN_DBG("System started (MINIMAL: mode23/30 only)");',
    'MAIN_DBG("System started");'],
  [SRC + 'main.c', 1,
    '    /* ---- 电流采样（模式23和30需要电流监视） ---- */',
    '    /* ---- 电流采样（FOC 各模式与 mode 30 都要电流监视） ---- */'],
  [WS + 'motor_config.h', 1,
    ' * @brief 电机控制参数主配置（精简版 — 仅保留模式23和30）',
    ' * @brief 电机控制参数主配置（FOC 实验模式 + mode 30 共用）'],
  [WS + 'motor_config.h', 1,
    ' * 环拓扑配置（精简版 — 模式23和30不需要速度环/电流环）',
    ' * 环拓扑配置（历史遗留：现行各 FOC 模式自持实现环路，这些宏已无引用）'],
  [WS + 'motor_config.h', 1,
    ' * FOC 参数（模式23 对齐校准 / 模式30 自增拖动）',
    ' * FOC 参数（对齐/吸附电压 + 电机规格 + 保护阈值）'],
  [WS + 'motor_config.h', 1,
    '/* 以下宏仅用于编译 FOC 代码（模式23和30不需要 I-F/电流环） */',
    '/* 以下宏仅用于编译 FOC 代码（历史遗留，见上方环拓扑说明） */'],
  [WS + 'foc_20_cal.h', 1,
    ' * 主循环完成（事件快照模式，与 mode 23/31/32 一致）。',
    ' * 主循环完成（事件快照模式，与 mode 31/32 一致）。'],
  [WS + 'foc_32_lockiq.c', 1,
    ' *        相位时序（沿用 mode 23 的两段式单侧逼近，破坏摩擦迟滞）：',
    ' *        相位时序（沿用老版对齐的两段式单侧逼近，破坏摩擦迟滞）：'],
  [WS + 'foc_32_lockiq.c', 1,
    ' * LockIqPi_OutputVolt - 对齐加压输出（磁场定 theta，模式23同款路径）',
    ' * LockIqPi_OutputVolt - 对齐加压输出（磁场定 theta，老版对齐同款路径）'],
  [WS + 'cur_loop.c', 1,
    ' *        模式23（对齐校准）和模式30（自增拖动）不需要电流环 PI 控制，',
    ' *        模式30（自增拖动）与各 FOC 模式不需要本模块的电流环 PI 控制，'],
  [WS + 'cur_loop.c', 1,
    'CURLOOP_DBG("Init done (current-loop PI DISABLED - only mode23/30)");',
    'CURLOOP_DBG("Init done (current-loop PI DISABLED)");'],
  [WS + 'dev_comm_runner.c', 1,
    ' * CommRunner_Update — 几乎空操作（FOC ISR 驱动模式23和30）',
    ' * CommRunner_Update — 几乎空操作（FOC 模式与 mode 30 都由 Foc_Isr 驱动）'],
  [WS + 'dev_comm_runner.c', 1,
    '    /* 模式23和30完全由 Foc_Isr 驱动，主循环无事可做 */',
    '    /* FOC 模式与 mode 30 完全由 Foc_Isr 驱动，主循环无事可做 */'],
  [WS + 'I.c', 1,
    ' * differs from the code labels. Watch tunable; run mode 23 and find the order',
    ' * differs from the code labels. Watch tunable; run mode 25 and find the order'],
];

let fail = 0, ok = 0;
const cache = new Map();
for (const [file, expect, from, to] of EDITS) {
  if (from === to) continue;                       // 占位条目
  if (!cache.has(file)) cache.set(file, fs.readFileSync(file, 'utf8'));
  const s = cache.get(file);
  const n = s.split(from).length - 1;
  if (n !== expect) { console.log(`  [X] ${file.split('/').pop()}: 命中 ${n} 次（期望 ${expect}）\n      "${from.slice(0, 60)}..."`); fail++; continue; }
  cache.set(file, s.split(from).join(to));
  ok++;
}
if (!fail) {
  for (const [file, s] of cache) fs.writeFileSync(file, s, 'utf8');
  console.log(`  已写入 ${cache.size} 个文件，共 ${ok} 处替换`);
} else {
  console.log(`  有 ${fail} 处不匹配，未写入任何文件`);
}
