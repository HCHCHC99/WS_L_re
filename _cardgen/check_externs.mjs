// 预检：某个 .c 的同名 .h 里声明的每个 extern 变量，是否在 .c 里有定义。
// 起因：mode 52 新增 5 个观测量，只在 .h 里 extern 声明 + 在 .c/foc_obs.c 里使用，
// 忘了在 .c 里定义 —— 编译全过，链接才报 L6218E "Undefined symbol ... referred from foc_obs.o"。
// 声明齐全时编译器不会报错（定义可以在别的编译单元里），所以必须静态查一遍。
// 用法：node _cardgen/check_externs.mjs ws/foc_52_ldlq.c
import fs from 'node:fs';
import path from 'node:path';

const ROOT = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2';
const TARGET = process.argv[2];
if (!TARGET) { console.error('用法: node check_externs.mjs <ws/xxx.c>'); process.exit(2); }
const HDR = TARGET.replace(/\.c$/, '.h');
const ABS_C = path.join(ROOT, TARGET);
const ABS_H = path.join(ROOT, HDR);
if (!fs.existsSync(ABS_H)) { console.log(`--- ${TARGET}：没有同名 .h，跳过 ---`); process.exit(0); }

const strip = s => s
  .replace(/\/\*[\s\S]*?\*\//g, m => m.replace(/[^\n]/g, ' '))
  .replace(/\/\/[^\n]*/g, m => ' '.repeat(m.length))
  .replace(/"(?:\\.|[^"\\\n])*"/g, m => ' '.repeat(m.length));

const hsrc = strip(fs.readFileSync(ABS_H, 'utf8').replace(/\r\n/g, '\n'));
let csrc = strip(fs.readFileSync(ABS_C, 'utf8').replace(/\r\n/g, '\n'));
const SELF_TEST = process.argv.includes('--selftest');

/* .h 里的 extern 变量声明（含 volatile / 多类型限定词） */
const declared = [];
for (const line of hsrc.split('\n')) {
  const m = /^\s*extern\s+([\w\s\*]+?)\s+([A-Za-z_]\w*)\s*(\[[^\]]*\])?\s*;/.exec(line);
  if (m) declared.push({ name: m[2], type: m[1].trim() });
}

if (SELF_TEST && declared.length) {          // 自测：把第一个定义从 .c 里抹掉
  const n = declared[0].name;
  csrc = csrc.replace(new RegExp(`^\\s*[\\w\\s\\*]*\\b${n}\\b\\s*=.*$`, 'm'),
                      `/* removed for selftest */`);
}

const missing = [];
for (const d of declared) {
  /* 定义形态：行首（未被 extern 修饰）出现 "类型 名字 =|;|["，且该行不是 extern 声明 */
  const re = new RegExp(`^\\s*(?!extern\\b)[\\w\\s\\*]*\\b${d.name}\\b\\s*(=|;|\\[)`, 'm');
  const re2 = new RegExp(`^\\s*(?!extern\\b)[\\w\\s\\*]*\\b${d.name}\\b\\s*(=[^=]|;)`, 'm');
  if (!re.test(csrc) && !re2.test(csrc)) missing.push(d.name);
}

console.log(`--- ${TARGET}${SELF_TEST ? '（自测：已抹掉第一个定义）' : ''}：.h 里 extern ${declared.length} 个 ---`);
if (missing.length) {
  console.log(missing.map(m => `  [X] .h 声明了 extern 但 .c 里没有定义: ${m}（会报 L6218E）`).join('\n'));
  process.exit(1);
}
console.log('  OK  每个 extern 都有对应定义');
