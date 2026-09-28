// 预检：某个 .c 里用到的全局符号/宏，是否在工程里有定义或声明
//（等价于"未定义标识符"那一类编译错误的静态替代）。
// 起因：foc_52_ldlq.c 引用了重写头文件时删掉的 FOC52_STEP_DONE，编译才暴露。
//
// 判定规则（严格版，避免被"只是别处用过"污染）：
//   符号必须能在**某个头文件**里找到，或者能在本 .c 里找到"定义形态"的行
//   （#define NAME / 行首类型 + NAME( 的函数定义）。
// 用法：node _cardgen/check_symbols.mjs [ws/foc_52_ldlq.c]
import fs from 'node:fs';
import path from 'node:path';

const ROOT = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2';
const DDL = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/drivers/hc32_ll_driver/inc';
const TARGET = process.argv[2] ?? 'ws/foc_52_ldlq.c';
const SELF_TEST = process.argv.includes('--selftest');

/* 头文件池：所有 .h（跨文件共享的符号按本工程惯例都声明在 .h 里） */
let pool = '';
const walkH = (dir, depth = 0) => {
  if (depth > 4) return;
  let ents; try { ents = fs.readdirSync(dir, { withFileTypes: true }); } catch { return; }
  for (const e of ents) {
    const p = path.join(dir, e.name);
    if (e.isDirectory()) { if (!/^(output|MDK|EWARM|GCC|docs|\.git)$/.test(e.name)) walkH(p, depth + 1); }
    else if (/\.h$/.test(e.name)) pool += fs.readFileSync(p, 'utf8').replace(/\r\n/g, '\n') + '\n';
  }
};
walkH(ROOT); walkH(DDL, 3);

let srcRaw = fs.readFileSync(path.join(ROOT, TARGET), 'utf8').replace(/\r\n/g, '\n');
if (SELF_TEST) srcRaw += '\nvoid probe(void) { g_ldlq52_state = FOC52_STEP_DONE; }\n';   // 故意注入
/* 注释与字符串字面量整体替换成等长空格（保留行结构）：
 * 注释里的示例名、printf 格式串（ADC1_CH%d 之类）否则会造成大量假报。 */
const src = srcRaw
  .replace(/\/\*[\s\S]*?\*\//g, m => m.replace(/[^\n]/g, ' '))
  .replace(/\/\/[^\n]*/g, m => ' '.repeat(m.length))
  .replace(/"(?:\\.|[^"\\\n])*"/g, m => ' '.repeat(m.length))
  .replace(/'(?:\\.|[^'\\\n])*'/g, m => ' '.repeat(m.length));

const PATTERNS = [
  /\bg_foc_[A-Za-z0-9_]+/g, /\bg_[a-z0-9_]+_[A-Za-z0-9_]+/g,
  /\bFOC[A-Z0-9_]+/g, /\bFoc_[A-Za-z0-9_]+/g, /\bTMR4_[A-Za-z0-9_]+/g,
  /\bTMRA_[A-Za-z0-9_]+/g, /\bENCODER_[A-Za-z0-9_]+/g, /\bUSART3_[A-Za-z0-9_]+/g,
  /\bMAIN_D\b/g, /\bCM_TMRA_[0-9]\b/g, /\bADC1_[A-Za-z0-9_]+/g, /\bAOS_[A-Za-z0-9_]+/g,
];
/* 末尾用负向先行断言而不是 \b：避免把长名字截断成前缀（如 FOC25_SUB_ms -> FOC25_SUB_） */
const names = new Set();
for (const re of PATTERNS) {
  const g = new RegExp(re.source.replace(/\/g$/, '') + '(?![A-Za-z0-9_])', 'g');
  for (const m of src.match(g) ?? []) names.add(m);
}

const esc = s => s.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
/** 本 .c 里是否存在该符号的"定义形态"（宏 / 函数定义 / 文件级变量定义） */
function definedHere(n) {
  const lines = src.split('\n');
  const isMacro = new RegExp(`^\\s*#\\s*define\\s+${esc(n)}\\b`);
  const isFuncDef = new RegExp(`^\\s*[A-Za-z_][\\w\\s\\*]*\\b${esc(n)}\\s*\\(`);
  const isVarDef = new RegExp(`^\\s*(?:static\\s+|volatile\\s+|const\\s+|extern\\s+)*`
                            + `[A-Za-z_][\\w\\s\\*,]*\\b${esc(n)}\\b\\s*(?:\\[[^\\]]*\\])?\\s*(?:=|;|,)`);
  for (const l of lines) {
    if (isMacro.test(l)) return true;
    if (isFuncDef.test(l) && !/;\s*$/.test(l)) return true;      // 函数定义（非原型）
    if (isVarDef.test(l)) return true;                           // 文件级变量定义
  }
  return false;
}

const missing = [];
for (const n of [...names].sort()) {
  if (new RegExp(`\\b${esc(n)}\\b`).test(pool)) continue;
  if (definedHere(n)) continue;
  missing.push(n);
}
/* 第二遍：文件级 static（s_ 前缀）必须先声明后使用。
 * 起因：foc_52_ldlq.c 里用了 s_tick0 但忘了写 static 声明，编译报 #20 identifier undefined ——
 * 第一遍只查 g_/FOC/Foc_/TMR4_ 这些外部符号，s_ 开头的静态量没覆盖到。 */
const staticsDeclared = new Set();
for (const l of src.split('\n')) {
  if (/^\s*static\b/.test(l)) {
    for (const m of l.match(/\bs_[A-Za-z0-9_]+/g) ?? []) staticsDeclared.add(m);
  }
  /* 结构体/数组定义的收尾行形如 "} s_step_meta[6] = {"，名字不在 static 那一行 */
  const t = l.replace(/^\s*\}\s*/, '');
  const m2 = /^(s_[A-Za-z0-9_]+)\s*(\[|=|;|,)/.exec(t);
  if (m2) staticsDeclared.add(m2[1]);
}
const sUsed = new Set();
for (const m of src.match(/\bs_[A-Za-z0-9_]+/g) ?? []) sUsed.add(m);
for (const n of [...sUsed].sort()) {
  if (staticsDeclared.has(n)) continue;
  if (new RegExp(`#\\s*define\\s+${esc(n)}\\b`).test(src)) continue;
  missing.push(`${n}（s_ 前缀静态量未声明）`);
}
console.log(`--- ${TARGET}${SELF_TEST ? '（自测：已注入一个不存在的 FOC52_STEP_DONE）' : ''}：外部符号 ${names.size} 个，s_ 静态量 ${sUsed.size} 个 ---`);
if (missing.length) {
  console.log(missing.map(m => `  [X] 头文件与本 .c 都没有定义/声明: ${m}`).join('\n'));
  console.log(`\n有 ${missing.length} 个符号可能在编译时报"未定义标识符"`);
  process.exit(1);
}
console.log('  OK  全部外部符号都能在头文件或本文件里找到定义/声明');
