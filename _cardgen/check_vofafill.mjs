// 静态校验所有 Foc_*_VofaFill：
//  1) cur[k] 下标必须是 0..n-1 且每个恰好出现一次；
//  2) return 值 == 下标个数，且 <= USART3_VOFA_MAX_CHANNELS；
//  3) 函数体里引用的每个 g_* / Foc_* 符号在工程里真实存在（防拼写错误）。
import fs from 'node:fs';
import path from 'node:path';
import { findSymbol } from './cardlib.mjs';

const WS = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws';
const MAXCH = 24;   // USART3_VOFA_MAX_CHANNELS
let fail = 0;

for (const f of fs.readdirSync(WS).filter(f => f.endsWith('.c')).sort()) {
  const src = fs.readFileSync(path.join(WS, f), 'utf8').replace(/\r\n/g, '\n');
  const lines = src.split('\n');
  lines.forEach((l, i) => {
    const m = l.match(/^\s*int\s+(Foc_\w*VofaFill)\s*\(\s*int32_t\s*\*\s*cur\s*\)/);
    if (!m) return;
    const name = m[1];
    // 取函数体
    let body = [];
    for (let j = i + 1; j < lines.length; j++) { if (/^\}/.test(lines[j])) break; body.push(lines[j]); }
    const bodyTxt = body.join('\n');
    const problems = [];
    // 变长布局：按 return 分段，每段各校验一次（mode 45 有三个分支）
    const segs = [];
    {
      let acc = [];
      for (const ln of body) {
        acc.push(ln);
        const r = ln.match(/return\s+(\d+)\s*;/);
        if (r) { segs.push({ n: Number(r[1]), txt: acc.join('\n') }); acc = []; }
      }
    }
    if (!segs.length) problems.push('没有 return 语句');
    for (const seg of segs) {
      const idx = [...seg.txt.matchAll(/\bcur\[(\d+)\]/g)].map(x => Number(x[1]));
      const n = seg.n;
      if (n > MAXCH) { problems.push(`通道数 ${n} 超过上限 ${MAXCH}`); continue; }
      const uniq = [...new Set(idx)].sort((a, b) => a - b);
      const expect = Array.from({ length: n }, (_, k) => k);
      if (uniq.length !== n || uniq.some((v, k) => v !== expect[k])) {
        problems.push(`分支 return ${n}: 下标集合 {${uniq.join(',')}} 与 0..${n - 1} 不一致`);
      }
      if (idx.some(v => v >= n)) problems.push(`分支 return ${n}: 存在越界下标`);
    }
    const idxAll = [...bodyTxt.matchAll(/\bcur\[(\d+)\]/g)].map(x => Number(x[1]));
    const n = segs[0]?.n;
    if (segs.length === 1) {
      const dup = idxAll.filter((v, k) => idxAll.indexOf(v) !== k);
      if (dup.length) problems.push(`重复下标 ${[...new Set(dup)].join(',')}`);
    }
    // 符号存在性（排除 cur / 局部量）
    const syms = [...new Set([...bodyTxt.matchAll(/\b(g_[A-Za-z0-9_]+|FOC[A-Z0-9_]+)\b/g)].map(x => x[1]))];
    const missing = syms.filter(s => findSymbol(s).length === 0);
    if (missing.length) problems.push(`符号不存在: ${missing.join(' ')}`);
    const tag = `${f} :: ${name}() -> ${segs.map(s => s.n).join('/')}ch`;
    if (problems.length) { fail++; console.log(`  [X] ${tag}\n      ${problems.join('\n      ')}`); }
    else console.log(`  OK  ${tag.padEnd(52)} ${segs.length > 1 ? `${segs.length} 个分支各自齐全` : `ch0..ch${n - 1} 齐全`}，${syms.length} 个符号全部存在`);
  });
}
console.log(fail ? `\n有 ${fail} 个填充函数有问题` : '\n全部填充函数静态校验通过');
