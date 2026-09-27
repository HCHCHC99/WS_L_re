// 证明"只动了注释"：去掉所有注释与空白后，逐文件比较改动前后；同时检查注释配对
import fs from 'node:fs';
import path from 'node:path';

const BAK = 'D:/WS_L_re/_cardgen/hdrbak';
const WS = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws';
const MAIN = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/source/main.c';

function stripComments(s) {
  let out = '', i = 0, inLine = false, inBlock = false, inStr = false;
  while (i < s.length) {
    const c = s[i], n = s[i + 1];
    if (inLine) { if (c === '\n') { inLine = false; out += c; } i++; continue; }
    if (inBlock) { if (c === '*' && n === '/') { inBlock = false; i += 2; continue; } i++; continue; }
    if (inStr) { out += c; if (c === '\\') { out += s[i + 1] ?? ''; i += 2; continue; } if (c === '"') inStr = false; i++; continue; }
    if (c === '"') { inStr = true; out += c; i++; continue; }
    if (c === '/' && n === '/') { inLine = true; i += 2; continue; }
    if (c === '/' && n === '*') { inBlock = true; i += 2; continue; }
    out += c; i++;
  }
  return out;
}
const codeOf = s => stripComments(s).split('\n').map(l => l.replace(/\s+/g, ' ').trim()).filter(Boolean).join('\n');

let bad = 0, n = 0;
for (const f of fs.readdirSync(BAK)) {
  const cur = path.join(f === 'main.c' ? path.dirname(MAIN) : WS, f);
  if (!fs.existsSync(cur)) { console.log(`  [X] ${f} 找不到当前文件`); bad++; continue; }
  const a = codeOf(fs.readFileSync(path.join(BAK, f), 'utf8').replace(/\r\n/g, '\n'));
  const b = codeOf(fs.readFileSync(cur, 'utf8').replace(/\r\n/g, '\n'));
  n++;
  if (a === b) console.log(`  OK  ${f.padEnd(22)} 代码部分逐行一致（只改了注释）`);
  else {
    bad++;
    const la = a.split('\n'), lb = b.split('\n');
    const diff = [];
    for (let i = 0; i < Math.max(la.length, lb.length); i++) if (la[i] !== lb[i]) diff.push(`第${i + 1}行: 旧"${(la[i] ?? '').slice(0, 50)}" 新"${(lb[i] ?? '').slice(0, 50)}"`);
    console.log(`  [X] ${f} 代码有差异（${diff.length} 行）\n      ${diff.slice(0, 6).join('\n      ')}`);
  }
}
console.log(`\n共比对 ${n} 个文件，代码有差异的 ${bad} 个`);
