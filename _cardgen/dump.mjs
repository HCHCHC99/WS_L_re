// 机械清点：每个 mode 的 VOFA 填充函数体、extern 符号、关键宏、main.c 派发行
// 输出 D:/WS_L_re/_cardgen/inventory.txt（分段，便于按需阅读）
import fs from 'node:fs';
import path from 'node:path';

const WS = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws';
const MAIN = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/source/main.c';
const OUT = 'D:/WS_L_re/_cardgen/inventory.txt';

const files = fs.readdirSync(WS).filter(f => /\.(c|h)$/.test(f)).sort();
const out = [];
const w = s => out.push(s);

/* ---- 1) VOFA 填充函数 ---- */
w('################ 1. VOFA 填充函数（每个 mode 自持布局） ################');
for (const f of files.filter(f => f.endsWith('.c'))) {
  const txt = fs.readFileSync(path.join(WS, f), 'utf8').replace(/\r\n/g, '\n');
  const lines = txt.split('\n');
  lines.forEach((l, i) => {
    const m = l.match(/^\s*int\s+(Foc_\w*VofaFill)\s*\(\s*int32_t\s*\*\s*cur\s*\)/);
    if (!m) return;
    w(`\n=== ${f} :: ${m[1]}()  (第 ${i + 1} 行) ===`);
    for (let j = i; j < lines.length; j++) {
      w(lines[j]);
      if (/^\}/.test(lines[j])) break;
    }
  });
}

/* ---- 2) 每个头文件的 extern 符号 ---- */
w('\n\n################ 2. 头文件 extern 符号 ################');
for (const f of files.filter(f => f.endsWith('.h'))) {
  const lines = fs.readFileSync(path.join(WS, f), 'utf8').replace(/\r\n/g, '\n').split('\n');
  const hits = [];
  lines.forEach((l, i) => { if (/^extern\s/.test(l)) hits.push(`${i + 1}: ${l.trim()}`); });
  if (!hits.length) continue;
  w(`\n=== ${f} (${hits.length}) ===`);
  hits.forEach(h => w(h));
}

/* ---- 3) 每个 mode 头文件里的关键宏 ---- */
w('\n\n################ 3. 模式头文件宏定义 ################');
for (const f of files.filter(f => f.endsWith('.h'))) {
  const lines = fs.readFileSync(path.join(WS, f), 'utf8').replace(/\r\n/g, '\n').split('\n');
  const hits = [];
  lines.forEach((l, i) => { if (/^#define\s+FOC\d+_/.test(l)) hits.push(`${i + 1}: ${l.trim()}`); });
  if (!hits.length) continue;
  w(`\n=== ${f} (${hits.length}) ===`);
  hits.forEach(h => w(h));
}

/* ---- 4) main.c 的 VOFA 派发 ---- */
w('\n\n################ 4. main.c VOFA 派发 ################');
{
  const lines = fs.readFileSync(MAIN, 'utf8').replace(/\r\n/g, '\n').split('\n');
  lines.forEach((l, i) => { if (/VofaFill|Vofa_SendScaled|cur\[32\]|COMM_MODE|n\s*=/.test(l)) w(`${i + 1}: ${l.trim()}`); });
}

/* ---- 5) 文件清单 ---- */
w('\n\n################ 5. ws 目录清单 ################');
for (const f of files) {
  const n = fs.readFileSync(path.join(WS, f), 'utf8').split('\n').length;
  w(`${f.padEnd(28)} ${n} 行`);
}

fs.writeFileSync(OUT, out.join('\n'), 'utf8');
console.log(`已写出 ${OUT}，共 ${out.length} 行`);
