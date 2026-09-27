// 从 Keil .uvoptx 里删除 foc_23_align.c 的 <File> 条目（.uvprojx 已删，这里要跟着一致）
import fs from 'node:fs';

const UV = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/MDK/template - 副本.uvoptx';
const lines = fs.readFileSync(UV, 'utf8').replace(/\r\n/g, '\n').split('\n');

const drop = new Set();
const blocks = [];
for (let i = 0; i < lines.length; i++) {
  if (!/foc_23_align\.c/.test(lines[i])) continue;
  let s = i;
  while (s >= 0 && !/<File>\s*$/.test(lines[s])) s--;
  let e = i;
  while (e < lines.length && !/<\/File>\s*$/.test(lines[e])) e++;
  if (s < 0 || e >= lines.length) throw new Error(`区间定位失败 ${s}..${e}`);
  blocks.push([s, e]);
  for (let k = s; k <= e; k++) drop.add(k);
}
console.log(`  定位到 ${blocks.length} 个条目：${blocks.map(([s, e]) => `第 ${s + 1}~${e + 1} 行`).join('，')}`);
const out = lines.filter((_, i) => !drop.has(i));
fs.writeFileSync(UV, out.join('\n'), 'utf8');

const t = fs.readFileSync(UV, 'utf8');
const c = re => (t.match(re) || []).length;
console.log(`  自检：<File> ${c(/<File>/g)}  </File> ${c(/<\/File>/g)}；foc_23_align 残留 ${c(/foc_23_align/g)}；行数 ${lines.length} -> ${out.length}`);
if (c(/<File>/g) !== c(/<\/File>/g)) { console.log('  [X] 标签不配对'); process.exit(1); }
console.log('  OK');
