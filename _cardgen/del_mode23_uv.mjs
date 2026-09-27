// 从 Keil .uvprojx 里删除 foc_23_align.c 的两个 <File> 条目
// 正确做法：先定位每个条目的完整行区间 [<File> .. </File>]，收集所有待删行号，最后一次性过滤。
// （上一版逐行边扫边输出，导致 <File> 开标签已被输出、闭标签却被跳过 —— 结果 XML 不闭合。）
import fs from 'node:fs';

const UV = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/MDK/template - 副本.uvprojx';
const lines = fs.readFileSync(UV, 'utf8').replace(/\r\n/g, '\n').split('\n');

const drop = new Set();
const blocks = [];
for (let i = 0; i < lines.length; i++) {
  if (!/<FileName>\s*foc_23_align\.c\s*<\/FileName>/.test(lines[i])) continue;
  let s = i;
  while (s >= 0 && !/<File>\s*$/.test(lines[s])) s--;
  let e = i;
  while (e < lines.length && !/<\/File>\s*$/.test(lines[e])) e++;
  if (s < 0 || e >= lines.length) throw new Error(`条目区间定位失败：${s}..${e}`);
  blocks.push([s, e]);
  for (let k = s; k <= e; k++) drop.add(k);
}
console.log(`  定位到 ${blocks.length} 个条目：`);
for (const [s, e] of blocks) {
  console.log(`    第 ${s + 1}~${e + 1} 行`);
  for (let k = s; k <= e; k++) console.log(`      - ${lines[k].trim()}`);
}
const out = lines.filter((_, i) => !drop.has(i));
fs.writeFileSync(UV, out.join('\n'), 'utf8');

const t = fs.readFileSync(UV, 'utf8');
const c = re => (t.match(re) || []).length;
console.log(`\n  自检：<File> ${c(/<File>/g)}  </File> ${c(/<\/File>/g)}  <Files> ${c(/<Files>/g)}  </Files> ${c(/<\/Files>/g)}`);
console.log(`  自检：foc_23_align 残留 ${c(/foc_23_align/g)} 处；总行数 ${lines.length} -> ${out.length}`);
if (c(/<File>/g) !== c(/<\/File>/g)) { console.log('  [X] File 标签仍不配对！'); process.exit(1); }
console.log('  OK  标签配对正常');
