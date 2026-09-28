// 往 Keil .uvprojx 的两个 target 里各插入一个 foc_51_rs.c 的 <File> 条目
// 做法：先找到每个 target 里 foc_20_cal.c 的条目区间，在它后面插入（保持组内顺序与缩进）
import fs from 'node:fs';

const UV = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/MDK/template - 副本.uvprojx';
const lines = fs.readFileSync(UV, 'utf8').replace(/\r\n/g, '\n').split('\n');

if (lines.some(l => /foc_51_rs/.test(l))) {
  console.log('  已存在 foc_51_rs 条目，跳过');
  process.exit(0);
}

// 找每个目标里 foc_20_cal.c 条目的结束行（</File>），在其后插入新条目
const inserts = [];   // {after: index, lines: [...]}
for (let i = 0; i < lines.length; i++) {
  if (!/<FileName>foc_20_cal\.c<\/FileName>/.test(lines[i])) continue;
  let e = i;
  while (e < lines.length && !/<\/File>\s*$/.test(lines[e])) e++;
  const indent = lines[i].match(/^\s*/)[0];
  const inner = indent + '  ';
  inserts.push({
    after: e,
    lines: [
      `${indent}<File>`,
      `${inner}<FileName>foc_51_rs.c</FileName>`,
      `${inner}<FileType>1</FileType>`,
      `${inner}<FilePath>..\\..\\ws\\foc_51_rs.c</FilePath>`,
      `${indent}</File>`,
    ],
  });
}
console.log(`  找到 ${inserts.length} 个插入点（应为 2：两个 target 各一份）`);
if (inserts.length !== 2) { console.log('  [X] 插入点数量异常，未写入'); process.exit(1); }

// 从后往前插，避免行号漂移
const out = lines.slice();
for (const ins of inserts.sort((a, b) => b.after - a.after)) {
  out.splice(ins.after + 1, 0, ...ins.lines);
}
fs.writeFileSync(UV, out.join('\n'), 'utf8');

const t = fs.readFileSync(UV, 'utf8');
const c = re => (t.match(re) || []).length;
console.log(`  自检：<File> ${c(/<File>/g)} / </File> ${c(/<\/File>/g)}；foc_51_rs 出现 ${c(/foc_51_rs/g)} 处；行数 ${lines.length} -> ${out.length}`);
if (c(/<File>/g) !== c(/<\/File>/g)) { console.log('  [X] File 标签不配对！'); process.exit(1); }
console.log('  OK');
