// 删除 mode 23：源文件 + Keil 工程(.uvprojx)里的 2 个 <File> 块 + 卡片数据
import fs from 'node:fs';
import path from 'node:path';

const PROJ = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2';
const UV = path.join(PROJ, 'template/MDK/template - 副本.uvprojx');

/* ---- 1) 删源文件 ---- */
for (const f of ['ws/foc_23_align.c', 'ws/foc_23_align.h']) {
  const p = path.join(PROJ, f);
  if (fs.existsSync(p)) { fs.rmSync(p); console.log(`  删除 ${f}`); }
  else console.log(`  [跳过] ${f} 不存在`);
}

/* ---- 2) 删 Keil 工程里的文件条目 ---- */
{
  const lines = fs.readFileSync(UV, 'utf8').replace(/\r\n/g, '\n').split('\n');
  const out = [];
  let removed = 0;
  for (let i = 0; i < lines.length; i++) {
    if (/<FileName>foc_23_align\.c<\/FileName>/.test(lines[i])) {
      // 条目形如：<File> / <FileName> / <FileType> / <FilePath> / </File>
      let s = i;
      while (s > 0 && !/<File>/.test(lines[s])) s--;
      let e = i;
      while (e < lines.length && !/<\/File>/.test(lines[e])) e++;
      console.log(`  删除工程条目：第 ${s + 1}~${e + 1} 行（缩进 ${lines[i].match(/^\s*/)[0].length}）`);
      for (let k = s; k <= e; k++) console.log(`      - ${lines[k].trim()}`);
      removed++;
      i = e;                       // 跳过整块
      continue;
    }
    out.push(lines[i]);
  }
  fs.writeFileSync(UV, out.join('\n'), 'utf8');
  console.log(`  工程文件共删除 ${removed} 个条目（应为 2：两个 target 各一份）`);
  // 结构自检
  const t = fs.readFileSync(UV, 'utf8');
  console.log(`  自检：<File> ${(t.match(/<File>/g) || []).length} 个，</File> ${(t.match(/<\/File>/g) || []).length} 个；`
    + `foc_23_align 残留 ${(t.match(/foc_23_align/g) || []).length} 处`);
}

/* ---- 3) 删卡片数据与渲染产物 ---- */
for (const f of ['_cardgen/cards/mode23.json', '_cardgen/out/mode23.txt', '_cardgen/hdrbak/foc_23_align.h']) {
  const p = `D:/WS_L_re/${f}`;
  if (fs.existsSync(p)) { fs.rmSync(p); console.log(`  删除 ${f}`); }
}
