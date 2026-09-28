// 把误插到中段的 mode51 卡片搬到头文件顶部（）的文档块内），并恢复被覆盖的 API 注释行
import fs from 'node:fs';

const H = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_51_rs.h';
const CARD = 'D:/WS_L_re/_cardgen/out/mode51.txt';

const lines = fs.readFileSync(H, 'utf8').replace(/\r\n/g, '\n').split('\n');
const card = fs.readFileSync(CARD, 'utf8').replace(/\r\n/g, '\n').split('\n');

// 1) 定位误插的卡片区间：从 "/*" 行（紧跟在 Foc_RsId_Stop 声明之后）到卡片末尾的 " */"
const stopIdx = lines.findIndex(l => /^void Foc_RsId_Stop\(void\);/.test(l));
if (stopIdx < 0) throw new Error('找不到 Foc_RsId_Stop 声明');
const start = stopIdx + 2;                       // 跳过一个空行
if (!/^\/\*$/.test(lines[start])) throw new Error(`预期 /* 开头，实际: ${lines[start]}`);
let end = start;
while (end < lines.length && !/^ \*\/$/.test(lines[end])) end++;
if (end >= lines.length) throw new Error('找不到卡片结束 */');
console.log(`  误插区间：第 ${start + 1} ~ ${end + 1} 行（共 ${end - start + 1} 行）`);

// 2) 删掉误插块，恢复 API 里那行说明注释
const rest = [...lines.slice(0, start), ...lines.slice(end + 1)];
const vofaIdx = rest.findIndex(l => /^int\s+Foc_RsId_VofaFill/.test(l));
if (vofaIdx < 0) throw new Error('找不到 VofaFill 声明');
rest.splice(vofaIdx, 0, '/* 模式自持 VOFA：固定 12ch，返回通道数；通道含义见本文件顶部速览卡。 */');

// 3) 在顶部文档块结尾之前插入卡片。
//    该文件的块尾形如：  " ***...***" 换行  " */"
//    所以先找 " */" 行，再插到它上面那行（星号行）之前。
const closeIdx = rest.findIndex((l, i) => i > 5 && /^ \*\/\s*$/.test(l));
if (closeIdx < 0) throw new Error('找不到顶部文档块的 */ 行');
const blockEnd = closeIdx - 1;
if (!/^ \*{5,}\s*$/.test(rest[blockEnd])) {
  throw new Error(`*/ 上一行不是星号分隔线: "${rest[blockEnd]}"`);
}
console.log(`  顶部文档块 */ 在第 ${closeIdx + 1} 行，卡片插到第 ${blockEnd + 1} 行之前`);
const out = [...rest.slice(0, blockEnd), ...card, ...rest.slice(blockEnd)];

fs.writeFileSync(H, out.join('\n'), 'utf8');
const t = fs.readFileSync(H, 'utf8');
console.log(`  自检：卡片标题出现 ${(t.match(/模式速览卡/g) || []).length} 次（应为 2 = 卡片内 + VofaFill 注释引用）`);
console.log(`  自检：行数 ${lines.length} -> ${out.length}；/* ${(t.match(/\/\*/g) || []).length}  vs  */ ${(t.match(/\*\//g) || []).length}`);
