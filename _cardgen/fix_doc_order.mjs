// 把设计文档里"ψf 节"与"Ld/Lq 节"的正文对调，并同步两节内部的 mode 号与变量前缀
// 目标终态：二、mode 52 = Ld/Lq (g_ldlq52_*, FOC52_*)；三、mode 53 = ψf (g_flx53_*, FOC53_*)
import fs from 'node:fs';

const P = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/md_record/参数辨识mode方案_51_52_53.md';
const lines = fs.readFileSync(P, 'utf8').replace(/\r\n/g, '\n').split('\n');

const idx = (re) => lines.findIndex(l => re.test(l));
const iA = idx(/^## 二、mode 52：Ld \/ Lq 辨识/);
const iB = idx(/^## 三、mode 53：ψf 辨识/);
const iC = lines.findIndex((l, i) => i > iB && /^## /.test(l));
if (iA < 0 || iB < 0 || iC < 0) throw new Error(`定位失败 A=${iA} B=${iB} C=${iC}`);

let bodyA = lines.slice(iA + 1, iB);   // 当前装的是 ψf 内容 → 应搬到"三"
let bodyB = lines.slice(iB + 1, iC);   // 当前装的是 Ld/Lq 内容 → 应搬到"二"

const rename = (arr, pairs) => arr.map(l => {
  let s = l;
  for (const [from, to] of pairs) s = s.split(from).join(to);
  return s;
});
// ψf 内容：52 -> 53
bodyA = rename(bodyA, [['g_flx52_', 'g_flx53_'], ['FOC52_', 'FOC53_'], ['mode 52', 'mode 53'], ['**52**', '**53**']]);
// Ld/Lq 内容：53 -> 52
bodyB = rename(bodyB, [['g_ldlq53_', 'g_ldlq52_'], ['FOC53_', 'FOC52_'], ['mode 53', 'mode 52'], ['**53**', '**52**']]);

const out = [...lines.slice(0, iA + 1), ...bodyB, ...lines.slice(iB, iB + 1), ...bodyA, ...lines.slice(iC)];
fs.writeFileSync(P, out.join('\n'), 'utf8');
console.log(`  二节正文 ${bodyA.length} 行（ψf）、三节正文 ${bodyB.length} 行（Ld/Lq）已对调`);
console.log(`  行数 ${lines.length} -> ${out.length}（应相等）`);
