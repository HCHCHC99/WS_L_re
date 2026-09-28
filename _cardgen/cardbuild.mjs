// 批量构建：读 cards/mode*.json -> 渲染 + 校验 + （可选）拼进头文件
// 用法:
//   node _tmp_cardbuild.mjs            仅渲染到 _tmp_out/mode<NN>.txt 并校验
//   node _tmp_cardbuild.mjs --splice   额外把卡片写进对应头文件
import fs from 'node:fs';
import path from 'node:path';
import { renderCard, verify, verifySymbols, splitRows, ownFilesOf, WS, MAIN, dispWidth } from './cardlib.mjs';

const CARDS = 'D:/WS_L_re/_cardgen/cards';
const OUTDIR = 'D:/WS_L_re/_cardgen/out';
const DO_SPLICE = process.argv.includes('--splice');
const DRY = process.argv.includes('--dry');
fs.mkdirSync(OUTDIR, { recursive: true });

const isBar = l => /^ \* ={10,}\s*$/.test(l);              // 纯横线
const isBarAny = l => /^ \* ={10,}/.test(l);               // 横线或"横线+标题"
// 定位旧卡区块：起始 = "模式速览卡"所在行（含标题横线）或其上方最近的横线；
// 结束 = 注释块结尾之前最后一条横线。
function locateCard(lines, cardAnchor) {
  // "标题行" = 行首（允许先有 ==== 横线）紧跟"模式速览卡"；正文里顺带提到它的句子不算，
  // 否则一张卡里的交叉引用会把锚点劫持到别的注释块上（曾因此误删 main.c 的 Keil Watch 段）。
  const titleRe = /^ \*\s*(={4,}\s*)?模式速览卡/;
  const titleIdx = lines.findIndex(l => titleRe.test(l));
  if (titleIdx < 0) {
    // 首次插入：优先放进**文件顶部文档块**（含 @file 的那个 /* ... */）的末尾。
    // 不能直接用填充函数名当锚点 —— .h 里通常也有 "int Foc_Xxx_VofaFill(...)" 声明，
    // 锚点会命中 API 注释块，把卡插到文件中部（mode 51 曾因此插错，靠一次性脚本搬回）。
    const docIdx = lines.findIndex(l => /@file\b/.test(l));
    if (docIdx >= 0) {
      let cstart = -1, cend = -1;
      for (let i = docIdx; i >= 0; i--) if (/^\/\*/.test(lines[i])) { cstart = i; break; }
      for (let i = docIdx; i < lines.length; i++) if (/\*\/\s*$/.test(lines[i])) { cend = i; break; }
      if (cstart >= 0 && cend >= 0) {
        // 文档块以 " ***...***" 星号分隔线收尾 => 插到它之前，保持块尾格式一致
        const insertAt = /^ \*{5,}\s*$/.test(lines[cend - 1]) ? cend - 1 : cend;
        return { start: insertAt, end: insertAt - 1, closeIdx: cend, insertOnly: true };
      }
    }
    // 没有文档块（理论上不会发生）：退回用填充函数定义作为锚点，替换它上方那个注释块
    const fn = (cardAnchor ?? 'Foc_Common_VofaFill');
    const fnIdx = lines.findIndex(l => new RegExp(`^\\s*(static\\s+)?int\\s+${fn}\\s*\\(`).test(l));
    if (fnIdx < 0) return null;
    let cstart = -1;
    for (let i = fnIdx; i >= 0; i--) if (/^\/\*/.test(lines[i])) { cstart = i; break; }
    let cend = -1;
    for (let i = fnIdx - 1; i >= cstart; i--) if (/\*\/\s*$/.test(lines[i])) { cend = i; break; }
    if (cstart < 0 || cend < 0) return null;
    return { start: cstart, end: cend, closeIdx: cend + 1, replaceBlock: true };
  }
  const closeIdx = lines.findIndex((l, i) => i > titleIdx && /^ \*\//.test(l));
  if (closeIdx < 0) return null;
  let start = titleIdx;
  if (!isBarAny(lines[start])) {
    for (let i = titleIdx; i >= 0; i--) if (isBarAny(lines[i])) { start = i; break; }
  }
  let end = -1;
  for (let i = closeIdx - 1; i > titleIdx; i--) if (isBarAny(lines[i])) { end = i; break; }
  if (end < 0) return null;
  return { start, end, closeIdx, insertOnly: false };
}
function splice(headerPath, cardLines, cardAnchor) {
  const lines = fs.readFileSync(headerPath, 'utf8').replace(/\r\n/g, '\n').split('\n');
  const loc = locateCard(lines, cardAnchor);
  if (!loc) throw new Error('无法定位插入点');
  const { start, end, closeIdx, insertOnly, replaceBlock } = loc;
  // 卡尾与 */ 之间只允许：空注释行，或文档块收尾的星号分隔线（" ***...***"）
  if (!replaceBlock) {
    for (let i = end + 1; i < closeIdx; i++) {
      if (!/^ \*(?:\s*|\*{5,}\s*)$/.test(lines[i])) {
        throw new Error(`卡尾与 */ 之间有多余内容: 第${i + 1}行 "${lines[i]}"`);
      }
    }
  }
  if (DRY) {
    return {
      start: start + 1, end: end + 1, insertOnly,
      from: lines[start]?.slice(0, 60) ?? '(文件开头)',
      to: lines[end]?.slice(0, 60) ?? '(无)',
    };
  }
  // 独立 /* ... */ 注释块整块替换时，需要自己补上注释定界符
  const body = replaceBlock ? ['/*', ...cardLines, ' */'] : cardLines;
  const out = insertOnly
    ? [...lines.slice(0, start), ' *', ...body, ...lines.slice(start)]
    : [...lines.slice(0, start), ...body, ...lines.slice(end + 1)];
  fs.writeFileSync(headerPath, out.join('\n'), 'utf8');
  return { start: start + 1, end: start + body.length, insertOnly, replaceBlock };
}

const files = fs.readdirSync(CARDS).filter(f => /^mode\d+\.json$/.test(f)).sort();
console.log(`发现 ${files.length} 个 mode 数据: ${files.join(' ')}\n`);
const summary = [];
let fail = 0;
for (const f of files) {
  const c = JSON.parse(fs.readFileSync(path.join(CARDS, f), 'utf8'));
  const tag = `mode${c.mode}`;
  console.log(`\n================ ${tag}  ${c.title ?? ''} ================`);
  // 1) schema 完整性
  const need = ['mode', 'title', 'entry', 'pre', 'end', 'watch', 'obs', 'vofa', 'notes', 'evidence'];
  const missing = need.filter(k => c[k] === undefined);
  if (missing.length) { console.log(`  [!] 缺字段: ${missing.join(', ')}`); fail++; }
  // 2) VOFA 行数与通道标号
  if (c.vofa?.rows) {
    if (c.vofa.rows.length !== c.vofa.n) { console.log(`  [!] vofa.rows 有 ${c.vofa.rows.length} 行，但 n = ${c.vofa.n}`); fail++; }
    c.vofa.rows.forEach((r, i) => { if (r.ch !== `ch${i}`) { console.log(`  [!] 第 ${i} 行通道标号是 ${r.ch}，应为 ch${i}`); fail++; } });
  }
  // 3) 渲染 + 版式校验
  const { text, rows } = renderCard(c);
  const p1 = verify(text, rows, `${tag} 版式校验`);
  fail += p1.length;
  // 3b) 合并写法拆出来的行：描述会重复，需要改成每个变量各自的说明
  const sp = splitRows(c);
  if (sp.length) console.log(`  [~] 由合并写法拆出的行（描述重复，建议手改）: ${sp.join(' ')}`);
  // 4) 符号核验
  const { errors: p2 } = verifySymbols(c, tag);
  fail += p2.length;
  // 5) 写出 + 可选拼接
  const outPath = path.join(OUTDIR, `${tag}.txt`);
  fs.writeFileSync(outPath, text, 'utf8');
  console.log(`  写出 ${outPath}`);
  if (DO_SPLICE) {
    const headerPath = c.mode === 0 ? MAIN : (() => {
      const cand = ownFilesOf(c.mode).filter(f => f.endsWith('.h'));
      return cand.length ? path.join(WS, cand[0]) : '';
    })();
    if (!headerPath || !fs.existsSync(headerPath)) { console.log(`  [!] 找不到头文件 (mode ${c.mode})`); fail++; }
    else {
      try {
        const r = splice(headerPath, text.split('\n'), c.vofa?.fn);
        console.log(`  ${DRY ? '[dry] 将替换' : (r.insertOnly ? '新增' : '替换')} ${path.basename(headerPath)} 第 ${r.start}~${r.end} 行`);
        if (DRY) { console.log(`        原首行: ${r.from}`); console.log(`        原尾行: ${r.to}`); }
      } catch (e) { console.log(`  [!] ${path.basename(headerPath)} 拼接失败: ${e.message}`); fail++; }
    }
  }
  const wrapped = rows.filter(r => r.role === 'row' && r.kind === 'line').length;
  summary.push(`mode ${String(c.mode).padEnd(3)} ${String(c.vofa?.n).padStart(2)}ch  watch ${String((c.watch ?? []).filter(x => !x.note).length).padStart(2)}  obs ${String((c.obs ?? []).filter(x => !x.note).length).padStart(2)}  notes ${String((c.notes ?? []).length).padStart(2)}  ${wrapped} 行`);
}
console.log('\n\n================ 汇总 ================');
summary.forEach(s => console.log('  ' + s));
console.log(fail ? `\n共 ${fail} 处问题需要处理` : '\n全部通过，无问题');
