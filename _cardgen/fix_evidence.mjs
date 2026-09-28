// 把 _cardgen/cards/*.json 的 evidence 行号按当前源码机械重算。
// 背景：卡片每被 splice 进 .h 顶部，其下方所有行号就整体下移，旧行号会静默腐烂
//       （实测 604 条里 343 条已过期）。evidence 不进卡片正文，靠 check_evidence.mjs 查。
// 规则：保持原文件不变（文件已删则按 mode 自己的文件 -> 全库搜索）；行号取"定义行"：
//   #define X        -> 该行
//   X( ... ) {       -> 函数定义（行尾不是分号）
//   类型 X = / X[ / X; -> 变量定义（优先非 extern）
import fs from 'node:fs';
import path from 'node:path';

const ROOT = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2';
const DDL = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/drivers/hc32_ll_driver/inc';
const CARDS = 'D:/WS_L_re/_cardgen/cards';
const APPLY = process.argv.includes('--apply');

/* 文件名索引：同名优先 ws/ > Adp/ > DDL inc > 其它 */
const index = new Map();
function walk(dir, depth = 0) {
  if (depth > 6) return;
  let ents; try { ents = fs.readdirSync(dir, { withFileTypes: true }); } catch { return; }
  for (const e of ents) {
    const p = path.join(dir, e.name);
    if (e.isDirectory()) {
      if (/^(output|MDK|EWARM|GCC|RTT|\.git|docs)$/.test(e.name)) continue;
      walk(p, depth + 1);
    } else if (/\.(c|h)$/.test(e.name)) {
      const rank = p.includes('\\ws\\') ? 0 : p.includes('\\Adp\\') ? 1 : p.startsWith(DDL.replace(/\//g, '\\')) ? 2 : 3;
      const cur = index.get(e.name);
      if (!cur || rank < cur.rank) index.set(e.name, { p, rank });
    }
  }
}
walk(ROOT);
walk(DDL, 4);

const esc = s => s.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
/** 在 lines 里挑该符号的"定义行"（1-based），找不到返回 0 */
function defLine(lines, sym) {
  // 带点的"伪符号"（结构体成员，如 g_x_cfg.kp）：先定位根变量，再取其后第一个 ".kp = ..."
  if (sym.includes('.')) {
    const parts = sym.split('.');
    const member = parts.pop();
    const rootLine = lines.findIndex(l => new RegExp(`\\b${esc(parts.join('.'))}\\b`).test(l));
    const mre = new RegExp(`\\.${esc(member)}\\b\\s*=`);
    const from = rootLine >= 0 ? rootLine : 0;
    for (let i = from; i < lines.length; i++) if (mre.test(lines[i])) return i + 1;
    return 0;
  }
  const re = new RegExp(`\\b${esc(sym)}\\b`);
  const isComment = l => /^\s*(\/\/|\*|\/\*)/.test(l);
  const cand = [];
  lines.forEach((l, i) => { if (re.test(l)) cand.push({ n: i + 1, l }); });
  if (!cand.length) return 0;
  const isMacro = new RegExp(`^\\s*#\\s*define\\s+${esc(sym)}\\b`);
  const isFuncDef = new RegExp(`\\b${esc(sym)}\\s*\\(`);
  const isVarDef = new RegExp(`\\b${esc(sym)}\\s*(=|\\[|;)`);
  const nextBraces = i => {   // 第 i 行(0-based)之后第一个非空行是否以 { 开头
    for (let k = i + 1; k < Math.min(i + 4, lines.length); k++) {
      if (!lines[k].trim()) continue;
      return /^\s*\{/.test(lines[k]);
    }
    return false;
  };
  for (const c of cand) if (isMacro.test(c.l)) return c.n;
  for (const c of cand) {     // 函数定义：非注释行、带参数表、且函数体跟着
    if (isComment(c.l)) continue;
    if (!isFuncDef.test(c.l)) continue;
    if (/;\s*$/.test(c.l)) continue;
    if (/\{\s*$/.test(c.l) || nextBraces(c.n - 1)) return c.n;
  }
  const code = cand.filter(c => !isComment(c.l));
  const nonExtern = code.filter(c => !/^\s*extern\b/.test(c.l) && isVarDef.test(c.l));
  if (nonExtern.length) return nonExtern[0].n;
  const anyVar = code.filter(c => isVarDef.test(c.l));
  if (anyVar.length) return anyVar[0].n;
  if (code.length) return code[0].n;
  return cand[0].n;
}

let changed = 0, missing = 0;
for (const f of fs.readdirSync(CARDS).filter(x => x.endsWith('.json')).sort()) {
  const p = path.join(CARDS, f);
  const c = JSON.parse(fs.readFileSync(p, 'utf8'));
  const ev = c.evidence ?? {};
  const diffs = [];
  for (const [sym, loc] of Object.entries(ev)) {
    const m = /^(.+?):(\d+)$/.exec(loc);
    if (!m) { diffs.push(`${sym}: 格式异常 "${loc}"`); continue; }
    const base = path.basename(m[1].replace(/\\/g, '/'));
    let ent = index.get(base);
    if (!ent) {                                   // 原文件没了：退回到 mode 自己的文件里找
      const nn = String(c.mode).padStart(2, '0');
      for (const cand of fs.readdirSync(path.join(ROOT, 'ws'))) {
        if (!cand.startsWith(`foc_${nn}_`)) continue;
        const t = fs.readFileSync(path.join(ROOT, 'ws', cand), 'utf8').split('\n');
        const n = defLine(t, sym);
        if (n) { ent = { p: path.join(ROOT, 'ws', cand) }; break; }
      }
    }
    if (!ent) { diffs.push(`${sym}: 找不到文件 ${base}`); missing++; continue; }
    const lines = fs.readFileSync(ent.p, 'utf8').replace(/\r\n/g, '\n').split('\n');
    const n = defLine(lines, sym);
    if (!n) { diffs.push(`${sym}: 在 ${path.basename(ent.p)} 里搜不到`); missing++; continue; }
    const rel = path.basename(ent.p);
    const now = `${rel}:${n}`;
    if (now !== loc) { diffs.push(`${sym}: ${loc}  ->  ${now}`); ev[sym] = now; changed++; }
  }
  if (diffs.length) {
    console.log(`--- ${f} ---`);
    console.log(diffs.map(d => '  ' + d).join('\n'));
    if (APPLY) fs.writeFileSync(p, JSON.stringify(c, null, 2) + '\n', 'utf8');
  }
}
console.log(APPLY ? `\n已回写：${changed} 条行号更新，${missing} 条未解决`
                  : `\n[dry] 需更新 ${changed} 条，未解决 ${missing} 条（加 --apply 回写）`);
