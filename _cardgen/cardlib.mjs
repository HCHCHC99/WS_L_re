// 速览卡渲染库：JSON 数据 -> 注释块文本，并做机械校验
// 数据 schema 见 D:/WS_L_re/_tmp_card_brief.md
import fs from 'node:fs';
import path from 'node:path';

export const W = 78;
export const BAR = '='.repeat(W);
export const THIN = '-'.repeat(W);
export const WS = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws';
export const MAIN = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/source/main.c';

/* ---------- 显示宽度 ---------- */
const WIDE_RANGES = [
  [0x1100, 0x115F], [0x2E80, 0x303E], [0x3041, 0x33FF], [0x3400, 0x4DBF],
  [0x4E00, 0x9FFF], [0xA000, 0xA4CF], [0xAC00, 0xD7A3], [0xF900, 0xFAFF],
  [0xFE30, 0xFE6F], [0xFF00, 0xFF60], [0xFFE0, 0xFFE6],
];
const isWide = c => WIDE_RANGES.some(([a, b]) => c >= a && c <= b);
export function dispWidth(s) {
  let w = 0;
  for (const ch of s) w += isWide(ch.codePointAt(0)) ? 2 : 1;
  return w;
}
const AMBIGUOUS = /[\u0370-\u03FF\u2010-\u2027\u2030-\u205E\u2190-\u21FF\u2200-\u22FF\u2460-\u24FF\u25A0-\u27BF\u00A1-\u00FF\u2B00-\u2BFF\uFE0F\u200B-\u200F]/;
export function widthHazards(s) {
  const bad = [];
  for (const ch of String(s)) {
    const c = ch.codePointAt(0);
    if (c < 0x80) continue;
    if (AMBIGUOUS.test(ch) || !isWide(c)) bad.push(ch);
  }
  return [...new Set(bad)];
}

/* ---------- 折行 ---------- */
function chunks(word) {
  const out = []; let ascii = '';
  for (const ch of word) {
    if (ch.codePointAt(0) < 0x80) { ascii += ch; continue; }
    if (ascii) { out.push(ascii); ascii = ''; }
    out.push(ch);
  }
  if (ascii) out.push(ascii);
  return out;
}
const CLOSE_PUNCT = '）)]}，。、；：！？》」』%';
const OPEN_PUNCT = '（([{《「『';
function glue(list) {
  const out = [];
  for (const ck of list) {
    const prev = out[out.length - 1];
    if (prev !== undefined && dispWidth(ck) === 2 && CLOSE_PUNCT.includes(ck)) { out[out.length - 1] = prev + ck; continue; }
    if (prev !== undefined && dispWidth(prev) === 2 && OPEN_PUNCT.includes(prev)) { out[out.length - 1] = prev + ck; continue; }
    out.push(ck);
  }
  return out;
}
export function wrap(text, width) {
  if (!text) return [''];
  const lines = [];
  let cur = '', curW = 0;
  const flush = () => { if (cur) { lines.push(cur); cur = ''; curW = 0; } };
  for (const word of String(text).split(' ')) {
    if (word === '') continue;
    const wW = dispWidth(word);
    const sep = cur ? 1 : 0;
    if (curW + sep + wW <= width) { cur += (cur ? ' ' : '') + word; curW += sep + wW; continue; }
    flush();
    if (wW <= width) { cur = word; curW = wW; continue; }
    let buf = '', bufW = 0;
    for (const ck of glue(chunks(word))) {
      const cW = dispWidth(ck);
      if (bufW + cW > width) { lines.push(buf); buf = ''; bufW = 0; }
      buf += ck; bufW += cW;
    }
    cur = buf; curW = bufW;
  }
  flush();
  return lines.length ? lines : [''];
}
const pad = (s, n) => s + ' '.repeat(Math.max(0, n - dispWidth(s)));

/* ---------- 渲染器 ---------- */
const INDENT = 2, GAP = 1;
const COLS = {
  watch: { mk: 3, name: 27, val: 7, unit: 8 },
  obs:   { mk: 3, name: 30 },
  vofa:  { mk: 3, ch: 5, what: 22, unit: 8 },
};
class Card {
  constructor() { this.L = []; this.rows = []; }
  p(s = '') { this.L.push(' *' + (s ? ' ' + s.replace(/\s+$/, '') : '')); return this; }
  cells(section, spec, cellsArr, desc, mk = '') {
    let prefix = ' '.repeat(INDENT) + pad(mk, spec.mk) + ' '.repeat(GAP);
    let prefixW = INDENT + spec.mk + GAP;
    for (const [k, w] of Object.entries(spec)) {
      if (k === 'mk') continue;
      const t = cellsArr[k] ?? '';
      this.rows.push({ section, role: 'row', kind: 'cell', field: k, text: t, width: w, overflow: dispWidth(t) > w });
      prefix += pad(t, w) + ' '.repeat(GAP);
      prefixW += w + GAP;
    }
    wrap(desc, W - prefixW).forEach((l, i) => {
      this.rows.push({ section, role: 'row', kind: 'line', prefixW });
      this.p(i === 0 ? prefix + l : ' '.repeat(prefixW) + l);
    });
  }
  note(section, mk, text) {
    const prefixW = INDENT + COLS.watch.mk + GAP;
    const prefix = ' '.repeat(INDENT) + pad(mk, COLS.watch.mk) + ' '.repeat(GAP);
    wrap(text, W - prefixW).forEach((l, i) => {
      this.rows.push({ section, role: 'note', kind: 'line', prefixW });
      this.p(i === 0 ? prefix + l : ' '.repeat(prefixW) + l);
    });
  }
  header(section, spec, titles, dash = true) {
    let prefix = ' '.repeat(INDENT) + pad('', spec.mk) + ' '.repeat(GAP);
    let dashLine = prefix;
    let prefixW = INDENT + spec.mk + GAP;
    for (const [k, w] of Object.entries(spec)) {
      if (k === 'mk') continue;
      const t = titles[k] ?? '';
      this.rows.push({ section, role: 'row', kind: 'cell', field: k, text: t, width: w, overflow: dispWidth(t) > w });
      prefix += pad(t, w) + ' '.repeat(GAP);
      dashLine += '-'.repeat(w) + ' '.repeat(GAP);
      prefixW += w + GAP;
    }
    this.p(prefix + (titles.desc ?? ''));
    if (dash) { this.p(dashLine + '-'.repeat(W - prefixW)); this.rows.push({ section, role: 'rule', kind: 'line', prefixW }); }
  }
}

/* ---------- 归一化：把 "a / b / c" 这种合并写法拆成每个变量单独一行 ---------- */
export function normalize(c) {
  const split = (list) => {
    const out = [];
    for (const it of list ?? []) {
      if (it.note || !it.name || !String(it.name).includes(' / ')) { out.push(it); continue; }
      for (const n of String(it.name).split(' / ')) out.push({ ...it, name: n.trim(), _split: true });
    }
    return out;
  };
  return { ...c, watch: split(c.watch), obs: split(c.obs) };
}
/** 拆分出来的行（描述会重复）——需要人工改成每个变量各自的说明 */
export function splitRows(cRaw) {
  const c = normalize(cRaw);
  return [...(c.obs ?? []), ...(c.watch ?? [])].filter(x => x._split).map(x => x.name);
}

/** schema 数据 -> { text, rows }  */
export function renderCard(cRaw) {
  const c = normalize(cRaw);
  const k = new Card();
  k.p(BAR);
  {
    const prefixW = INDENT + 10 + 3 + 7 + 3;          // "  模式速览卡   MODE NN   "
    const prefix = ' '.repeat(INDENT) + '模式速览卡' + '   ' + `MODE ${c.mode}`.padEnd(7) + '   ';
    wrap(c.title, W - prefixW).forEach((l, i) => k.p(i === 0 ? prefix + l : ' '.repeat(prefixW) + l));
  }
  k.p('  本卡是本模式唯一事实源；代码内注释与本卡冲突时，以本卡为准');
  k.p(BAR);
  const meta = (label, text) => {
    const prefixW = INDENT + 7;
    const prefix = ' '.repeat(INDENT) + pad(label, 7);
    wrap(text, W - prefixW).forEach((l, i) => k.p(i === 0 ? prefix + l : ' '.repeat(prefixW) + l));
  };
  meta('入口', c.entry);
  meta('前置', c.pre);
  meta('结束', c.end);
  k.p('  标记   [!] 易错点   (*) 主判据   [可调] Watch 可写   [只读] 仅观察');
  const sec = (t) => { k.p(THIN); k.p('  ' + t); k.p(THIN); };

  sec('[可调] Watch 变量 -- 改值即时生效，复位后回宏默认');
  k.header('watch', COLS.watch, { name: '变量', val: '当前值', unit: '单位', desc: '含义' });
  for (const it of c.watch ?? []) {
    if (it.note) k.note('watch', it.mk ?? '', it.note);
    else k.cells('watch', COLS.watch, { name: it.name, val: it.val ?? '', unit: it.unit ?? '' }, it.desc ?? '', it.mk ?? '');
  }

  sec('[只读] 关键观察变量');
  k.header('obs', COLS.obs, { name: '变量', desc: '含义' });
  for (const it of c.obs ?? []) {
    if (it.note) k.note('obs', it.mk ?? '', it.note);
    else k.cells('obs', COLS.obs, { name: it.name }, it.desc ?? '', it.mk ?? '');
  }

  const shared = c.vofa?.shared ? '（本 mode 走通用布局）' : '';
  sec(`VOFA 通道 -- ${c.vofa?.n}ch${shared}，填充见 ${c.vofa?.fn}`);
  k.header('vofa', COLS.vofa, { ch: '通道', what: '含义', unit: '单位', desc: '备注' });
  for (const it of c.vofa?.rows ?? []) {
    k.cells('vofa', COLS.vofa, { ch: it.ch, what: it.what, unit: it.unit ?? '' }, it.desc ?? '', it.mk ?? '');
  }
  if (c.vofa?.shared) {
    k.note('vofa', '[!]', '本表是通用布局的副本（权威定义 = main.c 里 Foc_Common_VofaFill 上方那张 MODE 0 卡）；'
      + '改通用通道时 MODE 0/20/23/24/25 五张卡要同步改。');
  }
  for (const v of c.vofa?.variants ?? []) {
    k.note('vofa', '[!]', `本 mode 通道数会变 -- ${v.cond} 改成 ${v.n}ch${v.title ? '：' + v.title : ''}`);
    if (v.rows?.length) {
      k.header('vofa', COLS.vofa, { ch: '通道', what: '含义', unit: '单位', desc: '备注' }, false);
      for (const it of v.rows) {
        k.cells('vofa', COLS.vofa, { ch: it.ch, what: it.what, unit: it.unit ?? '' }, it.desc ?? '', it.mk ?? '');
      }
    }
    if (v.note) k.note('vofa', '[!]', v.note);
  }

  sec('判据与坑');
  for (const n of c.notes ?? []) k.note('notes', n.mk ?? '', n.text);
  k.p(BAR);
  return { text: k.L.join('\n'), rows: k.rows };
}

/* ---------- 校验 ---------- */
export function verify(text, rows, label) {
  const lines = text.split('\n');
  const problems = [];
  lines.forEach((l, i) => {
    const body = l.replace(/^ \* ?/, '');
    const w = dispWidth(body);
    if (w > W) problems.push(`第${i + 1}行超宽 ${w}>${W}: ${body.slice(0, 56)}`);
    const hz = widthHazards(body);
    if (hz.length) problems.push(`第${i + 1}行含宽度不确定字符: ${hz.join(' ')}`);
  });
  for (const r of rows) {
    if (r.kind === 'cell' && r.overflow) problems.push(`[${r.section}] 字段 ${r.field} 溢出列宽: "${r.text}" (${dispWidth(r.text)}>${r.width})`);
  }
  const bySection = {};
  for (const r of rows) (bySection[r.section] ??= []).push(r);
  for (const [s, rs] of Object.entries(bySection)) {
    const starts = [...new Set(rs.filter(r => r.role === 'row' && r.kind === 'line').map(r => r.prefixW))];
    if (starts.length > 1) problems.push(`[${s}] 表格描述列起止位置不唯一: ${starts.join(',')}`);
  }
  console.log(`--- ${label} ---`);
  console.log(problems.length ? problems.map(p => '  [!] ' + p).join('\n') : `  OK  ${lines.length} 行，列宽/行宽/字符宽度全部通过`);
  return problems;
}

/* ---------- 源码符号核验 ---------- */
let wsText = null;
function loadWs() {
  if (wsText) return wsText;
  wsText = new Map();
  for (const f of fs.readdirSync(WS)) {
    if (!/\.(c|h)$/.test(f)) continue;
    wsText.set(f, fs.readFileSync(path.join(WS, f), 'utf8').replace(/\r\n/g, '\n').split('\n'));
  }
  wsText.set('main.c', fs.readFileSync(MAIN, 'utf8').replace(/\r\n/g, '\n').split('\n'));
  return wsText;
}
export function findSymbol(name) {
  const files = loadWs();
  const re = new RegExp(`\\b${name.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}\\b`);
  const hits = [];
  for (const [f, lines] of files) lines.forEach((l, i) => { if (re.test(l)) hits.push(`${f}:${i + 1}`); });
  return hits;
}
/** 该 mode 自己的源文件（由 mode 号推导，不依赖 JSON 里是否写了 files） */
export function ownFilesOf(mode) {
  if (mode === 0) return ['main.c'];
  const nn = String(mode).padStart(2, '0');
  return fs.readdirSync(WS).filter(f => new RegExp(`^foc_${nn}_\\d*\\.?.*\\.(c|h)$`).test(f) || f.startsWith(`foc_${nn}_`));
}
/** 校验数据里出现的所有符号名：必须真实存在；不在本 mode 文件里的只做提示，不算错 */
export function verifySymbols(c, label) {
  const own = ownFilesOf(c.mode);
  const names = new Set();
  for (const it of c.watch ?? []) if (it.name) String(it.name).split(' / ').forEach(s => names.add(s.trim()));
  for (const it of c.obs ?? []) if (it.name) String(it.name).split(' / ').forEach(s => names.add(s.trim()));
  if (c.vofa?.fn) names.add(c.vofa.fn);
  for (const k of Object.keys(c.evidence ?? {})) names.add(k);
  const errors = [], notes = [];
  for (const n of [...names].sort()) {
    const hits = findSymbol(n);
    // 结构体成员名（含 .）整体搜不到时退化为搜其根变量
    const rootHits = hits.length ? hits : findSymbol(n.split('.')[0]);
    if (!rootHits.length) { errors.push(`符号不存在: ${n}`); continue; }
    const inOwn = rootHits.some(h => own.some(f => h.startsWith(f + ':')));
    if (!inOwn) notes.push(`${n} 不在本 mode 文件里（在 ${rootHits[0]}）`);
  }
  console.log(`--- ${label} 符号核验 (${names.size} 个) ---`);
  if (errors.length) console.log(errors.map(p => '  [X] ' + p).join('\n'));
  if (notes.length) console.log(notes.map(p => '  [~] ' + p).join('\n'));
  if (!errors.length && !notes.length) console.log('  OK  全部符号都在本 mode 文件中存在');
  else if (!errors.length) console.log(`  OK  无杜撰符号（${notes.length} 个来自公共模块/其它文件，属正常）`);
  return { errors, notes };
}
