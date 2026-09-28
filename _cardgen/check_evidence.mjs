// 校验 _cardgen/cards/*.json 里 evidence 表的 "文件:行号" 是否真的指向该符号。
// 起因：mode51.json 里 Foc_Core_OverCurrent / TMR4_PWM_SetDuty3Phase 的行号已过期，
//       而 evidence 不进卡片正文，过期了没人会发现 —— 用机器查。
import fs from 'node:fs';
import path from 'node:path';

const ROOT = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2';
const CARDS = 'D:/WS_L_re/_cardgen/cards';

/* 文件名 -> 路径索引（同名优先 ws/，再 Adp/，再 template/） */
const index = new Map();
function walk(dir, depth = 0) {
  if (depth > 6) return;
  for (const e of fs.readdirSync(dir, { withFileTypes: true })) {
    const p = path.join(dir, e.name);
    if (e.isDirectory()) {
      if (/^(output|MDK|EWARM|GCC|RTT|\.git)$/.test(e.name)) continue;
      walk(p, depth + 1);
    } else if (/\.(c|h)$/.test(e.name)) {
      const rank = p.includes('\\ws\\') ? 0 : p.includes('\\Adp\\') ? 1 : 2;
      const cur = index.get(e.name);
      if (!cur || rank < cur.rank) index.set(e.name, { p, rank });
    }
  }
}
walk(ROOT);
walk('D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/drivers/hc32_ll_driver/inc', 4);

let bad = 0, total = 0;
for (const f of fs.readdirSync(CARDS).filter(x => x.endsWith('.json')).sort()) {
  const c = JSON.parse(fs.readFileSync(path.join(CARDS, f), 'utf8'));
  const problems = [];
  for (const [sym, loc] of Object.entries(c.evidence ?? {})) {
    total++;
    const m = /^(.+?):(\d+)$/.exec(loc);
    if (!m) { problems.push(`${sym}: 位置格式不是 文件:行号 -> "${loc}"`); continue; }
    const base = path.basename(m[1].replace(/\\/g, '/'));
    const ent = index.get(base);
    if (!ent) { problems.push(`${sym}: 找不到文件 ${base}`); continue; }
    const lines = fs.readFileSync(ent.p, 'utf8').replace(/\r\n/g, '\n').split('\n');
    const ln = Number(m[2]);
    // 带点的"伪符号"（结构体成员，如 g_x_cfg.kp）：按成员名匹配该行
    const probe = sym.includes('.') ? sym.split('.').pop() : sym;
    const re = new RegExp(`\\b${probe.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}\\b`);
    if (ln >= 1 && ln <= lines.length && re.test(lines[ln - 1])) continue;
    // 行号错了：找出它现在在哪
    const now = [];
    lines.forEach((l, i) => { if (re.test(l)) now.push(i + 1); });
    problems.push(`${sym}: ${base}:${ln} 处没有它 -> 实际在 ${now.length ? base + ':' + now.join(',') : '（全文件搜不到）'}`);
  }
  console.log(`--- ${f} (${Object.keys(c.evidence ?? {}).length} 条) ---`);
  if (problems.length) { bad += problems.length; console.log(problems.map(p => '  [X] ' + p).join('\n')); }
  else console.log('  OK  行号与符号全部对得上');
}
console.log(bad ? `\n共 ${bad}/${total} 条 evidence 行号不对` : `\n全部 ${total} 条 evidence 行号核验通过`);
