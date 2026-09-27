// 一致性校验：头文件里的卡片 == 由 cards/*.json 渲染出来的文本（逐行比对）
import fs from 'node:fs';
import path from 'node:path';
import { renderCard, ownFilesOf, WS, MAIN } from './cardlib.mjs';

const CARDS = 'D:/WS_L_re/_cardgen/cards';
const isBar = l => /^ \* ={10,}\s*$/.test(l);
const isBarAny = l => /^ \* ={10,}/.test(l);
const titleRe = /^ \*\s*(={4,}\s*)?模式速览卡/;

function extractCard(filePath) {
  const lines = fs.readFileSync(filePath, 'utf8').replace(/\r\n/g, '\n').split('\n');
  const titleIdx = lines.findIndex(l => titleRe.test(l));
  if (titleIdx < 0) return null;
  let start = titleIdx;
  if (!isBarAny(lines[start])) for (let i = titleIdx; i >= 0; i--) if (isBarAny(lines[i])) { start = i; break; }
  let end = -1;
  for (let i = lines.length - 1; i > titleIdx; i--) if (isBar(lines[i])) { end = i; break; }
  return lines.slice(start, end + 1);
}

let bad = 0;
for (const f of fs.readdirSync(CARDS).filter(f => /^mode\d+\.json$/.test(f)).sort()) {
  const c = JSON.parse(fs.readFileSync(path.join(CARDS, f), 'utf8'));
  const hdr = c.mode === 0 ? MAIN : path.join(WS, ownFilesOf(c.mode).find(x => x.endsWith('.h')));
  const want = renderCard(c).text.split('\n');
  const got = extractCard(hdr);
  const name = path.basename(hdr);
  if (!got) { console.log(`  [X] mode ${c.mode}: ${name} 里找不到卡片`); bad++; continue; }
  if (got.length !== want.length) {
    console.log(`  [X] mode ${c.mode}: ${name} 卡片 ${got.length} 行，JSON 渲染 ${want.length} 行（不同步）`);
    bad++; continue;
  }
  const diff = [];
  for (let i = 0; i < want.length; i++) if (want[i] !== got[i]) diff.push(`第${i + 1}行\n      头文件: ${got[i]}\n      JSON  : ${want[i]}`);
  if (diff.length) { console.log(`  [X] mode ${c.mode}: ${name} 有 ${diff.length} 行不同\n      ${diff.slice(0, 3).join('\n      ')}`); bad++; }
  else console.log(`  OK  mode ${String(c.mode).padEnd(3)} ${name.padEnd(20)} ${got.length} 行完全一致`);
}
console.log(bad ? `\n有 ${bad} 个文件不同步` : '\n全部头文件卡片与 JSON 数据完全同步');
