// 扫描 cards/*.json 里的"宽度不确定字符"，给出上下文，便于逐条替换
import fs from 'node:fs';
import { widthHazards } from './cardlib.mjs';

const dir = 'D:/WS_L_re/_cardgen/cards';
for (const f of fs.readdirSync(dir).filter(f => f.endsWith('.json')).sort()) {
  const raw = fs.readFileSync(`${dir}/${f}`, 'utf8');
  const bad = [];
  for (const line of raw.split('\n')) {
    const hz = widthHazards(line);
    if (!hz.length) continue;
    bad.push(`${hz.join(' ')}  <-  ${line.trim().slice(0, 90)}`);
  }
  if (bad.length) { console.log(`\n### ${f}  (${bad.length} 行)`); bad.forEach(b => console.log('   ' + b)); }
}
console.log('\n扫描完成');
