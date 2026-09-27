// 死宏扫描：对指定头/源文件里的每个 #define，统计它在整个工程里的引用情况
//   代码引用 = 去掉注释后还能搜到；注释引用 = 只在注释里出现；0 = 完全没人用
import fs from 'node:fs';
import path from 'node:path';

const ROOT = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2';
const TARGETS = process.argv.slice(2);

function walk(dir, out = []) {
  for (const e of fs.readdirSync(dir, { withFileTypes: true })) {
    const p = path.join(dir, e.name);
    if (e.isDirectory()) walk(p, out);
    else if (/\.(c|h)$/.test(e.name)) out.push(p);
  }
  return out;
}
// 把源码拆成"只有代码"和"只有注释"两份，保持字符位置以便计数
function split(src) {
  let code = '', cmt = '';
  let i = 0, inLine = false, inBlock = false, inStr = false;
  while (i < src.length) {
    const c = src[i], n = src[i + 1];
    if (inLine) { if (c === '\n') { inLine = false; code += c; cmt += ' '; } else { code += ' '; cmt += c; } i++; continue; }
    if (inBlock) {
      if (c === '*' && n === '/') { inBlock = false; code += '  '; cmt += '*/'; i += 2; continue; }
      code += c === '\n' ? '\n' : ' '; cmt += c; i++; continue;
    }
    if (inStr) { code += c; cmt += ' '; if (c === '\\') { code += src[i + 1] ?? ''; cmt += ' '; i += 2; continue; } if (c === '"') inStr = false; i++; continue; }
    if (c === '"') { inStr = true; code += c; cmt += ' '; i++; continue; }
    if (c === '/' && n === '/') { inLine = true; code += '  '; cmt += '//'; i += 2; continue; }
    if (c === '/' && n === '*') { inBlock = true; code += '  '; cmt += '/*'; i += 2; continue; }
    code += c; cmt += ' '; i++;
  }
  return { code, cmt };
}

const files = walk(ROOT);
const merged = { code: '', cmt: '' };
for (const f of files) {
  const { code, cmt } = split(fs.readFileSync(f, 'utf8').replace(/\r\n/g, '\n'));
  merged.code += '\n' + code;
  merged.cmt += '\n' + cmt;
}
const count = (txt, name) => (txt.match(new RegExp(`\\b${name}\\b`, 'g')) || []).length;

for (const t of TARGETS) {
  const p = path.join(ROOT, 'ws', t);
  const src = fs.readFileSync(p, 'utf8').replace(/\r\n/g, '\n');
  const names = [...src.matchAll(/^#define\s+([A-Za-z_]\w*)/gm)].map(m => m[1]);
  if (!names.length) { console.log(`\n### ${t}: 没有 #define`); continue; }
  const dead = [], cmtOnly = [], used = [];
  for (const n of names) {
    const inCode = count(merged.code, n) - 1;   // 减去定义本身
    const inCmt = count(merged.cmt, n);
    if (inCode <= 0 && inCmt <= 0) dead.push(n);
    else if (inCode <= 0) cmtOnly.push(`${n}(${inCmt})`);
    else used.push(`${n}(${inCode})`);
  }
  console.log(`\n### ${t}  (${names.length} 个宏)`);
  console.log(`  [死宏：代码与注释都没人用] ${dead.length ? dead.join('  ') : '无'}`);
  if (cmtOnly.length) console.log(`  [只在注释里被提到] ${cmtOnly.join('  ')}`);
  console.log(`  [正常使用] ${used.length} 个`);
}
