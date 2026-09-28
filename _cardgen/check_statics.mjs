// 预检：文件级静态变量的"标量/数组"用法是否一致。
// 起因：mode 52 把 s_dsum/s_dcnt 等改成数组后，探测分支仍按标量用 -> ARMCC 报
// "#171 invalid type conversion / #32 expression must have arithmetic type"。
// 规则：声明为 T name[N] 的，每次使用都必须跟 [；声明为标量的，出现 name[ 即报错。
// 用法：node _cardgen/check_statics.mjs ws/foc_52_ldlq.c
import fs from 'node:fs';
import path from 'node:path';

const ROOT = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2';
const TARGET = process.argv[2] ?? 'ws/foc_52_ldlq.c';

const raw = fs.readFileSync(path.join(ROOT, TARGET), 'utf8').replace(/\r\n/g, '\n');
/* 去注释与字符串，避免注释里的示例名干扰 */
const src = raw
  .replace(/\/\*[\s\S]*?\*\//g, m => m.replace(/[^\n]/g, ' '))
  .replace(/\/\/[^\n]*/g, m => ' '.repeat(m.length))
  .replace(/"(?:\\.|[^"\\\n])*"/g, m => ' '.repeat(m.length));

/* 收集文件级 static 声明：static <type...> name; / name[..]; / name, name2; */
const decls = new Map();   // name -> 'array' | 'scalar'
for (const line of src.split('\n')) {
  const m = /^\s*static\s+(?!.*\()([A-Za-z_][\w\s\*]*?)\s+([^;{}()]+);/.exec(line);
  if (!m) continue;
  const typePart = m[1];
  if (/\b(static|inline)\b/.test(typePart.replace(/^static\s*/, ''))) continue;
  for (const piece of m[2].split(',')) {
    const p = piece.trim();
    const d = /^([A-Za-z_]\w*)\s*(\[[^\]]*\])?\s*(=.*)?$/.exec(p);
    if (!d) continue;
    decls.set(d[1], d[2] ? 'array' : 'scalar');
  }
}

const problems = [];
/* 注：试过"块内 static（缩进>4）即报警"，但本工程大量合法使用这种写法
 *（foc_obs 里每个 mode 的打印节流 s_last_xxx_dbg 就写在各自的 if 块里，只在块内用），
 * 28 处误报。要区分"块内合法"与"跨块引用"必须做词法作用域分析，不划算 ——
 * 这类错误编译器会直接报 #20 identifier undefined，已足够。规则撤掉，不留噪音。 */
for (const [name, kind] of decls) {
  const uses = [...src.matchAll(new RegExp(`\\b${name}\\b(\\s*\\[)?`, 'g'))];
  let first = true;
  for (const u of uses) {
    if (first) { first = false; continue; }          // 跳过声明本身
    const bracketed = u[1] !== undefined;
    if (kind === 'array' && !bracketed) {
      problems.push(`数组 ${name} 被当标量用（缺下标）`);
    }
    if (kind === 'scalar' && bracketed) {
      problems.push(`标量 ${name} 被当下标访问`);
    }
  }
}
const uniq = [...new Set(problems)];
console.log(`--- ${TARGET}：文件级 static 变量 ${decls.size} 个（数组 ${[...decls.values()].filter(v => v === 'array').length}）---`);
if (uniq.length) {
  console.log(uniq.map(p => `  [X] ${p}`).join('\n'));
  process.exit(1);
}
console.log('  OK  标量/数组用法一致');
