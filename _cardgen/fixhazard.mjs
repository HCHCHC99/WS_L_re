// 把 JSON 数据里"宽度不确定字符"替换成 ASCII 等价写法（Keil 里宽度算不准会破坏整列对齐）
import fs from 'node:fs';

const MAP = {
  '°': 'deg', '±': '+/-', '—': '--', '–': '-', '→': '->', '←': '<-', '≈': '~=',
  '≥': '>=', '≤': '<=', '×': 'x', '·': '*', '∙': '*', '…': '...', '√': 'sqrt',
  '⇒': '=>', '⇐': '<=', '⇔': '<=>', '≡': '==', '∝': 'prop', '∠': 'ang',
  '’': "'", '‘': "'", '“': '"', '”': '"', '～': '~', '∈': 'in', '∞': 'inf',
  '①': '1.', '②': '2.', '③': '3.', '④': '4.', '⑤': '5.',
  '★': '*', '⚠': '!', '✓': 'v', 'Ω': 'ohm', 'µ': 'u', 'μ': 'u', '％': '%',
  'α': 'alpha', 'β': 'beta', 'γ': 'gamma', 'δ': 'delta', 'ε': 'eps', 'ζ': 'zeta',
  'η': 'eta', 'θ': 'theta', 'λ': 'lambda', 'ρ': 'rho', 'σ': 'sigma', 'τ': 'tau',
  'φ': 'phi', 'χ': 'chi', 'ψ': 'psi', 'ω': 'omega', 'Δ': 'D', 'Σ': 'Sum',
};
const dir = 'D:/WS_L_re/_cardgen/cards';
let total = 0;
for (const f of fs.readdirSync(dir).filter(f => f.endsWith('.json')).sort()) {
  const p = `${dir}/${f}`;
  const raw = fs.readFileSync(p, 'utf8');
  let out = raw, n = 0;
  for (const [bad, good] of Object.entries(MAP)) {
    const c = out.split(bad).length - 1;
    if (c) { out = out.split(bad).join(good); n += c; }
  }
  if (n) { fs.writeFileSync(p, out, 'utf8'); console.log(`${f}: 替换 ${n} 处`); total += n; }
}
console.log(total ? `共替换 ${total} 处` : '无需替换');
