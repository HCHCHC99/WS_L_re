// 去注释/字符串后统计括号平衡（无编译器时的最低限度语法自检）
import fs from 'node:fs';

const ROOT = 'D:/WS_L_re/ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2';
const files = [
  'ws/foc_20_cal.c', 'ws/foc_24_dcal.c', 'ws/foc_25_calangle.c',
  'ws/foc_20_cal.h', 'ws/foc_24_dcal.h', 'ws/foc_25_calangle.h',
  'ws/foc_core.c', 'ws/foc_core.h', 'ws/foc.c', 'ws/foc_obs.c', 'ws/dev_comm_runner.c',
  'ws/dev_comm_runner.h', 'ws/motor_config.h', 'template/source/main.c',
];
let bad = 0;
for (const f of files) {
  let s = fs.readFileSync(`${ROOT}/${f}`, 'utf8');
  s = s.replace(/\/\*[\s\S]*?\*\//g, ' ')
       .replace(/\/\/[^\n]*/g, ' ')
       .replace(/"(?:\\.|[^"\\])*"/g, '""')
       .replace(/'(?:\\.|[^'\\])*'/g, "''");
  const c = ch => s.split(ch).length - 1;
  const b = c('{') - c('}'), r = c('(') - c(')'), q = c('[') - c(']');
  const ok = b === 0 && r === 0 && q === 0;
  if (!ok) bad++;
  console.log(`  ${ok ? 'OK ' : '[X]'} ${f.padEnd(34)} {} ${b >= 0 ? '+' : ''}${b}   () ${r >= 0 ? '+' : ''}${r}   [] ${q >= 0 ? '+' : ''}${q}`);
}
console.log(bad ? `\n有 ${bad} 个文件括号不平衡` : '\n括号全部平衡');
