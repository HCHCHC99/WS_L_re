// 一次性：mode 52 卡片补上"偏置自抬 + 双配对一致性 + probe 行"这几项。
import fs from 'node:fs';

const P = 'D:/WS_L_re/_cardgen/cards/mode52.json';
const c = JSON.parse(fs.readFileSync(P, 'utf8'));

/* obs：在 cons_pct 之后插入 other/ vbias / probe_seq，替换掉原来的 pair_sel 描述位置 */
const i = c.obs.findIndex(o => o.name === 'g_ldlq52_cons_pct');
if (i < 0) throw new Error('找不到 g_ldlq52_cons_pct');
c.obs.splice(i + 1, 0,
  { mk: '(*)', name: 'g_ldlq52_cons_other_pct', desc: '另一套配对的一致性 (%) ---- 两者都低说明波形不干净（典型：电流过了零），不是选错配对' },
  { mk: '', name: 'g_ldlq52_vbias_v', desc: '当前指令偏置电压 (V) ---- 偏置不够时模式自己每次抬 0.1V 再探' },
  { mk: '', name: 'g_ldlq52_probe_seq', desc: '探测完成次数（每次变化 foc_obs 打一行 probe，含两套一致性）' });

/* notes：补一条偏置自适应的说明 */
c.notes.push({
  mk: '[!]',
  text: '偏置电流是自适应建立的，不依赖 R/V_dead 先验的准确性：每次探测后检查实测 |I_bias| 是否 >= 1.5 x 纹波半幅，不够就把偏置电压抬 FOC52_VBIAS_STEP_V(0.1V) 重探，直到够用或到 1.5V 上限。2026-09-27 两次上机就是因为实测偏置小于指令值（先验偏差）而导致电流过零、cons 只有 67.8% —— 这条环路就是为它加的。日志里每次探测都会打一行 probe#，直接看 bias / ripple/2 / cons / other 四个数。',
});

fs.writeFileSync(P, JSON.stringify(c, null, 2) + '\n', 'utf8');
console.log(`  obs ${c.obs.length} 条；notes ${c.notes.length} 条`);
console.log('  OK');
