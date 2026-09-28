// 一次性：mode 52 卡片同步"平台取 max"规则（去掉候选配对/线性闸门，加相位比）。
import fs from 'node:fs';

const P = 'D:/WS_L_re/_cardgen/cards/mode52.json';
const c = JSON.parse(fs.readFileSync(P, 'utf8'));

/* obs：删掉 cons_other/lin_ok/pair_sel，补 phase_ratio */
c.obs = c.obs.filter(o => !['g_ldlq52_cons_other_pct', 'g_ldlq52_lin_ok_pct', 'g_ldlq52_pair_sel'].includes(o.name));
const i = c.obs.findIndex(o => o.name === 'g_ldlq52_cons_pct');
if (i < 0) throw new Error('找不到 cons_pct');
c.obs[i].desc = '采用样本的符号一致性 (%) ---- 应 ~100，低于 85 判 evt=6 不更新结果';
c.obs.splice(i + 1, 0, {
  mk: '(*)',
  name: 'g_ldlq52_phase_ratio',
  desc: '平台内 max/min 之比 (x100) ---- 100 = 采样相位与平台锁定；>200 说明错开一拍（结果仍正确，是诊断量）',
});

/* notes：把"配对自选/线性闸门"那条改写成"平台取 max"规则 */
const oldIdx = c.notes.findIndex(n => n.text && n.text.includes('采样配对相位是自选的'));
if (oldIdx < 0) throw new Error('找不到旧的配对说明');
c.notes[oldIdx] = {
  mk: '[!]',
  text: '相位免疫的关键规则：平台长度正好是 2 个采样周期 => 每个平台内有两对相邻采样；翻转是按 ISR 次数计的，一旦某次写入错过峰点影子装载，采样相位就永久错开一拍，此时两对里必有一对跨过三角波顶点、其 |di| 被抵消一部分（实测均值只剩 0.72 倍、离散度 52%）。干净的那一对必然是两者中较大的那个，所以每个平台取 max(|di|) 即可，与相位无关（10ns 步长仿真：任意相位下误差 0.00%、离散度 0.00%；对照方案"相邻两对斜率差做闸门"会把顶点两侧的对全部误杀、有效样本归零，已弃用）。phase_ratio 顺带给出相位诊断。',
};

fs.writeFileSync(P, JSON.stringify(c, null, 2) + '\n', 'utf8');
console.log(`  obs ${c.obs.length} 条；notes ${c.notes.length} 条`);
console.log('  OK');
