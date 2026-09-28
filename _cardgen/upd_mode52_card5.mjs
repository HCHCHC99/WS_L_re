// 一次性：mode 52 卡片同步"平台均值法"（结果算法）+ 双算法对照打印。
import fs from 'node:fs';

const P = 'D:/WS_L_re/_cardgen/cards/mode52.json';
const c = JSON.parse(fs.readFileSync(P, 'utf8'));

/* obs：清理旧项，补上新观测量 */
const drop = ['g_ldlq52_di_pp_a', 'g_ldlq52_phase_ratio', 'g_ldlq52_cons_pct', 'g_ldlq52_d_scatter_pct', 'g_ldlq52_dcnt'];
c.obs = c.obs.filter(o => !drop.includes(o.name));
const i = c.obs.findIndex(o => o.name === 'g_ldlq52_l_probe_uh');
const add = [
  { mk: '(*)', name: 'g_ldlq52_l_env_uh', desc: '平台均值法反推的电感 (uH) ---- 最终结果用这个（每平台取两点平均，相邻平台均值求差）' },
  { mk: '', name: 'g_ldlq52_l_pair_uh', desc: '单点差分法反推的电感 (uH) ---- 对照用；易被快速交替干扰污染（实测会偏小约一半）' },
  { mk: '(*)', name: 'g_ldlq52_de_ma', desc: '相邻平台均值之差 (mA) ---- 结果的原始量；L = Vsq x 平台时长 / 它' },
  { mk: '', name: 'g_ldlq52_dp_ma', desc: '平台内两点之差 (mA) ---- 单点差分法的原始量（对照）' },
  { mk: '(*)', name: 'g_ldlq52_de_cons_pct', desc: '平台均值差的符号一致性 (%) ---- 平台均值法的有效性判据' },
  { mk: '', name: 'g_ldlq52_cons_pct', desc: '单点差分法的一致性 (%) ---- 仅供对照' },
  { mk: '', name: 'g_ldlq52_dcnt', desc: '参与统计的平台数' },
];
c.obs.splice(i + 1, 0, ...add);

/* notes：补平台均值法的由来与两种算法的对照 */
c.notes.push({
  mk: '(*)',
  text: '结果算法 = 平台均值法：平台长度正好是 2 个采样周期，把平台内两点取平均（这一步把采样序列里约 ±0.5A 的快速交替干扰抵消掉 —— 相邻两点符号相反），再取相邻平台均值之差 |dE|，则 L = Vsq x 平台时长 / |dE|。2026-09-27 的抓取波形显示：单点差分法里"跨平台边界那一跳"比平台内斜坡大 ~2 倍（边界瞬态），取 max 的规则专门挑到它，导致 L 被系统性压掉一半（报 20uH，而平台均值法给出 34~37uH、与手册 42.3uH 同量级）。两种算法的结果都在 RTT 里直接打印（env / pair），方便对照。',
});
c.notes.push({
  mk: '[!]',
  text: 'RTT 打印的 probe 行含三行：第一行是偏置/纹波/实测 ISR 频率，第二行是一致性与偏置需求，第三行直接给出两种算法各自的电感：probe L: env=..uH (dE=..mA) pair=..uH (dp=..mA) fs=..Hz。结束时再打 done（Ld/Lq/比值/手册倍数）与过程量行。无需手工推算。',
});

fs.writeFileSync(P, JSON.stringify(c, null, 2) + '\n', 'utf8');
console.log(`  obs ${c.obs.length} 条；notes ${c.notes.length} 条`);
console.log('  OK');
