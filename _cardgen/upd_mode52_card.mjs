// 一次性：把 mode 52 卡片数据更新为"双配对自选 + 运动/配对有效性判据"版本。
// 用脚本改而不是手改，因为 mode52.json 已被 fix_evidence 重排成每字段一行。
import fs from 'node:fs';

const P = 'D:/WS_L_re/_cardgen/cards/mode52.json';
const c = JSON.parse(fs.readFileSync(P, 'utf8'));

/* ---- obs：替换 di_pp 之后的段落，加入新诊断量与更新后的事件说明 ---- */
const head = c.obs.slice(0, c.obs.findIndex(o => o.name === 'g_ldlq52_di_pp_a'));
const tail = [
  { mk: '(*)', name: 'g_ldlq52_cons_pct', desc: '选中配对的符号一致性 (%) ---- 必须约 100，约 50 说明两次采样没落在同一平台' },
  { mk: '', name: 'g_ldlq52_pair_sel', desc: '自动选中的配对：0 = 组内，1 = 跨组（两种配对同时统计，取一致性高的）' },
  { mk: '(*)', name: 'g_ldlq52_path_cnts', desc: '测量窗内编码器路径累计 (counts) ---- 转子是否在动的真正判据' },
  { mk: '', name: 'g_ldlq52_di_pp_a', desc: '实测纹波峰峰值 (A) ---- 应贴近 di_target，否则说明幅值被上下限卡住' },
  { mk: '', name: 'g_ldlq52_v_inj_v', desc: '自适应后的方波幅值 Vsq (V)' },
  { mk: '', name: 'g_ldlq52_dcnt', desc: '参与统计的平台数（每平台一个 |di| 样本）' },
  { mk: '', name: 'g_ldlq52_state', desc: '0 IDLE / 1 零偏窗 / 2 探测 / 3 正式测量 / 4 间隔 / 5 OC' },
  { mk: '[!]', name: 'g_ldlq52_evt', desc: '1 完成 / 2 过流 / 3 纹波太小 / 4 偏置没建立 / 5 转子在动 / 6 采样配对异常（4/5/6 都不更新结果）' },
  { mk: '', name: 'g_ldlq52_axis', desc: '0 = d 轴，1 = q 轴（当前/最后测的轴）' },
  { mk: '', name: 'g_ldlq52_running', desc: '1 = 本轮有效（DONE 后仍为 1，便于回看；切模式时 Stop）' },
  { mk: '[!]', name: 'g_ldlq52_moved_d_cnts', desc: 'd 轴测量期间的净位移 (counts，已解回绕)' },
  { mk: '[!]', name: 'g_ldlq52_moved_q_cnts', desc: 'q 轴测量期间的净位移 (counts，已解回绕) ---- d 轴就不该动' },
  { mk: '', name: 'g_ldlq52_moved_cnts', desc: '全程净位移 (counts，已解回绕)' },
  { mk: '', name: 'g_ldlq52_elapsed_ms', desc: '累计耗时 (ms)' },
];
c.obs = [...head, ...tail];

/* ---- VOFA：ch9 由死区反推改成配对一致性（有效性指标更该在曲线上实时看） ---- */
const r9 = c.vofa.rows.find(r => r.ch === 'ch9');
r9.mk = '(*)';
r9.what = '配对一致性';
r9.unit = '%';
r9.desc = '必须约 100';

/* ---- 追加两条说明：配对自选机制 + 三个有效性判据 ---- */
c.notes.push({
  mk: '[!]',
  text: '采样配对相位是自选的，不用事先知道 ADC 触发与占空比装载的相位关系：平台内两次采样的差必然与当前平台极性同号，所以同时统计"组内配对"和"跨组配对"两套 |di|，取符号一致性高的那套（正确时约 100%，配错时约 50%），结果记在 cons_pct / pair_sel。若两套都低于 85%，置 evt = 6 (PAIR_FAIL) 且不更新结果。',
});
c.notes.push({
  mk: '[!]',
  text: '三条有效性判据（任一不过就不更新结果，只留过程量供排查）：1) cons_pct >= 85（采样配对正确）；2) |I_bias| > 纹波半幅（单极性成立）；3) 测量窗内编码器路径 path_cnts <= 64（转子基本不动）。另 path_cnts > 8 时追一行 warn。注意净位移 moved_cnts 会随 16 位计数器回绕（曾把 -206 打成 65330），所以判"转子是否在动"要看路径累计而不是净位移。',
});

fs.writeFileSync(P, JSON.stringify(c, null, 2) + '\n', 'utf8');
console.log(`  obs ${c.obs.length} 条；notes ${c.notes.length} 条；vofa ch9 = ${r9.what}`);
console.log('  OK');
