// 一次性：mode 52 卡片同步"精简打印 + 波形开关 + 重测说明"。
import fs from 'node:fs';

const P = 'D:/WS_L_re/_cardgen/cards/mode52.json';
const c = JSON.parse(fs.readFileSync(P, 'utf8'));

/* watch：加波形开关 */
if (!c.watch.some(w => w.name === 'g_ldlq52_wave_en')) {
  const i = c.watch.findIndex(w => w.name === 'g_ldlq52_r_used_ohm');
  c.watch.splice(i + 1, 0, {
    mk: '', name: 'g_ldlq52_wave_en', val: '0', unit: '-',
    desc: '1 = 抓取并打印原始波形（9 行，排查用）；默认 0，日志保持精简',
  });
}

/* notes：替换旧的 RTT 说明，补重测说明 */
const idx = c.notes.findIndex(n => n.text && n.text.includes('RTT 输出（开关 FOC_LDLQ52_DBG'));
if (idx >= 0) {
  c.notes[idx] = {
    mk: '(*)',
    text: 'RTT 输出（开关 FOC_LDLQ52_DBG）：每次成功跑完共 6~9 行 —— start（含 do_q，确认本轮会不会测 q）；每个轴的 probe 两行（配置/偏置 + cons/path/env/pair）；d done 一行（do_q=0/1 都会打，d 轴结果位置固定）；done 一行（Ld/Lq/Lq/Ld/vs_vendor/vdead）；last axis 一行；moved 一行。失败时只多打一条原因行（OC / bias not established / rotor moving / pair mismatch / implausible / no ripple）。原始波形默认不打印，要看就把 g_ldlq52_wave_en 写 1。',
  };
}
if (!c.notes.some(n => n.text && n.text.includes('重测'))) {
  c.notes.push({
    mk: '[!]',
    text: '重测：跑完后固件调用 CommRunner_ReleaseMode() 把模式号放回 STOP（Keil 里 comm_mode 会自动回 0），因此**直接再写一次 52 就能重测**，不必手动先写 0。原因：CommRunner_SetMode() 开头有"同模式早退"，若模式号停在 52，再写 52 会被吃掉 —— 这就是以前"有时能重测、有时不能"的根因。',
  });
}

fs.writeFileSync(P, JSON.stringify(c, null, 2) + '\n', 'utf8');
console.log(`  watch ${c.watch.length} 条；notes ${c.notes.length} 条`);
console.log('  OK');
