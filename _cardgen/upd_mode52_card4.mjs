// 一次性：mode 52 卡片补上"合理性闸门 + 波形抓取"这两项（本次排查的关键工具）。
import fs from 'node:fs';

const P = 'D:/WS_L_re/_cardgen/cards/mode52.json';
const c = JSON.parse(fs.readFileSync(P, 'utf8'));

const i = c.obs.findIndex(o => o.name === 'g_ldlq52_probe_seq');
if (i < 0) throw new Error('找不到 probe_seq');
c.obs.splice(i + 1, 0,
  { mk: '', name: 'g_ldlq52_l_probe_uh', desc: '探测档纹波反推的电感 (uH) ---- 落在 [3, 3000] 之外判 evt=7 并停止抬偏置' },
  { mk: '', name: 'g_ldlq52_wave', desc: '波形抓取缓冲 (mA, 64 点)：正式测量窗开头自动抓一窗，RTT 打印 + Keil Watch 可看' },
  { mk: '', name: 'g_ldlq52_wave_sgn', desc: '抓取窗内每拍的平台极性 (+1/-1)，用于对齐电压与电流' },
  { mk: '', name: 'g_ldlq52_wave_arm', desc: '写 1 请求抓取（记满自动清 0）；一般不用手写，正式测量会自动抓' });

const n = c.notes.length;
c.notes.push({
  mk: '[!]',
  text: 'evt=7 (IMPLAUSIBLE)：探测档纹波反推的电感不在 [3, 3000] uH 就立刻停手、不再抬偏置电压。2026-09-27 实测教训：波形异常时"纹波半幅"被高估到 1.4A（应为 0.59A），偏置自适应就一路把偏置电压抬到上限、d 轴电流冲到 3A。看到 evt=7 请先看打印出来的波形串（下面那条），确认采样结构再谈标定。',
});
c.notes.push({
  mk: '(*)',
  text: '波形抓取（排查采样结构用）：正式测量窗开头自动抓 64 拍被测轴电流与平台极性，RTT 打出 8 个数一行 + 一行 +/- 极性串。重复采样（相邻数相同）、相位错拍（极性串与电流拐点错位）、开关纹波、直流瞬态在这两行上一眼可见 —— 比反复推断"波形应该是什么样"可靠得多。',

});
fs.writeFileSync(P, JSON.stringify(c, null, 2) + '\n', 'utf8');
console.log(`  obs ${c.obs.length} 条；notes ${c.notes.length} 条（+${c.notes.length - n}）`);
console.log('  OK');
