// 验证"线性闸门"能否把跨过三角波顶点的采样对剔除掉。
// 场景：平台 100 µs（顶点在平台边界），采样每 50 µs 一次，但**配对相位 a 不确定**
//（翻转按 ISR 次数计，一旦某次写入错过影子装载，相位就永久错开一拍）。
// 模型：三角波 i(t)，顶点在 t = 0,100,200...；采样点 t = a, a+50, a+100...
// 闸门：相邻两段斜率之差 > FOC52_LIN_TOL_PCT% 就丢掉这一对。
const TS = 50e-6, PLAT = 100e-6, TOL = 0.25;

/* 理想三角波：平台内线性上升/下降，峰峰值 2A（A = Vsq*TS/L） */
function sample(a, Vsq, L, k) {
  const t = a + k * TS;
  const n = Math.floor(t / PLAT);            // 第几个平台
  const u = t - n * PLAT;                    // 平台内位置
  const A = Vsq * PLAT / 2 / L;              // 半幅
  const c = (n % 2 === 0) ? 1 : -1;          // 上升/下降
  return c * (2 * A * u / PLAT - A);         // 从 -A 到 +A
}

function estimate(a, Vsq, L, pairs = 200, useGate = true) {
  const A_ideal = Vsq * TS / L;              // 期望的 |di|
  let sum = 0, n = 0, rej = 0;
  const all = [];
  for (let k = 1; k <= pairs; k++) {
    const i2 = sample(a, Vsq, L, k), i1 = sample(a, Vsq, L, k - 1), i0 = sample(a, Vsq, L, k - 2);
    if (k === 1) continue;
    const d1 = i2 - i1, d0 = i1 - i0;
    const ok = !useGate || Math.abs(d1 - d0) <= TOL * Math.abs(d1);
    if (ok) { sum += Math.abs(d1); n++; } else { rej++; }
    all.push(Math.abs(d1));
  }
  const mean = sum / n;
  const meanAll = all.reduce((x, y) => x + y, 0) / all.length;
  return { Lgate: Vsq * TS / mean, Lall: Vsq * TS / meanAll, n, rej, A_ideal };
}

const Vsq = 0.57, Ltrue = 42.3e-6;
console.log('  相位 a(µs)   闸门后 L     偏差      未闸门 L    偏差    有效/丢弃');
for (let a = 0; a <= 50; a += 6.25) {
  const g = estimate(a * 1e-6, Vsq, Ltrue, 200, true);
  const u = estimate(a * 1e-6, Vsq, Ltrue, 200, false);
  const f = x => `${((x / Ltrue - 1) * 100).toFixed(1)}%`.padStart(7);
  console.log(`   ${a.toFixed(2).padStart(6)}   ${(g.Lgate * 1e6).toFixed(1).padStart(7)}uH${f(g.Lgate)} ` +
              `  ${(u.Lall * 1e6).toFixed(1).padStart(7)}uH${f(u.Lall)}   ${g.n}/${g.rej}`);
}

console.log('\n  统计：相位在 [0,50) 内均匀随机时，两种做法各自的均值/离散度（20 组）');
let ga = [], ua = [];
for (let t = 0; t < 20; t++) {
  const a = Math.random() * PLAT;
  ga.push(estimate(a, Vsq, Ltrue, 200, true).Lgate);
  ua.push(estimate(a, Vsq, Ltrue, 200, false).Lall);
}
const stat = arr => {
  const m = arr.reduce((x, y) => x + y, 0) / arr.length;
  const sd = Math.sqrt(arr.reduce((s, x) => s + (x - m) ** 2, 0) / arr.length);
  return `均值 ${(m * 1e6).toFixed(1)}uH (${((m / Ltrue - 1) * 100).toFixed(1)}%)  离散度 ${(sd / m * 100).toFixed(1)}%`;
};
console.log('   有闸门: ' + stat(ga));
console.log('   无闸门: ' + stat(ua));
