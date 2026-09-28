// 用仿真波形对比"采样 -> L"的几种估计式，挑残差最小的那个。
// 仿真与 foc_52_ldlq.c 一致：平台 100 µs、采样在平台内 25/75 µs、
// di/dt = (sgn*V - sign(i)*Vd - R*i)/L（死区等效电压始终顶住电流）。
const TS = 1 / 20000, PLAT = 2 * TS, DT = 1e-8, DT_S = 50e-6;

/** 跑出每平台的两个采样值（带平台极性） */
function sim({ L, R, Vd, V, cycles = 300 }) {
  let i = 0; const plats = []; const n = Math.round(PLAT / DT);
  for (let c = 0; c < cycles; c++) {
    const sgn = (c % 2 === 0) ? 1 : -1;
    const pair = [];
    for (let k = 0; k < n; k++) {
      const t = k * DT;
      i += ((sgn * V - (i > 0 ? Vd : (i < 0 ? -Vd : 0)) - R * i) / L) * DT;
      const tn = t + DT;
      if (Math.abs(tn - 25e-6) < DT / 2 || Math.abs(tn - 75e-6) < DT / 2) pair.push(i);
    }
    plats.push({ sgn, a: pair[0], b: pair[1] });
  }
  return plats.slice(-60);                     // 稳态 60 个平台
}

/* ---------- 候选估计式 ---------- */
const est = {
  // E1 现行：全局展布（= 真实峰峰值/2）
  E1: (P, V) => {
    const all = P.flatMap(p => [p.a, p.b]);
    return V * TS / (Math.max(...all) - Math.min(...all));
  },
  // E2 平台内 |Δ| 在两极性上的平均
  E2: (P, V) => {
    const d = P.map(p => Math.abs(p.b - p.a));
    return V * TS / (d.reduce((x, y) => x + y, 0) / d.length);
  },
  // E3 E1 + 死区电压修正
  E3: (P, V, { Vd }) => {
    const all = P.flatMap(p => [p.a, p.b]);
    return (V - Vd) * TS / (Math.max(...all) - Math.min(...all));
  },
  // E4 平台内 |Δ| + 死区 + R*i 修正（用两采样均值当窗内平均电流）
  E4: (P, V, { Vd, R, plus }) => {
    const d = P.map(p => ({ sgn: p.sgn, d: p.b - p.a, mid: (p.a + p.b) / 2 }));
    const use = d.filter(x => (plus ? x.sgn > 0 : x.sgn < 0));
    const num = use.reduce((s, x) => s + (V - Vd - R * (plus ? x.mid : -x.mid)) * TS, 0);
    const den = use.reduce((s, x) => s + Math.abs(x.d), 0);
    return num / den;
  },
  // E5 E1 的"双幅值拟合死区"版：用 V 和 V/ratio 两次测量的展布解 Vd 再修正
  E5: (P, V, { Vd, R }) => {
    const all = P.flatMap(p => [p.a, p.b]);
    const spread = Math.max(...all) - Math.min(...all);
    return V * TS / spread;                     // 仅给单点值，Vd 由外部两档解（这里占位）
  },
  // E6 单极性（加直流偏置使电流不过零）：L = 2*Vsq*TS/(|d+|+|d-|)
  E6: (P, V) => {
    let s = 0, n = 0;
    for (const p of P) { s += Math.abs(p.b - p.a); n++; }
    return 2 * V * TS / (s / n * 2);            // 两极性平均后 *2
  },
};

const cases = [];
for (const L of [20e-6, 42.3e-6, 80e-6, 130e-6, 260e-6]) {
  const V = Math.min(3.0, Math.max(0.3, L * 0.5 / TS));   // 自适应到展布 0.5A
  cases.push({ L, V });
}
const R = 0.098, Vd = 0.16;

console.log('  真值L     V       E1(现行)   E2       E3(+Vd)   E4(+R,i)    (E4 分极性)');
for (const { L, V } of cases) {
  const P = sim({ L, R, Vd, V });
  const e1 = est.E1(P, V), e2 = est.E2(P, V), e3 = est.E3(P, V, { Vd, R });
  const e4p = est.E4(P, V, { Vd, R, plus: true }), e4n = est.E4(P, V, { Vd, R, plus: false });
  const e4 = (e4p + e4n) / 2;
  const f = x => `${((x / L - 1) * 100).toFixed(1)}%`.padStart(7);
  console.log(`  ${(L * 1e6).toFixed(1).padStart(6)}uH ${V.toFixed(2)}V  ` +
    `${(e1 * 1e6).toFixed(1).padStart(7)}uH${f(e1)} ${(e2 * 1e6).toFixed(1).padStart(6)}uH${f(e2)} ` +
    `${(e3 * 1e6).toFixed(1).padStart(6)}uH${f(e3)} ${(e4 * 1e6).toFixed(1).padStart(6)}uH${f(e4)}` +
    `  (${(e4p * 1e6).toFixed(1)}/${(e4n * 1e6).toFixed(1)})`);
}

console.log('\n  补充：若 R 未知（用 R=0）时 E4 的残差——决定要不要依赖 mode 51 的 R');
for (const { L, V } of cases) {
  const P = sim({ L, R, Vd, V });
  const a = est.E4(P, V, { Vd, R, plus: true }), b = est.E4(P, V, { Vd, R, plus: false });
  const a0 = est.E4(P, V, { Vd, R: 0, plus: true }), b0 = est.E4(P, V, { Vd, R: 0, plus: false });
  console.log(`    L=${(L * 1e6).toFixed(1)}uH  用R: ${(((a + b) / 2 / L - 1) * 100).toFixed(1)}%   R=0: ${(((a0 + b0) / 2 / L - 1) * 100).toFixed(1)}%`);
}

console.log('\n  补充：E3 对 Vd 先验误差的敏感度（Vd 真值 0.16，先验给 0 / 0.10 / 0.20）');
for (const { L, V } of cases) {
  const P = sim({ L, R, Vd, V });
  const g = v => (((est.E3(P, V, { Vd: v }) / L) - 1) * 100).toFixed(1) + '%';
  console.log(`    L=${(L * 1e6).toFixed(1)}uH V=${V.toFixed(2)}V   Vd=0:${g(0).padStart(7)}  Vd=0.10:${g(0.10).padStart(7)}  Vd=0.20:${g(0.20).padStart(7)}`);
}
