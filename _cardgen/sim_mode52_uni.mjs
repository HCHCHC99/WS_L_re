// 单极性（直流偏置方波）方案仿真：让被测轴电流不过零 => 死区等效电压在两极性平台
// 上同号，做 |d+|+|d-| 时精确抵消，R 项也一阶抵消。
//   v_axis(t) = V_bias + sgn*Vsq        V_bias = R*I_bias + Vd（含死区补偿）
//   L = (2*Vsq - 2*R*I_bias) * TS / (|d+| + |d-|)     （两极性平台各取一个 |Δ| 的平均）
const TS = 1 / 20000, PLAT = 2 * TS, DT = 1e-8;

function simBias({ L, R, Vd, Vsq, Ibias, cycles = 200 }) {
  const Vb = R * Ibias + Vd;
  let i = 0; const plats = []; const n = Math.round(PLAT / DT);
  for (let c = 0; c < cycles; c++) {
    const sgn = (c % 2 === 0) ? 1 : -1;
    const pair = [];
    for (let k = 0; k < n; k++) {
      const t = k * DT;
      // 死区等效电压顶住电流；电流恒正 => 恒定 -Vd
      i += ((Vb + sgn * Vsq - Vd - R * i) / L) * DT;
      const tn = t + DT;
      if (Math.abs(tn - 25e-6) < DT / 2 || Math.abs(tn - 75e-6) < DT / 2) pair.push(i);
    }
    plats.push({ sgn, a: pair[0], b: pair[1] });
  }
  return plats.slice(-40);
}

/** 单极性估计式（用实测 R 修正 R*i 项；mid 取两采样均值） */
function estUni(P, Vsq, R) {
  let num = 0, den = 0;
  for (const p of P) {
    const d = Math.abs(p.b - p.a);
    const mid = (p.a + p.b) / 2;
    num += (Vsq - R * mid) * TS;         // 每个平台的伏秒（正负平台各一份）
    den += d;
  }
  return num / den;
}
/** 不修正 R 项的版本 */
function estUniNoR(P, Vsq) {
  let s = 0, n = 0;
  for (const p of P) { s += Math.abs(p.b - p.a); n++; }
  return 2 * Vsq * TS / (s / n * 2) / 2 * 2;   // = Vsq*TS/mean|d|
}

const R = 0.098, Vd = 0.16;
console.log('  真值L     Vsq    Ibias   |d+|    |d-|    零偏(mA)    L_est     偏差');
for (const L of [20e-6, 42.3e-6, 80e-6, 130e-6, 260e-6]) {
  const Vsq = Math.min(3.0, Math.max(0.3, L * 0.5 / TS));
  const Ibias = 0.45;                                  // 略高于纹波半幅
  const P = simBias({ L, R, Vd, Vsq, Ibias });
  const dp = P.filter(p => p.sgn > 0).map(p => Math.abs(p.b - p.a));
  const dn = P.filter(p => p.sgn < 0).map(p => Math.abs(p.b - p.a));
  const avg = a => a.reduce((x, y) => x + y, 0) / a.length;
  const all = P.flatMap(p => [p.a, p.b]);
  const dc = (all.reduce((x, y) => x + y, 0) / all.length) * 1000;
  const Le = estUni(P, Vsq, R);
  console.log(`  ${(L * 1e6).toFixed(1).padStart(6)}uH ${Vsq.toFixed(2)}V  ${Ibias.toFixed(2)}A  ` +
    `${(avg(dp) * 1000).toFixed(0).padStart(5)}mA ${(avg(dn) * 1000).toFixed(0).padStart(5)}mA  ` +
    `${dc.toFixed(0).padStart(7)}   ${(Le * 1e6).toFixed(1).padStart(6)}uH  ${(((Le / L) - 1) * 100).toFixed(1)}%`);
}

console.log('\n  对照：同样偏置下若不做直流偏置补偿（假设 Vd 未知、Vb 直接按 R*Ibias 给）');
console.log('  说明：Vb 给错 => 实际 Ibias 偏掉 => 只影响 R*mid 项，二阶小量。');
for (const L of [42.3e-6, 260e-6]) {
  const Vsq = Math.min(3.0, Math.max(0.3, L * 0.5 / TS));
  const P = simBias({ L, R, Vd, Vsq, Ibias: 0.45 });
  const Le = estUni(P, Vsq, R);
  const Le0 = estUni(P, Vsq, 0);              // 完全不修正 R
  console.log(`    L=${(L * 1e6).toFixed(1)}uH  修正R: ${(((Le / L) - 1) * 100).toFixed(1)}%   不修正R: ${(((Le0 / L) - 1) * 100).toFixed(1)}%`);
}
