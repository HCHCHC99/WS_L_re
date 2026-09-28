// 验证"平台均值法"(env) 与"单点差分法"(pair) 到底测得什么：按 foc_52_ldlq.c 的真实几何
//   - 方波极性每 FOC52_HALF_ISRS = 2 个采样周期翻转一次 => 平台 = 100 µs
//   - 每个平台内恰有 2 个采样点，两点间隔 T_s = 50 µs（成对 => 组的边界与平台边界对齐）
//   - 采样点相对平台起点的位置可调（真实硬件：谷点采样 + 峰点装载 => 存在固定相位差）
//   - 直流偏置使电流恒正 => 死区等效电压恒为 -Vd（两极性同号）
// 物理：di/dt = (V_bias + sgn*Vsq - Vd - R*i)/L
const TS = 1 / 20000, PLAT = 2 * TS, DT = 1e-8, N = Math.round(PLAT / DT);
const R = 0.098, Vd = 0.16, L = 42.3e-6, Vsq = 0.391, Ibias = 0.9;
const Vb = R * Ibias + Vd;                 // 命令偏置电压（= 0.238 V）

/** 跑稳态，返回每个平台的两个采样值 + 平台极性 */
function sim(phi_us, cycles = 200) {
  let i = 0; const plats = [];
  const t1 = (25 + phi_us) * 1e-6, t2 = (75 + phi_us) * 1e-6;   // 平台内两个采样时刻
  for (let c = 0; c < cycles; c++) {
    const sgn = (c % 2 === 0) ? 1 : -1;
    const pair = [];
    for (let k = 0; k < N; k++) {
      const t = k * DT;
      i += ((Vb + sgn * Vsq - Vd - R * i) / L) * DT;
      const tn = t + DT;
      if ((t1 >= 0 && Math.abs(tn - t1) < DT / 2) || (t2 <= PLAT && Math.abs(tn - t2) < DT / 2)) pair.push(i);
    }
    if (pair.length === 2) plats.push({ sgn, a: pair[0], b: pair[1] });
  }
  return plats.slice(-60);
}

const mean = a => a.reduce((x, y) => x + y, 0) / a.length;
const Lenv = (P) => {                       // env：相邻平台均值之差
  const m = P.map(p => (p.a + p.b) / 2), d = [];
  for (let k = 1; k < m.length; k++) d.push(Math.abs(m[k] - m[k - 1]));
  return Vsq * PLAT / mean(d);
};
const Lpair = (P) => {                      // pair：平台内两点之差
  return Vsq * TS / mean(P.map(p => Math.abs(p.b - p.a)));
};
const pp = (P) => { const all = P.flatMap(p => [p.a, p.b]); return Math.max(...all) - Math.min(...all); };

console.log(`  L真值 = ${(L * 1e6).toFixed(1)}uH  Vsq = ${Vsq}V  V_bias = ${Vb.toFixed(3)}V  R = ${R}  Vd = ${Vd}`);
console.log(`  理论：平台内斜坡 = Vsq*PLAT/L = ${(Vsq * PLAT / L * 1000).toFixed(0)}mA，一对之差 = Vsq*T_s/L = ${(Vsq * TS / L * 1000).toFixed(0)}mA\n`);
console.log('  相位φ    |\u0394P|    |\u0394E|   峰峰     L_pair     L_env    env/真值  pair/真值   周期均值漂移');
for (const phi of [0, 5, 10, 15, 20, 25, -5, -10, -15, -20, -25]) {
  const P = sim(phi);
  const dp = mean(P.map(p => Math.abs(p.b - p.a))) * 1000;
  const m = P.map(p => (p.a + p.b) / 2), de = mean(m.slice(1).map((v, k) => Math.abs(v - m[k]))) * 1000;
  const le = Lenv(P) * 1e6, lp = Lpair(P) * 1e6;
  const drift = (m[m.length - 1] - m[0]) * 1000;
  console.log(`  ${String(phi).padStart(4)}us ${dp.toFixed(0).padStart(6)}mA ${de.toFixed(0).padStart(6)}mA ` +
    `${(pp(P) * 1000).toFixed(0).padStart(5)}mA ${lp.toFixed(1).padStart(7)}uH ${le.toFixed(1).padStart(7)}uH ` +
    `${(le / (L * 1e6)).toFixed(3).padStart(8)} ${(lp / (L * 1e6)).toFixed(3).padStart(8)} ${drift.toFixed(1).padStart(9)}mA`);
}
