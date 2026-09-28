// mode 52 波形仿真（修正版）：稳态取窗，量化"采样展布 -> L"的系统偏差。
// 模型与 foc_52_ldlq.c 一致：平台 100 µs；采样在平台内 25/75 µs；
// di/dt = (sgn*V - sign(i)*Vd - R*i)/L。输出 L_simple = V*(1/fs)/di 相对真值的偏差。
const TS = 1 / 20000, PLAT = 2 * TS, DT = 1e-8;

function spread({ L, R, Vd, V, cycles = 300 }) {
  let i = 0; const s = []; const n = Math.round(PLAT / DT);
  for (let c = 0; c < cycles; c++) {
    const sgn = (c % 2 === 0) ? 1 : -1;
    for (let k = 0; k < n; k++) {
      const t = k * DT;
      i += ((sgn * V - (i > 0 ? Vd : (i < 0 ? -Vd : 0)) - R * i) / L) * DT;
      const tn = t + DT;
      if (Math.abs(tn - 25e-6) < DT / 2 || Math.abs(tn - 75e-6) < DT / 2) s.push(i);
    }
  }
  const tail = s.slice(-120);                       // 稳态：最后 60 个平台
  return Math.max(...tail) - Math.min(...tail);
}

const R = 0.098, Vd = 0.16;
console.log('  [1] 偏差 vs 幅值（L=42.3 uH，R=0.098，Vd=0.16）—— 看是否能靠抬幅值压掉 R 偏差');
for (const V of [0.30, 0.42, 0.60, 0.85, 1.20, 1.70, 2.40]) {
  const L = 42.3e-6, di = spread({ L, R, Vd, V });
  const Ls = V * TS / di;
  console.log(`    V=${V.toFixed(2)}V  展布=${(di * 1000).toFixed(0)}mA  峰值=${(di / 2 * 1000).toFixed(0)}mA  L_simple=${(Ls * 1e6).toFixed(1)}uH  偏差=${((Ls / L - 1) * 100).toFixed(1)}%`);
}

console.log('\n  [2] 已知真值时：偏差主要由 tau=L/R 决定（V 取各自的自适应值）');
for (const L of [20e-6, 42.3e-6, 80e-6, 130e-6, 260e-6]) {
  const V = Math.min(3.0, Math.max(0.3, (L * 0.5 / TS)));   // 目标展布 0.5A（真实峰峰值 1A）
  const di = spread({ L, R, Vd, V });
  const Ls = V * TS / di;
  console.log(`    L=${(L * 1e6).toFixed(1)}uH (tau=${(L / R * 1e3).toFixed(2)}ms)  V=${V.toFixed(2)}V  L_simple=${(Ls * 1e6).toFixed(1)}uH  偏差=${((Ls / L - 1) * 100).toFixed(1)}%  平台/tau=${(PLAT / (L / R) * 100).toFixed(1)}%`);
}

console.log('\n  [3] 一阶修正候选：除以 (1 - k*(平台/2)/tau)，k 待定；先看 k=1 的效果');
for (const L of [20e-6, 42.3e-6, 80e-6, 130e-6, 260e-6]) {
  const V = Math.min(3.0, Math.max(0.3, (L * 0.5 / TS)));
  const di = spread({ L, R, Vd, V });
  const Ls = V * TS / di;
  const tau = L / R;
  const corr = Ls / (1 - (TS / tau));               // 用采样间隔 TS=50us 作特征时间
  console.log(`    L=${(L * 1e6).toFixed(1)}uH  修正后=${(corr * 1e6).toFixed(1)}uH  残差=${((corr / L - 1) * 100).toFixed(1)}%`);
}
