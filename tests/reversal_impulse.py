"""How big are the reversal force peaks? Duration above mu_s N, extra impulse, and the robot-speed error it causes."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
import wheel_bench as wb

for delta, zeta, fade in ((2e-5, 2.0, 5e-2), (3e-5, 3.0, 5e-2)):
    for N in (3.0, 7.75):
        wb.N = N
        out, ok = wb.run(delta, zeta, fade, lambda t: 15.0 if t < 0.15 else -15.0, 0.3)
        t, F = out[:, 0], out[:, 4]
        lim = wb.mu_s * N
        over = np.abs(F) > lim
        excess = np.sum(np.maximum(np.abs(F) - lim, 0.0)) * wb.T
        k = np.argmax(np.abs(F))
        print(f"delta {1e6 * delta:.0f} um zeta {zeta:g} fade {1e3 * fade:g} mm/s, N {N:5.2f}: peak {np.abs(F).max() / lim:.2f} mu_s N "
              f"at t = {1e3 * t[k]:.2f} ms (slip {1e3 * out[k, 5]:+.1f} mm/s); time above mu_s N {1e3 * over.sum() * wb.T:.2f} ms; "
              f"extra impulse {1e3 * excess:.2f} mN s -> robot-speed error at most {1e3 * excess / wb.m_c:.2f} mm/s")
