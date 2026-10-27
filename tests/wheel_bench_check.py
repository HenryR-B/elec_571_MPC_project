"""Candidates from wheel_bench.py: 50 us fixed step vs a 1 us reference, across wheel loads."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
import wheel_bench as wb

CANDS = [(2e-5, 2.0, 5e-2), (3e-5, 3.0, 5e-2), (2e-5, 3.0, 5e-2), (3e-5, 2.0, 1e-1)]
for delta, zeta, fade in CANDS:
    print(f"=== delta {1e6 * delta:.0f} um, zeta {zeta:g}, damping fades above {1e3 * fade:g} mm/s")
    for N in (3.0, 5.48, 7.75, 11.0):
        wb.N = N
        wb.I_BREAK = wb.mu_s * N * wb.r * (wb.m_c + wb.m_rim) / (wb.Kt * wb.m_c)
        rows = []
        for sub, label in ((1, "50 us"), (50, "1 us ref")):
            res = wb.summarize(delta, zeta, fade, sub)
            if res is None:
                rows.append(f"{label}: UNSTABLE"); continue
            rows.append(f"{label}: grips {str(res['grip']):5} ramp {res['ramp_peak']:.3f} slides@107% {str(res['slides_107']):5} "
                        f"(peak {res['step_peak_107']:.3f}) slide err {100 * res['slide_err']:.2f}% peak F {res['slide_peak']:.2f}")
            if sub == 1:
                traj50 = res["traj"]
            else:
                dv = np.abs(traj50[:, 3] - res["traj"][:, 3]).max()
                rows.append(f"robot-speed difference 50 us vs ref {1e3 * dv:.1f} mm/s")
        print(f"   N {N:5.2f} N (breakaway {wb.I_BREAK:5.2f} A) | " + " | ".join(rows), flush=True)
