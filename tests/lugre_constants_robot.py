"""LuGre constants in the full (implicit) model: the current set (2 um, zeta 1, fade 5 mm/s) vs the tested softer set
(30 um, zeta 3, fade 50 mm/s), on Dana's manoeuvres and a slow current ramp. Noise off, 30/45 layout.
Usage: python tests/lugre_constants_robot.py RobotSimulationOpenLoopModel_HenryConstants.ipynb"""
import json, sys, time
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # robot_constants

nb = json.load(open(sys.argv[1], encoding="utf-8"))
d = {c.get("id"): "".join(c["source"]) for c in nb["cells"]}
g = {}
exec(d["ebf0a37d"], g)
exec(d["21f5c2d7"].split("# 2. Instantiate")[0], g)
base = dict(g["simulation_parameters"], sigma_V=0.0, sigma_mu=0.0, alpha_f=np.radians(30.0), alpha_b=np.radians(45.0))
SETS = {"2 um / zeta 1 / fade 5 mm/s": dict(lugre_deflection=2e-6, lugre_damping_ratio=1.0, lugre_damping_slip=5e-3),
        "30 um / zeta 3 / fade 50 mm/s": dict(lugre_deflection=3e-5, lugre_damping_ratio=3.0, lugre_damping_slip=5e-2)}
FWD, STR = np.array([1, -1, -1, 1.0]), np.array([-1, -1, 1, 1.0])
SCEN = [("2 A forward (grips)", 1.5, lambda t: 2.0 * FWD),
        ("2 A strafe", 1.5, lambda t: 2.0 * STR),
        ("5 A forward (front slides)", 1.5, lambda t: 5.0 * FWD),
        ("4 A fwd / back / coast", 3.0, lambda t: 4.0 * FWD * (1 if t < 1 else (-1 if t < 2 else 0))),
        ("ramp 0 -> 8 A forward", 1.5, lambda t: 8.0 * t / 1.5 * FWD)]
dt = base["dt"]
for name, T, fn in SCEN:
    n = int(round(T / dt))
    res = {}
    for label, extra in SETS.items():
        sim, s = g["OmniRobotSimulation"](dict(base, **extra)), g["OmniMotorState"]()
        t0, path, ev, first_slip, slip_steps = time.time(), [], 0, [None] * 4, np.zeros(4)
        for k in range(n):
            s, _, S = sim.step(s, fn(k * dt))
            path.append(s.pos_global.copy()); ev += sim.last_nfev
            slip_steps += S
            for w in range(4):
                if first_slip[w] is None and S[w]:
                    first_slip[w] = (k + 1) * dt
        res[label] = (np.array(path), s, time.time() - t0, ev / n, first_slip, slip_steps / n)
    (p0, s0, w0, e0, f0, sl0), (p1, s1, w1, e1, f1, sl1) = res.values()
    dev = np.linalg.norm(p1[:, :2] - p0[:, :2], axis=1).max()
    print(f"--- {name}: path difference {1e3 * dev:.1f} mm, heading {np.degrees(s1.pos_global[2]):+.1f} vs {np.degrees(s0.pos_global[2]):+.1f} deg")
    for label, (p, s, wall, evs, fs, sl) in res.items():
        fs_txt = ", ".join("-" if f is None else (f"{f:.3f} s" + (f" ({8.0 * f / 1.5:.2f} A)" if name.startswith("ramp") else "")) for f in fs)
        print(f"   {label:30}: end speed {np.hypot(*s.v_local[:2]):.3f} m/s | first slip per wheel [{fs_txt}] | "
              f"time slipping {np.round(100 * sl)}% | {evs:.0f} evaluations per step, {wall:.1f} s wall", flush=True)
