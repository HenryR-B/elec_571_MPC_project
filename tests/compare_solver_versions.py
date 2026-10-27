"""Compare two versions of the working notebook on Dana's manoeuvres (noise off): end speed, path, heading,
run time, and the newer version's solver steps. Usage: python tests/compare_solver_versions.py OLD.ipynb NEW.ipynb"""
import json, sys, time
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # robot_constants
def load(path):
    nb = json.load(open(path, encoding="utf-8")); d = {c.get("id"): "".join(c["source"]) for c in nb["cells"]}
    g = {}; exec(d["ebf0a37d"], g); exec(d["21f5c2d7"].split("# 2. Instantiate")[0], g)
    return g, dict(g["simulation_parameters"], sigma_V=0.0, sigma_mu=0.0, alpha_f=np.radians(30.0), alpha_b=np.radians(45.0))
go, po = load(sys.argv[1]); gn, pn = load(sys.argv[2])
FWD, STR, ROT = np.array([1, -1, -1, 1.0]), np.array([-1, -1, 1, 1.0]), np.ones(4)
SCEN = [("2 A forward", 1.5, lambda t: 2.0 * FWD), ("2 A strafe", 1.5, lambda t: 2.0 * STR), ("2 A rotation", 1.5, lambda t: 2.0 * ROT),
        ("5 A forward", 1.5, lambda t: 5.0 * FWD), ("4 A fwd / back / coast", 3.0, lambda t: 4.0 * FWD * (1 if t < 1 else (-1 if t < 2 else 0)))]
dt = po["dt"]
for name, T, fn in SCEN:
    n = int(round(T / dt)); out = {}
    for tag, g, p in (("old", go, po), ("new", gn, pn)):
        sim, s = g["OmniRobotSimulation"](p), g["OmniMotorState"](); t0 = time.time(); path = []; steps = []; ev = 0
        for k in range(n):
            s, _, _ = sim.step(s, fn(k * dt)); path.append(s.pos_global.copy())
            if tag == "new": steps.extend(sim.last_steps); ev += sim.last_nfev
        out[tag] = (np.array(path), s, time.time() - t0, np.array(steps), ev)
    po_, so, wo = out["old"][:3]; pn_, sn, wn, hs, ev = out["new"]
    dev = np.linalg.norm(pn_[:, :2] - po_[:, :2], axis=1).max()
    print(f"{name:24}: end speed new {np.hypot(*sn.v_local[:2]):.3f} vs old {np.hypot(*so.v_local[:2]):.3f} m/s, path difference "
          f"{1e3 * dev:6.1f} mm, heading {np.degrees(sn.pos_global[2]):+8.1f} vs {np.degrees(so.pos_global[2]):+8.1f} deg | wall new {wn:.1f} s "
          f"(old {wo:.1f} s), solver steps per 1 ms: median {np.median(np.bincount(np.minimum((np.cumsum(hs) / dt).astype(int), n - 1))):.0f}, "
          f"smallest step {1e6 * hs.min():.2f} us, {ev / n:.0f} evaluations per step", flush=True)
