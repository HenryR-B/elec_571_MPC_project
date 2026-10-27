"""5 A strafe on the 30/45 layout: the normal model vs no slipping (mu = 5) vs no load transfer (CoG height ~ 0).
Noise off. Usage: python tests/strafe_5a_causes.py RobotSimulationOpenLoopModel_HenryConstants.ipynb"""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
nb = json.load(open(sys.argv[1], encoding="utf-8"))
d = {c.get("id"): "".join(c["source"]) for c in nb["cells"]}
g = {}
exec(d["ebf0a37d"], g); exec(d["21f5c2d7"].split("# 2. Instantiate")[0], g)
base = dict(g["simulation_parameters"], sigma_V=0.0, sigma_mu=0.0, alpha_f=np.radians(30.0), alpha_b=np.radians(45.0))
cmd = 5.0 * np.array([-1, -1, 1, 1.0])
for label, extra in (("normal model", {}), ("no slipping (mu_s = mu_k = 5)", dict(mu_s=5.0, mu_k=5.0)),
                     ("no load transfer (h = 1 mm)", dict(h=1e-3))):
    sim, s = g["OmniRobotSimulation"](dict(base, **extra)), g["OmniMotorState"]()
    slip = np.zeros(4); pts = []
    for k in range(1500):
        s, _, S = sim.step(s, cmd); slip += S
        if (k + 1) % 300 == 0:
            pts.append(f"({s.pos_global[0]:+.2f},{s.pos_global[1]:+.2f})")
    v = s.v_global
    print(f"{label:32}: path every 0.3 s {' '.join(pts)} m | end heading {np.degrees(s.pos_global[2]):+7.0f} deg, "
          f"yaw rate {v[2]:+6.1f} rad/s, travel direction {np.degrees(np.arctan2(v[1], v[0])):+5.0f} deg | "
          f"time slipping {np.round(100 * slip / 1500)}%", flush=True)
