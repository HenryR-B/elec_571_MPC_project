"""Rotating-frame check on the working notebook's model: a robot coasting at 1 m/s forward while spinning at 3 rad/s,
with the floor friction and rolling resistance switched off (mu = 0, Crr = 0), so no force acts on it.
Correct physics: a straight line at 1 m/s (2 m in 2 s). Usage: python tests/frame_fix_check.py NOTEBOOK.ipynb"""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
nb = json.load(open(sys.argv[1], encoding="utf-8"))
d = {c.get("id"): "".join(c["source"]) for c in nb["cells"]}
g = {}
exec(d["ebf0a37d"], g); exec(d["21f5c2d7"].split("# 2. Instantiate")[0], g)
p = dict(g["simulation_parameters"], sigma_V=0.0, sigma_mu=0.0, mu_s=1e-12, mu_k=1e-12, Crr=0.0)
sim, s = g["OmniRobotSimulation"](p), g["OmniMotorState"]()
s.v_local = np.array([0.0, 1.0, 3.0]); s.v_global = s.v_local.copy()     # same at theta = 0
path = []
for k in range(int(2.0 / p["dt"])):
    s, _, _ = sim.step(s, np.zeros(4))
    path.append(s.pos_global[:2].copy())
path = np.array(path)
print(f"after 2 s: position {path[-1].round(3)} m, distance from start {np.linalg.norm(path[-1]):.3f} m "
      f"(straight line: 2.000 m), farthest sideways {np.abs(path[:, 0]).max() * 1e3:.1f} mm")
