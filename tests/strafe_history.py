"""The 30/45 strafe through the model's history: Dana's original (her constants), her model with Henry's constants,
after the axis swap, and the current model. Noise off, 1.5 s. Usage: python tests/strafe_history.py LABEL=NOTEBOOK ..."""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
for arg in sys.argv[1:]:
    label, path = arg.split("=", 1)
    nb = json.load(open(path, encoding="utf-8"))
    d = {c.get("id"): "".join(c["source"]) for c in nb["cells"]}
    g = {}
    exec(d["ebf0a37d"], g); exec(d["21f5c2d7"].split("# 2. Instantiate")[0], g)
    p = dict(g["simulation_parameters"], sigma_V=0.0, sigma_mu=0.0, alpha_f=np.radians(30.0), alpha_b=np.radians(45.0))
    current_input = "I_bw" in p
    for mag in ((2.0, 5.0)):
        sim, s = g["OmniRobotSimulation"](p), g["OmniMotorState"]()
        slip = np.zeros(4); loads_min = np.full(4, np.inf)
        for k in range(int(round(1.5 / p["dt"]))):
            s, _, S = sim.step(s, mag * np.array([-1, -1, 1, 1.0])); slip += S
        unit = "A" if current_input else "V"
        n = int(round(1.5 / p["dt"]))
        print(f"{label:34} {mag:.0f} {unit} strafe: end ({s.pos_global[0]:+.2f}, {s.pos_global[1]:+.2f}) m, heading "
              f"{np.degrees(s.pos_global[2]):+8.0f} deg, yaw rate {s.v_local[2]:+6.1f} rad/s, time slipping per wheel "
              f"{np.round(100 * slip / n)}%", flush=True)
