"""MECHANICAL_DESIGN 'built' (CAD CoG and height) vs 'proper' (CoG at the robot's centre, Dana's lower height)
on strafes and a forward run.
Noise off, 30/45 layout, 1.5 s. Usage: python tests/design_toggle_compare.py RobotSimulationOpenLoopModel_HenryConstants.ipynb"""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
nb = json.load(open(sys.argv[1], encoding="utf-8"))
d = {c.get("id"): "".join(c["source"]) for c in nb["cells"]}
FWD, STR = np.array([1, -1, -1, 1.0]), np.array([-1, -1, 1, 1.0])
SCEN = [("2 A strafe", 2.0 * STR), ("5 A strafe", 5.0 * STR), ("5 A forward", 5.0 * FWD)]
for design in ("built", "proper"):
    g = {}
    exec(d["ebf0a37d"], g)
    exec(d["21f5c2d7"].split("# 2. Instantiate")[0].replace("MECHANICAL_DESIGN = 'built'", f"MECHANICAL_DESIGN = '{design}'"), g)
    p = dict(g["simulation_parameters"], sigma_V=0.0, sigma_mu=0.0, alpha_f=np.radians(30.0), alpha_b=np.radians(45.0))
    for name, cmd in SCEN:
        sim, s = g["OmniRobotSimulation"](p), g["OmniMotorState"]()
        slip = np.zeros(4)
        for k in range(1500):
            s, _, S = sim.step(s, cmd); slip += S
        v = s.v_global
        print(f"{design:6} {name:11}: end ({s.pos_global[0]:+.2f}, {s.pos_global[1]:+.2f}) m, heading {np.degrees(s.pos_global[2]):+8.0f} deg, "
              f"yaw rate {v[2]:+6.1f} rad/s, speed {np.hypot(v[0], v[1]):.2f} m/s, time slipping per wheel {np.round(100 * slip / 1500)}%",
              flush=True)
