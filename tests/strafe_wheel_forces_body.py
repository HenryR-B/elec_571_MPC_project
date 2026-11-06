"""The 30/45 5 A strafe right, 'built' vs 'proper': each wheel's floor force and the net force and yaw moment in the
robot's own frame, averaged over the first 50 ms (before the robot has turned much). Shows which wheel imbalance
pushes the robot backwards (-y_r) and turns it. Noise off.
Usage: python tests/strafe_wheel_forces_body.py RobotSimulationOpenLoopModel_HenryConstants.ipynb"""
import json, re, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
nb = json.load(open(sys.argv[1], encoding="utf-8"))
d = {c.get("id"): "".join(c["source"]) for c in nb["cells"]}
STR = np.array([-1, -1, 1, 1.0])
N_STEPS = 50
for design in ("built", "proper"):
    g = {}
    exec(d["ebf0a37d"], g)
    exec(re.sub(r"MECHANICAL_DESIGN = '\w+'", f"MECHANICAL_DESIGN = '{design}'", d["21f5c2d7"].split("# 2. Instantiate")[0]), g)
    p = dict(g["simulation_parameters"], sigma_V=0.0, sigma_mu=0.0, alpha_f=np.radians(30.0), alpha_b=np.radians(45.0))
    sim, s = g["OmniRobotSimulation"](p), g["OmniMotorState"]()
    F_sum = np.zeros(4)
    for k in range(N_STEPS):
        s, T_load, S = sim.step(s, 5.0 * STR)
        F_sum += T_load / p["r"]                  # floor force along each wheel's drive direction (incl. rolling resistance)
    F = F_sum / N_STEPS
    Fn = sim.compute_dynamic_normal_forces(s.a_local_past[:2], np.zeros(4))
    contrib = sim.H * F[:, None]                  # each wheel's [F_x_r, F_y_r, yaw moment about the CoG]
    print(f"{design}: first {N_STEPS} ms, heading at the end {np.degrees(s.pos_global[2]):+.1f} deg, slipping {S.astype(int)} (FR FL RL RR)")
    for i, nm in enumerate(("FR", "FL", "RL", "RR")):
        print(f"   {nm}: drive force {F[i]:+5.2f} N (load {Fn[i]:4.2f} N, mu_s*N {p['mu_s'] * Fn[i]:4.2f})"
              f" -> body x {contrib[i, 0]:+5.2f} N, body y {contrib[i, 1]:+5.2f} N, yaw {1000 * contrib[i, 2]:+6.1f} mN m")
    tot = contrib.sum(axis=0)
    print(f"   net: body x {tot[0]:+5.2f} N (sideways, commanded), body y {tot[1]:+5.2f} N (fore/aft), yaw moment {1000 * tot[2]:+6.1f} mN m\n")
