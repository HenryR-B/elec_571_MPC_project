"""Why does the 5 A strafe on the 30/45 layout drift towards -y? Decompose the world-frame force by wheel over time.
Noise off. Usage: python tests/strafe_5a_forces.py RobotSimulationOpenLoopModel_HenryConstants.ipynb"""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
nb = json.load(open(sys.argv[1], encoding="utf-8"))
d = {c.get("id"): "".join(c["source"]) for c in nb["cells"]}
g = {}
exec(d["ebf0a37d"], g); exec(d["21f5c2d7"].split("# 2. Instantiate")[0], g)
p = dict(g["simulation_parameters"], sigma_V=0.0, sigma_mu=0.0, alpha_f=np.radians(30.0), alpha_b=np.radians(45.0))
sim, s = g["OmniRobotSimulation"](p), g["OmniMotorState"]()
cmd = 5.0 * np.array([-1, -1, 1, 1.0])
dt, n = p["dt"], 1500
imp_wheel = np.zeros((4, 2))                       # world-frame impulse from each wheel (N s)
rows = []
for k in range(n):
    th = s.pos_global[2]
    s, T_load, S = sim.step(s, cmd)
    F = (T_load - p["Crr"] * 0) / p["r"]          # approximate: step-average floor force incl. rolling torque / r
    # exact step-average floor force: rebuild from the solver integral is not exposed, so use T_load minus rolling resistance
    th_mid = 0.5 * (th + s.pos_global[2])
    c, sn = np.cos(th_mid), np.sin(th_mid)
    d_body = sim.H[:, :2]                          # drive directions in body axes
    d_world = np.column_stack((c * d_body[:, 0] - sn * d_body[:, 1], sn * d_body[:, 0] + c * d_body[:, 1]))
    imp_wheel += (F[:, None] * d_world) * dt
    if (k + 1) % 150 == 0:
        rows.append((dt * (k + 1), s.pos_global.copy(), s.v_global.copy(), imp_wheel.copy(), S.copy(), s.omega.copy()))
for t, pos, vg, imp, S, w in rows:
    print(f"t {t:.2f} s: pos ({pos[0]:+.3f}, {pos[1]:+.3f}) m, heading {np.degrees(pos[2]):+8.1f} deg, world velocity ({vg[0]:+.3f}, {vg[1]:+.3f}) m/s, "
          f"yaw rate {vg[2]:+6.1f} rad/s | slipping {S.astype(int)} | world impulse per wheel x/y (N s): "
          + " ".join(f"[{imp[i, 0]:+.2f},{imp[i, 1]:+.2f}]" for i in range(4)))
print(f"total world impulse / mass = {imp_wheel.sum(axis=0) / p['Mr']} m/s (compare with the world velocity above)")
