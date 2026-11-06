"""Choosing the LuGre bristle deflection delta (2 / 30 / 50 / 80 um) in the voltage-input notebook model: run time and
solves per step, how far the robot's path moves from the 2 um result, the voltage at which a wheel first breaks loose on a
slow ramp, and how much a gripping wheel creeps. Only delta changes (the bristle damping scales with it). Noise off.
Usage: python tests/lugre_stiffness_choice.py RobotSimulationOpenLoopModel_HenryConstants.ipynb"""
import json, sys, time
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
nb = json.load(open(sys.argv[1], encoding="utf-8"))
d = {c.get("id"): "".join(c["source"]) for c in nb["cells"]}
g = {}
exec(d["ebf0a37d"], g)
exec(d["21f5c2d7"].split("# 2. Instantiate")[0], g)
base = dict(g["simulation_parameters"], sigma_V=0.0, sigma_mu=0.0)
FWD, STR = np.array([1, -1, -1, 1.0]), np.array([-1, -1, 1, 1.0])
A30 = dict(alpha_f=np.radians(30.0), alpha_b=np.radians(45.0))
A45 = dict(alpha_f=np.radians(45.0), alpha_b=np.radians(45.0))
TESTS = [("45/45 2 V forward", A45, lambda k: 2.0 * FWD, 1500), ("30/45 2 V strafe", A30, lambda k: 2.0 * STR, 1500),
         ("30/45 5 V strafe", A30, lambda k: 5.0 * STR, 1500),
         ("30/45 +-4 V side-to-side", A30, lambda k: 4.0 * -STR * (1 if k < 1000 else (-1 if k < 2000 else 0)), 3000)]
DELTAS = [2e-6, 30e-6, 50e-6, 80e-6]


def run(p, cmd, n):
    sim, s = g["OmniRobotSimulation"](p), g["OmniMotorState"]()
    path, nfev, t0 = np.zeros((n, 3)), 0, time.perf_counter()
    for k in range(n):
        s, _, S = sim.step(s, cmd(k)); path[k] = s.pos_global; nfev += sim.last_iterations
    return path, nfev / n, time.perf_counter() - t0


ref = {}
print(f"{'delta':>6} {'test':26} {'wall (s)':>8} {'solves/step':>11} {'path diff from 2 um':>20} {'end heading':>12}")
for delta in DELTAS:
    for name, geo, cmd, n in TESTS:
        p = dict(base, lugre_deflection=delta, **geo)
        path, ev, wall = run(p, cmd, n)
        if delta == DELTAS[0]:
            ref[name] = path
        diff = 1e3 * np.max(np.hypot(*(path[:, :2] - ref[name][:, :2]).T))
        print(f"{delta * 1e6:4.0f} um {name:26} {wall:8.1f} {ev:11.1f} {diff:17.1f} mm {np.degrees(path[-1, 2]):+10.0f} deg", flush=True)

print("\nSlow ramp, 45/45 forward, 0 to 3 V over 1.5 s: first wheel to break loose, and the largest creep of a gripping wheel")
for delta in DELTAS:
    p = dict(base, lugre_deflection=delta, **A45)
    sim, s = g["OmniRobotSimulation"](p), g["OmniMotorState"]()
    creep, V_break = 0.0, None
    for k in range(1500):
        V = 3.0 * k / 1500
        s, _, S = sim.step(s, V * FWD)
        slip = sim.H @ s.v_local - p["r"] * s.omega
        if V_break is None and np.any(S):
            V_break = V
        if V_break is None:
            creep = max(creep, np.abs(slip).max())
    print(f"   {delta * 1e6:4.0f} um: breaks loose at {V_break if V_break is None else round(V_break, 3)} V, "
          f"largest gripping slip speed before that {1e3 * creep:.2f} mm/s, bristle deflection at the static limit {delta * 1e6:.0f} um", flush=True)
