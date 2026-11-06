"""Checks of the roller (a-axis) friction added in change 9. Noise off.
1. Off reproduces the model without it exactly (against a notebook saved before the change, if given).
2. Geometry: a pure spin gives zero axle slip (axle radial, spin velocity tangential), so no roller force.
3. Stability of the explicit, one-step-lagged force: how often each wheel's roller force flips sign from one step to the
   next, including while the robot coasts to rest (where the axle slip crosses zero).
4. Its effect on Dana's tests: path, speed, heading, slipping, run time, with it on and off.
Usage: python tests/roller_friction_check.py RobotSimulationOpenLoopModel_HenryConstants.ipynb [BEFORE.ipynb]"""
import json, sys, time
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def load(path):
    d = {c.get("id"): "".join(c["source"]) for c in json.load(open(path, encoding="utf-8"))["cells"]}
    g = {}
    exec(d["ebf0a37d"], g)
    exec(d["21f5c2d7"].split("# 2. Instantiate")[0], g)
    return g


g = load(sys.argv[1])
p0 = dict(g["simulation_parameters"], sigma_V=0.0, sigma_mu=0.0, roller_friction=True)   # on here: the notebook's default is off
FWD, STR, ROT = np.array([1, -1, -1, 1.0]), np.array([-1, -1, 1, 1.0]), np.ones(4)
A30 = dict(alpha_f=np.radians(30.0), alpha_b=np.radians(45.0)); A45 = dict(alpha_f=np.radians(45.0), alpha_b=np.radians(45.0))
TESTS = [("B1 45/45 1 V forward", A45, lambda t: 1.0 * FWD, 0.5), ("B1 45/45 1 V strafe", A45, lambda t: 1.0 * STR, 0.5),
         ("B1 45/45 1 V rotation", A45, lambda t: 1.0 * ROT, 0.5), ("B2 30/45 1 V forward", A30, lambda t: 1.0 * FWD, 0.5),
         ("B2 30/45 1 V strafe", A30, lambda t: 1.0 * STR, 0.5), ("B2 30/45 1 V rotation", A30, lambda t: 1.0 * ROT, 0.5),
         ("B3 30/45 10 V forward", A30, lambda t: 10.0 * FWD, 0.5), ("B3 30/45 10 V strafe", A30, lambda t: 10.0 * STR, 0.5),
         ("B3 30/45 10 V rotation", A30, lambda t: 10.0 * ROT, 0.5),
         ("MS 30/45 4 V fwd/back/coast", A30, lambda t: 4.0 * FWD * (1 if t < 1 else (-1 if t < 2 else 0)), 3.0),
         ("MS 30/45 4 V side-to-side", A30, lambda t: 4.0 * -STR * (1 if t < 1 else (-1 if t < 2 else 0)), 3.0),
         ("MS 30/45 4 V rotation", A30, lambda t: 4.0 * ROT * (1 if t < 1 else (-1 if t < 2 else 0)), 3.0)]


def run(gg, p, cmd, T):
    sim, s = gg["OmniRobotSimulation"](p), gg["OmniMotorState"]()
    n = int(round(T / p["dt"]))
    P, Fa, Va, slip, t0 = np.zeros((n, 3)), np.zeros((n, 4)), np.zeros((n, 4)), np.zeros(4), time.perf_counter()
    for k in range(n):
        s, _, S = sim.step(s, cmd(k * p["dt"])); P[k] = s.pos_global; slip += S
        if hasattr(sim, "last_roller"):
            Fa[k], Va[k] = sim.last_roller
    return P, s, Fa, Va, slip / n, time.perf_counter() - t0


dmm = lambda a, b: 1e3 * np.max(np.hypot(*(a[:, :2] - b[:, :2]).T))

if len(sys.argv) > 2:
    gb = load(sys.argv[2])
    pb = dict(gb["simulation_parameters"], sigma_V=0.0, sigma_mu=0.0)
    worst = max(dmm(run(g, dict(p0, roller_friction=False, **geo), cmd, T)[0], run(gb, dict(pb, **geo), cmd, T)[0]) for _, geo, cmd, T in TESTS)
    print(f"1. Off against the notebook before the change: largest path difference over all tests {worst:.6f} mm")

sim = g["OmniRobotSimulation"](dict(p0, **A30))
W, Fa, Va = sim.compute_roller_friction(np.array([0.0, 0.0, 5.0]), np.full(4, 6.6))
print(f"2. Pure spin at 5 rad/s: axle slips {np.round(Va, 12)} m/s, roller wrench {np.round(W, 12)}")
W, Fa, Va = sim.compute_roller_friction(np.array([0.0, 0.5, 0.0]), np.full(4, 6.6))
print(f"   Pure forward 0.5 m/s: axle slips {np.round(Va, 3)} m/s, roller forces {np.round(Fa, 3)} N, wrench [Fx, Fy, Mz] {np.round(W, 3)}")

print("3. Sign flips of each wheel's roller force between consecutive steps (only steps where it is above 1 mN):")
for name, geo, cmd, T in (("MS 30/45 4 V fwd/back/coast", A30, lambda t: 4.0 * FWD * (1 if t < 1 else (-1 if t < 2 else 0)), 3.0),
                          ("MS 30/45 4 V side-to-side", A30, lambda t: 4.0 * -STR * (1 if t < 1 else (-1 if t < 2 else 0)), 3.0),
                          ("B2 30/45 1 V strafe", A30, lambda t: 1.0 * STR, 0.5)):
    P, s, Fa, Va, sl, _ = run(g, dict(p0, **geo), cmd, T)
    big = np.abs(Fa) > 1e-3
    flips = (np.sign(Fa[1:]) != np.sign(Fa[:-1])) & big[1:] & big[:-1]
    coast = slice(int(2.0 / p0["dt"]), None)
    print(f"   {name:30}: flips per wheel {flips.sum(axis=0)} over {len(Fa)} steps; while coasting the axle slip ends at "
          f"{np.round(1e3 * np.abs(Va[-1]), 3)} mm/s" if T > 2 else f"   {name:30}: flips per wheel {flips.sum(axis=0)} over {len(Fa)} steps")

print("4. Dana's tests, roller friction on vs off:")
print(f"   {'test':28} | {'on: end (x, y), heading, speed':40} | {'off: end (x, y), heading, speed':40} | path diff | slipping % on / off | time on/off")
for name, geo, cmd, T in TESTS:
    P1, s1, _, _, S1, w1 = run(g, dict(p0, **geo), cmd, T)
    P0, s0, _, _, S0, w0 = run(g, dict(p0, roller_friction=False, **geo), cmd, T)
    f = lambda P, s: f"({P[-1, 0]:+.3f}, {P[-1, 1]:+.3f}) {np.degrees(P[-1, 2]):+7.1f}° {np.hypot(*s.v_global[:2]):.3f} m/s"
    print(f"   {name:28} | {f(P1, s1):40} | {f(P0, s0):40} | {dmm(P1, P0):6.1f} mm | {np.round(100 * S1).astype(int)} / {np.round(100 * S0).astype(int)} | {w1:.2f}/{w0:.2f} s", flush=True)
