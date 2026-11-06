"""Dana's v3 model (dana-models/robot-simulation-open-loop-v3.ipynb) against our working model, with her constants in
both (Pololu 37D 30:1 motor, her Jzz, h, Jw; CoG at the centre), on her tests at her voltages. Noise off.
Reports the end of each run, how far the paths differ, the slip speeds, how often the wheel speeds chatter (the step
change flips sign), and run time. Usage:
python tests/compare_dana_v3.py dana-models/robot-simulation-open-loop-v3.ipynb RobotSimulationOpenLoopModel_HenryConstants.ipynb"""
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


gD, gH = load(sys.argv[1]), load(sys.argv[2])
pD = dict(gD["simulation_parameters"], sigma_V=0.0, sigma_mu=0.0)
# ours with Dana's constants
pH = dict(gH["simulation_parameters"], sigma_V=0.0, sigma_mu=0.0, cog_y=0.0,
          **{k: pD[k] for k in ("Mr", "Jzz", "Rb", "h", "r", "Jw", "R", "Lm", "Kt", "Kb", "Crr", "mu_s", "mu_k",
                                "lugre_deflection", "lugre_damping_ratio", "lugre_damping_slip", "lugre_creep",
                                "alpha_d", "roll_smoothing", "lugre_viscous", "slip_band")})
FWD, STR, ROT = np.array([1, -1, -1, 1.0]), np.array([-1, -1, 1, 1.0]), np.ones(4)
A30 = dict(alpha_f=np.radians(30.0), alpha_b=np.radians(45.0))
A45 = dict(alpha_f=np.radians(45.0), alpha_b=np.radians(45.0))
TESTS = [("Block 1 45/45 1 V forward", A45, lambda t: 1.0 * FWD, 0.5), ("Block 1 45/45 1 V strafe", A45, lambda t: 1.0 * STR, 0.5),
         ("Block 1 45/45 1 V rotation", A45, lambda t: 1.0 * ROT, 0.5),
         ("Block 2 30/45 1 V forward", A30, lambda t: 1.0 * FWD, 0.5), ("Block 2 30/45 1 V strafe", A30, lambda t: 1.0 * STR, 0.5),
         ("Block 2 30/45 1 V rotation", A30, lambda t: 1.0 * ROT, 0.5),
         ("Block 3 30/45 10 V forward", A30, lambda t: 10.0 * FWD, 0.5), ("Block 3 30/45 10 V strafe", A30, lambda t: 10.0 * STR, 0.5),
         ("Block 3 30/45 10 V rotation", A30, lambda t: 10.0 * ROT, 0.5),
         ("Multi 30/45 +-4 V fwd/back/coast", A30, lambda t: 4.0 * FWD * (1 if t < 1.0 else (-1 if t < 2.0 else 0)), 3.0),
         ("Multi 30/45 +-4 V side-to-side", A30, lambda t: 4.0 * -STR * (1 if t < 1.0 else (-1 if t < 2.0 else 0)), 3.0),
         ("Multi 30/45 +-4 V rotation", A30, lambda t: 4.0 * ROT * (1 if t < 1.0 else (-1 if t < 2.0 else 0)), 3.0)]


def run(g, p, cmd, T):
    sim, s = g["OmniRobotSimulation"](p), g["OmniMotorState"]()
    n = int(round(T / p["dt"]))
    P, W, slip, t0 = np.zeros((n, 3)), np.zeros((n, 4)), np.zeros((n, 4)), time.perf_counter()
    for k in range(n):
        s, _, S = sim.step(s, cmd(k * p["dt"]))
        P[k], W[k] = s.pos_global, s.omega
        slip[k] = sim.H @ s.v_local - p["r"] * s.omega
    dW = np.diff(W, axis=0)
    chatter = np.mean(np.sign(dW[1:]) * np.sign(dW[:-1]) < 0, axis=0)   # fraction of steps the wheel-speed change flips sign
    return P, s, slip, chatter, time.perf_counter() - t0


tD = tH = 0.0
for name, geo, cmd, T in TESTS:
    PD, sD, slD, chD, wD = run(gD, dict(pD, **geo), cmd, T)
    PH, sH, slH, chH, wH = run(gH, dict(pH, **geo), cmd, T)
    tD += wD; tH += wH
    diff = 1e3 * np.max(np.hypot(*(PD[:, :2] - PH[:, :2]).T))
    print(f"\n{name}: path difference {diff:.1f} mm")
    for lab, P, s, sl, ch, w in (("Dana v3", PD, sD, slD, chD, wD), ("ours   ", PH, sH, slH, chH, wH)):
        print(f"   {lab}: end ({P[-1, 0]:+.3f}, {P[-1, 1]:+.3f}) m, heading {np.degrees(P[-1, 2]):+7.1f} deg, "
              f"speed {np.hypot(*s.v_global[:2]):.3f} m/s, yaw rate {s.v_global[2]:+6.2f} rad/s | max |slip| {1e3 * np.abs(sl).max():7.1f} mm/s | "
              f"wheel-speed chatter {np.round(100 * ch).astype(int)} % | {w:.2f} s", flush=True)
print(f"\ntotal run time: Dana v3 {tD:.1f} s, ours {tH:.1f} s")
