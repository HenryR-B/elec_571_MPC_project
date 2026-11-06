"""Why the 30/45 strafe spins out: each wheel's grip limit mu_s*N against the drive force its current asks for,
for the four combinations of CoG position (built 10.2 mm forward / centred) and CoG height (built / Dana's).
Then the full model: when each wheel first slips on a 5 A strafe, the end yaw rate, and the lowest strafe
current that spins the robot out. Noise off, 30/45.
Usage: python tests/strafe_grip_margin.py RobotSimulationOpenLoopModel_HenryConstants.ipynb"""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
nb = json.load(open(sys.argv[1], encoding="utf-8"))
d = {c.get("id"): "".join(c["source"]) for c in nb["cells"]}
g = {}
exec(d["ebf0a37d"], g)
exec(d["21f5c2d7"].split("# 2. Instantiate")[0], g)
base = dict(g["simulation_parameters"], sigma_V=0.0, sigma_mu=0.0, alpha_f=np.radians(30.0), alpha_b=np.radians(45.0))
STR = np.array([-1, -1, 1, 1.0])
H_BUILT, H_DANA = g["_DESIGNS"]["built"]["h"], g["_DESIGNS"]["proper"]["h"]
C_BUILT = g["_DESIGNS"]["built"]["cog_y"]
COMBOS = [("CoG +10.2 mm, h 61 mm (built)", C_BUILT, H_BUILT), ("CoG +10.2 mm, h 32 mm", C_BUILT, H_DANA),
          ("CoG centred, h 61 mm", 0.0, H_BUILT), ("CoG centred, h 32 mm (proper)", 0.0, H_DANA)]


def spins(p, amps, T=1.0):
    sim, s = g["OmniRobotSimulation"](p), g["OmniMotorState"]()
    first = np.full(4, np.nan)
    for k in range(int(round(T / p["dt"]))):
        s, _, S = sim.step(s, amps * STR)
        first[np.isnan(first) & S] = (k + 1) * p["dt"]
    return s, first


I_A = 5.0
print("Drive force asked of each wheel at 5 A, wheel held at the floor: Kt*I/r = %.2f N" % (base["Kt"] * I_A / base["r"]))
for name, c, h in COMBOS:
    p = dict(base, cog_y=c, h=h)
    sim = g["OmniRobotSimulation"](p)
    # Gripping strafe: chassis acceleration from equal wheel forces, then the loads it causes
    F = base["Kt"] * I_A / base["r"] * STR
    M = np.diag([p["Mr"], p["Mr"], p["Jzz"]])
    a = np.linalg.solve(M, sim.H.T @ F)
    N0 = sim.compute_dynamic_normal_forces(np.zeros(2), np.zeros(4))
    N1 = sim.compute_dynamic_normal_forces(a[:2], np.zeros(4))
    print(f"\n{name}: a_x {a[0]:+.2f} m/s^2, yaw accel {a[2]:+.0f} rad/s^2")
    print("   loads at rest        N  " + "  ".join(f"{x:5.2f}" for x in N0) + "   (FR FL RL RR)")
    print("   loads while strafing N  " + "  ".join(f"{x:5.2f}" for x in N1))
    print("   grip mu_s*N / demand    " + "  ".join(f"{x:5.2f}" for x in p["mu_s"] * N1 / abs(F)))
    s, first = spins(p, I_A, 1.5)
    print("   5 A strafe: first slip (s) " + "  ".join("  -  " if np.isnan(x) else f"{x:5.3f}" for x in first)
          + f" | end yaw rate {s.v_global[2]:+.1f} rad/s, heading {np.degrees(s.pos_global[2]):+.0f} deg")
    lo, hi = 1.0, 5.0
    for _ in range(6):
        mid = 0.5 * (lo + hi)
        _, f = spins(p, mid, 1.0)
        lo, hi = (lo, mid) if np.any(~np.isnan(f)) else (mid, hi)
    print(f"   lowest strafe current that makes a wheel slip within 1 s: {lo:.2f}-{hi:.2f} A", flush=True)
