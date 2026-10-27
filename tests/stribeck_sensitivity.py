"""How much the 30/45 5 A strafe ('proper' design) depends on the unmeasured Stribeck drop: how far mu falls
(mu_k) and how fast (alpha_d). First the rear-right wheel's slip speed and force over the first 100 ms with the
current constants, then the end of the run for other drops. Noise off.
Usage: python tests/stribeck_sensitivity.py RobotSimulationOpenLoopModel_HenryConstants.ipynb"""
import json, re, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
nb = json.load(open(sys.argv[1], encoding="utf-8"))
d = {c.get("id"): "".join(c["source"]) for c in nb["cells"]}
g = {}
exec(d["ebf0a37d"], g)
exec(re.sub(r"MECHANICAL_DESIGN = '\w+'", "MECHANICAL_DESIGN = 'proper'", d["21f5c2d7"].split("# 2. Instantiate")[0]), g)
base = dict(g["simulation_parameters"], sigma_V=0.0, sigma_mu=0.0, alpha_f=np.radians(30.0), alpha_b=np.radians(45.0))
CMD = 5.0 * np.array([-1, -1, 1, 1.0])
demand = base["Kt"] * 5.0 / base["r"]


def run(p, T=1.5, trace=False):
    sim, s = g["OmniRobotSimulation"](p), g["OmniMotorState"]()
    for k in range(int(round(T / p["dt"]))):
        s, T_load, S = sim.step(s, CMD)
        if trace and (k + 1) in (1, 2, 5, 10, 20, 30, 50, 100):
            slip = sim.H @ s.v_local - p["r"] * s.omega
            Fn = sim.compute_dynamic_normal_forces(s.a_local_past[:2], np.zeros(4))
            mu = p["mu_k"] + (p["mu_s"] - p["mu_k"]) * np.exp(-p["alpha_d"] * abs(slip[3]))
            print(f"   t {k + 1:3d} ms: RR slip {slip[3]:+.3f} m/s, Stribeck mu {mu:.3f}, floor force {T_load[3] / p['r']:+.2f} N "
                  f"(asked {demand:.2f}), load {Fn[3]:.2f} N, heading {np.degrees(s.pos_global[2]):+.1f} deg")
    return s


print(f"Current constants: mu_s {base['mu_s']}, mu_k {base['mu_k']}, alpha_d {base['alpha_d']} s/m")
run(base, T=0.1, trace=True)
print()
for label, mk, ad in [("current: mu_k 0.422, half-way at 0.14 m/s", base["mu_k"], base["alpha_d"]),
                      ("same drop, 5x slower (alpha_d 1)", base["mu_k"], 1.0),
                      ("smaller drop: mu_k 0.75", 0.75, base["alpha_d"]),
                      ("no drop: mu_k = mu_s", base["mu_s"], base["alpha_d"])]:
    s = run(dict(base, mu_k=mk, alpha_d=ad))
    v = s.v_global
    print(f"{label:42}: end ({s.pos_global[0]:+.2f}, {s.pos_global[1]:+.2f}) m, heading {np.degrees(s.pos_global[2]):+7.0f} deg, "
          f"yaw rate {v[2]:+6.1f} rad/s, speed {np.hypot(v[0], v[1]):.2f} m/s", flush=True)
