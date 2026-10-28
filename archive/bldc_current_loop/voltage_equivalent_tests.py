"""Dana's voltage tests reproduced on the current-input model. Each step the test layer commands the current Dana's
voltage V would drive through our motor at the present wheel speed, I_cmd = clip((V - Kb*omega) / R, +-MOTOR_CURRENT_LIMIT),
so a wheel that spins up loses current through back-EMF exactly as under voltage drive. Compared with the flat-current
tests now in the notebook (the same numbers in amps). 30/45, noise off, both mechanical designs.
Usage: python archive/bldc_current_loop/voltage_equivalent_tests.py RobotSimulationOpenLoopModel_HenryConstants.ipynb
(It needs a notebook version that still takes current commands.)"""
import json, re, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # repository root
nb = json.load(open(sys.argv[1], encoding="utf-8"))
d = {c.get("id"): "".join(c["source"]) for c in nb["cells"]}
FWD, STR, ROT = np.array([1, -1, -1, 1.0]), np.array([-1, -1, 1, 1.0]), np.ones(4)
SIDE = np.array([1, 1, -1, -1.0])                                    # the multi-stage side-to-side pattern


def staged(v):                                                       # +v for 1 s, -v for 1 s, 0 for 1 s
    return lambda k: v if k < 1000 else (-v if k < 2000 else 0.0 * v)


TESTS = [("Block 3, 5 V forward", lambda k: 5.0 * FWD, 1500), ("Block 3, 5 V strafe", lambda k: 5.0 * STR, 1500),
         ("Block 3, 5 V rotation", lambda k: 5.0 * ROT, 1500), ("Block 2, 2 V strafe", lambda k: 2.0 * STR, 1500),
         ("Multi-stage, +-4 V side-to-side", staged(4.0 * SIDE), 3000)]

for design in ("built", "proper"):
    g = {}
    exec(d["ebf0a37d"], g)
    exec(re.sub(r"MECHANICAL_DESIGN = '\w+'", f"MECHANICAL_DESIGN = '{design}'", d["21f5c2d7"].split("# 2. Instantiate")[0]), g)
    p = dict(g["simulation_parameters"], sigma_V=0.0, sigma_mu=0.0, alpha_f=np.radians(30.0), alpha_b=np.radians(45.0))
    for name, cmd, n in TESTS:
        for mode in ("flat amps", "voltage"):
            sim, s = g["OmniRobotSimulation"](p), g["OmniMotorState"]()
            slip, I_peak, V_err = np.zeros(4), 0.0, []
            for k in range(n):
                V = cmd(k)
                I = V if mode == "flat amps" else np.clip((V - p["Kb"] * s.omega) / p["R"], -p["Iq_limit"], p["Iq_limit"])
                s, _, S = sim.step(s, I)
                slip += S; I_peak = max(I_peak, np.abs(s.I).max())
                if mode == "voltage" and np.all(np.abs(I) < p["Iq_limit"]):
                    V_err.append(np.abs(s.V_applied - V).max())
            v = s.v_global
            extra = f", applied V within {np.median(V_err):.2f} V of Dana's (median)" if V_err else ""
            print(f"{design:6} {name:32} {mode:9}: end ({s.pos_global[0]:+.2f}, {s.pos_global[1]:+.2f}) m, heading {np.degrees(s.pos_global[2]):+7.0f} deg, "
                  f"speed {np.hypot(v[0], v[1]):.2f} m/s, peak current {I_peak:4.1f} A, slipping {np.round(100 * slip / n).astype(int)} %{extra}", flush=True)
