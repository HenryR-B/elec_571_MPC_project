"""Compare the working notebook's model with the functions-and-constants rewrite (change 10), state by state: pose, velocities, wheel speeds,
motor currents, bristle states, past acceleration, applied voltages, load torque, slip flags and solve counts, at every step.
Cases: all 12 test motions with the noise off and on, then again with the roller friction on and with the CAD CoG offset.
Then the rewrite's run_simulation against the logging of the working notebook's test cells, and whether its model cells keep to the separation.
Exits with 1 if anything differs by more than rounding.
Usage: python tests/compare_henry_style.py RobotSimulationOpenLoopModel_HenryConstants.ipynb RobotSimulationOpenLoopModel_HenryStyle.ipynb"""
import json, re, sys, time
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

TOLERANCE = 1e-6     # largest difference in any state accepted as rounding


def cells(path):
    return {c.get("id"): "".join(c["source"]) for c in json.load(open(path, encoding="utf-8"))["cells"]}


a, b = cells(sys.argv[1]), cells(sys.argv[2])
go = {}
exec(a["ebf0a37d"], go)
exec(a["21f5c2d7"].split("# 2. Instantiate")[0], go)
gn = {}
for cell_id in ("model", "timestep", "simulation", "controller", "constants"):
    exec(b[cell_id], gn)

FWD, STR, ROT = gn["FORWARD"], gn["STRAFE_RIGHT"], gn["ROTATE_CCW"]
A30 = (np.radians(30.0), np.radians(45.0))
A45 = (np.radians(45.0), np.radians(45.0))
TESTS = []
for label, angles, voltage in (("45/45 1 V", A45, 1.0), ("30/45 1 V", A30, 1.0), ("30/45 10 V", A30, 10.0)):
    for motion, pattern in (("forward", FWD), ("strafe", STR), ("rotation", ROT)):
        TESTS.append((f"{label} {motion}", angles, gn["constant_voltage"](voltage, pattern), 500))
for motion, pattern in (("fwd/back/coast", FWD), ("side-to-side", -STR), ("rotation", ROT)):
    TESTS.append((f"30/45 4 V {motion}", A30, gn["multi_stage_voltage"](4.0, pattern, 1000), 3000))


def old_plant(angles, sigma_V, sigma_mu, roller=False, cog=0.0):
    p = dict(go["simulation_parameters"], alpha_f=angles[0], alpha_b=angles[1], sigma_V=sigma_V, sigma_mu=sigma_mu, roller_friction=roller, cog_y=cog)
    return go["OmniRobotSimulation"](p, random_seed=42), go["OmniMotorState"]()


def run_old(angles, controller, steps, sigma_V, sigma_mu, roller, cog):
    sim, s = old_plant(angles, sigma_V, sigma_mu, roller, cog)
    out, t0 = [], time.perf_counter()
    for k in range(steps):
        s, T_load, S = sim.step(s, controller(k, None))
        out.append(np.concatenate((s.pos_global, s.v_global, s.v_local, s.omega, s.I, s.z, s.a_local_past, s.V_actual, T_load, S, [sim.last_iterations])))
    return np.array(out), time.perf_counter() - t0


def run_new(angles, controller, steps, sigma_V, sigma_mu, roller, cog):
    gn["ROLLER_FRICTION"], gn["COG_FORWARD"] = roller, cog
    geometry, rng, s = gn["wheel_geometry"](*angles), np.random.default_rng(seed=42), gn["initial_state"]()
    out, t0 = [], time.perf_counter()
    for k in range(steps):
        w_F, w_mu = gn["draw_disturbances"](rng, sigma_V, sigma_mu)
        s, T_load, slipping, solves = gn["step"](s, controller(k, gn["measured_outputs"](s)), geometry, w_F, w_mu)
        out.append(np.concatenate((s["pos_global"], s["v_global"], s["v_local"], s["omega"], s["I"], s["z"], s["a_local_past"], s["V_actual"], T_load, slipping, [solves])))
    return np.array(out), time.perf_counter() - t0


CASES = [("noise off", 0.0, 0.0, False, 0.0), ("noise on", 0.15, 0.03, False, 0.0),
         ("noise off, roller on", 0.0, 0.0, True, 0.0), ("noise on, roller on", 0.15, 0.03, True, 0.0),
         ("noise off, CoG 10.2 mm", 0.0, 0.0, False, 10.206e-3), ("noise on, CoG 10.2 mm, roller on", 0.15, 0.03, True, 10.206e-3)]
failed = False
t_old = t_new = 0.0
for label, sigma_V, sigma_mu, roller, cog in CASES:
    worst_path, worst_state, flag_diffs, solve_diffs = 0.0, 0.0, 0, 0
    for name, angles, controller, steps in TESTS:
        A, ta = run_old(angles, controller, steps, sigma_V, sigma_mu, roller, cog)
        B, tb = run_new(angles, controller, steps, sigma_V, sigma_mu, roller, cog)
        t_old += ta
        t_new += tb
        # The last column is the solve count, the four before it the slip flags
        worst_path = max(worst_path, 1e3 * np.max(np.hypot(*(A[:, :2] - B[:, :2]).T)))
        worst_state = max(worst_state, np.max(np.abs(A[:, :-5] - B[:, :-5])))
        flag_diffs += int(np.sum(A[:, -5:-1] != B[:, -5:-1]))
        solve_diffs += int(np.sum(A[:, -1] != B[:, -1]))
    failed |= worst_state > TOLERANCE or flag_diffs > 0 or solve_diffs > 0
    print(f"{label:34}: largest path difference {worst_path:.2e} mm, largest difference in any state {worst_state:.2e}, "
          f"slip flags differing {flag_diffs}, solve counts differing {solve_diffs}", flush=True)
print(f"run time of the plant steps, all cases: working copy {t_old:.1f} s, rewrite {t_new:.1f} s")

# The rewrite's own loop against the way the working notebook's test cells log a run: record, then step
gn["ROLLER_FRICTION"], gn["COG_FORWARD"] = False, 0.0
worst_logged = 0.0
for name, angles, controller, steps in TESTS:
    for sigma_V, sigma_mu in ((0.0, 0.0), (gn["SIGMA_LOAD"], gn["SIGMA_MU"])):
        history = gn["run_simulation"](controller, steps, gn["wheel_geometry"](*angles), sigma_V, sigma_mu)
        sim, s = old_plant(angles, sigma_V, sigma_mu)
        for k in range(steps):
            V = controller(k, None)
            worst_logged = max(worst_logged, np.max(np.abs(history["V"][k] - V)), np.max(np.abs(history["omega"][k] - s.omega)),
                               np.max(np.abs(history["v_local"][k] - s.v_local)), np.max(np.abs(history["pos_global"][k] - s.pos_global)))
            s, _, _ = sim.step(s, V)
failed |= worst_logged > TOLERANCE
print(f"run_simulation against the test cells' logging: largest difference in any logged signal {worst_logged:.2e}")


def code_only(source):
    return re.sub(r"#.*", "", re.sub(r'"""[\s\S]*?"""', "", source))


RULES = [("model", r"\bDT\b", "no time step"), ("model", r"\brng\b|sigma_|\.normal\(", "no noise generator"), ("model", r"controller|measurement", "no controller"),
         ("timestep", r"\brng\b|sigma_|\.normal\(", "no noise generator"), ("timestep", r"controller|measurement", "no controller"),
         ("controller", r"state\[|\bstep\(", "no access to the plant state")]
for cell_id, pattern, rule in RULES:
    broken = bool(re.search(pattern, code_only(b[cell_id])))
    failed |= broken
    print(f"{'FAIL' if broken else 'ok  '} {cell_id}: {rule}")

print("FAILED" if failed else "PASSED: same results to rounding")
sys.exit(1 if failed else 0)
