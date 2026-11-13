"""Checks the IMU observer and the camera. argv[1]: folder of the notebooks to load (new, or the backup)."""
import json, os, sys, matplotlib
import numpy as np
matplotlib.use("Agg")
folder = sys.argv[1]
mode = sys.argv[2]
os.chdir(folder)
c = ["".join(x["source"]) for x in json.load(open("ControllerAnalysis_Linearized.ipynb", encoding="utf-8"))["cells"]]
g = {"__name__": "__main__"}
for s in c[:15]:
    if any(k in s for k in ("path_generator =", "analysis_params", "def ", "import", "class ")) and "run_and_report(\"" not in s:
        exec(s, g)

if mode == "closed":
    exec('''
paths = {"Dana 4 s": path_generator.generate_coupled_segment(np.zeros(3), np.array([3.0, 3.0, np.pi / 4]), duration=4.0)}
ctrls = ["mpc_lin_obs"] + (["mpc_lin_obs_imu"] if "OmniRobotCamera" in globals() else [])
res = run_and_report("check", paths, ctrls, analysis_params)
for k, v in res.items():
    print(k, "final pos", np.round(v["pos"][-1], 6) if isinstance(v, dict) and "pos" in v else "")
''', g)
    sys.exit()

# Open loop: both observers against the true state, the observer test's three runs
p = dict(g["simulation_parameters"])
STRAFE = np.array([-1.0, -1.0, 1.0, 1.0])
rand_V = np.random.default_rng(3).uniform(-8, 8, (20, 4))
runs = {
    "6 V strafe, reversed": (lambda t: 6 * STRAFE * (1.0 if t < 0.5 else -1.0), 1.5),
    "4 V side-to-side": (lambda t: -4 * STRAFE * (1.0 if t < 1 else (-1.0 if t < 2 else 0.0)), 3.0),
    "random 8 V steps": (lambda t: rand_V[min(int(t / 0.2), 19)], 4.0),
}
for delay_on in (False, True):
    params = dict(p, camera_delay_on=delay_on)
    print(f"\ncamera_delay_on={delay_on}   (median / worst at the 20 ms controller steps)")
    print(f"{'run':24s} {'observer':10s} {'v err mm/s':>14s} {'yaw rate err':>14s} {'a_past err':>14s} {'z err':>12s} {'pose err mm':>13s}")
    for name, (V_of_t, T) in runs.items():
        for obs_name in ("RobotObserver1ms", "RobotObserver1msIMU"):
            plant = g["OmniRobotSimulationODESolveIMU"](params, 42)
            cam = g["OmniRobotCamera"](params, 42)
            obs = g[obs_name](params)
            m = obs.model
            st = g["OmniRobotStateODESolve"]()
            e = {"v": [], "w": [], "a": [], "z": [], "p": []}
            delays = []
            for k in range(int(round(T / params["dt"]))):
                st = plant.step(st, V_of_t(k * params["dt"]))
                o = plant.get_observable_state(st)
                pose = cam.step(st.pos_global)
                if pose is not None:
                    delays.append(cam.last_frame_delay)
                if obs_name.endswith("IMU"):
                    obs.update(st.V_actual, o.omega, o.I, pose, o.imu_accel, o.imu_yaw_rate)
                else:
                    obs.update(st.V_actual, o.omega, o.I, pose)
                if (k + 1) % 20 == 0:
                    x = obs.estimate()
                    e["v"].append(1000 * np.hypot(*(x[3:5] - st.v_global[:2])))
                    e["w"].append(abs(x[5] - st.v_global[2]))
                    e["a"].append(np.hypot(*(x[m.ACCEL_PAST] - st.a_local_past[:2])))
                    e["z"].append(np.abs(x[m.BRISTLES] - st.z).max())
                    e["p"].append(1000 * np.hypot(*(x[:2] - st.pos_global[:2])))
            f = lambda a: f"{np.median(a):6.3g} /{np.max(a):6.3g}"
            print(f"{name:24s} {obs_name[13:] or 'no IMU':10s} {f(e['v']):>14s} {f(e['w']):>14s} {f(e['a']):>14s} {f(e['z']):>12s} {f(e['p']):>13s}")
        print(f"{'':24s} camera frame delays seen: {sorted(set(np.round(delays, 3)))}")
