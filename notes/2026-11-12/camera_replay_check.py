"""
Open loop: the observers' estimates against the plant's true state, with a perfect camera, with 50 ms late frames
used as if on time, and with them replayed to now. Notebooks are only read. Run from dana-mpc-copy-with-estimator.
"""
import json, time, matplotlib
import numpy as np
matplotlib.use("Agg")
c = ["".join(x["source"]) for x in json.load(open("ControllerAnalysis_Linearized.ipynb", encoding="utf-8"))["cells"]]
g = {"__name__": "__main__"}
exec(c[1], g)
p = dict(g["simulation_parameters"])
STRAFE = np.array([-1.0, -1.0, 1.0, 1.0])
rand_V = np.random.default_rng(3).uniform(-8, 8, (20, 4))
runs = {
    "6 V strafe, reversed": (lambda t: 6 * STRAFE * (1.0 if t < 0.5 else -1.0), 1.5),
    "4 V side-to-side": (lambda t: -4 * STRAFE * (1.0 if t < 1 else (-1.0 if t < 2 else 0.0)), 3.0),
    "random 8 V steps": (lambda t: rand_V[min(int(t / 0.2), 19)], 4.0),
}
cases = {"perfect camera": (False, True), "late, no replay": (True, False), "late, replayed": (True, True)}
print(f"{'run':22s} {'observer':8s} {'camera':16s} {'pose err mm':>14s} {'heading deg':>14s} {'v err mm/s':>14s} {'z err':>12s} {'ms/frame':>9s}")
for name, (V_of_t, T) in runs.items():
    for obs_name in ("RobotObserver1ms", "RobotObserver1msIMU"):
        for case, (delay_on, replay) in cases.items():
            params = dict(p, camera_delay_on=delay_on)
            plant = g["OmniRobotSimulationODESolveIMU"](params, 42)
            cam = g["OmniRobotCamera"](params, 42)
            obs = g[obs_name](params, camera_replay=replay)
            st = g["OmniRobotStateODESolve"]()
            e = {"p": [], "h": [], "v": [], "z": []}
            frame_time = []
            for k in range(int(round(T / params["dt"]))):
                st = plant.step(st, V_of_t(k * params["dt"]))
                o = plant.get_observable_state(st)
                pose = cam.step(st.pos_global)
                t0 = time.perf_counter()
                if obs_name.endswith("IMU"):
                    obs.update(st.V_actual, o.omega, o.I, pose, o.imu_accel, o.imu_yaw_rate, pose_age=cam.last_frame_delay)
                else:
                    obs.update(st.V_actual, o.omega, o.I, pose, pose_age=cam.last_frame_delay)
                if pose is not None:
                    frame_time.append(1000 * (time.perf_counter() - t0))
                if (k + 1) % 20 == 0:                       # when the MPC reads the estimate
                    x = obs.estimate()
                    e["p"].append(1000 * np.hypot(*(x[:2] - st.pos_global[:2])))
                    e["h"].append(abs(np.degrees((x[2] - st.pos_global[2] + np.pi) % (2 * np.pi) - np.pi)))
                    e["v"].append(1000 * np.hypot(*(x[3:5] - st.v_global[:2])))
                    e["z"].append(np.abs(x[obs.model.BRISTLES] - st.z).max())
            f = lambda a: f"{np.median(a):6.3g} /{np.max(a):6.3g}"
            print(f"{name:22s} {'IMU' if obs_name.endswith('IMU') else 'no IMU':8s} {case:16s} {f(e['p']):>14s} {f(e['h']):>14s} "
                  f"{f(e['v']):>14s} {f(e['z']):>12s} {np.median(frame_time):9.1f}")
print("(median / worst at the 20 ms controller steps; ms/frame: observer time on a step a frame arrives)")
