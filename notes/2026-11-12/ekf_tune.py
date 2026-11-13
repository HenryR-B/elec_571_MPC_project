import json, sys, time, itertools, matplotlib
import numpy as np
matplotlib.use("Agg")
S = sys.argv[1]; mode = sys.argv[2]
c = ["".join(x["source"]) for x in json.load(open("ControllerAnalysis_Linearized.ipynb", encoding="utf-8"))["cells"]]
g = {"__name__": "__main__"}; exec(c[1], g); exec(open(f"{S}/ekf_cell.py", encoding="utf-8").read(), g)
p = dict(g["simulation_parameters"], camera_delay_on=True)
STRAFE = np.array([-1.0, -1.0, 1.0, 1.0]); DIAG = np.array([-1.0, 0.0, 1.0, 0.0])
train_V = np.random.default_rng(7).uniform(-8, 8, (20, 4)); test_V = np.random.default_rng(3).uniform(-8, 8, (20, 4))
train = {"train: random 8 V (seed 7)": (lambda t: train_V[min(int(t / 0.2), 19)], 3.0),
         "train: 5 V diagonal, reversed": (lambda t: 5 * DIAG * (1.0 if t < 0.7 else -1.0), 1.5)}
test = {"6 V strafe, reversed": (lambda t: 6 * STRAFE * (1.0 if t < 0.5 else -1.0), 1.5),
        "4 V side-to-side": (lambda t: -4 * STRAFE * (1.0 if t < 1 else (-1.0 if t < 2 else 0.0)), 3.0),
        "random 8 V steps": (lambda t: test_V[min(int(t / 0.2), 19)], 4.0)}

def run(make_obs, V_of_t, T):
    plant = g["OmniRobotSimulationODESolveIMU"](p, 42); cam = g["OmniRobotCamera"](p, 42); obs = make_obs()
    st = g["OmniRobotStateODESolve"](); e = {"p": [], "h": [], "v": [], "z": []}; t_total = 0.0
    for k in range(int(round(T / p["dt"]))):
        st = plant.step(st, V_of_t(k * p["dt"])); o = plant.get_observable_state(st); pose = cam.step(st.pos_global)
        t0 = time.perf_counter()
        obs.update(st.V_actual, o.omega, o.I, pose, o.imu_accel, o.imu_yaw_rate, pose_age=cam.last_frame_delay)
        t_total += time.perf_counter() - t0
        if (k + 1) % 20 == 0:
            x = obs.estimate()
            e["p"].append(1000 * np.hypot(*(x[:2] - st.pos_global[:2])))
            e["h"].append(abs(np.degrees((x[2] - st.pos_global[2] + np.pi) % (2 * np.pi) - np.pi)))
            e["v"].append(1000 * np.hypot(*(x[3:5] - st.v_global[:2])))
            e["z"].append(np.abs(x[14:18] - st.z).max())
    return {k: (np.median(v), np.max(v)) for k, v in e.items()}, 1000 * t_total / (k + 1)

f = lambda r: " ".join(f"{k} {r[k][0]:6.3g}/{r[k][1]:6.3g}" for k in ("p", "h", "v", "z"))
if mode == "tune":
    for qp in (1e-6, 1e-5, 3e-5, 1e-4):
        qv, qz, qa = 1e-5, 2e-2, 0.1
        q = {"pose": qp}
        score = []
        for name, (V, T) in train.items():
            r, ms = run(lambda: g["RobotObserverEKF"](p, process_noise=q), V, T)
            score.append(r["v"][1] / 10 + r["p"][1] + r["p"][0])             # worst velocity (cm/s) + worst pose (mm)
        print(f"q_pose {qp:7.0e} q_v {qv:7.0e} q_z {qz:7.0e} q_a {qa:5.2f}  score {np.mean(score):7.2f}", flush=True)
else:
    q = json.loads(sys.argv[3]) if len(sys.argv) > 3 else None
    for name, (V, T) in test.items():
        for label, make in (("hand gains 0.1/0.01", lambda: g["RobotObserver1msIMU"](p, gain_pose=0.1, gain_velocity=0.01)),
                            ("hand gains default", lambda: g["RobotObserver1msIMU"](p)),
                            ("EKF", lambda: g["RobotObserverEKF"](p, process_noise=q))):
            r, ms = run(make, V, T)
            print(f"{name:22s} {label:20s} {f(r)}   {ms:.2f} ms/step", flush=True)
