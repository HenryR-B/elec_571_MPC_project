"""
Dana's ControllerAnalysis runner (her plant, parameters and path generator) with every MPC variant plugged in.
Run from the elec_571_MPC_project folder:  python suite.py <controllers comma-separated> <paths comma-separated>
Writes results to suite_results.json (appends/updates).
"""
import json
import os
import sys
import time
import numpy as np
import osqp
from scipy import sparse
from scipy.linalg import expm

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mpc_models as mm

DANA = os.path.join(os.path.dirname(HERE), "dana_zip", "extracted", "Project Design Folder")


def cells(path):
    return json.load(open(path, encoding="utf-8"))["cells"]


# --- Dana's classes ---------------------------------------------------------------------------------------
dana = {"__name__": "dana"}
exec("import numpy as np\nimport matplotlib\nmatplotlib.use('Agg')\nimport matplotlib.pyplot as plt", dana)
_p = cells(os.path.join(DANA, "PlantModels.ipynb"))
for i in (2, 6, 9, 11, 15):
    exec("".join(_p[i]["source"]), dana)
_c = cells(os.path.join(DANA, "ControllerModels.ipynb"))
for i in (1, 4, 6, 9, 10):
    exec("".join(_c[i]["source"]), dana)
exec("".join(cells(os.path.join(DANA, "PathGeneration.ipynb"))[4]["source"]), dana)
_a = cells(os.path.join(DANA, "ControllerAnalysis.ipynb"))
exec("".join(_a[2]["source"]), dana)
PlantClass = dana["OmniRobotSimulationODESolve"]

# True state of the plant, kept as it steps (the MPC variants get it: ideal observer)
TRUE = {}
_orig_step = PlantClass.step
def _step_keep(self, state, V):
    nxt = _orig_step(self, state, V)
    TRUE["state"] = nxt
    return nxt
PlantClass.step = _step_keep

# --- our current bristle MPC (held model), as the baseline ------------------------------------------------
ours = {"__name__": "ours", "np": np, "expm": expm, "sparse": sparse, "osqp": osqp, "time": time}
def _ids(path):
    return {c.get("id"): "".join(c["source"]) for c in cells(path)}
ROOT = os.getcwd()
_m = _ids(os.path.join(ROOT, "RobotSimulationOpenLoopModel_Merged.ipynb"))
exec(_m["constants"], ours); exec(_m["plant"], ours)
exec(_ids(os.path.join(ROOT, "MPCModel_Bristle.ipynb"))["mpc_model"], ours)
_r = _ids(os.path.join(ROOT, "RobotSimulationMPC.ipynb"))
exec(_r["mpc_params"], ours); exec(_r["bristle_controller"], ours)
MPC_PARAMS = ours["mpc_parameters"]


def ref_rows_from(p_h, v_h, N):
    rows = np.zeros((N, 6))
    for j in range(N):
        pose = p_h[j + 1]
        c, s = np.cos(pose[2]), np.sin(pose[2])
        vl = v_h[j + 1]
        rows[j] = np.concatenate((pose, [c * vl[0] - s * vl[1], s * vl[0] + c * vl[1], vl[2]]))
    return rows


class HeldBristleForDana:
    """Our current BristleMPC (held 100 um model). true_state: give it the plant's state instead of its estimate."""
    def __init__(self, params, true_state=False, deflection=100e-6, sigmas=None):
        mp = dict(MPC_PARAMS, bristle_deflection=deflection)
        if sigmas is not None:
            mp['deviation_sigmas'] = sigmas
        self.mpc = ours["BristleMPC"](params, mp)
        self.true_state = true_state
        self.dt, self.K = MPC_PARAMS['Ts'], MPC_PARAMS['N']
        if true_state:
            own = self.mpc.estimate
            def est(measurement):
                s = TRUE.get("state")
                if s is None:
                    return own(measurement)
                self.mpc.omega_prev = s.omega.copy()
                return np.concatenate((s.pos_global, s.v_local, s.omega, s.z))
            self.mpc.estimate = est

    def compute_optimal_u0(self, measured_State, path_ref_horizon, vel_ref_horizon, return_info=False):
        measurement = np.concatenate((measured_State.pos_global, measured_State.omega))
        rows = ref_rows_from(path_ref_horizon, vel_ref_horizon, self.K)
        f0 = self.mpc.failures
        u = self.mpc.step(0.0, measurement, lambda t: rows[int(round(t / self.dt)) - 1])
        info = {'optimal': self.mpc.failures == f0}
        return (u, info) if return_info else u

    @property
    def solve_times(self):
        return [t[2] for t in self.mpc.solve_times]


class LTVForDana:
    def __init__(self, params, model, sigmas=None, measured=False, **kw):
        self.model = model
        self.p = params
        mp = MPC_PARAMS if sigmas is None else dict(MPC_PARAMS, deviation_sigmas=sigmas)
        self.mpc = mm.LTVMPC(params, mp, model, **kw)
        self.dt, self.K = MPC_PARAMS['Ts'], MPC_PARAMS['N']
        self.measured = measured
        self.omega_prev = None

    def estimate(self, obs):
        """Only what Dana's observable state gives: pose, omega, I. v from the model's prediction corrected by the
        pose error; z from each wheel's torque balance, F = (Kt I - T_roll - Jw domega/dt) / r, z = -F / (mu_s F_n)."""
        p, m, Ts = self.p, self.model, self.dt
        pose, omega, I = obs.pos_global.copy(), obs.omega.copy(), obs.I.copy()
        X_plan = self.mpc.X_plan
        if X_plan is None:
            v, z = np.zeros(3), np.zeros(4)
        else:
            pred = X_plan[1]
            err = pose - pred[0:3]
            err[2] = (err[2] + np.pi) % (2 * np.pi) - np.pi
            c, s = np.cos(pose[2]), np.sin(pose[2])
            v = pred[3:6] + np.array([c * err[0] + s * err[1], -s * err[0] + c * err[1], err[2]]) / Ts
            z = pred[14:18].copy()
        x = np.concatenate((pose, v, omega, I, z))
        if self.omega_prev is not None:
            F_n = m.held(x[None])['F_n'][0]
            T_roll = p['Crr'] * F_n * p['r'] * np.tanh(omega / p['roll_smoothing'])
            F = (p['Kt'] * I - T_roll - p['Jw'] * (omega - self.omega_prev) / Ts) / p['r']
            x[14:18] = np.clip(-F / (p['mu_s'] * np.maximum(F_n, 1e-3)), -1.2, 1.2)
        self.omega_prev = omega
        return x

    def compute_optimal_u0(self, measured_State, path_ref_horizon, vel_ref_horizon, return_info=False):
        if self.measured:
            x_hat = self.estimate(measured_State)
        else:
            s = TRUE.get("state")
            x_hat = self.model.from_plant(s) if s is not None else np.zeros(self.model.n_x)
        rows = ref_rows_from(path_ref_horizon, vel_ref_horizon, self.K)
        f0 = self.mpc.failures
        u = self.mpc.step(x_hat, rows)
        info = {'optimal': self.mpc.failures == f0}
        return (u, info) if return_info else u

    @property
    def solve_times(self):
        return self.mpc.solve_times


def make(name, P):
    B = lambda **kw: mm.BristleJacobianModel(P, PlantClass, **kw)
    S = lambda **kw: mm.SlipFrictionModel(P, PlantClass, **kw)
    table = {
        "held100": lambda: HeldBristleForDana(P),
        "held100_true": lambda: HeldBristleForDana(P, true_state=True),
        "held2_true": lambda: HeldBristleForDana(P, true_state=True, deflection=2e-6),
        "J1": lambda: LTVForDana(P, B(), substeps=1),
        "J2": lambda: LTVForDana(P, B(), substeps=2),
        "J4": lambda: LTVForDana(P, B(), substeps=4),
        "J2_zbound": lambda: LTVForDana(P, B(), substeps=2, z_bound=True),
        "JP5": lambda: LTVForDana(P, B(), substeps=5, mode="plan"),
        "JP10": lambda: LTVForDana(P, B(), substeps=10, mode="plan"),
        "J2s": lambda: LTVForDana(P, B(slide_threshold=0.002), substeps=2),
        "J2s_zbound": lambda: LTVForDana(P, B(slide_threshold=0.002), substeps=2, z_bound=True),
        "J1s_zbound": lambda: LTVForDana(P, B(slide_threshold=0.002), substeps=1, z_bound=True),
        "JP5_zbound": lambda: LTVForDana(P, B(), substeps=5, mode="plan", z_bound=True),
        "JP5s": lambda: LTVForDana(P, B(slide_threshold=0.002), substeps=5, mode="plan"),
        "JP5s_zbound": lambda: LTVForDana(P, B(slide_threshold=0.002), substeps=5, mode="plan", z_bound=True),
        "JP5_z1sig": lambda: LTVForDana(P, B(), substeps=5, mode="plan", z_bound=True, sigmas=1.0),
        "JP5_z2sig": lambda: LTVForDana(P, B(), substeps=5, mode="plan", z_bound=True, sigmas=2.0),
        "JP5_z0sig": lambda: LTVForDana(P, B(), substeps=5, mode="plan", z_bound=True, sigmas=0.0),
        "held2_z2sig": lambda: HeldBristleForDana(P, true_state=True, deflection=2e-6, sigmas=2.0),
        "held100_z2sig": lambda: HeldBristleForDana(P, sigmas=2.0),
        "J2_z2sig": lambda: LTVForDana(P, B(), substeps=2, z_bound=True, sigmas=2.0),
        "JP3_z2sig": lambda: LTVForDana(P, B(), substeps=3, mode="plan", z_bound=True, sigmas=2.0),
        "JP10_z2sig": lambda: LTVForDana(P, B(), substeps=10, mode="plan", z_bound=True, sigmas=2.0),
        "JP3_z2sig_meas": lambda: LTVForDana(P, B(), substeps=3, mode="plan", z_bound=True, sigmas=2.0, measured=True),
        "S2": lambda: LTVForDana(P, S(), substeps=2),
        "SP5": lambda: LTVForDana(P, S(), substeps=5, mode="plan"),
        "SP10": lambda: LTVForDana(P, S(), substeps=10, mode="plan"),
    }
    return table[name]()


# --- Dana's execute_simulation with any controller ---------------------------------------------------------
src = "".join(_a[5]["source"])
i0 = src.index("    if controller_type.lower() == 'pid':\n        ctrl = CascadedPIDController(")
i1 = src.index("    # Extract reference trajectory data")
src = src[:i0] + """    if controller_type.lower() == 'pid':
        ctrl = CascadedPIDController(
            plant_parm,
            pos_pid_gains={'kp': 1.5, 'ki': 0.1, 'kd': 0.05},
            wheel_pid_gains={'kp': 0.15, 'ki': 0.8, 'kd': 0.0},
            dt=0.02, a_max=3.5, alpha_max=7.0,
            k_theta=3.0, voltage_limit=12.0
        )
    elif controller_type.lower() == 'mpc':
        ctrl = MPCControllerSimple(
            plant_parm, horizon_K=8, dt=0.02,
            q_pose=(100.0, 100.0, 60.0), q_vel=(15.0, 15.0, 8.0),
            r_voltage=0.015, r_slew=0.008,
            voltage_limit=12.0, current_limit = 20.0, delta_v_max=5.0
            )
    else:
        ctrl = make_controller(controller_type, plant_parm)
    LAST['ctrl'] = ctrl

""" + src[i1:]
dana["make_controller"] = make
LAST = {}
dana["LAST"] = LAST
exec(src, dana)
execute_simulation = dana["execute_simulation"]


# --- paths --------------------------------------------------------------------------------------------------
def figure_eight_path(size, period, laps=2, ramp=0.3, swing=0.3, dt=0.02):
    t = np.arange(0.0, laps * period + 1.0, dt)
    rate = 2 * np.pi / period
    ph = rate * (t - ramp * (1 - np.exp(-t / ramp)))
    phd = rate * (1 - np.exp(-t / ramp))
    pos = np.column_stack((size * np.sin(ph), 0.5 * size * np.sin(2 * ph), swing * np.sin(ph)))
    vel = np.column_stack((size * np.cos(ph) * phd, size * np.cos(2 * ph) * phd, swing * np.cos(ph) * phd))
    return {'t': t, 'pos': pos, 'vel': vel}


pg = dana["PathGenerator"](dt=0.02)
seg = lambda tgt, T: pg.generate_coupled_segment(np.zeros(3), np.array(tgt), duration=T)
PATHS = {
    "dana_8s": lambda: seg([3.0, 3.0, np.pi / 4], 8.0),
    "dana_4s": lambda: seg([3.0, 3.0, np.pi / 4], 4.0),
    "short_1.2s": lambda: seg([0.5, 0.5, np.pi / 4], 1.2),
    "short_1.0s": lambda: seg([0.5, 0.5, np.pi / 4], 1.0),
    "eight_0.5m_4s": lambda: figure_eight_path(0.5, 4.0),
    "eight_0.4m_3s": lambda: figure_eight_path(0.4, 3.0),
    "eight_0.4m_2.5s": lambda: figure_eight_path(0.4, 2.5),
    "eight_0.3m_2s": lambda: figure_eight_path(0.3, 2.0),
}


def run(cname, pname, params=None):
    P = dict(dana["simulation_parameters"]) if params is None else params
    ref = PATHS[pname]()
    TRUE.clear()
    t0 = time.perf_counter()
    r = execute_simulation(P, cname, ref)
    wall = time.perf_counter() - t0
    e = np.hypot(*(r['pos'][:-1, :2] - r['pos_ref'][1:, :2]).T)      # aligned: pos[k] is at t[k+1]
    ctrl = LAST['ctrl']
    times = np.array(getattr(ctrl, 'solve_times', [])) * 1e3
    out = {
        'max_cm': 100 * e.max(), 'rms_cm': 100 * np.sqrt(np.mean(e**2)), 'final_cm': 100 * np.hypot(*(r['pos'][-1, :2] - r['pos_ref'][-1, :2])),
        'sliding_pct': 100 * np.mean(np.abs(r['vs']) > 5e-3), 'optimal_pct': 100 * np.mean(r['log_opt']),
        'step_ms_avg': 1e3 * r['elapsed'] / len(r['t']), 'wall_s': wall,
    }
    if len(times):
        out.update({'solve_ms_median': float(np.median(times)), 'solve_ms_p99': float(np.percentile(times, 99)),
                    'solve_ms_max': float(times.max()), 'over_20ms': int((times > 20).sum()), 'n_solves': int(len(times))})
    return out, r


if __name__ == "__main__":
    names = sys.argv[1].split(",")
    paths = sys.argv[2].split(",") if len(sys.argv) > 2 else list(PATHS)
    out_file = os.path.join(HERE, "suite_results.json")
    results = json.load(open(out_file)) if os.path.exists(out_file) else {}
    for pname in paths:
        for cname in names:
            try:
                res, _ = run(cname, pname)
            except Exception as ex:
                res = {'error': repr(ex)[:200]}
            results.setdefault(pname, {})[cname] = res
            if 'error' in res:
                print(f"{pname:16s} {cname:13s} ERROR {res['error']}", flush=True)
            else:
                print(f"{pname:16s} {cname:13s} max {res['max_cm']:6.1f} cm  rms {res['rms_cm']:6.1f}  slide {res['sliding_pct']:4.0f}%  "
                      f"opt {res['optimal_pct']:5.1f}%  solve med/p99/max {res.get('solve_ms_median', float('nan')):5.1f}/"
                      f"{res.get('solve_ms_p99', float('nan')):5.1f}/{res.get('solve_ms_max', float('nan')):5.1f} ms", flush=True)
            json.dump(results, open(out_file, "w"), indent=1)
