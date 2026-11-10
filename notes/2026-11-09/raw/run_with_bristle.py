"""
Dana's ControllerAnalysis benchmark (her plant, parameters, paths, PID and MPC), with our bristle MPC added as a
third controller through an adapter. Her notebooks are read, not changed.

Run from the elec_571_MPC_project folder (for our notebooks); Dana's folder is given below.
"""
import json
import os
import sys
import time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import osqp
from scipy import sparse
from scipy.linalg import expm

DANA = r"C:\Users\Henry\AppData\Local\Temp\claude\406281af493eb1a3\7a7d4aa2-230b-4b94-95b4-c77435123d47\scratchpad\dana_zip\extracted\Project Design Folder"
OUT = os.path.dirname(os.path.abspath(__file__))


def cells(path):
    return json.load(open(path, encoding="utf-8"))["cells"]


# ---------------------------------------------------------------------------
# Dana's classes and parameters (the cells that define things; her test and plot cells are skipped)
# ---------------------------------------------------------------------------
dana = {"__name__": "dana"}
exec("import numpy as np\nimport matplotlib.pyplot as plt", dana)
plant_nb = cells(os.path.join(DANA, "PlantModels.ipynb"))
for i in (2, 6, 9, 11, 15):
    exec("".join(plant_nb[i]["source"]), dana)
ctrl_nb = cells(os.path.join(DANA, "ControllerModels.ipynb"))
for i in (1, 4, 6, 9, 10):
    exec("".join(ctrl_nb[i]["source"]), dana)
path_nb = cells(os.path.join(DANA, "PathGeneration.ipynb"))
exec("".join(path_nb[4]["source"]), dana)
analysis_nb = cells(os.path.join(DANA, "ControllerAnalysis.ipynb"))
exec("".join(analysis_nb[2]["source"]), dana)          # CodeTimer
print("Dana's model_choice:", dana["model_choice"], "(2 = OmniRobotSimulationODESolve)")

# ---------------------------------------------------------------------------
# Our bristle MPC (its model takes geometry from our plant class, built with Dana's parameters)
# ---------------------------------------------------------------------------
ours = {"__name__": "ours", "np": np, "expm": expm, "sparse": sparse, "osqp": osqp, "time": time}
merged = {c.get("id"): "".join(c["source"]) for c in cells("RobotSimulationOpenLoopModel_Merged.ipynb")}
exec(merged["constants"], ours)
exec(merged["plant"], ours)
model_cells = {c.get("id"): "".join(c["source"]) for c in cells("MPCModel_Bristle.ipynb")}
exec(model_cells["mpc_model"], ours)
mpc_cells = {c.get("id"): "".join(c["source"]) for c in cells("RobotSimulationMPC.ipynb")}
exec(mpc_cells["mpc_params"], ours)
exec(mpc_cells["bristle_controller"], ours)


class BristleMPCForDana:
    """Our BristleMPC behind Dana's controller interface (compute_optimal_u0)."""

    def __init__(self, params, mpc_params):
        self.mpc = ours["BristleMPC"](params, mpc_params)
        self.dt = mpc_params['Ts']
        self.K = mpc_params['N']        # Dana's runner hands over K + 1 reference rows

    def compute_optimal_u0(self, measured_State, path_ref_horizon, vel_ref_horizon, return_info=False):
        measurement = np.concatenate((measured_State.pos_global, measured_State.omega))
        Ts = self.dt

        def reference(t):
            # Our controller asks for the reference at t = Ts (j + 1); Dana's rows start at now
            j = int(round(t / Ts))
            pose = path_ref_horizon[j]
            c, s = np.cos(pose[2]), np.sin(pose[2])
            v_local = vel_ref_horizon[j]
            v_global = np.array([c * v_local[0] - s * v_local[1], s * v_local[0] + c * v_local[1], v_local[2]])
            return np.concatenate((pose, v_global))

        failures_before = self.mpc.failures
        u = self.mpc.step(0.0, measurement, reference)
        info = {'optimal': self.mpc.failures == failures_before}
        return (u, info) if return_info else u


# ---------------------------------------------------------------------------
# Dana's execute_simulation, with one more branch for the bristle MPC
# ---------------------------------------------------------------------------
src = "".join(analysis_nb[5]["source"])
old = """    else:
        ctrl = MPCControllerSimple("""
new = """    elif controller_type.lower() == 'bristle':
        ctrl = BristleMPCForDana(plant_parm, mpc_parameters)
    else:
        ctrl = MPCControllerSimple("""
assert src.count(old) == 1
src = src.replace(old, new)
dana["BristleMPCForDana"] = BristleMPCForDana
dana["mpc_parameters"] = ours["mpc_parameters"]
exec(src, dana)
execute_simulation = dana["execute_simulation"]

# ---------------------------------------------------------------------------
# Dana's benchmark: the same two paths
# ---------------------------------------------------------------------------
sim_params = dana["simulation_parameters"].copy()
pg = dana["PathGenerator"](dt=0.02)
start_pose = np.array([0.0, 0.0, 0.0])
target_pose = np.array([3.0, 3.0, np.pi / 4.0])
paths = {"T = 8 s": pg.generate_coupled_segment(start_pose, target_pose, duration=8.0),
         "T = 4 s": pg.generate_coupled_segment(start_pose, target_pose, duration=4.0)}

results = {}
for pname, ref in paths.items():
    for cname in ("pid", "mpc", "bristle"):
        t0 = time.perf_counter()
        results[(pname, cname)] = execute_simulation(sim_params, cname, ref)
        print(f"  ran {cname:8s} {pname}: {time.perf_counter() - t0:.1f} s wall")

label = {"pid": "PID", "mpc": "Dana's MPC", "bristle": "Bristle MPC"}
print()
print(f"{'':28}" + "".join(f"{label[c] + ' ' + p:>24}" for p in paths for c in ("pid", "mpc", "bristle")))
rows = [
    ("Final position error (cm)", lambda r: r['dist_err'][-1] * 100),
    ("Max position error (cm)", lambda r: np.max(r['dist_err']) * 100),
    ("RMS position error (cm)", lambda r: np.sqrt(np.mean(r['dist_err'] ** 2)) * 100),
    ("Final heading error (deg)", lambda r: np.degrees(r['th_err'][-1])),
    ("Peak voltage (V)", lambda r: np.max(np.abs(r['V']))),
    ("Peak slip (mm/s)", lambda r: np.max(np.abs(r['vs'])) * 1000),
    ("Optimal solutions (%)", lambda r: 100 * np.mean(r['log_opt'])),
    ("Controller time/step (ms)", lambda r: r['elapsed'] / len(r['t']) * 1000),
]
for name, f in rows:
    print(f"{name:28}" + "".join(f"{f(results[(p, c)]):24.2f}" for p in paths for c in ("pid", "mpc", "bristle")))

# Figure: paths and position errors
colors = {"pid": "royalblue", "mpc": "darkorange", "bristle": "seagreen"}
fig, axs = plt.subplots(2, 2, figsize=(14, 12))
for col, pname in enumerate(paths):
    ax = axs[0, col]
    ref = results[(pname, "pid")]['pos_ref']
    ax.plot(ref[:, 0], ref[:, 1], 'k:', lw=1.8, label='Reference')
    for cname in ("pid", "mpc", "bristle"):
        pos = results[(pname, cname)]['pos']
        ax.plot(pos[:, 0], pos[:, 1], color=colors[cname], lw=2, label=label[cname])
    ax.set_aspect('equal', 'datalim'); ax.grid(True); ax.legend(); ax.set_title(f"Path, {pname}")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax = axs[1, col]
    for cname in ("pid", "mpc", "bristle"):
        r = results[(pname, cname)]
        ax.plot(r['t'], r['dist_err'] * 100, color=colors[cname], lw=2, label=label[cname])
    ax.grid(True); ax.legend(); ax.set_title(f"Position error, {pname}")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Position error (cm)")
plt.tight_layout()
fig.savefig(os.path.join(OUT, "dana_benchmark_with_bristle.png"), dpi=70)
print("figure saved")
