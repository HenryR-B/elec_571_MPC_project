"""
How often are the MPC's constraints active? Wraps compute_optimal_u0 to record each solve's plan, then counts the
steps where any limit is reached anywhere in the horizon (voltage +-12 V, current +-20 A, voltage change +-5 V per
step), and on the first move u0 only. Notebooks are only read. Run from dana-mpc-copy-with-estimator.
"""
import json, sys, matplotlib
import numpy as np
matplotlib.use("Agg")
c = ["".join(x["source"]) for x in json.load(open("ControllerAnalysis_Linearized.ipynb", encoding="utf-8"))["cells"]]
g = {"__name__": "__main__"}
for s in c[:18]:
    if "_results = run_and_report(" in s:
        s = s[:s.index("_results = run_and_report(")].rsplit("\n", 1)[0]     # the paths only, not the runs
    if any(k in s for k in ("path_generator =", "analysis_params", "def ", "import", "class ", "_paths = {")):
        exec(s, g)

records = []
originals = {cls: g[cls].__dict__["compute_optimal_u0"] for cls in ("MPCControllerSimple", "MPCControllerLinearized")}


def recording(self, *args, **kwargs):
    own = "MPCControllerLinearized" if isinstance(self, g["MPCControllerLinearized"]) else "MPCControllerSimple"
    out = originals[own](self, *args, **kwargs)
    U = self.U_var.value
    X = self.X_var.value
    if U is not None:
        tol = 1e-3
        du = np.diff(np.hstack((self.u_prev_param.value[:, None], U)), axis=1) if hasattr(self, "u_prev_param") else np.diff(U, axis=1)
        records.append({
            "V_any": np.any(np.abs(U) >= self.voltage_limit - tol),
            "V_u0": np.any(np.abs(U[:, 0]) >= self.voltage_limit - tol),
            "I_any": np.any(np.abs(X[6:10, 1:]) >= self.current_limit - tol),
            "dV_any": np.any(np.abs(du) >= self.delta_v_max - tol),
            "dV_u0": np.any(np.abs(du[:, 0]) >= self.delta_v_max - tol),
        })
    return out


for cls in originals:
    g[cls].compute_optimal_u0 = recording
paths = {**g["benchmark_paths"], **g["motion_paths"]}
for controller in sys.argv[1:] or ["mpc", "mpc_lin"]:
    print(f"\n{controller}")
    print(f"{'path':30s} {'any limit':>10s} {'V in plan':>10s} {'V on u0':>8s} {'dV in plan':>11s} {'dV on u0':>9s} {'I in plan':>10s}")
    for name, path in paths.items():
        records.clear()
        g["execute_simulation"](g["analysis_params"], controller, path)
        r = {k: 100 * np.mean([x[k] for x in records]) for k in records[0]}
        any_limit = 100 * np.mean([x["V_any"] or x["I_any"] or x["dV_any"] for x in records])
        print(f"{name:30s} {any_limit:9.0f}% {r['V_any']:9.0f}% {r['V_u0']:7.0f}% {r['dV_any']:10.0f}% {r['dV_u0']:8.0f}% {r['I_any']:9.0f}%")
