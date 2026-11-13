"""
Test: loads held every 1 ms (worked out from the lagged acceleration at each point, not differentiated) against the
current model (a_past a lagged state, loads differentiated). Notebooks are only read. Run from
dana-mpc-copy-with-estimator.
"""
import json
import sys
import time
import numpy as np
import matplotlib
matplotlib.use("Agg")

g = {"__name__": "__main__"}
# The analysis notebook's own loader (cell 1) and runner (cell 3)
analysis = [("".join(c["source"])) for c in json.load(open("ControllerAnalysis_Linearized.ipynb", encoding="utf-8"))["cells"]]
if len(sys.argv) > 1 and sys.argv[1] == "closed":
    exec(analysis[1], g)
    exec(analysis[3], g)
    exec(analysis[13], g)                   # helpers: figure_eight_path, run_and_report, ...
else:
    # No solver needed: only the plant and the linearized model (the solver DLLs are blocked on this PC)
    exec("import json, time; import numpy as np", g)
    plant_cells = {c.get("id"): "".join(c["source"]) for c in json.load(open("PlantModels.ipynb", encoding="utf-8"))["cells"]}
    for cell_id in ("b5584b9d", "e4823acb", "ce831979", "f56abec6", "bf71ff62"):
        exec(plant_cells[cell_id], g)
    model_cells = ["".join(c["source"]) for c in json.load(open("ControllerModels_Estimator.ipynb", encoding="utf-8"))["cells"]]
    exec(next(c for c in model_cells if "class OmniRobotLinearizedModel" in c), g)
Base = g["OmniRobotLinearizedModel"]


class HeldLoadsModel(Base):
    """
    Loads held at every point (from that point's lagged acceleration), not differentiated; the lagged acceleration
    set to the acceleration each sub-step produces (the plant's rule), not a 1 ms lag.
    """
    jac_every_default = 1

    def get_linearization_batch(self, X, U):
        bristle_decay = self.get_bristle_decay_batch(X)
        F_n = self.get_wheel_loads_batch(X)                         # from this point's a_past, held
        A_c, B_c = self.get_continuous_matrices_batch(X, U, bristle_decay, F_n)
        f_bar = self.get_state_derivative_batch(X, U, bristle_decay, F_n)
        acc = self.ACCEL_PAST
        A_c[:, acc, :] = 0.0                                        # a_past is set in the sub-step, not integrated
        B_c[:, acc, :] = 0.0
        f_bar[:, acc] = 0.0
        offset = f_bar - np.einsum('nij,nj->ni', A_c, X) - np.einsum('nij,nj->ni', B_c, U)
        return A_c, B_c, offset, bristle_decay

    def get_substep_matrices_batch(self, A_c, B_c, offset, bristle_decay, X_points, h):
        a, b, c = super().get_substep_matrices_batch(A_c, B_c, offset, bristle_decay, X_points, h)
        N, J, n = X_points.shape
        # The acceleration this sub-step produces, along the robot's axes [right, forward], linear in x_next:
        #   a_local = R(theta)^T (A_v x_next + offset_v)   (velocity rows of the linearization; voltages do not enter)
        R = self.get_rotation_matrix_batch(X_points[..., 2].reshape(-1)).reshape(N, J, 3, 3)
        R_T_rows = np.swapaxes(R, -1, -2)[:, :, 0:2, :]                       # (N, J, 2, 3)
        A_v = A_c[:, self.VELOCITY, :]                                          # (N, 3, 20)
        offset_v = offset[:, self.VELOCITY]                                     # (N, 3)
        C_a = R_T_rows @ A_v[:, None]                                           # (N, J, 2, 20)
        c_a = np.einsum('njik,nk->nji', R_T_rows, offset_v)                     # (N, J, 2)
        acc = self.ACCEL_PAST
        a[:, :, acc, :] = C_a @ a
        b[:, :, acc, :] = C_a @ b
        c[:, :, acc] = np.einsum('njik,njk->nji', C_a, c) + c_a
        return a, b, c

    def get_discrete_matrices(self, x0, u, dt, substeps, jac_every=None):
        return super().get_discrete_matrices(x0, u, dt, substeps, self.jac_every_default if jac_every is None else jac_every)

    def get_discrete_matrices_along_plan(self, X_sub, U, dt, jac_every=None):
        return super().get_discrete_matrices_along_plan(X_sub, U, dt, self.jac_every_default if jac_every is None else jac_every)


class HeldLoadsModel5ms(HeldLoadsModel):
    """The same, but the loads (with the Jacobians) only every 5 ms."""
    jac_every_default = 5


models = {"current (a_past lag, loads differentiated)": Base,
          "held loads every 1 ms": HeldLoadsModel,
          "held loads every 5 ms": HeldLoadsModel5ms}

# ---------------------------------------------------------------------------------------------------------------
# 1. 0.4 s predictions against the plant (the analysis notebook's Step 3 check)
# ---------------------------------------------------------------------------------------------------------------
P = dict(g["simulation_parameters"], sigma_V=0.0, sigma_mu=0.0)
Ts, horizon = 0.02, 20
STRAFE = np.array([-1.0, -1.0, 1.0, 1.0])
rv = np.random.default_rng(3).uniform(-8, 8, (20, 4))
manoeuvres = {"10 V strafe": (lambda t: 10 * STRAFE, 2.0),
              "4 V side-to-side": (lambda t: -4 * STRAFE * (1.0 if t < 1 else (-1.0 if t < 2 else 0.0)), 3.0),
              "random 8 V steps": (lambda t: rv[min(int(t / 0.2), 19)], 4.0)}
runs = {}
for name, (V_of_t, duration) in manoeuvres.items():
    plant = g["OmniRobotSimulationODESolve"](P)
    state = g["OmniRobotStateODESolve"]()
    states, voltages = [], []
    for k in range(int(round(duration / Ts))):
        V = V_of_t(k * Ts)
        states.append(state)
        voltages.append(V)
        for _ in range(20):
            state = plant.step(state, V)
    states.append(state)
    runs[name] = (states, voltages)

print("Position error after 0.4 s [mm], median / worst   (and the loads' error against the plant over the 0.4 s, N)")
for label, cls in models.items():
    m = cls(P)
    row = []
    for name, (states, voltages) in runs.items():
        errs = []
        for k in range(5, len(voltages) - horizon, 5):
            x = m.get_state_from_plant(states[k])
            for j in range(horizon):
                _, _, _, x = m.get_discrete_matrices(x, voltages[k + j], Ts, 20)
            errs.append(1e3 * np.hypot(*(x[:2] - states[k + horizon].pos_global[:2])))
        row.append(f"{name}: {np.median(errs):5.1f} / {np.max(errs):6.1f}")
    print(f"  {label:44s} " + " | ".join(row), flush=True)

# ---------------------------------------------------------------------------------------------------------------
# 2. Model build time along a plan (K = 8 steps, 20 sub-steps each), as the MPC calls it
# ---------------------------------------------------------------------------------------------------------------
states, voltages = runs["random 8 V steps"]
print("\nModel build along a plan, K = 8 controller steps x 20 sub-steps [ms], median / worst of 30")
for label, cls in models.items():
    m = cls(P)
    x0 = m.get_state_from_plant(states[40])
    U = np.array(voltages[40:48])
    X_sub = np.zeros((8, 20, m.n_x))
    x = x0.copy()
    for j in range(8):
        for q in range(20):
            X_sub[j, q] = x
            _, _, _, x = m.get_discrete_matrices(x, U[j], Ts / 20, 1)
    times = []
    for _ in range(30):
        t0 = time.perf_counter()
        m.get_discrete_matrices_along_plan(X_sub, U, Ts)
        times.append(1e3 * (time.perf_counter() - t0))
    print(f"  {label:44s} {np.median(times):6.2f} / {np.max(times):6.2f}", flush=True)

# ---------------------------------------------------------------------------------------------------------------
# 3. Closed loop: the linearized MPC with each model, Dana's runner (true state, as 'mpc_lin_true', and 'mpc_lin')
# ---------------------------------------------------------------------------------------------------------------
if len(sys.argv) > 1 and sys.argv[1] == "closed":
    pg = g["path_generator"]
    small_area = {'x_min': -0.5, 'x_max': 0.5, 'y_min': -0.5, 'y_max': 0.5, 'theta_min': -np.pi / 2, 'theta_max': np.pi / 2}
    paths = {
        "Dana 8 s": pg.generate_coupled_segment(np.zeros(3), np.array([3.0, 3.0, np.pi / 4]), duration=8.0),
        "Dana 4 s": pg.generate_coupled_segment(np.zeros(3), np.array([3.0, 3.0, np.pi / 4]), duration=4.0),
        "Quick hops": pg.generate_random_sequence(num_segments=6, workspace_bounds=small_area, segment_duration=1.25, random_seed=7),
        "Eight 0.4 m 2.5 s": g["figure_eight_path"](0.4, 2.5),
        "Eight 0.4 m 3.4 s": g["figure_eight_path"](0.4, 3.4),
    }
    for label, cls in models.items():
        g["OmniRobotLinearizedModel"] = cls
        print(f"\n=== {label}")
        g["run_and_report"]("", paths, ['mpc_lin'], g["analysis_params"])
    g["OmniRobotLinearizedModel"] = Base
