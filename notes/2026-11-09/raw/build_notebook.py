"""Builds MPCModel_Jacobian.ipynb from the tested development code. Run from the project folder."""
import json

cells = []


def md(cid, text):
    cells.append({"cell_type": "markdown", "id": cid, "metadata": {}, "source": text.strip("\n").splitlines(keepends=True)})


def code(cid, text):
    cells.append({"cell_type": "code", "id": cid, "metadata": {}, "execution_count": None, "outputs": [],
                  "source": text.strip("\n").splitlines(keepends=True)})


md("header", r"""
# MPC models linearized with Jacobians, compared in Dana's test suite

The held bristle model (`MPCModel_Bristle.ipynb`) holds five values over each 20 ms step and uses a softer bristle. This notebook builds MPC models that are **as close to the plant as we could make them while staying linear for the QP**, and runs every one of them in Dana's `ControllerAnalysis` setup (her plant, her parameters, her path generator, her PID and her MPC as the baselines).

**Result:** `JP3` (bristle model, exact Jacobians, 3 sub-steps per 20 ms step, linearized along the last plan, friction bound at 2 standard deviations) is the best controller on almost every path, and solves within the 20 ms budget (median about 3-7 ms, worst under 15 ms with the OSQP iteration cap of 300). It works with only the measured pose, wheel speeds and currents.

Contents:
1. Loading the plant, Dana's notebooks (read straight from her zip) and our held bristle MPC
2. What the held model simplifies, and what changed
3. The models
4. The MPC
5. Dana's test suite with every model plugged in
6. Model accuracy: sub-steps
7. Results
""")

code("load", r'''
# ===========================================================================
# The plant, Dana's notebooks and our held bristle MPC
# ===========================================================================
import json
import time
import zipfile
import numpy as np
import matplotlib.pyplot as plt
import osqp
from scipy import sparse
from scipy.linalg import expm


def notebook_cells(nb):
    return {c.get("id"): "".join(c["source"]) for c in nb["cells"]}


# Our plant (frozen) and our held bristle MPC, as the baseline
plant = {}
merged = notebook_cells(json.load(open("RobotSimulationOpenLoopModel_Merged.ipynb", encoding="utf-8")))
exec(merged["constants"], plant)
exec(merged["plant"], plant)
simulation_parameters = plant["simulation_parameters"]
OmniRobotSimulation = plant["OmniRobotSimulation"]
OmniMotorState = plant["OmniMotorState"]

held = {"np": np, "expm": expm, "sparse": sparse, "osqp": osqp, "time": time, "OmniRobotSimulation": OmniRobotSimulation}
exec(notebook_cells(json.load(open("MPCModel_Bristle.ipynb", encoding="utf-8")))["mpc_model"], held)
mpc_notebook = notebook_cells(json.load(open("RobotSimulationMPC.ipynb", encoding="utf-8")))
exec(mpc_notebook["mpc_params"], held)
exec(mpc_notebook["bristle_controller"], held)
BristleMPC = held["BristleMPC"]                # the held bristle MPC (100 um, values held over each step)
mpc_parameters = held["mpc_parameters"]

# Dana's notebooks, read from her zip (nothing extracted, nothing changed). Only the cells that define things run.
DANA_ZIP = "dana-models/Project Design Folder.zip"
with zipfile.ZipFile(DANA_ZIP) as z:
    dana_nb = {name.split("/")[-1]: json.load(z.open(name)) for name in z.namelist() if name.endswith(".ipynb")}
dana = {}
exec("import numpy as np", dana)
code_of = lambda name, i: "".join(dana_nb[name]["cells"][i]["source"])
for i in (2, 6, 9, 11, 15):
    exec(code_of("PlantModels.ipynb", i), dana)            # her plant classes, parameters, model_choice
for i in (1, 4, 6, 9, 10):
    exec(code_of("ControllerModels.ipynb", i), dana)       # her PID and MPC
exec(code_of("PathGeneration.ipynb", 4), dana)             # her path generator
exec(code_of("ControllerAnalysis.ipynb", 2), dana)         # her timer
dana_parameters = dana["simulation_parameters"]
DanaPlant = dana["OmniRobotSimulationODESolve"]            # her model_choice = 2: the plant with the bristle solve
PathGenerator = dana["PathGenerator"]
print(f"Loaded our plant, the held bristle MPC and Dana's notebooks (model_choice = {dana['model_choice']})")
''')

md("simplifications_md", r"""
## What the held model simplifies, and what changed

| | Plant | Held model (`MPCModel_Bristle`) | Jacobian models here |
|---|---|---|---|
| Bristle | 2 µm | 100 µm | 2 µm (the plant's) |
| Motor current | with inductance | settled at once | with inductance (4 more states) |
| Relax rate $d$ (from the slip) | changes every 1 ms | held per 20 ms step | held at each **sub-step** point |
| Loads $F_n$ (load transfer) | from the acceleration | held per 20 ms step | held at each sub-step point |
| Roller friction, rolling drag, turning, $R(\theta)$ | exact | held per 20 ms step | **exact Jacobian** |
| How often it is linearized | | every 20 ms | every **sub-step** (20 ms / 3), chained into one 20 ms model for the QP |

**Why $d$ and $F_n$ stay held (at each linearization point) instead of differentiated:**
* $d = \frac{\mu_s}{\mu_d}\sqrt{v_s^2 + v_c^2}$ has a sharp corner at zero slip (the $|v_s|$ in $\mu_d$, on a scale of $v_c$ = 0.1 mm/s). Its derivative there is meaningless over a sub-step and made the linearization blow up (eigenvalues up to +390 /s).
* Differentiating the same-step load transfer gives a lopsided wheel-to-wheel stiffness and a fast unstable flutter (about 816 Hz). The plant does not show it: its loads use the previous step's acceleration and its backward Euler damps anything that fast.

**What mattered most, from the tests below:**
1. **Sub-steps.** Linearizing only every 20 ms misses a wheel switching between gripping and sliding inside the step. Prediction error over 0.4 s falls steadily with more sub-steps.
2. **The friction bound $|z| \le z_{max}$** acts as a trust region: a linear model says more slip gives more force, so without the bound the MPC spins the wheels (85-95% sliding). With $z_{max} = (\mu_s - 2\sigma_\mu)/\mu_s$ the wheels slide a few percent of the time and tracking is best. (3 standard deviations, 0.57 with Dana's $\sigma_\mu$ = 0.1, was too cautious.)
3. **Linearizing along the last plan, in one batch** (instead of a step-by-step rollout) makes the sub-steps affordable: all 60 sub-step Jacobians in one call.
4. **OSQP iteration cap of 300** bounds the worst case without losing accuracy.

Tried and dropped: the true Stribeck slope of $d$ while sliding (no help on its own), friction as a function of slip, F($v_s$) (good on easy paths, but slides 60-95% and loses badly on hard ones), more sub-steps (5, 10: no better than 3, slower).
""")

code("models", r'''
"""
The MPC models
==============

Each model is dx/dt = f(x, u). Its Jacobians come from central differences, all perturbed points in one batched call.
Each sub-step is discretized with backward Euler, (I - h A)^-1, like the plant's own step, and the sub-steps of a
20 ms controller step are chained (chain rule) into one model x_next = A_d x + B_d u + c_d for the QP.

    BristleJacobianModel   x = [pose, v, omega, I, z] (18): the plant's equations
    SlipFrictionModel      x = [pose, v, omega, I] (14): friction a function of slip, no bristle (tried, see results)
"""


class JacobianModel:
    """Shared parts: geometry and loads (from the plant class), Jacobians, sub-step maps, rollouts."""

    # Where each part of the state sits in x
    POSE = slice(0, 3)          # [X, Y, theta]
    VELOCITY = slice(3, 6)      # v = [v_x, v_y, theta_dot], along the robot's axes
    WHEELS = slice(6, 10)       # omega_1..4
    CURRENTS = slice(10, 14)    # I_1..4
    n_u = 4
    z_index = None              # where z sits in x, for models with bristles

    def __init__(self, params, plant_class):
        self.p = params
        geometry = plant_class(params)          # fixed values only, never a state
        self.H_drive = geometry.H_drive
        self.H_axial = geometry.H_axial
        self.M_inv = np.diag(geometry.M_inv)
        self.F_n_static = np.array([geometry.F_n_front_static, geometry.F_n_front_static,
                                    geometry.F_n_rear_static, geometry.F_n_rear_static])
        pitch = geometry.pitch_gain
        roll_front = geometry.roll_gain_front
        roll_rear = geometry.roll_gain_rear
        self.load_transfer = np.array([
            [-roll_front, -pitch, 0.0],     # 1 FR
            [roll_front, -pitch, 0.0],      # 2 FL
            [roll_rear, pitch, 0.0],        # 3 RL
            [-roll_rear, pitch, 0.0],       # 4 RR
        ])

    # -----------------------------------------------------------------------
    # Physics shared by the models (batched: one row per point)
    # -----------------------------------------------------------------------

    def roller_factor(self, v):
        """Roller force per unit normal force, from the axle slip v_slip_a = H_axial v (Step 5.8)."""
        p = self.p
        if not p['roller_friction']:
            return np.zeros((len(v), 4))
        v_slip_a = v @ self.H_axial.T
        mu_a = p['mu_a_k'] + (p['mu_a_s'] - p['mu_a_k']) * np.exp(-p['alpha_a'] * np.abs(v_slip_a))
        return -mu_a * (2.0 / np.pi) * np.arctan(p['roller_smoothing'] * v_slip_a)

    def loads(self, drive_factor, axial_factor):
        """
        Normal forces with this state's acceleration, when the floor forces are a factor times each load:
        F = drive_factor F_n, F_a = axial_factor F_n, so (I - load_transfer M^-1 K) F_n = F_n_static.
        """
        n_points = len(drive_factor)
        K = self.H_drive.T[None] * drive_factor[:, None, :] + self.H_axial.T[None] * axial_factor[:, None, :]
        mat = np.eye(4)[None] - self.load_transfer[None] @ (self.M_inv[None, :, None] * K)
        rhs = np.broadcast_to(self.F_n_static, (n_points, 4))[..., None]
        return np.maximum(0.0, np.linalg.solve(mat, rhs)[..., 0])

    def chassis_and_motor(self, X, U, F, F_a, F_n):
        """Rates of the pose, velocity, wheel speeds and currents from the floor and roller forces."""
        p = self.p
        theta = X[:, 2]
        v = X[:, self.VELOCITY]
        omega = X[:, self.WHEELS]
        I = X[:, self.CURRENTS]

        # Pose: d[X, Y, theta]/dt = R(theta) v
        pose_dot = np.column_stack((np.cos(theta) * v[:, 0] - np.sin(theta) * v[:, 1],
                                    np.sin(theta) * v[:, 0] + np.cos(theta) * v[:, 1],
                                    v[:, 2]))

        # Chassis, along the robot's axes, plus the turning of the axes (theta_dot v_y, -theta_dot v_x)
        v_dot = self.M_inv * (F @ self.H_drive + F_a @ self.H_axial)
        v_dot[:, 0] += v[:, 2] * v[:, 1]
        v_dot[:, 1] -= v[:, 2] * v[:, 0]

        # Wheels and motors (Steps 5.3 and 5.7)
        T_roll = p['Crr'] * F_n * p['r'] * np.tanh(omega / p['roll_smoothing'])
        omega_dot = (p['Kt'] * I - T_roll - p['r'] * F) / p['Jw']
        I_dot = (U - p['R'] * I - p['Kb'] * omega) / p['Lm']
        return pose_dot, v_dot, omega_dot, I_dot

    # -----------------------------------------------------------------------
    # Linearization
    # -----------------------------------------------------------------------

    def jacobians(self, X, U):
        """
        A = df/dx and B = df/du at a batch of points, by central differences; the held values come from held(X) at
        each point and stay fixed while x and u are moved. Returns (A, B, f at the points).
        """
        n, m = self.n_x, self.n_u
        n_points = len(X)
        k = 1 + 2 * (n + m)                       # the point itself, then +h and -h in each x and each u
        h_x = 1e-6 * (1.0 + np.abs(X))
        h_u = 1e-6 * (1.0 + np.abs(U))
        X_all = np.repeat(X[:, None, :], k, axis=1)
        U_all = np.repeat(U[:, None, :], k, axis=1)
        i, j = np.arange(n), np.arange(m)
        X_all[:, 1 + i, i] += h_x
        X_all[:, 1 + n + i, i] -= h_x
        U_all[:, 1 + 2 * n + j, j] += h_u
        U_all[:, 1 + 2 * n + m + j, j] -= h_u
        held_values = {key: np.repeat(value[:, None, :], k, axis=1).reshape(n_points * k, -1)
                       for key, value in self.held(X).items()}
        f_all = self.f(X_all.reshape(-1, n), U_all.reshape(-1, m), held_values).reshape(n_points, k, n)

        A = ((f_all[:, 1:1 + n] - f_all[:, 1 + n:1 + 2 * n]) / (2 * h_x[:, :, None])).transpose(0, 2, 1)
        B = ((f_all[:, 1 + 2 * n:1 + 2 * n + m] - f_all[:, 1 + 2 * n + m:]) / (2 * h_u[:, :, None])).transpose(0, 2, 1)
        return A, B, f_all[:, 0]

    def substep_maps(self, X, U, h):
        """
        One backward-Euler sub-step of the linearization at each point: x+ = a x + b u + c.
        From x+ = x + h (A x+ + B u + c0) with c0 = f - A x - B u:   a = (I - hA)^-1,  b = h a B,  c = h a c0.
        """
        n = self.n_x
        A, B, f0 = self.jacobians(X, U)
        c0 = f0 - np.einsum('pij,pj->pi', A, X) - np.einsum('pij,pj->pi', B, U)
        a = np.linalg.inv(np.eye(n)[None] - h * A)
        return a, h * a @ B, h * np.einsum('pij,pj->pi', a, c0)

    def rollout(self, x0, U, Ts, substeps):
        """Step the model from x0 with U, relinearizing at every sub-step. Returns the points at each controller step,
        one chained model per controller step (A_d, B_d, c_d), and the sub-step points."""
        n, m = self.n_x, self.n_u
        N = len(U)
        X_bar = np.zeros((N + 1, n))
        X_bar[0] = x0
        X_sub = np.zeros((N, substeps, n))
        A_d, B_d, c_d = np.zeros((N, n, n)), np.zeros((N, n, m)), np.zeros((N, n))
        x = x0.copy()
        for j in range(N):
            A_chain, B_chain, c_chain = np.eye(n), np.zeros((n, m)), np.zeros(n)
            for q in range(substeps):
                X_sub[j, q] = x
                a, b, c = (item[0] for item in self.substep_maps(x[None], U[j][None], Ts / substeps))
                x = a @ x + b @ U[j] + c
                A_chain, B_chain, c_chain = a @ A_chain, a @ B_chain + b, a @ c_chain + c
            A_d[j], B_d[j], c_d[j] = A_chain, B_chain, c_chain
            X_bar[j + 1] = x
        return X_bar, A_d, B_d, c_d, X_sub

    def along_points(self, X_sub, U, Ts):
        """One chained model per controller step, linearized at given sub-step points, all in one batch.
        X_sub: (N, substeps, n). Returns (A_d, B_d, c_d) and the sub-step maps (for the next step's points)."""
        N, S, n = X_sub.shape
        m = self.n_u
        a, b, c = self.substep_maps(X_sub.reshape(-1, n), np.repeat(U, S, axis=0), Ts / S)
        a, b, c = a.reshape(N, S, n, n), b.reshape(N, S, n, m), c.reshape(N, S, n)
        A_d = np.broadcast_to(np.eye(n), (N, n, n)).copy()
        B_d, c_d = np.zeros((N, n, m)), np.zeros((N, n))
        for q in range(S):                         # chain rule over the sub-steps
            A_d = a[:, q] @ A_d
            B_d = a[:, q] @ B_d + b[:, q]
            c_d = np.einsum('nij,nj->ni', a[:, q], c_d) + c[:, q]
        return A_d, B_d, c_d, (a, b, c)


class BristleJacobianModel(JacobianModel):
    """The plant's equations with the bristle state: x = [pose, v, omega, I, z] (18)."""
    BRISTLES = slice(14, 18)    # z_1..4
    n_x = 18
    z_index = np.arange(14, 18)

    def __init__(self, params, plant_class, deflection=None):
        super().__init__(params, plant_class)
        self.delta = params['lugre_deflection'] if deflection is None else deflection

    def slip(self, X):
        """Drive slip of each wheel: v_slip_d = H_drive v - r omega."""
        return X[:, self.VELOCITY] @ self.H_drive.T - self.p['r'] * X[:, self.WHEELS]

    def held(self, X):
        """The bristle relax rate d and the loads F_n at each linearization point (held: see the notes above)."""
        p = self.p
        v_slip_d = self.slip(X)
        mu_d = p['mu_k'] + (p['mu_s'] - p['mu_k']) * np.exp(-p['alpha_d'] * np.abs(v_slip_d))
        d = (p['mu_s'] / mu_d) * np.sqrt(v_slip_d**2 + p['lugre_creep']**2)
        F_n = self.loads(-p['mu_s'] * X[:, self.BRISTLES], self.roller_factor(X[:, self.VELOCITY]))
        return {'d': d, 'F_n': F_n}

    def f(self, X, U, held_values):
        p = self.p
        z = X[:, self.BRISTLES]
        F_n = held_values['F_n']
        d = held_values['d']
        F = -p['mu_s'] * F_n * z                                  # Step 5.6
        F_a = self.roller_factor(X[:, self.VELOCITY]) * F_n
        pose_dot, v_dot, omega_dot, I_dot = self.chassis_and_motor(X, U, F, F_a, F_n)
        z_dot = (self.slip(X) - d * z) / self.delta               # Step 5.5
        return np.hstack((pose_dot, v_dot, omega_dot, I_dot, z_dot))

    def from_plant(self, state):
        return np.concatenate((state.pos_global, state.v_local, state.omega, state.I, state.z))


class SlipFrictionModel(JacobianModel):
    """Friction a function of slip, F = -mu_d(v_s) F_n (2/pi) atan(v_s / v_eps), no bristle: x = [pose, v, omega, I]."""
    n_x = 14

    def __init__(self, params, plant_class, v_eps=0.005):
        super().__init__(params, plant_class)
        self.v_eps = v_eps

    def drive_factor(self, X):
        p = self.p
        v_slip_d = X[:, self.VELOCITY] @ self.H_drive.T - p['r'] * X[:, self.WHEELS]
        mu_d = p['mu_k'] + (p['mu_s'] - p['mu_k']) * np.exp(-p['alpha_d'] * np.abs(v_slip_d))
        return -mu_d * (2.0 / np.pi) * np.arctan(v_slip_d / self.v_eps)

    def held(self, X):
        return {'F_n': self.loads(self.drive_factor(X), self.roller_factor(X[:, self.VELOCITY]))}

    def f(self, X, U, held_values):
        F_n = held_values['F_n']
        F = self.drive_factor(X) * F_n
        F_a = self.roller_factor(X[:, self.VELOCITY]) * F_n
        return np.hstack(self.chassis_and_motor(X, U, F, F_a, F_n))

    def from_plant(self, state):
        return np.concatenate((state.pos_global, state.v_local, state.omega, state.I))
''')

code("mpc", r'''
"""
The MPC
=======

LTVMPC works with any JacobianModel. Every controller step:
    1. linearize: a rollout from the estimate (mode "rollout"), or (mode "plan") the last plan's sub-step points moved
       one step on, with all sub-step Jacobians in one batch
    2. one QP (OSQP): pose and velocity tracking, voltage changes weighted, |u| <= V_max, and for bristle models
       the friction bound |z| <= z_max, z_max = (mu_s - deviation_sigmas sigma_mu) / mu_s
"""


class LTVMPC:

    def __init__(self, params, mpc_params, model, substeps=3, mode="plan", z_bound=True):
        self.p = params
        self.mp = mpc_params
        self.model = model
        self.Ts = mpc_params['Ts']
        self.N = mpc_params['N']
        self.substeps = substeps
        self.mode = mode
        n, m, N = model.n_x, model.n_u, self.N
        self.n, self.m = n, m
        self.z_bound = z_bound and model.z_index is not None
        self.z_max = (params['mu_s'] - mpc_params['deviation_sigmas'] * params['sigma_mu']) / params['mu_s']
        self.n_x, self.n_u = n * (N + 1), m * N
        x_at = lambda j: j * n
        u_at = lambda j: self.n_x + j * m

        # Cost: pose and velocity at steps 1..N (the last step weighted more), and voltage changes
        self.q_track = np.zeros((N, n))
        for j in range(N):
            weight = mpc_params['q_terminal'] if j == N - 1 else 1.0
            self.q_track[j, 0:3] = weight * np.array(mpc_params['q_pos'])
            self.q_track[j, 3:6] = weight * np.array(mpc_params['q_vel'])
        du = np.zeros((N, N))
        for j in range(N):
            du[j, j] += 1.0
            if j > 0:
                du[j - 1, j - 1] += 1.0
                du[j, j - 1] -= 1.0
                du[j - 1, j] -= 1.0
        P = sparse.block_diag((sparse.diags(np.concatenate((np.zeros(n), 2.0 * self.q_track.ravel()))),
                               sparse.kron(2.0 * mpc_params['r_du'] * du, sparse.eye(m))), format='csc')

        # Constraint rows: x_0 = x_hat, x_{j+1} - A_d x_j - B_d u_j = c_d, |u| <= V_max, |z_j| <= z_max
        rows, cols = [], []

        def block(r0, c0, height, width):
            r, c = np.meshgrid(np.arange(height), np.arange(width), indexing='ij')
            rows.append((r0 + r).ravel())
            cols.append((c0 + c).ravel())

        row = 0
        block(row, x_at(0), n, n)
        row += n
        for j in range(N):
            block(row, x_at(j), n, n)
            block(row, x_at(j + 1), n, n)
            block(row, u_at(j), n, m)
            row += n
        rows.append(row + np.arange(self.n_u))
        cols.append(self.n_x + np.arange(self.n_u))
        row += self.n_u
        if self.z_bound:
            for j in range(1, N + 1):
                rows.append(row + np.arange(4))
                cols.append(x_at(j) + model.z_index)
                row += 4
        rows, cols = np.concatenate(rows), np.concatenate(cols)
        self.n_rows = row
        self.order = np.lexsort((rows, cols))
        indptr = np.concatenate(([0], np.cumsum(np.bincount(cols, minlength=self.n_x + self.n_u))))
        A_start = sparse.csc_matrix((self._values(np.tile(np.eye(n), (N, 1, 1)), np.zeros((N, n, m))),
                                     rows[self.order], indptr), shape=(self.n_rows, self.n_x + self.n_u))
        self.solver = osqp.OSQP()
        self.solver.setup(P, np.zeros(self.n_x + self.n_u), A_start, -np.ones(self.n_rows), np.ones(self.n_rows),
                          verbose=False, warm_starting=True, polishing=False, eps_abs=1e-4, eps_rel=1e-4,
                          max_iter=mpc_params['max_iterations'])

        # Memory between steps
        self.u_applied = np.zeros(m)
        self.X_plan = self.U_plan = None
        self.sub_maps = None
        self.failures = 0
        self.solve_times = []

    def _values(self, A_d, B_d):
        """The constraint matrix's values, in the solver's order."""
        n, N = self.n, self.N
        parts = [np.eye(n).ravel()]
        for j in range(N):
            parts += [(-A_d[j]).ravel(), np.eye(n).ravel(), (-B_d[j]).ravel()]
        parts.append(np.ones(self.n_u))
        if self.z_bound:
            parts.append(np.ones(4 * N))
        return np.concatenate(parts)[self.order]

    def _target(self, ref_rows, theta_now):
        """ref_rows (N, 6): [X, Y, theta, x_dot, y_dot, theta_dot] (room frame) at steps 1..N. Velocities go into the
        robot's axes; the heading is unwrapped next to the current one."""
        ref = ref_rows.copy()
        ref[:, 2] = np.unwrap(ref[:, 2])
        ref[:, 2] -= 2 * np.pi * np.round((ref[0, 2] - theta_now) / (2 * np.pi))
        target = np.zeros((self.N, self.n))
        target[:, 0:3] = ref[:, 0:3]
        cos_ref, sin_ref = np.cos(ref[:, 2]), np.sin(ref[:, 2])
        target[:, 3] = cos_ref * ref[:, 3] + sin_ref * ref[:, 4]
        target[:, 4] = -sin_ref * ref[:, 3] + cos_ref * ref[:, 4]
        target[:, 5] = ref[:, 5]
        return target

    def step(self, x_hat, ref_rows):
        """One controller step: the voltages to hold until the next step [V]."""
        start = time.perf_counter()
        N, n, m, Ts, S = self.N, self.n, self.m, self.Ts, self.substeps
        target = self._target(ref_rows, x_hat[2])
        q = np.concatenate((np.zeros(n), -2.0 * (self.q_track * target).ravel(), np.zeros(self.n_u)))
        q[self.n_x:self.n_x + m] = -2.0 * self.mp['r_du'] * self.u_applied
        U_guess = (np.tile(self.u_applied, (N, 1)) if self.U_plan is None
                   else np.vstack((self.U_plan[1:], self.U_plan[-1:])))

        # 1. Linearize
        if self.mode == "rollout":
            _, A_d, B_d, c_d, _ = self.model.rollout(x_hat, U_guess, Ts, S)
        elif self.sub_maps is None:
            # First step (or after a failed solve): a one-sub-step rollout gives the points, then one batch
            X_bar = self.model.rollout(x_hat, U_guess, Ts, 1)[0]
            X_sub = np.repeat(X_bar[:-1, None, :], S, axis=1)
            A_d, B_d, c_d, self.sub_maps = self.model.along_points(X_sub, U_guess, Ts)
        else:
            # The last sub-step maps, moved one step on, carry the estimate along the guess to give the new points
            a, b, c = (np.concatenate((arr[1:], arr[-1:])) for arr in self.sub_maps)
            X_sub = np.zeros((N, S, n))
            x = x_hat.copy()
            for j in range(N):
                for qq in range(S):
                    X_sub[j, qq] = x
                    x = a[j, qq] @ x + b[j, qq] @ U_guess[j] + c[j, qq]
            A_d, B_d, c_d, self.sub_maps = self.model.along_points(X_sub, U_guess, Ts)

        # 2. Solve
        lower = [x_hat] + list(c_d) + [-self.p['V_max'] * np.ones(self.n_u)]
        upper = [x_hat] + list(c_d) + [self.p['V_max'] * np.ones(self.n_u)]
        if self.z_bound:
            lower.append(np.full(4 * N, -self.z_max))
            upper.append(np.full(4 * N, self.z_max))
        self.solver.update(q=q, l=np.concatenate(lower), u=np.concatenate(upper), Ax=self._values(A_d, B_d))
        if self.X_plan is not None:
            X_warm = np.vstack((x_hat, self.X_plan[2:], self.X_plan[-1:]))
            self.solver.warm_start(x=np.concatenate((X_warm.ravel(), U_guess.ravel())))
        result = self.solver.solve()
        if result.info.status in ('solved', 'solved inaccurate', 'maximum iterations reached') and np.all(np.isfinite(result.x)):
            self.X_plan = result.x[:self.n_x].reshape(N + 1, n)
            self.U_plan = result.x[self.n_x:].reshape(N, m)
            self.u_applied = np.clip(self.U_plan[0], -self.p['V_max'], self.p['V_max'])
        else:
            self.failures += 1
            self.X_plan = self.U_plan = self.sub_maps = None
        self.solve_times.append(time.perf_counter() - start)
        return self.u_applied.copy()


# Settings for the Jacobian MPCs: the held MPC's weights, the bound at 2 standard deviations, OSQP capped at 300
jacobian_mpc_parameters = dict(mpc_parameters, deviation_sigmas=2.0, max_iterations=300)
''')

md("suite_md", r"""
## Dana's test suite with every model plugged in

Her `execute_simulation` runs unchanged except for one line that builds the controller: `'pid'` and `'mpc'` are hers, and any other name comes from `make_controller` below. Each of our MPCs is wrapped in her controller interface, `compute_optimal_u0(observable_state, pose horizon, velocity horizon)`.

**Estimation.** `_meas` controllers see only Dana's observable state (pose, wheel speeds, currents): the velocity is the model's prediction corrected by the pose error, and $z$ comes from each wheel's torque balance, $F = (K_t I - T_{roll} - J_w \dot\omega)/r$, $z = -F/(\mu_s F_n)$. The others are given the plant's true state (an ideal observer), to compare the models alone.

**Error** is measured with the reference one controller step later, because Dana's runner logs the pose after stepping the plant (comparing with the reference before the step adds speed x 20 ms, about 2 cm at 1 m/s).
""")

code("suite", r'''
# ===========================================================================
# Our MPCs in Dana's runner
# ===========================================================================
TRUE = {}                                   # the plant's latest true state, for the ideal-observer controllers


class RecordingDanaPlant(DanaPlant):
    """Dana's plant, keeping its latest state (the ideal-observer controllers read it; the plant is unchanged)."""
    def step(self, state, V):
        next_state = super().step(state, V)
        TRUE['state'] = next_state
        return next_state


dana["OmniRobotSimulationODESolve"] = RecordingDanaPlant


def reference_rows(path_ref_horizon, vel_ref_horizon, N):
    """Dana's horizon (rows from now: pose, velocity along the reference's axes) as our rows for steps 1..N."""
    rows = np.zeros((N, 6))
    for j in range(N):
        pose = path_ref_horizon[j + 1]
        v_local = vel_ref_horizon[j + 1]
        cos_ref, sin_ref = np.cos(pose[2]), np.sin(pose[2])
        rows[j] = np.concatenate((pose, [cos_ref * v_local[0] - sin_ref * v_local[1],
                                         sin_ref * v_local[0] + cos_ref * v_local[1], v_local[2]]))
    return rows


class JacobianMPCForDana:
    """An LTVMPC behind Dana's controller interface."""

    def __init__(self, params, model, measured=True, **mpc_options):
        self.p = params
        self.model = model
        self.mpc = LTVMPC(params, jacobian_mpc_parameters, model, **mpc_options)
        self.dt, self.K = jacobian_mpc_parameters['Ts'], jacobian_mpc_parameters['N']
        self.measured = measured
        self.omega_prev = None

    def estimate(self, observed):
        """The state from the pose, wheel speeds and currents only (bristle models)."""
        p, Ts = self.p, self.dt
        pose, omega, I = observed.pos_global.copy(), observed.omega.copy(), observed.I.copy()
        if self.mpc.X_plan is None:
            v, z = np.zeros(3), np.zeros(4)
        else:
            predicted = self.mpc.X_plan[1]
            error = pose - predicted[0:3]
            error[2] = (error[2] + np.pi) % (2 * np.pi) - np.pi
            cos_th, sin_th = np.cos(pose[2]), np.sin(pose[2])
            v = predicted[3:6] + np.array([cos_th * error[0] + sin_th * error[1],
                                           -sin_th * error[0] + cos_th * error[1], error[2]]) / Ts
            z = predicted[14:18].copy()
        x = np.concatenate((pose, v, omega, I, z))
        if self.omega_prev is not None:
            F_n = self.model.held(x[None])['F_n'][0]
            T_roll = p['Crr'] * F_n * p['r'] * np.tanh(omega / p['roll_smoothing'])
            F = (p['Kt'] * I - T_roll - p['Jw'] * (omega - self.omega_prev) / Ts) / p['r']
            x[14:18] = np.clip(-F / (p['mu_s'] * np.maximum(F_n, 1e-3)), -1.2, 1.2)
        self.omega_prev = omega
        return x

    def compute_optimal_u0(self, measured_State, path_ref_horizon, vel_ref_horizon, return_info=False):
        if self.measured:
            x_hat = self.estimate(measured_State)
        else:
            state = TRUE.get('state')
            x_hat = self.model.from_plant(state) if state is not None else np.zeros(self.model.n_x)
        failures_before = self.mpc.failures
        u = self.mpc.step(x_hat, reference_rows(path_ref_horizon, vel_ref_horizon, self.K))
        info = {'optimal': self.mpc.failures == failures_before}
        return (u, info) if return_info else u

    @property
    def solve_times(self):
        return self.mpc.solve_times


class HeldMPCForDana:
    """The held bristle MPC (BristleMPC, its own estimate from pose and wheel speeds) behind Dana's interface."""

    def __init__(self, params, **changes):
        self.mpc = BristleMPC(params, dict(mpc_parameters, **changes))
        self.dt, self.K = mpc_parameters['Ts'], mpc_parameters['N']

    def compute_optimal_u0(self, measured_State, path_ref_horizon, vel_ref_horizon, return_info=False):
        rows = reference_rows(path_ref_horizon, vel_ref_horizon, self.K)
        measurement = np.concatenate((measured_State.pos_global, measured_State.omega))
        failures_before = self.mpc.failures
        u = self.mpc.step(0.0, measurement, lambda t: rows[int(round(t / self.dt)) - 1])
        info = {'optimal': self.mpc.failures == failures_before}
        return (u, info) if return_info else u

    @property
    def solve_times(self):
        return [t[2] for t in self.mpc.solve_times]


def make_controller(name, params):
    """Every model we tried, by name. J: bristle Jacobian model; P: linearized along the last plan; the number: sub-steps."""
    bristle = lambda: BristleJacobianModel(params, DanaPlant)
    slip = lambda: SlipFrictionModel(params, DanaPlant)
    controllers = {
        'held':          lambda: HeldMPCForDana(params),
        'J1_true':       lambda: JacobianMPCForDana(params, bristle(), measured=False, substeps=1, mode="rollout"),
        'J2_true':       lambda: JacobianMPCForDana(params, bristle(), measured=False, substeps=2, mode="rollout"),
        'JP3_true':      lambda: JacobianMPCForDana(params, bristle(), measured=False, substeps=3, mode="plan"),
        'JP5_true':      lambda: JacobianMPCForDana(params, bristle(), measured=False, substeps=5, mode="plan"),
        'JP3_meas':      lambda: JacobianMPCForDana(params, bristle(), measured=True, substeps=3, mode="plan"),
        'JP3_nobound':   lambda: JacobianMPCForDana(params, bristle(), measured=False, substeps=3, mode="plan", z_bound=False),
        'SP5_true':      lambda: JacobianMPCForDana(params, slip(), measured=False, substeps=5, mode="plan"),
    }
    return controllers[name]()


# Dana's execute_simulation, with the controller line opened up to make_controller
source = code_of("ControllerAnalysis.ipynb", 5)
start = source.index("    if controller_type.lower() == 'pid':\n        ctrl = CascadedPIDController(")
end = source.index("    # Extract reference trajectory data")
dana_controllers = source[start:end].replace("    else:\n        ctrl = MPCControllerSimple(",
                                             "    elif controller_type.lower() == 'mpc':\n        ctrl = MPCControllerSimple(")
source = source[:start] + dana_controllers + "    else:\n        ctrl = make_controller(controller_type, plant_parm)\n    LAST['ctrl'] = ctrl\n\n" + source[end:]
LAST = {}
dana.update(make_controller=make_controller, LAST=LAST)
exec(source, dana)
execute_simulation = dana["execute_simulation"]


# ===========================================================================
# Paths: Dana's segment (two speeds), a short fast segment, and figure eights that make the wheels slide
# ===========================================================================
def figure_eight_path(size, period, laps=2, ramp=0.3, swing=0.3, dt=0.02):
    """Figure eight in Dana's path format: t, pos [X, Y, theta], vel (room frame)."""
    t = np.arange(0.0, laps * period + 1.0, dt)
    rate = 2 * np.pi / period
    phase = rate * (t - ramp * (1 - np.exp(-t / ramp)))
    phase_dot = rate * (1 - np.exp(-t / ramp))
    pos = np.column_stack((size * np.sin(phase), 0.5 * size * np.sin(2 * phase), swing * np.sin(phase)))
    vel = np.column_stack((size * np.cos(phase) * phase_dot, size * np.cos(2 * phase) * phase_dot,
                           swing * np.cos(phase) * phase_dot))
    return {'t': t, 'pos': pos, 'vel': vel}


path_generator = PathGenerator(dt=0.02)
segment = lambda target, T: path_generator.generate_coupled_segment(np.zeros(3), np.array(target), duration=T)
paths = {
    "Dana 8 s":        lambda: segment([3.0, 3.0, np.pi / 4], 8.0),
    "Dana 4 s":        lambda: segment([3.0, 3.0, np.pi / 4], 4.0),
    "short 1.2 s":     lambda: segment([0.5, 0.5, np.pi / 4], 1.2),
    "short 1.0 s":     lambda: segment([0.5, 0.5, np.pi / 4], 1.0),
    "eight 0.5 m 4 s": lambda: figure_eight_path(0.5, 4.0),
    "eight 0.4 m 3 s": lambda: figure_eight_path(0.4, 3.0),
    "eight 0.4 m 2.5 s": lambda: figure_eight_path(0.4, 2.5),
    "eight 0.3 m 2 s": lambda: figure_eight_path(0.3, 2.0),
}


def run(controller, path, params=None):
    """One closed-loop run in Dana's runner. Returns the metrics and her result dictionary."""
    params = dict(dana_parameters) if params is None else dict(params)
    TRUE.clear()
    result = execute_simulation(params, controller, paths[path]())
    error = np.hypot(*(result['pos'][:-1, :2] - result['pos_ref'][1:, :2]).T)      # pos[k] is at t[k + 1]
    times = 1e3 * np.array(getattr(LAST['ctrl'], 'solve_times', []))
    metrics = {'max error (cm)': 100 * error.max(), 'RMS error (cm)': 100 * np.sqrt(np.mean(error**2)),
               'sliding (%)': 100 * np.mean(np.abs(result['vs']) > 5e-3), 'optimal (%)': 100 * np.mean(result['log_opt'])}
    if len(times):
        metrics.update({'solve median (ms)': np.median(times), 'solve worst (ms)': times.max(),
                        'over 20 ms after start': int((times[1:] > 20).sum())})
    return metrics, result
''')

md("accuracy_md", r"""
## Model accuracy: sub-steps

From the plant's own states (our parameters, noise off), the bristle Jacobian model predicts 0.4 s ahead with the voltages the plant got; position error against the plant. More sub-steps per 20 ms step catch wheels that switch between gripping and sliding inside the step. (About 1 minute.)
""")

code("accuracy", r'''
P_quiet = dict(simulation_parameters, sigma_V=0.0, sigma_mu=0.0)
model = BristleJacobianModel(P_quiet, OmniRobotSimulation)
Ts, N_steps = 0.02, 20
STRAFE = np.array([-1.0, -1.0, 1.0, 1.0])
stage = lambda t: 1.0 if t < 1 else (-1.0 if t < 2 else 0.0)
random_V = np.random.default_rng(3).uniform(-8, 8, (20, 4))
manoeuvres = [("10 V strafe", lambda t: 10 * STRAFE, 2.0),
              ("4 V side-to-side", lambda t: -4 * STRAFE * stage(t), 3.0),
              ("random 8 V steps", lambda t: random_V[min(int(t / 0.2), 19)], 4.0)]

print("Position error after 0.4 s [mm], median (90th percentile)")
for name, V_of_t, T in manoeuvres:
    robot = OmniRobotSimulation(P_quiet)
    state = OmniMotorState()
    X, U = [], []
    for k in range(int(round(T / Ts))):
        V = V_of_t(k * Ts)
        X.append(model.from_plant(state))
        U.append(V)
        for _ in range(int(round(Ts / P_quiet['dt']))):
            state, _, _ = robot.step(state, V)
    X.append(model.from_plant(state))
    X, U = np.array(X), np.array(U)
    line = []
    for substeps in (1, 3, 5, 10):
        errors = [1e3 * np.hypot(*(model.rollout(X[k], U[k:k + N_steps], Ts, substeps)[0][-1, :2] - X[k + N_steps, :2]))
                  for k in range(5, len(U) - N_steps, 5)]
        line.append(f"{substeps:2d} sub-steps: {np.median(errors):5.1f} ({np.percentile(errors, 90):5.1f})")
    print(f"   {name:18s}" + "   ".join(line))
''')

md("results_md", r"""
## Results

PID and Dana's MPC against the held bristle MPC and the Jacobian MPCs, on every path, in Dana's runner with her parameters. (About 3 minutes.)
""")

code("results", r'''
controllers = ['pid', 'mpc', 'held', 'JP3_meas', 'JP3_true']
names = {'pid': "PID", 'mpc': "Dana's MPC", 'held': "held bristle MPC", 'JP3_meas': "JP3, measured", 'JP3_true': "JP3, true state"}
results = {}
for path in paths:
    for controller in controllers:
        results[(path, controller)], _ = run(controller, path)

print(f"{'max error (cm)':18s}" + "".join(f"{names[c]:>19s}" for c in controllers))
for path in paths:
    print(f"{path:18s}" + "".join(f"{results[(path, c)]['max error (cm)']:19.1f}" for c in controllers))
print(f"\n{'worst solve (ms)':18s}" + "".join(f"{names[c]:>19s}" for c in controllers[2:]))
for path in paths:
    print(f"{path:18s}" + "".join(f"{results[(path, c)]['solve worst (ms)']:19.1f}" for c in controllers[2:]))
''')

code("plot", r'''
# The hard figure eight: paths and errors
path = "eight 0.4 m 3 s"
fig, axs = plt.subplots(1, 2, figsize=(14, 5.5))
for controller, color in (('pid', 'royalblue'), ('mpc', 'darkorange'), ('held', 'grey'), ('JP3_meas', 'seagreen')):
    _, result = run(controller, path)
    error = np.hypot(*(result['pos'][:-1, :2] - result['pos_ref'][1:, :2]).T)
    axs[0].plot(result['pos'][:, 0], result['pos'][:, 1], color=color, lw=1.8, label=names[controller])
    axs[1].plot(result['t'][1:], 100 * error, color=color, lw=1.8, label=names[controller])
axs[0].plot(result['pos_ref'][:, 0], result['pos_ref'][:, 1], 'k:', lw=1.5, label="Reference")
axs[0].set_aspect('equal', 'datalim'); axs[0].set_xlabel("X (m)"); axs[0].set_ylabel("Y (m)"); axs[0].set_title(f"Path, {path}")
axs[1].set_xlabel("Time (s)"); axs[1].set_ylabel("Position error (cm)"); axs[1].set_title("Position error")
for ax in axs:
    ax.grid(True)
    ax.legend()
plt.tight_layout()
plt.show()
''')

md("findings_md", r"""
## Findings (full runs: `make_controller` lists every variant; any of them can be run with `run(name, path)`)

Max position error (cm), Dana's parameters. JP3 with the measured state uses only pose, wheel speeds and currents.

| Path | PID | Dana's MPC | Held bristle MPC | **JP3, measured** | JP3, true state |
|---|---|---|---|---|---|
| Dana 8 s | 61.4 | 5.8 | 0.0 | **0.0** | 0.0 |
| Dana 4 s (needs more wheel speed than 12 V gives) | 136.0 | 65.0 | 46.3 | **16.7** | 16.7 |
| short 1.2 s | 47.1 | 4.6 | 0.7 | **0.2** | 0.1 |
| short 1.0 s | 50.6 | 13.7 | 8.9 | **2.7** | 1.9 |
| eight 0.5 m 4 s | 52.9 | 4.4 | 1.6 | **1.0** | 1.0 |
| eight 0.4 m 3 s | 47.1 | 15.5 | 45.7 | **4.9** | 3.4 |
| eight 0.4 m 2.5 s | 49.0 | 50.6 | 70.1 | **54.3** | 50.5 |
| eight 0.3 m 2 s | 37.8 | 35.8 | 52.6 | **34.6** | 33.0 |

* **Time:** JP3 median 3-7 ms, worst 6-15 ms on every path (iteration cap 300), including the first, cold solve. Within the 20 ms budget; the held bristle MPC peaks at 21 ms on the hard eights. Python on Windows adds occasional scheduling spikes that can hit any controller.
* **With our parameters** (rollers on, $\sigma_\mu$ = 0.03), JP3 is also best on Dana 4 s (23 cm against 74 for Dana's MPC), short 1.0 s (0.8 cm) and the 3 s eight (0.8 cm); on the 2 s eight Dana's MPC is slightly better (11.8 against 13-15 cm).
* **Where nothing works:** the two fastest eights ask for more acceleration than the grip allows; every controller ends up 30-55 cm off. Dana's 4 s segment asks for 63 rad/s at the wheels, more than the 47 rad/s the 12 V Pololu motors give, so every controller falls behind.
* **Tried and not kept:** the exact Jacobian of $d$ and of the load transfer (blow up); the Jacobian models without the friction bound (spin the wheels, 85-95% sliding); the true Stribeck slope while sliding (no help on its own); friction as a function of slip (slides 60-95% on hard paths, up to 4 m off); 5 or 10 sub-steps (no better than 3, slower); a bound at 3 standard deviations (too cautious with Dana's $\sigma_\mu$ = 0.1).
""")

nb = {"cells": cells, "metadata": {"kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
                                   "language_info": {"name": "python"}}, "nbformat": 4, "nbformat_minor": 5}
with open("MPCModel_Jacobian.ipynb", "w", encoding="utf-8", newline="\n") as f:
    json.dump(nb, f, ensure_ascii=False, indent=1)
    f.write("\n")
print("wrote MPCModel_Jacobian.ipynb")
