"""
Development version: MPC on the exact plant equations, linearized (Jacobian) along a rollout every controller step.
Run from the elec_571_MPC_project folder.
"""
import json
import time
import numpy as np
import osqp
from scipy import sparse
from scipy.linalg import expm


def load_cells(path):
    return {c.get("id"): "".join(c["source"]) for c in json.load(open(path, encoding="utf-8"))["cells"]}


plant_ns = {"__name__": "plant"}
_cells = load_cells("RobotSimulationOpenLoopModel_Merged.ipynb")
exec(_cells["constants"], plant_ns)
exec(_cells["plant"], plant_ns)
simulation_parameters = plant_ns["simulation_parameters"]
OmniRobotSimulation = plant_ns["OmniRobotSimulation"]
OmniMotorState = plant_ns["OmniMotorState"]


class ExactModel:
    """
    The plant's continuous equations, f(x, u), with nothing held.
    State x = [X, Y, theta, v_x, v_y, theta_dot (robot axes), omega_1..4, I_1..4, z_1..4] (18); input u = voltages (4).
    """
    POSE = slice(0, 3)
    VELOCITY = slice(3, 6)
    WHEELS = slice(6, 10)
    CURRENTS = slice(10, 14)
    BRISTLES = slice(14, 18)
    n_x, n_u = 18, 4

    def __init__(self, params, deflection=None, hold_d=True, hold_F_n=False):
        self.p = params
        self.hold_d = hold_d
        self.hold_F_n = hold_F_n
        self.delta = params['lugre_deflection'] if deflection is None else deflection
        geometry = OmniRobotSimulation(params)
        self.H_drive = geometry.H_drive
        self.H_axial = geometry.H_axial
        self.M_inv = np.diag(geometry.M_inv)
        self.F_n_static = np.array([geometry.F_n_front_static, geometry.F_n_front_static,
                                    geometry.F_n_rear_static, geometry.F_n_rear_static])
        pitch, rf, rr = geometry.pitch_gain, geometry.roll_gain_front, geometry.roll_gain_rear
        self.load_transfer = np.array([[-rf, -pitch, 0.0], [rf, -pitch, 0.0], [rr, pitch, 0.0], [-rr, pitch, 0.0]])

    def normal_forces(self, X):
        p = self.p
        v, z = X[:, self.VELOCITY], X[:, self.BRISTLES]
        B = len(X)
        if p['roller_friction']:
            v_slip_a = v @ self.H_axial.T
            mu_a = p['mu_a_k'] + (p['mu_a_s'] - p['mu_a_k']) * np.exp(-p['alpha_a'] * np.abs(v_slip_a))
            phi_a = -mu_a * (2.0 / np.pi) * np.arctan(p['roller_smoothing'] * v_slip_a)
        else:
            phi_a = np.zeros((B, 4))
        K = -p['mu_s'] * self.H_drive.T[None] * z[:, None, :] + self.H_axial.T[None] * phi_a[:, None, :]
        mat = np.eye(4)[None] - self.load_transfer[None] @ (self.M_inv[None, :, None] * K)
        return np.maximum(0.0, np.linalg.solve(mat, np.broadcast_to(self.F_n_static, (B, 4))[..., None])[..., 0])

    def relax_rate(self, X):
        p = self.p
        v_slip_d = X[:, self.VELOCITY] @ self.H_drive.T - p['r'] * X[:, self.WHEELS]
        mu_d = p['mu_k'] + (p['mu_s'] - p['mu_k']) * np.exp(-p['alpha_d'] * np.abs(v_slip_d))
        return (p['mu_s'] / mu_d) * np.sqrt(v_slip_d**2 + p['lugre_creep']**2)

    def f(self, X, U, d=None, F_n_held=None):
        """dx/dt for a batch of states X (B, 18) and inputs U (B, 4). d: relax rates to use (held), or None."""
        p = self.p
        theta = X[:, 2]
        v = X[:, self.VELOCITY]
        omega = X[:, self.WHEELS]
        I = X[:, self.CURRENTS]
        z = X[:, self.BRISTLES]
        B = len(X)

        # Slip, Stribeck curve and bristle relax rate
        v_slip_d = v @ self.H_drive.T - p['r'] * omega
        if d is None:
            mu_d = p['mu_k'] + (p['mu_s'] - p['mu_k']) * np.exp(-p['alpha_d'] * np.abs(v_slip_d))
            d = (p['mu_s'] / mu_d) * np.sqrt(v_slip_d**2 + p['lugre_creep']**2)

        # Roller force per unit normal force
        if p['roller_friction']:
            v_slip_a = v @ self.H_axial.T
            mu_a = p['mu_a_k'] + (p['mu_a_s'] - p['mu_a_k']) * np.exp(-p['alpha_a'] * np.abs(v_slip_a))
            phi_a = -mu_a * (2.0 / np.pi) * np.arctan(p['roller_smoothing'] * v_slip_a)
        else:
            phi_a = np.zeros((B, 4))

        # Normal forces with this state's acceleration: (I - T M^-1 K) F_n = F_n_static
        K = -p['mu_s'] * self.H_drive.T[None] * z[:, None, :] + self.H_axial.T[None] * phi_a[:, None, :]
        mat = np.eye(4)[None] - self.load_transfer[None] @ (self.M_inv[None, :, None] * K)
        F_n = np.maximum(0.0, np.linalg.solve(mat, np.broadcast_to(self.F_n_static, (B, 4))[..., None])[..., 0])
        if F_n_held is not None:
            F_n = F_n_held

        # Forces
        F = -p['mu_s'] * F_n * z
        F_a = phi_a * F_n
        T_roll = p['Crr'] * F_n * p['r'] * np.tanh(omega / p['roll_smoothing'])

        # Rates
        c, s = np.cos(theta), np.sin(theta)
        pose_dot = np.column_stack((c * v[:, 0] - s * v[:, 1], s * v[:, 0] + c * v[:, 1], v[:, 2]))
        v_dot = self.M_inv * (F @ self.H_drive + F_a @ self.H_axial)
        v_dot[:, 0] += v[:, 2] * v[:, 1]
        v_dot[:, 1] -= v[:, 2] * v[:, 0]
        omega_dot = (p['Kt'] * I - T_roll - p['r'] * F) / p['Jw']
        I_dot = (U - p['R'] * I - p['Kb'] * omega) / p['Lm']
        z_dot = (v_slip_d - d * z) / self.delta
        return np.hstack((pose_dot, v_dot, omega_dot, I_dot, z_dot))

    def jacobians(self, X, U):
        """A = df/dx, B = df/du and f at a batch of points (central differences, one batched call)."""
        n, m = self.n_x, self.n_u
        P = len(X)
        hx = 1e-6 * (1.0 + np.abs(X))
        hu = 1e-6 * (1.0 + np.abs(U))
        Xs = np.repeat(X[:, None, :], 1 + 2 * (n + m), axis=1)
        Us = np.repeat(U[:, None, :], 1 + 2 * (n + m), axis=1)
        idx = np.arange(n)
        Xs[:, 1 + idx, idx] += hx
        Xs[:, 1 + n + idx, idx] -= hx
        jdx = np.arange(m)
        Us[:, 1 + 2 * n + jdx, jdx] += hu
        Us[:, 1 + 2 * n + m + jdx, jdx] -= hu
        rep = lambda a: np.repeat(a[:, None, :], 1 + 2 * (n + m), axis=1).reshape(-1, 4)
        d = rep(self.relax_rate(X)) if self.hold_d else None
        F_n = rep(self.normal_forces(X)) if self.hold_F_n else None
        Fs = self.f(Xs.reshape(-1, n), Us.reshape(-1, m), d, F_n).reshape(P, 1 + 2 * (n + m), n)
        f0 = Fs[:, 0]
        A = ((Fs[:, 1:1 + n] - Fs[:, 1 + n:1 + 2 * n]) / (2 * hx[:, :, None])).transpose(0, 2, 1)
        Bm = ((Fs[:, 1 + 2 * n:1 + 2 * n + m] - Fs[:, 1 + 2 * n + m:]) / (2 * hu[:, :, None])).transpose(0, 2, 1)
        return A, Bm, f0

    def discretize(self, A, Bm, f0, X, U, Ts):
        """Exact zero-order hold of the linearization x_dot = f0 + A (x - X) + B (u - U), for a batch."""
        n, m = self.n_x, self.n_u
        P = len(A)
        c = f0 - np.einsum('pij,pj->pi', A, X) - np.einsum('pij,pj->pi', Bm, U)
        block = np.zeros((P, n + m + 1, n + m + 1))
        block[:, :n, :n] = A
        block[:, :n, n:n + m] = Bm
        block[:, :n, n + m] = c
        E = expm(block * Ts)
        return E[:, :n, :n], E[:, :n, n:n + m], E[:, :n, n + m]

    def rollout(self, x0, U, Ts):
        """Step the model from x0 with the inputs U, linearizing at each point reached. Returns the points and the
        discrete models (A_d, B_d, c_d) of each step."""
        N = len(U)
        Xbar = np.zeros((N + 1, self.n_x))
        Xbar[0] = x0
        Ad, Bd, cd = [], [], []
        for j in range(N):
            A, Bm, f0 = self.jacobians(Xbar[j:j + 1], U[j:j + 1])
            a, b, c = (item[0] for item in self.discretize(A, Bm, f0, Xbar[j:j + 1], U[j:j + 1], Ts))
            Ad.append(a); Bd.append(b); cd.append(c)
            Xbar[j + 1] = a @ Xbar[j] + b @ U[j] + c
        return Xbar, np.array(Ad), np.array(Bd), np.array(cd)

    def rollout_sub(self, x0, U, Ts, substeps, method="be"):
        """Rollout with `substeps` fresh linearizations per controller step, chained into one discrete model per
        controller step (chain rule). method: "be" backward Euler per sub-step, "expm" exact hold per sub-step."""
        n, m = self.n_x, self.n_u
        N = len(U)
        h = Ts / substeps
        Xbar = np.zeros((N + 1, n))
        Xbar[0] = x0
        Ad, Bd, cd = np.zeros((N, n, n)), np.zeros((N, n, m)), np.zeros((N, n))
        I_n = np.eye(n)
        x = x0.copy()
        for j in range(N):
            u = U[j]
            A_tot, B_tot, c_tot = I_n.copy(), np.zeros((n, m)), np.zeros(n)
            for _ in range(substeps):
                A, Bm, f0 = self.jacobians(x[None], u[None])
                A, Bm, f0 = A[0], Bm[0], f0[0]
                c = f0 - A @ x - Bm @ u
                if method == "be":
                    # x+ = x + h (A x+ + B u + c)  ->  x+ = (I - hA)^-1 (x + h B u + h c)
                    M = np.linalg.inv(I_n - h * A)
                    a, b, cc = M, h * M @ Bm, h * M @ c
                else:
                    blk = np.zeros((n + m + 1, n + m + 1))
                    blk[:n, :n], blk[:n, n:n + m], blk[:n, n + m] = A, Bm, c
                    E = expm(blk * h)
                    a, b, cc = E[:n, :n], E[:n, n:n + m], E[:n, n + m]
                x = a @ x + b @ u + cc
                A_tot, B_tot, c_tot = a @ A_tot, a @ B_tot + b, a @ c_tot + cc
            Ad[j], Bd[j], cd[j] = A_tot, B_tot, c_tot
            Xbar[j + 1] = x
        return Xbar, Ad, Bd, cd

    def linearize_along(self, Xbar, U, Ts):
        """Discrete models linearized at given points (batched, not a rollout)."""
        A, Bm, f0 = self.jacobians(Xbar, U)
        return self.discretize(A, Bm, f0, Xbar, U, Ts)


def state_vector(state):
    """The plant's true state as the exact model's x."""
    return np.concatenate((state.pos_global, state.v_local, state.omega, state.I, state.z))


class ExactMPC:
    """MPC on ExactModel, linearized along a rollout of the last plan every step (optionally repeated)."""

    def __init__(self, params, mpc_params, iterations=1, z_bound=True, mode="rollout"):
        self.p = params
        self.mp = mpc_params
        self.Ts = mpc_params['Ts']
        self.N = mpc_params['N']
        self.iterations = iterations
        self.mode = mode
        self.model = ExactModel(params)
        n, m, N = self.model.n_x, self.model.n_u, self.N
        self.n, self.m = n, m
        self.z_bound = z_bound
        self.z_max = (params['mu_s'] - mpc_params['deviation_sigmas'] * params['sigma_mu']) / params['mu_s']
        self.n_x, self.n_u = n * (N + 1), m * N
        x_at = lambda j: j * n
        u_at = lambda j: self.n_x + j * m

        self.q_track = np.zeros((N, n))
        for j in range(N):
            w = mpc_params['q_terminal'] if j == N - 1 else 1.0
            self.q_track[j, 0:3] = w * np.array(mpc_params['q_pos'])
            self.q_track[j, 3:6] = w * np.array(mpc_params['q_vel'])
        du = np.zeros((N, N))
        for j in range(N):
            du[j, j] += 1.0
            if j > 0:
                du[j - 1, j - 1] += 1.0
                du[j, j - 1] -= 1.0
                du[j - 1, j] -= 1.0
        P = sparse.block_diag((sparse.diags(np.concatenate((np.zeros(n), 2.0 * self.q_track.ravel()))),
                               sparse.kron(2.0 * mpc_params['r_du'] * du, sparse.eye(m))), format='csc')

        rows, cols = [], []

        def block(r0, c0, h, w):
            r, c = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
            rows.append((r0 + r).ravel()); cols.append((c0 + c).ravel())

        row = 0
        block(row, x_at(0), n, n); row += n
        for j in range(N):
            block(row, x_at(j), n, n)
            block(row, x_at(j + 1), n, n)
            block(row, u_at(j), n, m)
            row += n
        rows.append(row + np.arange(self.n_u)); cols.append(self.n_x + np.arange(self.n_u)); row += self.n_u
        if z_bound:
            for j in range(1, N + 1):
                rows.append(row + np.arange(4)); cols.append(x_at(j) + 14 + np.arange(4)); row += 4
        rows, cols = np.concatenate(rows), np.concatenate(cols)
        self.n_rows = row
        self.order = np.lexsort((rows, cols))
        indptr = np.concatenate(([0], np.cumsum(np.bincount(cols, minlength=self.n_x + self.n_u))))
        A0 = sparse.csc_matrix((self._values(np.tile(np.eye(n), (N, 1, 1)), np.zeros((N, n, m))), rows[self.order], indptr),
                               shape=(self.n_rows, self.n_x + self.n_u))
        self.solver = osqp.OSQP()
        self.solver.setup(P, np.zeros(self.n_x + self.n_u), A0, -np.ones(self.n_rows), np.ones(self.n_rows),
                          verbose=False, warm_starting=True, polishing=False, eps_abs=1e-4, eps_rel=1e-4,
                          max_iter=mpc_params['max_iterations'])
        self.u_applied = np.zeros(m)
        self.X_plan = self.U_plan = None
        self.failures = 0
        self.solve_times = []

    def _values(self, Ad, Bd):
        n, N = self.n, self.N
        parts = [np.eye(n).ravel()]
        for j in range(N):
            parts += [(-Ad[j]).ravel(), np.eye(n).ravel(), (-Bd[j]).ravel()]
        parts.append(np.ones(self.n_u))
        if self.z_bound:
            parts.append(np.ones(4 * N))
        return np.concatenate(parts)[self.order]

    def _targets(self, t, reference, theta_now):
        N, n = self.N, self.n
        ref = np.array([reference(t + self.Ts * (j + 1)) for j in range(N)])
        ref[:, 2] = np.unwrap(ref[:, 2])
        ref[:, 2] -= 2 * np.pi * np.round((ref[0, 2] - theta_now) / (2 * np.pi))
        target = np.zeros((N, n))
        target[:, 0:3] = ref[:, 0:3]
        c, s = np.cos(ref[:, 2]), np.sin(ref[:, 2])
        target[:, 3] = c * ref[:, 3] + s * ref[:, 4]
        target[:, 4] = -s * ref[:, 3] + c * ref[:, 4]
        target[:, 5] = ref[:, 5]
        return target

    def step(self, t, x_hat, reference):
        start = time.perf_counter()
        N, n, m, Ts = self.N, self.n, self.m, self.Ts
        target = self._targets(t, reference, x_hat[2])
        q = np.concatenate((np.zeros(n), -2.0 * (self.q_track * target).ravel(), np.zeros(self.n_u)))
        q[self.n_x:self.n_x + m] = -2.0 * self.mp['r_du'] * self.u_applied

        U_guess = np.tile(self.u_applied, (N, 1)) if self.U_plan is None else np.vstack((self.U_plan[1:], self.U_plan[-1:]))
        for it in range(self.iterations):
            if self.mode == "rollout" or self.X_plan is None:
                Xbar, Ad, Bd, cd = self.model.rollout(x_hat, U_guess, Ts)
            else:     # linearize along the last plan, shifted (batched)
                Xbar = np.vstack((x_hat, self.X_plan[2:], self.X_plan[-1:]))[:N]
                Ad, Bd, cd = self.model.linearize_along(Xbar, U_guess, Ts)
            lower = [x_hat] + [c for c in cd] + [-self.p['V_max'] * np.ones(self.n_u)]
            upper = [x_hat] + [c for c in cd] + [self.p['V_max'] * np.ones(self.n_u)]
            if self.z_bound:
                lower.append(np.full(4 * N, -self.z_max)); upper.append(np.full(4 * N, self.z_max))
            self.solver.update(q=q, l=np.concatenate(lower), u=np.concatenate(upper), Ax=self._values(Ad, Bd))
            if self.X_plan is not None:
                X_ws = np.vstack((x_hat, self.X_plan[2:], self.X_plan[-1:]))
                self.solver.warm_start(x=np.concatenate((X_ws.ravel(), U_guess.ravel())))
            res = self.solver.solve()
            if res.info.status in ('solved', 'solved inaccurate', 'maximum iterations reached') and np.all(np.isfinite(res.x)):
                self.X_plan = res.x[:self.n_x].reshape(N + 1, n)
                self.U_plan = res.x[self.n_x:].reshape(N, m)
                U_guess = self.U_plan
            else:
                self.failures += 1
                break
        if self.U_plan is not None:
            self.u_applied = np.clip(self.U_plan[0], -self.p['V_max'], self.p['V_max'])
        self.solve_times.append(time.perf_counter() - start)
        return self.u_applied.copy()


def figure_eight(t, size=0.5, period=4.0, ramp=0.5):
    rate = 2.0 * np.pi / period
    phase = rate * (t - ramp * (1.0 - np.exp(-t / ramp)))
    phase_dot = rate * (1.0 - np.exp(-t / ramp))
    return np.array([size * np.sin(phase), 0.5 * size * np.sin(2.0 * phase), 0.5 * np.sin(phase),
                     size * np.cos(phase) * phase_dot, size * np.cos(2.0 * phase) * phase_dot, 0.5 * np.cos(phase) * phase_dot])


def run(controller_step, reference, T, params, Ts=0.02, seed=42):
    """Closed loop on the plant. controller_step(t, state) -> voltages; the controller is given the true state."""
    robot = OmniRobotSimulation(params, random_seed=seed)
    state = OmniMotorState()
    every = int(round(Ts / params['dt']))
    n_steps = int(round(T / params['dt']))
    err, sliding = [], []
    V = np.zeros(4)
    for k in range(n_steps):
        t = k * params['dt']
        if k % every == 0:
            V = controller_step(t, state)
        state, _, S = robot.step(state, V)
        ref = reference(t + params['dt'])
        err.append(np.hypot(*(state.pos_global[:2] - ref[:2])))
        sliding.append(S.any())
    return np.array(err), 100 * np.mean(sliding)
