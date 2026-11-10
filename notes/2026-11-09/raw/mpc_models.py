"""
MPC models linearized with Jacobians, and one MPC that takes any of them.

Models (all continuous dx/dt = f(x, u), linearized by central differences, discretized per sub-step):
    BristleJacobianModel   the plant's equations: pose, v, omega, I, z (18 states); relax rate d and loads F_n held at
                           each linearization point (their derivatives are not usable, see notes), the rest exact
    SlipFrictionModel      friction as a smooth function of slip, F = -mu_d(v_s) F_n (2/pi) atan(v_s / v_eps);
                           no bristle state: pose, v, omega, I (14 states)

LTVMPC: linearizes the model along a rollout (or along the last plan, batched), with several sub-steps per
controller step chained into one 20 ms model, then solves one QP (OSQP).
"""
import time
import numpy as np
import osqp
from scipy import sparse
from scipy.linalg import expm


# ===========================================================================
# Models
# ===========================================================================

class JacobianModel:
    """Shared parts: geometry, loads, Jacobians, discretization, rollouts. Subclasses give f and the state layout."""
    POSE = slice(0, 3)
    VELOCITY = slice(3, 6)
    WHEELS = slice(6, 10)
    CURRENTS = slice(10, 14)
    n_u = 4
    z_index = None          # where z sits in x, if the model has bristles

    def __init__(self, params, geometry_class):
        self.p = params
        geometry = geometry_class(params)
        self.H_drive = geometry.H_drive
        self.H_axial = geometry.H_axial
        self.M_inv = np.diag(geometry.M_inv)
        self.F_n_static = np.array([geometry.F_n_front_static, geometry.F_n_front_static,
                                    geometry.F_n_rear_static, geometry.F_n_rear_static])
        pitch, rf, rr = geometry.pitch_gain, geometry.roll_gain_front, geometry.roll_gain_rear
        self.load_transfer = np.array([[-rf, -pitch, 0.0], [rf, -pitch, 0.0], [rr, pitch, 0.0], [-rr, pitch, 0.0]])

    # --- shared physics -------------------------------------------------------------------------------
    def roller_factor(self, v):
        p = self.p
        if not p['roller_friction']:
            return np.zeros((len(v), 4))
        v_slip_a = v @ self.H_axial.T
        mu_a = p['mu_a_k'] + (p['mu_a_s'] - p['mu_a_k']) * np.exp(-p['alpha_a'] * np.abs(v_slip_a))
        return -mu_a * (2.0 / np.pi) * np.arctan(p['roller_smoothing'] * v_slip_a)

    def loads(self, drive_factor, axial_factor):
        """F_n with this state's acceleration, when F_drive = drive_factor F_n and F_axial = axial_factor F_n."""
        B = len(drive_factor)
        K = self.H_drive.T[None] * drive_factor[:, None, :] + self.H_axial.T[None] * axial_factor[:, None, :]
        mat = np.eye(4)[None] - self.load_transfer[None] @ (self.M_inv[None, :, None] * K)
        return np.maximum(0.0, np.linalg.solve(mat, np.broadcast_to(self.F_n_static, (B, 4))[..., None])[..., 0])

    def chassis_and_motor(self, X, U, F, F_a, F_n):
        """Rates of the pose, v, omega and I from the forces."""
        p = self.p
        theta, v = X[:, 2], X[:, self.VELOCITY]
        omega, I = X[:, self.WHEELS], X[:, self.CURRENTS]
        c, s = np.cos(theta), np.sin(theta)
        pose_dot = np.column_stack((c * v[:, 0] - s * v[:, 1], s * v[:, 0] + c * v[:, 1], v[:, 2]))
        v_dot = self.M_inv * (F @ self.H_drive + F_a @ self.H_axial)
        v_dot[:, 0] += v[:, 2] * v[:, 1]
        v_dot[:, 1] -= v[:, 2] * v[:, 0]
        T_roll = p['Crr'] * F_n * p['r'] * np.tanh(omega / p['roll_smoothing'])
        omega_dot = (p['Kt'] * I - T_roll - p['r'] * F) / p['Jw']
        I_dot = (U - p['R'] * I - p['Kb'] * omega) / p['Lm']
        return pose_dot, v_dot, omega_dot, I_dot

    # --- linearization --------------------------------------------------------------------------------
    def jacobians(self, X, U):
        """A = df/dx, B = df/du, f at a batch of points; held values from held(X) at each point."""
        n, m = self.n_x, self.n_u
        P = len(X)
        k = 1 + 2 * (n + m)
        hx = 1e-6 * (1.0 + np.abs(X))
        hu = 1e-6 * (1.0 + np.abs(U))
        Xs = np.repeat(X[:, None, :], k, axis=1)
        Us = np.repeat(U[:, None, :], k, axis=1)
        i, j = np.arange(n), np.arange(m)
        Xs[:, 1 + i, i] += hx
        Xs[:, 1 + n + i, i] -= hx
        Us[:, 1 + 2 * n + j, j] += hu
        Us[:, 1 + 2 * n + m + j, j] -= hu
        held = {key: np.repeat(val[:, None, :], k, axis=1).reshape(P * k, -1) for key, val in self.held(X).items()}
        Fs = self.f(Xs.reshape(-1, n), Us.reshape(-1, m), held).reshape(P, k, n)
        A = ((Fs[:, 1:1 + n] - Fs[:, 1 + n:1 + 2 * n]) / (2 * hx[:, :, None])).transpose(0, 2, 1)
        Bm = ((Fs[:, 1 + 2 * n:1 + 2 * n + m] - Fs[:, 1 + 2 * n + m:]) / (2 * hu[:, :, None])).transpose(0, 2, 1)
        return A, Bm, Fs[:, 0]

    def substep_maps(self, X, U, h):
        """Backward-Euler sub-step maps x+ = a x + b u + c of the linearization at a batch of points."""
        n = self.n_x
        A, Bm, f0 = self.jacobians(X, U)
        c = f0 - np.einsum('pij,pj->pi', A, X) - np.einsum('pij,pj->pi', Bm, U)
        Minv = np.linalg.inv(np.eye(n)[None] - h * A)
        return Minv, h * Minv @ Bm, h * np.einsum('pij,pj->pi', Minv, c)

    def rollout(self, x0, U, Ts, substeps):
        """Rollout from x0 with U, relinearizing every sub-step; returns points and one model per controller step."""
        n, m = self.n_x, self.n_u
        N = len(U)
        h = Ts / substeps
        Xbar = np.zeros((N + 1, n)); Xbar[0] = x0
        Xsub = np.zeros((N, substeps, n))
        Ad, Bd, cd = np.zeros((N, n, n)), np.zeros((N, n, m)), np.zeros((N, n))
        x = x0.copy()
        for j in range(N):
            A_t, B_t, c_t = np.eye(n), np.zeros((n, m)), np.zeros(n)
            for q in range(substeps):
                Xsub[j, q] = x
                a, b, c = (item[0] for item in self.substep_maps(x[None], U[j][None], h))
                x = a @ x + b @ U[j] + c
                A_t, B_t, c_t = a @ A_t, a @ B_t + b, a @ c_t + c
            Ad[j], Bd[j], cd[j] = A_t, B_t, c_t
            Xbar[j + 1] = x
        return Xbar, Ad, Bd, cd, Xsub

    def along_points(self, Xsub, U, Ts):
        """Models linearized at given sub-step points (all in one batch). Xsub (N, substeps, n), U (N, m)."""
        N, S, n = Xsub.shape
        m = self.n_u
        a, b, c = self.substep_maps(Xsub.reshape(-1, n), np.repeat(U, S, axis=0), Ts / S)
        a, b, c = a.reshape(N, S, n, n), b.reshape(N, S, n, m), c.reshape(N, S, n)
        A_t = np.broadcast_to(np.eye(n), (N, n, n)).copy()
        B_t, c_t = np.zeros((N, n, m)), np.zeros((N, n))
        for q in range(S):
            A_t, B_t, c_t = a[:, q] @ A_t, a[:, q] @ B_t + b[:, q], np.einsum('nij,nj->ni', a[:, q], c_t) + c[:, q]
        return A_t, B_t, c_t, (a, b, c)


class BristleJacobianModel(JacobianModel):
    """The plant's equations with the bristle state. x = [pose, v, omega, I, z] (18)."""
    BRISTLES = slice(14, 18)
    n_x = 18
    z_index = np.arange(14, 18)

    def __init__(self, params, geometry_class, deflection=None, slide_threshold=None):
        """slide_threshold: wheels slipping faster than this [m/s] use the true slope of d (the Stribeck curve);
        slower ones keep d held. None: d held for every wheel."""
        super().__init__(params, geometry_class)
        self.delta = params['lugre_deflection'] if deflection is None else deflection
        self.slide_threshold = slide_threshold

    def slip(self, X):
        return X[:, self.VELOCITY] @ self.H_drive.T - self.p['r'] * X[:, self.WHEELS]

    def held(self, X):
        """Relax rate and loads at each linearization point (held: their derivatives are not usable)."""
        p = self.p
        v_s = self.slip(X)
        mu_d = p['mu_k'] + (p['mu_s'] - p['mu_k']) * np.exp(-p['alpha_d'] * np.abs(v_s))
        d = (p['mu_s'] / mu_d) * np.sqrt(v_s**2 + p['lugre_creep']**2)
        z = X[:, self.BRISTLES]
        F_n = self.loads(-p['mu_s'] * z, self.roller_factor(X[:, self.VELOCITY]))
        sliding = np.abs(v_s) > (np.inf if self.slide_threshold is None else self.slide_threshold)
        return {'d': d, 'F_n': F_n, 'sliding': sliding.astype(float)}

    def relax_rate(self, v_s):
        p = self.p
        mu_d = p['mu_k'] + (p['mu_s'] - p['mu_k']) * np.exp(-p['alpha_d'] * np.abs(v_s))
        return (p['mu_s'] / mu_d) * np.sqrt(v_s**2 + p['lugre_creep']**2)

    def f(self, X, U, held):
        p = self.p
        z = X[:, self.BRISTLES]
        F_n = held['F_n']
        v_s = self.slip(X)
        d = np.where(held['sliding'] > 0.5, self.relax_rate(v_s), held['d'])
        F = -p['mu_s'] * F_n * z
        F_a = self.roller_factor(X[:, self.VELOCITY]) * F_n
        pose_dot, v_dot, omega_dot, I_dot = self.chassis_and_motor(X, U, F, F_a, F_n)
        z_dot = (v_s - d * z) / self.delta
        return np.hstack((pose_dot, v_dot, omega_dot, I_dot, z_dot))

    def from_plant(self, state):
        return np.concatenate((state.pos_global, state.v_local, state.omega, state.I, state.z))


class SlipFrictionModel(JacobianModel):
    """Friction as a function of slip, no bristle: F = -mu_d(v_s) F_n (2/pi) atan(v_s / v_eps). x = [pose, v, omega, I]."""
    n_x = 14

    def __init__(self, params, geometry_class, v_eps=0.005):
        super().__init__(params, geometry_class)
        self.v_eps = v_eps

    def drive_factor(self, X):
        p = self.p
        v_s = X[:, self.VELOCITY] @ self.H_drive.T - p['r'] * X[:, self.WHEELS]
        mu_d = p['mu_k'] + (p['mu_s'] - p['mu_k']) * np.exp(-p['alpha_d'] * np.abs(v_s))
        return -mu_d * (2.0 / np.pi) * np.arctan(v_s / self.v_eps)

    def held(self, X):
        return {'F_n': self.loads(self.drive_factor(X), self.roller_factor(X[:, self.VELOCITY]))}

    def f(self, X, U, held):
        F_n = held['F_n']
        F = self.drive_factor(X) * F_n
        F_a = self.roller_factor(X[:, self.VELOCITY]) * F_n
        return np.hstack(self.chassis_and_motor(X, U, F, F_a, F_n))

    def from_plant(self, state):
        return np.concatenate((state.pos_global, state.v_local, state.omega, state.I))


# ===========================================================================
# The MPC
# ===========================================================================

class LTVMPC:
    """
    MPC on a JacobianModel. Every step: linearize (rollout, or batched along the last plan's sub-step points),
    chain the sub-steps into one model per 20 ms step, solve one QP. Pose and velocity tracking, voltage changes
    weighted, |u| <= V_max, and optionally |z| <= z_max for bristle models.
    """

    def __init__(self, params, mpc_params, model, substeps=1, mode="rollout", z_bound=False):
        self.p, self.mp, self.model = params, mpc_params, model
        self.Ts, self.N = mpc_params['Ts'], mpc_params['N']
        self.substeps, self.mode = substeps, mode
        n, m, N = model.n_x, model.n_u, self.N
        self.n, self.m = n, m
        self.z_bound = z_bound and model.z_index is not None
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
                du[j - 1, j - 1] += 1.0; du[j, j - 1] -= 1.0; du[j - 1, j] -= 1.0
        P = sparse.block_diag((sparse.diags(np.concatenate((np.zeros(n), 2.0 * self.q_track.ravel()))),
                               sparse.kron(2.0 * mpc_params['r_du'] * du, sparse.eye(m))), format='csc')
        rows, cols = [], []

        def block(r0, c0, h, w):
            r, c = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
            rows.append((r0 + r).ravel()); cols.append((c0 + c).ravel())

        row = 0
        block(row, x_at(0), n, n); row += n
        for j in range(N):
            block(row, x_at(j), n, n); block(row, x_at(j + 1), n, n); block(row, u_at(j), n, m); row += n
        rows.append(row + np.arange(self.n_u)); cols.append(self.n_x + np.arange(self.n_u)); row += self.n_u
        if self.z_bound:
            for j in range(1, N + 1):
                rows.append(row + np.arange(4)); cols.append(x_at(j) + model.z_index); row += 4
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
        self.sub_maps = None
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

    def _target(self, ref_rows, theta_now):
        """ref_rows (N, 6): [X, Y, theta, x_dot, y_dot, theta_dot] (global) at steps 1..N."""
        N, n = self.N, self.n
        ref = ref_rows.copy()
        ref[:, 2] = np.unwrap(ref[:, 2])
        ref[:, 2] -= 2 * np.pi * np.round((ref[0, 2] - theta_now) / (2 * np.pi))
        target = np.zeros((N, n))
        target[:, 0:3] = ref[:, 0:3]
        c, s = np.cos(ref[:, 2]), np.sin(ref[:, 2])
        target[:, 3] = c * ref[:, 3] + s * ref[:, 4]
        target[:, 4] = -s * ref[:, 3] + c * ref[:, 4]
        target[:, 5] = ref[:, 5]
        return target

    def step(self, x_hat, ref_rows):
        start = time.perf_counter()
        N, n, m, Ts, S = self.N, self.n, self.m, self.Ts, self.substeps
        target = self._target(ref_rows, x_hat[2])
        q = np.concatenate((np.zeros(n), -2.0 * (self.q_track * target).ravel(), np.zeros(self.n_u)))
        q[self.n_x:self.n_x + m] = -2.0 * self.mp['r_du'] * self.u_applied
        U_guess = np.tile(self.u_applied, (N, 1)) if self.U_plan is None else np.vstack((self.U_plan[1:], self.U_plan[-1:]))

        if self.mode == "rollout":
            Xbar, Ad, Bd, cd, Xsub = self.model.rollout(x_hat, U_guess, Ts, S)
        elif self.sub_maps is None:
            # First step (or after a failed solve): a quick one-sub-step rollout gives the points, then one batch
            Xbar = self.model.rollout(x_hat, U_guess, Ts, 1)[0]
            Xsub = np.repeat(Xbar[:-1, None, :], S, axis=1)
            Ad, Bd, cd, self.sub_maps = self.model.along_points(Xsub, U_guess, Ts)
        else:
            # Sub-step points from the last linearization (shifted one step), moved by the new guess; then one batch
            a, b, c = (np.concatenate((arr[1:], arr[-1:])) for arr in self.sub_maps)
            Xsub = np.zeros((N, S, n))
            x = x_hat.copy()
            for j in range(N):
                for qq in range(S):
                    Xsub[j, qq] = x
                    x = a[j, qq] @ x + b[j, qq] @ U_guess[j] + c[j, qq]
            Ad, Bd, cd, self.sub_maps = self.model.along_points(Xsub, U_guess, Ts)

        lower = [x_hat] + list(cd) + [-self.p['V_max'] * np.ones(self.n_u)]
        upper = [x_hat] + list(cd) + [self.p['V_max'] * np.ones(self.n_u)]
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
            self.u_applied = np.clip(self.U_plan[0], -self.p['V_max'], self.p['V_max'])
        else:
            self.failures += 1
            self.X_plan = self.U_plan = None
            self.sub_maps = None
        self.solve_times.append(time.perf_counter() - start)
        return self.u_applied.copy()
