"""Remove repeated work from the plant's right-hand side and the PID step; results unchanged.

1. Wheel geometry is computed once per layout (keyed on the rc layout, so tests that change the
   wheel angles still get new values), instead of three times per evaluation.
2. 2-D cross products written out instead of np.cross on zero-padded 3-D vectors (same operations).
3. Wheel loss torque returns zeros directly when both loss coefficients are zero (the default).
4. Contact solve: the all-gripping diagnostic matrix is built once per plant; H diag(w) H^T as
   (H * w) H^T; the current is saturated once per evaluation, not twice; no copies of arrays
   built fresh in the same evaluation.
5. A plant step already integrated during the step (the PID's feasibility check of the same
   current) is reused instead of integrated again. The cache is cleared at every step.
Usage: python patch_speed_a.py NOTEBOOK...
"""
import json
import sys


def rep(s, old, new, count=1):
    assert s.count(old) == count, (s.count(old), old[:80])
    return s.replace(old, new)


def lines(text):
    parts = text.rstrip('\n').split('\n')
    return [p + '\n' for p in parts[:-1]] + [parts[-1]]


def geometry(s):
    s = rep(s, '''def wheel_directions_body():
    """Return drive and axle unit vectors for all wheels in body coordinates."""
    alpha = np.radians(np.asarray(rc.WHEEL_ALPHA_DEG, dtype=float))
    delta = alpha + np.pi / 2

    d_hat = np.vstack((np.cos(delta), np.sin(delta)))
    a_hat = np.vstack((np.cos(alpha), np.sin(alpha)))

    return d_hat, a_hat

def wheel_positions_body():
    """Contact-point positions r_i from the robot centre P [m], body axes, shape (2, 4).

    Each wheel sits at WHEEL_POSITION_DEG on the circle of radius WHEEL_ROBOT_DIST. Its
    direction comes separately from WHEEL_ALPHA_DEG (wheel_directions_body). With equal
    arrays the wheels are tangential: r_i is parallel to a_i and every yaw lever is L.
    """
    beta = np.radians(np.asarray(rc.WHEEL_POSITION_DEG, dtype=float))
    return rc.WHEEL_ROBOT_DIST * np.vstack((np.cos(beta), np.sin(beta)))

def wheel_center_velocity_body(velocity_body, omega):
    """Return each wheel center's velocity in body coordinates [m/s]."""
    radius = wheel_positions_body()

    omega_vector = np.array([0.0, 0.0, omega])
    radius_vector = np.column_stack((radius.T, np.zeros(N_WHEELS)))

    v_center = np.asarray(velocity_body, dtype=float)[:, None] + np.cross(omega_vector, radius_vector).T[:2]

    return v_center''', '''# The geometry depends only on the wheel layout in robot_constants, so it is computed once per
# layout and reused: the plant evaluates it tens of thousands of times per simulated second.
# The cache key is the layout itself, so a test that changes rc.WHEEL_ALPHA_DEG or
# rc.WHEEL_POSITION_DEG gets the new geometry. The arrays are read-only because they are shared.
_wheel_geometry_cache = {}


def _wheel_geometry():
    """(d_hat, a_hat, positions) for the current rc wheel layout."""
    key = (tuple(rc.WHEEL_ALPHA_DEG), tuple(rc.WHEEL_POSITION_DEG), float(rc.WHEEL_ROBOT_DIST))
    geometry = _wheel_geometry_cache.get(key)
    if geometry is None:
        alpha = np.radians(np.asarray(rc.WHEEL_ALPHA_DEG, dtype=float))
        delta = alpha + np.pi / 2
        beta = np.radians(np.asarray(rc.WHEEL_POSITION_DEG, dtype=float))
        geometry = (
            np.vstack((np.cos(delta), np.sin(delta))),
            np.vstack((np.cos(alpha), np.sin(alpha))),
            rc.WHEEL_ROBOT_DIST * np.vstack((np.cos(beta), np.sin(beta))),
        )
        for array in geometry:
            array.flags.writeable = False
        _wheel_geometry_cache[key] = geometry
    return geometry

def wheel_directions_body():
    """Return drive and axle unit vectors for all wheels in body coordinates."""
    d_hat, a_hat, _ = _wheel_geometry()
    return d_hat, a_hat

def wheel_positions_body():
    """Contact-point positions r_i from the robot centre P [m], body axes, shape (2, 4).

    Each wheel sits at WHEEL_POSITION_DEG on the circle of radius WHEEL_ROBOT_DIST. Its
    direction comes separately from WHEEL_ALPHA_DEG (wheel_directions_body). With equal
    arrays the wheels are tangential: r_i is parallel to a_i and every yaw lever is L.
    """
    return _wheel_geometry()[2]

def wheel_center_velocity_body(velocity_body, omega):
    """Return each wheel center's velocity in body coordinates [m/s]."""
    radius = wheel_positions_body()
    # omega x r_i for omega along z: (-omega r_y, omega r_x), as in cross_2d_z.
    return np.asarray(velocity_body, dtype=float)[:, None] + omega * np.array([-radius[1], radius[0]])''')
    s = rep(s, '''    theta_dot = np.asarray(theta_dot, dtype=float)
    loads = np.full(N_WHEELS, rc.F_N, dtype=float) if normal_loads is None else np.asarray(normal_loads, dtype=float)
    rolling_resistance = (''', '''    if rc.WHEEL_ROLLING_RESISTANCE_COEFF == 0.0 and rc.WHEEL_BEARING_VISCOUS_DRAG == 0.0:
        return np.zeros(N_WHEELS)  # both losses off (the default): nothing to evaluate
    theta_dot = np.asarray(theta_dot, dtype=float)
    loads = np.full(N_WHEELS, rc.F_N, dtype=float) if normal_loads is None else np.asarray(normal_loads, dtype=float)
    rolling_resistance = (''')
    return s


def slip_plant(s):
    s = rep(s, '''        self.load_model = WHEEL_LOAD_MODEL if load_model is None else load_model''',
               '''        # Plant steps already integrated in this step, reused instead of integrated again
        # (the PID's feasibility check integrates the same step it then applies).
        self._step_cache = {}
        self.load_model = WHEEL_LOAD_MODEL if load_model is None else load_model''')
    s = rep(s, '''        self.initial_state = self.state.copy()
        self.mass_inverse = np.linalg.inv(self.body_mass)
''', '''        self.initial_state = self.state.copy()
        self.mass_inverse = np.linalg.inv(self.body_mass)
        # Mass matrix with every wheel gripping, for the force_to_roll diagnostic.
        self.all_roll_mass = (
            self.body_mass + self.H @ np.diag(np.full(N_WHEELS, rc.WHEEL_INERTIA / rc.WHEEL_RADIUS**2)) @ self.H.T
        )
''')
    s = rep(s, '''        super().update_contact_disturbances()
        # The base class applied''', '''        super().update_contact_disturbances()
        self._step_cache = {}  # loads and friction change: earlier integrations no longer apply
        # The base class applied''')
    s = rep(s, '''        drive, axle = wheel_directions_body()
        radius = self.positions
        radius_3d = np.column_stack((radius.T, np.zeros(N_WHEELS)))
        force_a_vectors = axle * force_a
        force_a_3d = np.column_stack((force_a_vectors.T, np.zeros(N_WHEELS)))
        a_hat_wrench = np.r_[
            np.sum(force_a_vectors, axis=1),
            np.sum(np.cross(radius_3d, force_a_3d)[:, 2]),
        ]''', '''        drive, axle = wheel_directions_body()
        radius = self.positions
        force_a_vectors = axle * force_a
        # Roller-force moment about P: z component of r_i x F_a,i, summed.
        a_hat_wrench = np.r_[
            np.sum(force_a_vectors, axis=1),
            np.sum(radius[0] * force_a_vectors[1] - radius[1] * force_a_vectors[0]),
        ]''')
    s = rep(s, '''        all_roll_acceleration = np.linalg.solve(
            self.body_mass + self.H @ np.diag(np.full(N_WHEELS, rc.WHEEL_INERTIA / rc.WHEEL_RADIUS**2)) @ self.H.T,
            self.H @ (-wheel_torque / rc.WHEEL_RADIUS) + a_hat_wrench + wrench_body - coriolis,
        )''', '''        all_roll_acceleration = np.linalg.solve(
            self.all_roll_mass,
            self.H @ (-wheel_torque / rc.WHEEL_RADIUS) + a_hat_wrench + wrench_body - coriolis,
        )''')
    s = rep(s, '''                + self.H @ np.diag(rolling * blend * rc.WHEEL_INERTIA / rc.WHEEL_RADIUS**2) @ self.H.T''',
               '''                + (self.H * (rolling * blend * rc.WHEEL_INERTIA / rc.WHEEL_RADIUS**2)) @ self.H.T''')
    s = rep(s, '''        self._force_selection_diagnostics = {
            "wheel_slip_d": slip_d.copy(),
            "force_mode": np.where(rolling, "rolling", "sliding"),
            "rolling_mask": rolling.copy(),
            "raw_force_d": force_d_raw.copy(),
            "force_required": force_required.copy(),
            "force_to_roll": force_to_roll.copy(),
            "force_used": force_used.copy(),
            "traction_limits": traction_limits.copy(),
            "loss_torque": loss_torque.copy(),
            "grip_restoring_force": grip_restoring_force.copy(),
            "normal_loads": np.asarray(normal_loads, dtype=float).copy(),
            "blend": blend.copy(),
        }''', '''        # Every array here is built fresh in this evaluation, so none is copied (readers copy);
        # only the loads, which are the plant's own array, are.
        self._force_selection_diagnostics = {
            "wheel_slip_d": slip_d,
            "force_mode": np.where(rolling, "rolling", "sliding"),
            "rolling_mask": rolling,
            "raw_force_d": force_d_raw,
            "force_required": force_required,
            "force_to_roll": force_to_roll,
            "force_used": force_used,
            "traction_limits": traction_limits,
            "loss_torque": loss_torque,
            "motor_torque": motor_torque,
            "grip_restoring_force": grip_restoring_force,
            "normal_loads": np.array(normal_loads, dtype=float),
            "blend": blend,
        }''')
    s = rep(s, '''        """Return chassis and wheel-speed derivatives."""
        current = self.saturate_current(state_world, current)
        (velocity_body, _, force_used, _, slip_d, slip_a,
         drive_wrench, a_hat_wrench, generalized_acceleration) = self._contact_wrenches(
            state_world, current, wrench_body
        )''', '''        """Return chassis and wheel-speed derivatives."""
        # The contact solve saturates the current; its motor torque is used below.
        (velocity_body, _, force_used, _, slip_d, slip_a,
         drive_wrench, a_hat_wrench, generalized_acceleration) = self._contact_wrenches(
            state_world, current, wrench_body
        )''')
    s = rep(s, '''        acceleration_world = body_to_world(acceleration_body, state_world[2])
        motor_torque = rc.MOTOR_KT * np.asarray(current, dtype=float)
        loss_torque = self._force_selection_diagnostics["loss_torque"]''',
               '''        acceleration_world = body_to_world(acceleration_body, state_world[2])
        motor_torque = self._force_selection_diagnostics["motor_torque"]
        loss_torque = self._force_selection_diagnostics["loss_torque"]''')
    s = rep(s, '''        """Advance the plant state over one controller interval with PLANT_INTEGRATOR."""
        result = solve_ivp(''', '''        """Advance the plant state over one controller interval with PLANT_INTEGRATOR."""
        key = (np.asarray(state, dtype=float).tobytes(), np.asarray(current, dtype=float).tobytes(),
               np.asarray(wrench_body, dtype=float).tobytes())
        if key in self._step_cache:
            final, self.last_nfev, contact_force = self._step_cache[key]
            self._last_contact_force = contact_force.copy()
            return final.copy()
        result = solve_ivp(''')
    s = rep(s, '''            self.physical_dynamics(final, current, wrench_body)
            self._last_contact_force = self._latest_contact_force.copy()
        return final''', '''            self.physical_dynamics(final, current, wrench_body)
            self._last_contact_force = self._latest_contact_force.copy()
        self._step_cache[key] = (final.copy(), self.last_nfev, self._last_contact_force.copy())
        return final''')
    return s


EDITS = [('f26c71d2', geometry), ('9676641c', slip_plant)]

if __name__ == "__main__":
    for path in sys.argv[1:]:
        nb = json.load(open(path, encoding='utf-8'))
        cells = {c.get('id'): c for c in nb['cells']}
        for cid, fn in EDITS:
            c = cells[cid]
            c['source'] = lines(fn(''.join(c['source'])))
            c['outputs'] = []
            c['execution_count'] = None
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            json.dump(nb, f, ensure_ascii=False, indent=1)
            f.write('\n')
        print('patched', path)
