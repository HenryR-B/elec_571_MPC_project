# From robot_model_voltage.ipynb, plant cell 72c90d84: where the positions feed the plant

        wheel_load_fn=None, disturbances=True, disturbance_seed=None):
        self.dt = float(dt)
        # wheel_load_fn(state, dt) -> four normal loads is an optional hook (None: equal loads).
        self.wheel_load_fn = wheel_load_fn
        # disturbances=True follows the configuration switches; False is the ideal plant.
        self.friction_fn = (
            make_friction_disturbance(disturbance_seed)
            if disturbances and ENABLE_FRICTION_OFFSET else None
        )
        self.load_vibration_fn = (
            make_load_vibration(disturbance_seed)
            if disturbances and ENABLE_LOAD_VIBRATION else None
        )

        self.normal_loads = np.full(N_WHEELS, rc.F_N, dtype=float)
        self.friction_scale = np.ones(N_WHEELS)
        self.load_scale = np.ones(N_WHEELS)  # load-vibration multipliers g_i (ones when off)
        self.wheel_traction_limits = rc.MU_D_STAT * self.normal_loads

        self.max_acceleration = MAX_ACCELERATION
        self.max_velocity = MAX_VELOCITY
        self.max_yaw_rate = MAX_YAW_RATE

        self.disturbance_fn = disturbance_fn
        self.disturbance_forecast = disturbance_forecast

        if heading_reference_fn is None:
            self.heading_reference_fn = configured_heading_reference()
        else:
            self.heading_reference_fn = heading_reference_fn

        # Geometry is fixed in body coordinates. Each column describes one wheel.
        self.drive, self.axle = wheel_directions_body()
        # Contact positions come separately from the directions, so non-tangential wheels work.
        self.positions = wheel_positions_body()
        wheel_radius_vectors = self.positions

        # Signed yaw lever arm: z component of r_i x d_i.
        # z component of r_i x d_i, written out explicitly (np.cross on 2-D vectors
        # is deprecated in NumPy 2.0 and removed in later releases).
        yaw_levers = wheel_radius_vectors[0] * self.drive[1] - wheel_radius_vectors[1] * self.drive[0]

        # H maps four drive forces to [Fx, Fy, tau_z]; its transpose maps body
        # twist/acceleration to the matching wheel-direction kinematic quantities.
        self.H = np.vstack((self.drive, yaw_levers))  # (3, 4)
        # H maps body twist [u, v, omega] to wheel drive speeds and yaw lever arms.
        # It is the matrix form of the old wheel-geometry force/moment sums.

        # Rigid-body mass matrix for the twist q = [u, v, omega] of the reference point P
        # (centre of the wheel-contact circle). The CoG sits at c = COG_OFFSET from P, so
        # translation and yaw couple. I_P = I_zz(CoG) + m |c|^2 = ROBOT_INERTIA.
        # The plant keeps its own copy of c, so a later set_center_of_gravity() call
        # cannot make the mass matrix and the velocity terms disagree.
        self.cog_offset = np.array(rc.COG_OFFSET, dtype=float)
        self.cog_height = float(rc.COG_HEIGHT)
        c_x, c_y = self.cog_offset
        m = rc.ROBOT_MASS
        self.body_mass = np.array([
            [m, 0.0, -m * c_y],
            [0.0, m, m * c_x],
            [-m * c_y, m * c_x, rc.ROBOT_INERTIA],
        ])

        # Under no slip, theta_ddot = -(H.T @ q_dot) / R_w. Substituting this
        # into wheel torque balance reflects wheel inertia into body motion:
        # M_eff = M_body + (J_w / R_w^2) H H.T. This is the model's key constraint.
        self.effective_mass = self.body_mass + (rc.WHEEL_INERTIA / rc.WHEEL_RADIUS**2) * (self.H @ self.H.T)
        self.mass_inverse = np.linalg.inv(self.effective_mass)  # M_eff^-1 for the force balance.

        # Motor torque is tau_m = K_t * Iq. G maps four wheel currents to the
        # three body-twist derivatives after applying the no-slip wheel constraint.
        self.G = -(rc.MOTOR_KT / rc.WHEEL_RADIUS) * self.mass_inverse @ self.H

        facet_angles = 2.0 * np.pi * np.arange(8) / 8.0
        self.facets = np.column_stack((np.cos(facet_angles), np.sin(facet_angles)))
        self.acceleration_limit_poly =