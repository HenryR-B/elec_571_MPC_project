# Henry's 1 ms observers with the late-frame replay (7d17ac4): a late frame corrects the step it was taken at and every
# step since is run again from a 0.3 s history. Not used (no camera delay). The on-time versions are in merged/ and
# imu_and_linearized_observers/.


# ==============================================================================================================
# ControllerModels: BristleTorqueEstimator and RobotObserver1ms, with the replay
# ==============================================================================================================
import numpy as np
from collections import deque


class BristleTorqueEstimator:
    """
    Reconstructs each wheel's bristle state z from its torque balance, with the measured motor current and the wheel
    acceleration from two encoder readings:
        J_w dw/dt = K_t i - T_roll - r F    =>    F = (K_t i - T_roll - J_w dw/dt) / r,    z = -F / (mu_s F_n)
    Same interface as Dana's estimators (reset, estimate), plus update, which takes each new encoder reading.
    Used every 20 ms by MPCControllerLinearized (state_estimate='pose') and every 1 ms by RobotObserver1ms.

    Subfunctions:
      1. __init__: the constants of the torque balance, and the model (for the rolling drag)
      2. reset: forgets the last encoder reading
      3. update: the wheel acceleration over the last dt, from the newest encoder reading
      4. estimate: z for each wheel, with the given wheel loads
    """
    def __init__(self, simulation_params: dict, model: OmniRobotLinearizedModel, dt: float):
        self.model = model
        self.dt = float(dt)
        self.r = float(simulation_params['r'])
        self.Jw = float(simulation_params['Jw'])
        self.Kt = float(simulation_params['Kt'])
        self.mu_s = float(simulation_params['mu_s'])
        self.reset()

    def reset(self):
        self.omega_past = None
        self.omega_dot = np.zeros(4)
        self.acceleration_known = False            # True once two encoder readings have come in

    def update(self, current_omega: np.ndarray):
        """
        Parameters:
            current_omega (4,): [w1, w2, w3, w4] wheel angular speeds in rad/s
        """
        if self.omega_past is not None:
            self.omega_dot = (current_omega - self.omega_past) / self.dt
            self.acceleration_known = True
        self.omega_past = current_omega.copy()

    def estimate(self, current_I: np.ndarray, current_omega: np.ndarray, F_n: np.ndarray) -> np.ndarray:
        """
        Parameters:
            current_I (4,): measured motor currents in A
            current_omega (4,): measured wheel speeds in rad/s
            F_n (4,): wheel loads in N
        Returns:
            z (4,): bristle states, |z| <= 1 (zeros until the wheel acceleration is known)
        """
        if not self.acceleration_known:
            return np.zeros(4)

        # 1. Floor force on each wheel, from its torque balance
        T_roll = self.model.get_rolling_drag(current_omega, F_n)
        F = (self.Kt * current_I - T_roll - self.Jw * self.omega_dot) / self.r

        # 2. Bristle state: at most mu_s F_n; a lifted wheel (F_n = 0) has no friction state
        z = -F / (self.mu_s * np.maximum(F_n, 1e-3))
        z = np.clip(z, -1.0, 1.0)
        return z


class RobotObserver1ms:
    """
    1 ms State Observer for the 20 states of OmniRobotLinearizedModel.

    Runs every plant step (1 ms), on its own core on the robot. It predicts with the linearized model and corrects
    with what the robot measures (no IMU):
      - every 1 ms: the wheel speeds (encoders) and the motor currents (drivers);
      - every 20 ms: the pose [x, y, theta] (camera), possibly late. With camera_replay, a late frame corrects the
        estimate of the step it was taken at (kept in the history), and every step since is advanced again with
        its kept readings, back to now. Without it, a late frame corrects the current estimate as if on time.
    Same interface as Dana's estimators (reset, estimate), plus update, called every plant step.

    Subfunctions:
      1. __init__: its own linearized model and BristleTorqueEstimator, and the correction gains
      2. reset: starts from a known state (default: at rest at the origin)
      3. predict: one 1 ms backward-Euler sub-step of the model, with the voltages applied
      4. correct_wheels: the measured currents and wheel speeds; z moved toward its torque-balance value
      5. correct_camera: the camera pose, and the velocity from how far the pose drifted
      6. update: every plant step: advance, remember, and correct_camera_late when a frame arrives
      7. estimate: the latest estimate (20,), what the MPC reads every 20 ms
      8. advance: one plant step without the camera: predict, correct_wheels
      9. remember: keeps the step's readings, estimate and bristle-estimator memory in the history
      10. correct_camera_late: a frame pose_age late, corrected at its own step and replayed to now
    """
    def __init__(self, simulation_params: dict, gain_bristle: float = 1.0, gain_pose: float = 1.0,
                 gain_velocity: float = 0.5, camera_period: float = 0.02, camera_replay: bool = True,
                 history_length: float = 0.3):
        self.p = simulation_params.copy()
        self.dt = float(self.p['dt'])              # the observer runs every plant step (1 ms)
        self.model = OmniRobotLinearizedModel(self.p)
        self.bristle_estimator = BristleTorqueEstimator(self.p, self.model, self.dt)
        self.gain_bristle = gain_bristle           # L_z: how far z moves toward the torque-balance value each 1 ms
        self.gain_pose = gain_pose                 # L_p: how far the pose moves toward the camera's
        self.gain_velocity = gain_velocity         # L_v: how much of the pose drift goes into the velocity
        self.camera_period = camera_period         # time between camera frames (s)
        self.camera_replay = camera_replay         # True: a late frame is corrected at its own step and replayed
        self.history_steps = int(round(history_length / self.dt))   # steps kept for the replay (0.3 s: Rustware's buffer)
        self.reset()

    def reset(self, x0: np.ndarray = None):
        """Subfunction 2: Start from a known state (default: at rest at the origin)."""
        self.x_hat = np.zeros(self.model.n_x) if x0 is None else x0.copy()
        self.bristle_estimator.reset()
        self.history = deque(maxlen=self.history_steps + 1)   # (readings, estimate, bristle memory), one per step

    def predict(self, V: np.ndarray):
        """Subfunction 3: One plant step (1 ms) of the linearized model from the estimate, with the voltages applied."""
        a, b, c = self.model.get_substep_matrices(self.x_hat, V, self.dt)
        self.x_hat = a @ self.x_hat + b @ V + c

    def correct_wheels(self, omega: np.ndarray, I: np.ndarray):
        """
        Subfunction 4: The 1 ms sensors. The measured currents and wheel speeds replace the estimate's, and the
        bristle state moves toward each wheel's torque-balance value (BristleTorqueEstimator):
            z_hat <- z_hat + L_z (z_m - z_hat)
        """
        model = self.model
        self.x_hat[model.CURRENTS] = I
        self.x_hat[model.WHEELS] = omega

        self.bristle_estimator.update(omega)
        if self.bristle_estimator.acceleration_known:
            z_measured = self.bristle_estimator.estimate(I, omega, model.get_wheel_loads(self.x_hat))
            z_hat = self.x_hat[model.BRISTLES]
            self.x_hat[model.BRISTLES] = z_hat + self.gain_bristle * (z_measured - z_hat)

    def correct_camera(self, pose: np.ndarray):
        """
        Subfunction 5: The camera pose (every camera_period): the pose error e = pose - pose_hat (heading wrapped to
        +-pi), then
            pose_hat += L_p e,    velocity_hat += L_v e / camera_period
        """
        model = self.model
        error = pose - self.x_hat[model.POSE]
        error[2] = (error[2] + np.pi) % (2.0 * np.pi) - np.pi
        self.x_hat[model.POSE] = self.x_hat[model.POSE] + self.gain_pose * error
        self.x_hat[model.VELOCITY] = self.x_hat[model.VELOCITY] + self.gain_velocity * error / self.camera_period

    def update(self, V: np.ndarray, omega: np.ndarray, I: np.ndarray, pose: np.ndarray = None, pose_age: float = 0.0):
        """
        Subfunction 6: Every plant step (1 ms): advance with the readings at its end, keep them in the history, and
        correct with the camera when a frame arrives (correct_camera_late: a late frame is replayed to now).

        Parameters:
            V (4,): the voltages applied over the step; omega (4,), I (4,): the encoder and current readings
            pose (3,), optional: the camera pose when a frame arrives, None between frames
            pose_age (float): how old the frame is (s), 0 when it is on time
        """
        readings = (V, omega, I)
        self.advance(*readings)
        self.remember(readings)
        if pose is not None:
            self.correct_camera_late(pose, pose_age)

    def estimate(self) -> np.ndarray:
        """Subfunction 7: The latest estimate of the 20 states (what the MPC reads every 20 ms)."""
        return self.x_hat.copy()

    def advance(self, V: np.ndarray, omega: np.ndarray, I: np.ndarray):
        """Subfunction 8: One plant step without the camera: predict, then the wheel sensors."""
        self.predict(V)
        self.correct_wheels(omega, I)

    def remember(self, readings: tuple):
        """
        Subfunction 9: Keeps the step's readings (what advance takes), the estimate at its end and the bristle
        estimator's memory (its last encoder reading), so the step can be advanced again in a replay.
        """
        b = self.bristle_estimator
        bristle_memory = (b.omega_past, b.omega_dot, b.acceleration_known)
        self.history.append((readings, self.x_hat.copy(), bristle_memory))

    def correct_camera_late(self, pose: np.ndarray, pose_age: float):
        """
        Subfunction 10: A camera frame pose_age late. It shows the pose at the end of the step pose_age ago: that
        step's kept estimate is corrected (correct_camera), then every step since is advanced again with its kept
        readings, back to now. On time, or without camera_replay, the frame corrects the current estimate. A frame
        older than the history is dropped.

        Parameters:
            pose (3,): the camera pose; pose_age (float): how old the frame is (s)
        """
        age_steps = int(round(pose_age / self.dt))
        if age_steps == 0 or not self.camera_replay:
            self.correct_camera(pose)
            return
        if age_steps >= len(self.history):
            return

        # 1. Back to the step the frame was taken at, and correct it there
        start = len(self.history) - 1 - age_steps
        readings, x_then, bristle_memory = self.history[start]
        self.x_hat = x_then.copy()
        b = self.bristle_estimator
        b.omega_past, b.omega_dot, b.acceleration_known = bristle_memory
        self.correct_camera(pose)
        self.history[start] = (readings, self.x_hat.copy(), bristle_memory)

        # 2. Every step since, advanced again with its kept readings (the history is updated with the new estimates)
        for j in range(start + 1, len(self.history)):
            readings = self.history[j][0]
            self.advance(*readings)
            self.history[j] = (readings, self.x_hat.copy(), (b.omega_past, b.omega_dot, b.acceleration_known))


# ==============================================================================================================
# ControllerModels: RobotObserver1msIMU, with the replay
# ==============================================================================================================
import numpy as np


class RobotObserver1msIMU(RobotObserver1ms):
    """
    RobotObserver1ms with an IMU at the CoG (OmniRobotSimulationODESolveIMU): the same prediction and the same wheel
    and camera corrections, plus, every 1 ms, the IMU readings where they measure a state:
      - accelerometer [right, forward]: the lagged acceleration a_past is what it reads (the last step's acceleration):
            a_past_hat <- a_past_hat + L_a (a_imu - a_past_hat)
      - accelerometer, again: the velocity of the step just taken, from the last step's velocity and the measured
        acceleration rotated with the heading over that step (the plant's Step 5.9):
            v_imu = v_hat(before the step) + dt R(theta_hat) a_imu,     v_hat <- v_hat + L_iv (v_imu - v_hat)
        (L_iv = 1: the velocity is the IMU's integral; the model's forces only enter through z and the loads.)
      - gyro: the yaw rate, theta_dot_hat <- theta_dot_hat + L_g (gyro - theta_dot_hat)
    The IMU correction goes before the wheel correction, so the bristle estimate uses the loads of the measured
    acceleration. Each gain at 0 switches that part off.

    Subfunctions (the rest as RobotObserver1ms):
      1. __init__: RobotObserver1ms's, plus the IMU gains
      2. reset: RobotObserver1ms's, plus the estimate from before the step
      3. predict: RobotObserver1ms's, keeping the estimate from before the step (for the IMU's velocity)
      4. correct_imu: the accelerometer and gyro readings
      5. advance: one plant step without the camera: predict, correct_imu, correct_wheels
      6. update: every plant step: advance, remember, and correct_camera_late when a frame arrives
    """
    def __init__(self, simulation_params: dict, gain_accel: float = 1.0, gain_imu_velocity: float = 1.0,
                 gain_yaw_rate: float = 1.0, **observer_gains):
        self.gain_accel = gain_accel               # L_a: how far a_past moves toward the accelerometer
        self.gain_imu_velocity = gain_imu_velocity # L_iv: how far the velocity moves toward the IMU's integral
        self.gain_yaw_rate = gain_yaw_rate         # L_g: how far the yaw rate moves toward the gyro
        super().__init__(simulation_params, **observer_gains)

    def reset(self, x0: np.ndarray = None):
        """Subfunction 2: RobotObserver1ms's reset, plus the estimate from before the step."""
        super().reset(x0)
        self.x_before = self.x_hat.copy()

    def predict(self, V: np.ndarray):
        """Subfunction 3: RobotObserver1ms's prediction, keeping the estimate from before the step."""
        self.x_before = self.x_hat.copy()
        super().predict(V)

    def correct_imu(self, imu_accel: np.ndarray, imu_yaw_rate: float):
        """
        Subfunction 4: The IMU readings at the end of the step (see the class notes).

        Parameters:
            imu_accel (2,): accelerometer [right, forward] in m/s^2; imu_yaw_rate (float): gyro in rad/s
        """
        model = self.model
        planar_velocity = slice(model.VELOCITY.start, model.VELOCITY.start + 2)     # [vx_g, vy_g]
        yaw_rate = model.VELOCITY.start + 2

        # 1. Lagged acceleration: what the accelerometer reads
        a_past_hat = self.x_hat[model.ACCEL_PAST]
        self.x_hat[model.ACCEL_PAST] = a_past_hat + self.gain_accel * (imu_accel - a_past_hat)

        # 2. Planar velocity: the last step's velocity plus the measured acceleration, rotated with the heading
        #    the step started from
        R_planar = model.get_rotation_matrix(self.x_before[2])[0:2, 0:2]
        v_imu = self.x_before[planar_velocity] + self.dt * R_planar @ imu_accel
        v_hat = self.x_hat[planar_velocity]
        self.x_hat[planar_velocity] = v_hat + self.gain_imu_velocity * (v_imu - v_hat)

        # 3. Yaw rate: what the gyro reads
        self.x_hat[yaw_rate] += self.gain_yaw_rate * (imu_yaw_rate - self.x_hat[yaw_rate])

    def advance(self, V: np.ndarray, omega: np.ndarray, I: np.ndarray, imu_accel: np.ndarray, imu_yaw_rate: float):
        """Subfunction 5: One plant step without the camera: predict, then the IMU, then the wheels."""
        self.predict(V)
        self.correct_imu(imu_accel, imu_yaw_rate)
        self.correct_wheels(omega, I)

    def update(self, V: np.ndarray, omega: np.ndarray, I: np.ndarray, pose: np.ndarray = None,
               imu_accel: np.ndarray = None, imu_yaw_rate: float = None, pose_age: float = 0.0):
        """
        Subfunction 6: Every plant step (1 ms): advance with the readings at its end, keep them in the history, and
        correct with the camera when a frame arrives (correct_camera_late: a late frame is replayed to now).

        Parameters:
            V (4,): the voltages applied over the step; omega (4,), I (4,): the encoder and current readings
            pose (3,), optional: the camera pose when a frame arrives, None between frames
            imu_accel (2,), imu_yaw_rate (float): the IMU readings
            pose_age (float): how old the frame is (s), 0 when it is on time
        """
        readings = (V, omega, I, imu_accel, imu_yaw_rate)
        self.advance(*readings)
        self.remember(readings)
        if pose is not None:
            self.correct_camera_late(pose, pose_age)


# ==============================================================================================================
# ControllerModels: RobotObserverEKF, with the replay
# ==============================================================================================================
import numpy as np
from collections import deque


class RobotObserverEKF:
    """
    1 ms extended Kalman filter for the 20 states of OmniRobotLinearizedModel, built the way the robot's own filter is
    (Rustware master, crates/common_robot_kalman, RobotKalman by Mathew MacDougall): every sensor is a Kalman update
    weighted by its noise, and late camera frames are replayed from a time-stamped buffer.

    The one change from the robot's filter is the prediction. The robot's is kinematic (constant acceleration, no
    physics); this one is the MPC's own model, the 1 ms backward-Euler step of OmniRobotLinearizedModel, linearized at
    the estimate every step:
        x <- a x + b V + c,      P <- a P a^T + Q

    Measurements (each picks its states out of x; the noise is the robot's config.toml value where it has one):
      - every 1 ms: wheel speeds w (encoders), motor currents I (drivers), the accelerometer [right, forward] (it reads
        the lagged acceleration a_past) and the gyro (the yaw rate)
      - every camera frame: the pose [X, Y, theta], the heading error wrapped to +-pi

    Replay (RobotKalman's): every step's readings, and the estimate and covariance after them, are kept for
    history_length (0.3 s, the robot's buffer). A frame taken k steps ago is applied at that step, then every step
    since is run again with its kept readings. RobotKalman rebuilds from the start of its buffer instead; the result is
    the same, because the steps before the frame do not change.

    Process noise Q (diagonal, a standard deviation per 1 ms step for each part of the state): how far each row of the
    model can be off in one step. Tuned here; the robot's Q is for its kinematic model, so it does not carry over.

    Subfunctions:
      1. __init__: the model, the sensor noise R, the process noise Q, the replay settings
      2. reset: starts from a known state (default: at rest at the origin), with a small covariance
      3. predict: one 1 ms step of the model, with its covariance
      4. correct: one Kalman update for the measured states
      5. advance: one plant step without the camera: predict, then the 1 ms sensors
      6. update: every plant step: advance, remember, and correct_camera_late when a frame arrives
      7. remember: keeps the step's readings, estimate and covariance
      8. correct_camera_late: a frame pose_age late, applied at its own step and replayed to now
      9. estimate: the latest estimate (20,), what the MPC reads every 20 ms
    """
    def __init__(self, simulation_params: dict, sigma_camera_position: float = 0.005,
                 sigma_camera_heading: float = 0.005, sigma_accel: float = 0.05, sigma_gyro: float = 0.0019,
                 sigma_wheel: float = 0.01, sigma_current: float = 0.01, process_noise: dict = None,
                 camera_replay: bool = True, history_length: float = 0.3):
        self.p = simulation_params.copy()
        self.dt = float(self.p['dt'])
        self.model = OmniRobotLinearizedModel(self.p)
        model = self.model

        # 1. Sensor noise (standard deviations). Camera, accelerometer and gyro: the robot's config.toml at rest.
        #    Wheel speeds and currents are measured directly here (the robot's encoder value is for the velocity it
        #    derives from them), so they get small values of their own.
        self.sigma_camera = np.array([sigma_camera_position, sigma_camera_position, sigma_camera_heading])
        self.yaw_rate = model.VELOCITY.start + 2
        rows = lambda part: np.arange(part.start, part.stop)
        self.rows_1ms = np.r_[rows(model.WHEELS), rows(model.CURRENTS), rows(model.ACCEL_PAST), self.yaw_rate]
        self.rows_pose = rows(model.POSE)
        self.sigma_1ms = np.r_[np.full(4, sigma_wheel), np.full(4, sigma_current), np.full(2, sigma_accel), sigma_gyro]

        # 2. Process noise, per 1 ms step, for each part of the state
        q = {'pose': 1e-6, 'velocity': 1e-5, 'current': 5e-3, 'wheel': 5e-2, 'bristle': 2e-2, 'accel_past': 0.1}
        if process_noise is not None:
            q.update(process_noise)
        q_std = np.zeros(model.n_x)
        q_std[model.POSE] = q['pose']
        q_std[model.VELOCITY] = q['velocity']
        q_std[model.CURRENTS] = q['current']
        q_std[model.WHEELS] = q['wheel']
        q_std[model.BRISTLES] = q['bristle']
        q_std[model.ACCEL_PAST] = q['accel_past']
        self.Q = np.diag(q_std**2)

        # 3. Replay
        self.camera_replay = camera_replay
        self.history_steps = int(round(history_length / self.dt))
        self.reset()

    def reset(self, x0: np.ndarray = None):
        """Subfunction 2: Start from a known state (default: at rest at the origin), with a small covariance."""
        self.x_hat = np.zeros(self.model.n_x) if x0 is None else x0.copy()
        self.P = np.eye(self.model.n_x) * 1e-8
        self.history = deque(maxlen=self.history_steps + 1)   # (readings, estimate, covariance), one per step

    def predict(self, V: np.ndarray):
        """Subfunction 3: One 1 ms backward-Euler step of the model at the estimate, and its covariance."""
        a, b, c = self.model.get_substep_matrices(self.x_hat, V, self.dt)
        self.x_hat = a @ self.x_hat + b @ V + c
        self.P = a @ self.P @ a.T + self.Q

    def correct(self, rows: np.ndarray, measured: np.ndarray, sigma: np.ndarray, heading: int = None):
        """
        Subfunction 4: One Kalman update. The measurement is the states in rows (H picks them out of x):
            y = measured - x[rows],  S = P[rows, rows] + R,  K = P[:, rows] S^-1,  x += K y,  P -= K P[rows, :]

        Parameters:
            rows (M,): the measured states; measured (M,): the readings; sigma (M,): their noise (standard deviations)
            heading (int), optional: the position in rows of the heading, whose error is wrapped to +-pi
        """
        y = measured - self.x_hat[rows]
        if heading is not None:
            y[heading] = (y[heading] + np.pi) % (2.0 * np.pi) - np.pi
        S = self.P[np.ix_(rows, rows)] + np.diag(sigma**2)
        K = np.linalg.solve(S, self.P[rows, :]).T          # P[:, rows] S^-1 (S and P symmetric)
        self.x_hat = self.x_hat + K @ y
        self.P = self.P - K @ self.P[rows, :]
        self.P = 0.5 * (self.P + self.P.T)                  # keep it symmetric against round-off

    def advance(self, V: np.ndarray, omega: np.ndarray, I: np.ndarray, imu_accel: np.ndarray, imu_yaw_rate: float):
        """Subfunction 5: One plant step without the camera: predict, then the 1 ms sensors in one update."""
        self.predict(V)
        self.correct(self.rows_1ms, np.r_[omega, I, imu_accel, imu_yaw_rate], self.sigma_1ms)

    def update(self, V: np.ndarray, omega: np.ndarray, I: np.ndarray, pose: np.ndarray = None,
               imu_accel: np.ndarray = None, imu_yaw_rate: float = None, pose_age: float = 0.0):
        """
        Subfunction 6: Every plant step (1 ms): advance with the readings at its end, keep them, and correct with the
        camera when a frame arrives (correct_camera_late: a late frame is replayed to now). Same call as
        RobotObserver1msIMU.update.

        Parameters:
            V (4,): the voltages applied over the step; omega (4,), I (4,): the encoder and current readings
            pose (3,), optional: the camera pose when a frame arrives, None between frames
            imu_accel (2,), imu_yaw_rate (float): the IMU readings
            pose_age (float): how old the frame is (s), 0 when it is on time
        """
        readings = (V, omega, I, imu_accel, imu_yaw_rate)
        self.advance(*readings)
        self.remember(readings)
        if pose is not None:
            self.correct_camera_late(pose, pose_age)

    def remember(self, readings: tuple):
        """Subfunction 7: Keeps the step's readings, and the estimate and covariance at its end."""
        self.history.append((readings, self.x_hat.copy(), self.P.copy()))

    def correct_camera_late(self, pose: np.ndarray, pose_age: float):
        """
        Subfunction 8: A camera frame pose_age late. It is applied to the kept estimate of the step it was taken at,
        then every step since is run again with its kept readings, back to now. On time, or without camera_replay,
        the frame updates the current estimate. A frame older than the history is dropped.
        """
        age_steps = int(round(pose_age / self.dt))
        if age_steps == 0 or not self.camera_replay:
            self.correct(self.rows_pose, pose, self.sigma_camera, heading=2)
            return
        if age_steps >= len(self.history):
            return

        # 1. Back to the step the frame was taken at, and the camera update there
        start = len(self.history) - 1 - age_steps
        readings, self.x_hat, self.P = self.history[start]
        self.x_hat = self.x_hat.copy()
        self.P = self.P.copy()
        self.correct(self.rows_pose, pose, self.sigma_camera, heading=2)
        self.history[start] = (readings, self.x_hat.copy(), self.P.copy())

        # 2. Every step since, run again with its kept readings (the history keeps the new estimates)
        for j in range(start + 1, len(self.history)):
            readings = self.history[j][0]
            self.advance(*readings)
            self.history[j] = (readings, self.x_hat.copy(), self.P.copy())

    def estimate(self) -> np.ndarray:
        """Subfunction 9: The latest estimate of the 20 states (what the MPC reads every 20 ms)."""
        return self.x_hat.copy()
