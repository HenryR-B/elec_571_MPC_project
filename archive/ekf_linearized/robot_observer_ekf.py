# Henry's RobotObserverEKF (commit 7d17ac4, camera on time): a Kalman filter on the linearized model, every 1 ms.
# Archived 11-14 (Henry: no EKF; the observer should use only the IMU and the camera, no linearized model).


# ==============================================================================================================
# ControllerModels: RobotObserverEKF
# ==============================================================================================================
import numpy as np


class RobotObserverEKF:
    """
    1 ms extended Kalman filter for the 20 states of OmniRobotLinearizedModel, built the way the robot's own filter is
    (Rustware master, crates/common_robot_kalman, RobotKalman by Mathew MacDougall): every sensor is a Kalman update
    weighted by its noise.

    The one change from the robot's filter is the prediction. The robot's is kinematic (constant acceleration, no
    physics); this one is the MPC's own model, the 1 ms backward-Euler step of OmniRobotLinearizedModel, linearized at
    the estimate every step:
        x <- a x + b V + c,      P <- a P a^T + Q

    Measurements (each picks its states out of x; the noise is the robot's config.toml value where it has one):
      - every 1 ms: wheel speeds w (encoders), motor currents I (drivers), the accelerometer [right, forward] (it reads
        the lagged acceleration a_past) and the gyro (the yaw rate)
      - every camera frame: the pose [X, Y, theta], the heading error wrapped to +-pi


    Process noise Q (diagonal, a standard deviation per 1 ms step for each part of the state): how far each row of the
    model can be off in one step. Tuned here; the robot's Q is for its kinematic model, so it does not carry over.

    Subfunctions:
      1. __init__: the model, the sensor noise R, the process noise Q
      2. reset: starts from a known state (default: at rest at the origin), with a small covariance
      3. predict: one 1 ms step of the model, with its covariance
      4. correct: one Kalman update for the measured states
      5. advance: one plant step without the camera: predict, then the 1 ms sensors
      6. update: every plant step: advance, and the camera update when a frame arrives
      7. estimate: the latest estimate (20,), what the MPC reads every 20 ms
    """
    def __init__(self, simulation_params: dict, sigma_camera_position: float = 0.005,
                 sigma_camera_heading: float = 0.005, sigma_accel: float = 0.05, sigma_gyro: float = 0.0019,
                 sigma_wheel: float = 0.01, sigma_current: float = 0.01, process_noise: dict = None):
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
        self.reset()

    def reset(self, x0: np.ndarray = None):
        """Subfunction 2: Start from a known state (default: at rest at the origin), with a small covariance."""
        self.x_hat = np.zeros(self.model.n_x) if x0 is None else x0.copy()
        self.P = np.eye(self.model.n_x) * 1e-8

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
               imu_accel: np.ndarray = None, imu_yaw_rate: float = None):
        """
        Subfunction 6: Every plant step (1 ms): advance with the readings at its end, and the camera update when a
        frame arrives. Same call as RobotObserver1msIMU.update.

        Parameters:
            V (4,): the voltages applied over the step; omega (4,), I (4,): the encoder and current readings
            pose (3,), optional: the camera pose when a frame arrives, None between frames
            imu_accel (2,), imu_yaw_rate (float): the IMU readings
        """
        self.advance(V, omega, I, imu_accel, imu_yaw_rate)
        if pose is not None:
            self.correct(self.rows_pose, pose, self.sigma_camera, heading=2)

    def estimate(self) -> np.ndarray:
        """Subfunction 7: The latest estimate of the 20 states (what the MPC reads every 20 ms)."""
        return self.x_hat.copy()
