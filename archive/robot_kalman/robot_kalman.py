# The robot's Kalman filter, ported from Rustware (archived 2026-11-13). Was a cell of merged/ControllerModels.ipynb.
# Needs OmniRobotSimulationODESolve (merged/PlantModels.ipynb) defined first.

import numpy as np
import bisect

"""
The robot's own state estimator, ported from Rustware (master 2e48c6d9, Mathew MacDougall):
  - crates/common_robot_kalman/src/lib.rs: Kalman (here RobotKalmanCore) and RobotKalman (the 300 ms measurement
    buffer that replays late or out-of-order measurements);
  - crates/robot/firmware-app/src/state_estimation.rs: StateEstimator (here RobotStateEstimator), which turns the
    sensor readings into Kalman measurements (frames, speed-dependent noise, which sensors are fused);
  - crates/robot_protocol/config/config.toml [state_estimation]: the settings (ROBOT_STATE_ESTIMATION_CONFIG).
It uses only what the robot measures, with no robot model and no linearization, so every controller can read it.

Left out (off in config.toml, or no such sensor in the simulation): the IMU notch / low-pass / slew filters
(apply_imu_filtering = false), the encoder low-pass filters (fuse_encoders = false), optical flow (no sensor here),
the vision innovation clamp (None in push_vision).
"""

# config.toml [state_estimation], [state_estimation.kalman_process_noise], [state_estimation.measurement_variance.*],
# and state_lookahead. Each std dev is SpeedDependentStdDev: (linear samples [(speed m/s, std)], angular samples
# [(rate rad/s, std)]), combined in quadrature.
ROBOT_STATE_ESTIMATION_CONFIG = {
    'fuse_vision': True,
    'use_t_capture_offset': False,
    'vision_delay_ms': -80.0,
    'fuse_gyro': True,
    'fuse_linear_imu': False,
    'imu_delay': 0.0,
    'imu_near_stop_attenuation_enabled': True,
    'imu_near_stop_linear_speed_threshold_mps': 0.05,
    'imu_near_stop_angular_speed_threshold_rad_s': 0.3,
    'imu_near_stop_accel_threshold_mps2': 0.15,
    'fuse_encoders': False,
    'encoder_delay': 0.0,
    'state_lookahead': 0.005,
    'buffer_duration': 0.3,                       # RobotKalman::new(Duration::from_millis(300)) in StateEstimator
    'process_noise': {'position': 1.0, 'velocity': 3.2852, 'acceleration': 50.0,
                      'orientation': 20597.9153, 'angular_velocity': 5000.0, 'angular_acceleration': 2047.4575},
    'vision_position_std_dev': ([(0.0, 0.005)], [(0.0, 0.005)]),
    'vision_theta_std_dev': ([(0.0, 0.005), (0.1, 0.01)], [(0.0, 0.005), (1.0, 0.02)]),
    'imu_linear_accel_std_dev': ([(0.0, 0.05)], [(0.0, 0.05)]),
    'gyro_omega_std_dev': ([(0.0, 0.0)], [(0.0, 0.0019), (0.5, 0.1), (3.0, 0.05)]),
    'encoder_linear_velocity_std_dev': ([(0.0, 0.03)], [(0.0, 0.03)]),
    'encoder_omega_velocity_std_dev': ([(0.0, 0.03)], [(0.0, 0.03)]),
}


def wrap_angle(angle: float) -> float:
    """The angle in [-pi, pi) (Rustware's clamp_pos_neg_pi)."""
    wrapped = (angle + np.pi) % (2.0 * np.pi) - np.pi
    return wrapped


def interpolated_value(samples: list, speed: float) -> float:
    """InterpolatedValue::get: linear between the (input, value) samples, held flat beyond the ends."""
    if len(samples) == 1 or speed <= samples[0][0]:
        return samples[0][1]
    if speed >= samples[-1][0]:
        return samples[-1][1]
    inputs = [sample[0] for sample in samples]
    right = bisect.bisect_left(inputs, speed)
    left = right - 1
    x0, y0 = samples[left]
    x1, y1 = samples[right]
    value = y0 + (y1 - y0) * (speed - x0) / (x1 - x0)
    return value


def speed_dependent_variance(std_dev: tuple, linear_speed: float, angular_speed: float) -> float:
    """SpeedDependentStdDev::variance: the linear and angular parts in quadrature, squared."""
    linear_samples, angular_samples = std_dev
    linear_part = interpolated_value(linear_samples, linear_speed)
    angular_part = interpolated_value(angular_samples, angular_speed)
    variance = linear_part**2 + angular_part**2
    return variance


class RobotKalmanCore:
    """
    Rustware's Kalman: 9 states [x, x', x'', y, y', y'', theta, theta', theta''] in the global frame, constant
    acceleration, jerk-driven process noise, the heading kept in [-pi, pi).

    Measurements are tuples (kind, z, variances): 'vision' z = [x, y, theta]; 'imu' z = [ax, ay] (global);
    'gyro' z = [omega]; 'encoder' z = [vx, vy, omega] (global). Each picks its states with a fixed observation matrix.
    """
    STATE_SIZE = 9
    X, VX, AX, Y, VY, AY, THETA, OMEGA, ALPHA = range(9)
    OBSERVED = {'vision': [0, 3, 6], 'imu': [2, 5], 'gyro': [7], 'encoder': [1, 4, 7]}

    def __init__(self):
        self.state = np.zeros(self.STATE_SIZE)
        self.covariance = np.eye(self.STATE_SIZE) * 0.1

    def copy(self):
        clone = RobotKalmanCore()
        clone.state = self.state.copy()
        clone.covariance = self.covariance.copy()
        return clone

    def predict(self, dt: float, process_noise: dict):
        if dt <= 0.0:
            return
        d2 = dt * dt / 2.0
        d3 = dt * dt * dt / 6.0

        # The state transition: constant acceleration in each of the three axes
        F_axis = np.array([[1.0, dt, d2],
                           [0.0, 1.0, dt],
                           [0.0, 0.0, 1.0]])
        state_transition_matrix = np.zeros((9, 9))
        state_transition_matrix[0:3, 0:3] = F_axis
        state_transition_matrix[3:6, 3:6] = F_axis
        state_transition_matrix[6:9, 6:9] = F_axis

        # Jerk-driven process noise, Q = g g^T in each axis, with a separate scale on each derivative
        g_linear = np.array([d3 * process_noise['position'], d2 * process_noise['velocity'], dt * process_noise['acceleration']])
        g_angular = np.array([d3 * process_noise['orientation'], d2 * process_noise['angular_velocity'],
                              dt * process_noise['angular_acceleration']])
        q = np.zeros((9, 9))
        q[0:3, 0:3] = np.outer(g_linear, g_linear)
        q[3:6, 3:6] = np.outer(g_linear, g_linear)
        q[6:9, 6:9] = np.outer(g_angular, g_angular)

        self.state = state_transition_matrix @ self.state
        self.state[self.THETA] = wrap_angle(self.state[self.THETA])
        self.covariance = state_transition_matrix @ self.covariance @ state_transition_matrix.T + q

    def update(self, measurement: tuple):
        kind, z, variances = measurement
        observation_matrix = np.zeros((len(z), self.STATE_SIZE))
        observation_matrix[np.arange(len(z)), self.OBSERVED[kind]] = 1.0
        noise_covariance = np.diag(variances)
        angle_wrap_index = 2 if kind == 'vision' else None
        self.update_impl(z, observation_matrix, noise_covariance, angle_wrap_index)

    def update_impl(self, measurement: np.ndarray, observation_matrix: np.ndarray, noise_covariance: np.ndarray,
                    angle_wrap_index: int = None):
        # Innovation, the heading part wrapped
        predicted = observation_matrix @ self.state
        y = measurement - predicted
        if angle_wrap_index is not None:
            y[angle_wrap_index] = wrap_angle(y[angle_wrap_index])

        # Innovation covariance, gain, update (skipped if S cannot be inverted, as try_inverse)
        s = observation_matrix @ self.covariance @ observation_matrix.T + noise_covariance
        if abs(np.linalg.det(s)) < 1e-300:
            return
        k = self.covariance @ observation_matrix.T @ np.linalg.inv(s)
        self.state = self.state + k @ y
        self.state[self.THETA] = wrap_angle(self.state[self.THETA])
        self.covariance = (np.eye(self.STATE_SIZE) - k @ observation_matrix) @ self.covariance

    def set_pose(self, x: float, y: float, theta: float, position_variance: float, theta_variance: float):
        self.state[self.X] = x
        self.state[self.Y] = y
        self.state[self.THETA] = theta
        for idx in (self.X, self.Y, self.THETA):
            self.covariance[idx, :] = 0.0
            self.covariance[:, idx] = 0.0
        self.covariance[self.X, self.X] = position_variance
        self.covariance[self.Y, self.Y] = position_variance
        self.covariance[self.THETA, self.THETA] = theta_variance


class RobotKalman:
    """
    Rustware's RobotKalman: a base filter (every measurement older than buffer_duration applied for good), and the
    current posterior: the base plus every buffered measurement in time order. A measurement in order is applied
    incrementally; one that arrives late (out of order, e.g. a delayed vision frame) is inserted at its time and the
    current posterior is rebuilt from the base. It is also rebuilt every MAX_INCREMENTAL_UPDATES_BEFORE_REBUILD
    incremental batches. Times are in seconds. The filter waits for the first vision frame before it is initialized.
    """
    MAX_INCREMENTAL_UPDATES_BEFORE_REBUILD = 64

    def __init__(self, buffer_duration: float):
        self.kalman = RobotKalmanCore()
        self.current_kalman = RobotKalmanCore()
        self.base_time = 0.0
        self.posterior_time = 0.0
        self.measurements = []                  # (timestamp, measurement), in time order
        self.buffer_duration = buffer_duration
        self.latest_vision_measurement_time = None
        self.initialized = False
        self.history_dirty = True
        self.incremental_updates_since_rebuild = 0

    def add_measurement(self, timestamp: float, measurement: tuple, process_noise: dict):
        self.add_measurement_batch([(timestamp, measurement)], process_noise)

    def add_measurement_batch(self, measurements: list, process_noise: dict):
        if not measurements:
            return
        was_initialized = self.initialized
        batch_appended_in_order = True
        newest_time = None

        for timestamp, measurement in measurements:
            if measurement[0] == 'vision':
                if self.latest_vision_measurement_time is None:
                    self.latest_vision_measurement_time = timestamp
                else:
                    self.latest_vision_measurement_time = max(self.latest_vision_measurement_time, timestamp)

            # Wait for the first vision frame (the global position) before the filter is initialized
            if not self.initialized and measurement[0] == 'vision':
                z, variances = measurement[1], measurement[2]
                self.kalman.set_pose(z[0], z[1], z[2], variances[0], variances[2])
                self.current_kalman = self.kalman.copy()
                self.base_time = timestamp
                self.posterior_time = timestamp
                self.initialized = True
                self.history_dirty = True
                self.incremental_updates_since_rebuild = 0

            appended_in_order = not self.measurements or timestamp >= self.measurements[-1][0]
            batch_appended_in_order = batch_appended_in_order and appended_in_order
            if appended_in_order:
                newest_time = timestamp
                self.measurements.append((timestamp, measurement))
            else:
                times = [m[0] for m in self.measurements]
                insert_idx = bisect.bisect_right(times, timestamp)       # partition_point(m.timestamp <= t)
                self.measurements.insert(insert_idx, (timestamp, measurement))
                self.history_dirty = True
                newest_time = self.measurements[-1][0]

        if newest_time is None and self.measurements:
            newest_time = self.measurements[-1][0]
        if newest_time is None:
            return
        self.advance_base(newest_time, process_noise)

        if not self.initialized:
            return
        if not was_initialized or self.history_dirty or not batch_appended_in_order:
            self.rebuild_current_posterior(process_noise)
            return

        # In order: apply only the measurements newer than the current posterior
        newer = [m for m in self.measurements if m[0] > self.posterior_time]
        for timestamp, measurement in newer:
            self.current_kalman.predict(max(0.0, timestamp - self.posterior_time), process_noise)
            self.current_kalman.update(measurement)
            self.posterior_time = timestamp
        if newer:
            self.incremental_updates_since_rebuild += 1
        if self.incremental_updates_since_rebuild >= self.MAX_INCREMENTAL_UPDATES_BEFORE_REBUILD:
            self.rebuild_current_posterior(process_noise)

    def estimate_state(self, current_time: float, process_noise: dict):
        """The current posterior predicted forward to current_time (None before the first vision frame)."""
        if not self.initialized:
            return None
        kalman = self.current_kalman.copy()
        kalman.predict(max(0.0, current_time - self.posterior_time), process_noise)
        return kalman.state

    def state(self):
        if not self.initialized:
            return None
        return self.current_kalman.state.copy()

    def advance_base(self, current_time: float, process_noise: dict):
        """Applies every measurement older than buffer_duration to the base filter for good."""
        cutoff = max(0.0, current_time - self.buffer_duration)
        while self.measurements and self.measurements[0][0] <= cutoff:
            timestamp, measurement = self.measurements.pop(0)
            self.kalman.predict(max(0.0, timestamp - self.base_time), process_noise)
            self.base_time = timestamp
            self.kalman.update(measurement)

    def rebuild_current_posterior(self, process_noise: dict):
        """The base filter plus every buffered measurement, in time order."""
        self.current_kalman = self.kalman.copy()
        t = self.base_time
        for timestamp, measurement in self.measurements:
            self.current_kalman.predict(max(0.0, timestamp - t), process_noise)
            self.current_kalman.update(measurement)
            t = timestamp
        self.posterior_time = t
        self.history_dirty = False
        self.incremental_updates_since_rebuild = 0


class RobotStateEstimator:
    """
    Rustware's firmware StateEstimator, for the simulation: turns the readings into RobotKalman measurements, in the
    firmware's order each tick (every plant step here):
      1. the reference state: the estimate at the current time (its heading rotates the body readings, its speeds
         set the speed-dependent noise)
      2. push_imu: the gyro (fuse_gyro), and the accelerometer rotated to the global frame, scaled down near a stop
         (fuse_linear_imu); queued
      3. push_encoders: the body velocity the wheel speeds give with no slip, r pinv(H_drive) w, rotated to the
         global frame (fuse_encoders); queued
      4. commit: the queued measurements to RobotKalman as one batch
      5. push_vision, when a camera frame arrives: added at once, timestamped as config.toml says (the current time
         minus max(vision_delay_ms, 0); vision_delay_ms is -80, so the arrival time)
    The controllers read estimate_pose(t + state_lookahead).

    Simulation frames: the IMU reads [right, forward] and the body velocity is [right, forward, omega]; both rotate to
    the global frame with this simulation's R(theta) (the robot's forward/left with its own convention).


    Subfunctions:
      1. __init__: the config, the wheel geometry, the filter
      2. tick: one plant step (steps 1 to 5 above)
      3. push_imu, push_encoders, push_vision, commit: as the firmware's
      4. near_stop_imu_attenuation_scale: as the firmware's
      5. estimate_pose: the pose for the controllers, its heading made continuous (the filter keeps it in [-pi, pi))
    """
    def __init__(self, simulation_params: dict, config: dict = None):
        self.config = ROBOT_STATE_ESTIMATION_CONFIG if config is None else config
        self.process_noise = self.config['process_noise']
        geometry = OmniRobotSimulationODESolve(simulation_params)
        self.H_drive_pinv = np.linalg.pinv(geometry.H_drive)
        self.r = simulation_params['r']
        self.kalman = RobotKalman(self.config['buffer_duration'])
        self.pending_measurements = []
        self.theta_output = 0.0

    def speeds(self, state) -> tuple:
        if state is None:
            return 0.0, 0.0
        linear_speed = np.hypot(state[RobotKalmanCore.VX], state[RobotKalmanCore.VY])
        angular_speed = abs(state[RobotKalmanCore.OMEGA])
        return linear_speed, angular_speed

    def rotation_matrix(self, theta: float) -> np.ndarray:
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                    [np.sin(theta),  np.cos(theta)]])
        return rotation_matrix

    def near_stop_imu_attenuation_scale(self, local_accel: np.ndarray, linear_speed: float, angular_speed: float) -> float:
        config = self.config
        if not config['imu_near_stop_attenuation_enabled']:
            return 1.0
        linear_speed_scale = np.clip(linear_speed / max(config['imu_near_stop_linear_speed_threshold_mps'], 1e-6), 0.0, 1.0)
        angular_speed_scale = np.clip(angular_speed / max(config['imu_near_stop_angular_speed_threshold_rad_s'], 1e-6), 0.0, 1.0)
        accel_scale = np.clip(np.hypot(*local_accel) / max(config['imu_near_stop_accel_threshold_mps2'], 1e-6), 0.0, 1.0)
        scale = max(linear_speed_scale, angular_speed_scale, accel_scale)
        return scale

    def push_imu(self, measurement_time: float, imu_accel: np.ndarray, imu_yaw_rate: float, reference_state):
        config = self.config
        linear_speed, angular_speed = self.speeds(reference_state if reference_state is not None else self.kalman.state())
        adjusted_measurement_time = max(0.0, measurement_time - config['imu_delay'])
        if config['fuse_gyro']:
            omega_variance = speed_dependent_variance(config['gyro_omega_std_dev'], linear_speed, angular_speed)
            self.pending_measurements.append((adjusted_measurement_time, ('gyro', np.array([imu_yaw_rate]), np.array([omega_variance]))))
        if reference_state is not None:
            imu_scale = self.near_stop_imu_attenuation_scale(imu_accel, linear_speed, angular_speed)
            global_accel = self.rotation_matrix(reference_state[RobotKalmanCore.THETA]) @ (imu_accel * imu_scale)
            acceleration_variance = speed_dependent_variance(config['imu_linear_accel_std_dev'], linear_speed, angular_speed)
            if config['fuse_linear_imu']:
                self.pending_measurements.append((adjusted_measurement_time, ('imu', global_accel, np.array([acceleration_variance] * 2))))

    def push_encoders(self, current_time: float, omega: np.ndarray, reference_state):
        config = self.config
        if reference_state is None:
            return
        body_velocity = self.r * self.H_drive_pinv @ omega                 # [right, forward, omega], no slip
        global_velocity = self.rotation_matrix(reference_state[RobotKalmanCore.THETA]) @ body_velocity[0:2]
        linear_speed, angular_speed = self.speeds(reference_state)
        velocity_variance = speed_dependent_variance(config['encoder_linear_velocity_std_dev'], linear_speed, angular_speed)
        omega_variance = speed_dependent_variance(config['encoder_omega_velocity_std_dev'], linear_speed, angular_speed)
        adjusted_measurement_time = max(0.0, current_time - config['encoder_delay'])
        if config['fuse_encoders']:
            z = np.array([global_velocity[0], global_velocity[1], body_velocity[2]])
            self.pending_measurements.append((adjusted_measurement_time, ('encoder', z, np.array([velocity_variance, velocity_variance, omega_variance]))))

    def commit(self):
        if not self.pending_measurements:
            return
        measurements = self.pending_measurements
        self.pending_measurements = []
        self.kalman.add_measurement_batch(measurements, self.process_noise)

    def push_vision(self, pose: np.ndarray, current_time: float):
        config = self.config
        linear_speed, angular_speed = self.speeds(self.kalman.state())
        adjusted_measurement_time = max(0.0, current_time - max(config['vision_delay_ms'], 0.0) / 1000.0)
        position_variance = speed_dependent_variance(config['vision_position_std_dev'], linear_speed, angular_speed)
        theta_variance = speed_dependent_variance(config['vision_theta_std_dev'], linear_speed, angular_speed)
        if config['fuse_vision']:
            measurement = ('vision', np.array([pose[0], pose[1], wrap_angle(pose[2])]),
                           np.array([position_variance, position_variance, theta_variance]))
            self.kalman.add_measurement(adjusted_measurement_time, measurement, self.process_noise)

    def tick(self, current_time: float, omega: np.ndarray, imu_accel: np.ndarray, imu_yaw_rate: float,
             camera_pose: np.ndarray = None):
        """One plant step, in the firmware's order (see the class notes)."""
        reference_state = self.kalman.estimate_state(current_time, self.process_noise)
        self.push_imu(current_time, imu_accel, imu_yaw_rate, reference_state)
        self.push_encoders(current_time, omega, reference_state)
        self.commit()
        if camera_pose is not None:
            self.push_vision(camera_pose, current_time)

    def estimate_pose(self, current_time: float) -> np.ndarray:
        """
        The pose at current_time + state_lookahead (the robot's default state before the first frame: zeros). The
        filter keeps the heading in [-pi, pi); for the controllers it is made continuous with the last output.
        """
        state = self.kalman.estimate_state(current_time + self.config['state_lookahead'], self.process_noise)
        if state is None:
            state = np.zeros(RobotKalmanCore.STATE_SIZE)
        self.theta_output = self.theta_output + wrap_angle(state[RobotKalmanCore.THETA] - self.theta_output)
        pose = np.array([state[RobotKalmanCore.X], state[RobotKalmanCore.Y], self.theta_output])
        return pose

    def estimate_velocity(self, current_time: float) -> np.ndarray:
        """The global velocity [vx, vy, omega] at current_time + state_lookahead (zeros before the first frame)."""
        state = self.kalman.estimate_state(current_time + self.config['state_lookahead'], self.process_noise)
        if state is None:
            state = np.zeros(RobotKalmanCore.STATE_SIZE)
        velocity = np.array([state[RobotKalmanCore.VX], state[RobotKalmanCore.VY], state[RobotKalmanCore.OMEGA]])
        return velocity
