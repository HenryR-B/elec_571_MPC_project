# Henry's 1 ms observer on the linearized model with the IMU (commit 7d17ac4), with the camera on time. (His EKF is archived: archive/ekf_linearized/.)
# New cells of merged/ControllerModels.ipynb (ids cad39837, ca2b33c0). They predict with the linearized model every 1 ms.


# ==============================================================================================================
# ControllerModels: RobotObserver1msIMU
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
      6. update: every plant step: advance, and correct_camera when a frame arrives
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
               imu_accel: np.ndarray = None, imu_yaw_rate: float = None):
        """
        Subfunction 6: Every plant step (1 ms): advance with the readings at its end, and correct with the camera
        when a frame arrives.

        Parameters:
            V (4,): the voltages applied over the step; omega (4,), I (4,): the encoder and current readings
            pose (3,), optional: the camera pose when a frame arrives, None between frames
            imu_accel (2,), imu_yaw_rate (float): the IMU readings
        """
        self.advance(V, omega, I, imu_accel, imu_yaw_rate)
        if pose is not None:
            self.correct_camera(pose)
