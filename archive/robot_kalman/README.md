### `RobotStateEstimator`: the robot's own Kalman filter (ported from Rustware)

A port of the team robot's state estimator (Rustware master 2e48c6d9, by Mathew MacDougall): `RobotKalmanCore` and `RobotKalman` from `crates/common_robot_kalman/src/lib.rs`, `RobotStateEstimator` from `crates/robot/firmware-app/src/state_estimation.rs`, and the settings from `crates/robot_protocol/config/config.toml` (`ROBOT_STATE_ESTIMATION_CONFIG`).

* **Measured quantities only.** 9 states [x, x', x'', y, y', y'', θ, θ', θ''] in the global frame, constant acceleration, jerk-driven process noise. No robot model, nothing linearized, so every controller can read it the same way.
* **Sensors**, each with a switch as on the robot:
  * vision (pose)
  * gyro
  * linear IMU (the accelerometer, rotated to the global frame, scaled down near a stop)
  * encoders (the no-slip body velocity from the wheel speeds)

  The robot's config fuses vision, the gyro and optical flow. The simulation has no optical flow sensor.
* **Speed-dependent noise:** each sensor's standard deviation is interpolated from the config's samples against the estimated speed and turn rate, the two parts added in quadrature.
* **Measurement buffer:** every measurement is kept for 300 ms; one that arrives out of order is inserted at its own time and the estimate is rebuilt from the base filter (Matt's). Vision is stamped as the robot's config does (its arrival time). The camera here hands each frame over on time (the camera delay is in `archive/camera_delay/`).
* **Between frames** it predicts from the last measured state with the constant-acceleration model, corrected every 1 ms by the gyro (and by the IMU's accelerometer when it is fused).
* The controllers read the pose at the current time + `state_lookahead` (5 ms, as the robot).

## Set aside (2026-11-13)
Henry: "we're not using kalman, get rid of it ... archive it" (observers are not part of the course; the linearized MPC is as accurate from the measured pose, currents and wheel speeds alone: `notes/2026-11-13/2000_8PM/`).

- `robot_kalman.py`: the port (RobotKalmanCore, RobotKalman, RobotStateEstimator, ROBOT_STATE_ESTIMATION_CONFIG), as it was in `merged/ControllerModels.ipynb` with the camera on time.
- `section7_test.py`: the closed-loop test that used it.
- The version with the late camera (frames replayed at their capture time) is in `archive/camera_delay/`.
- Results: `notes/2026-11-13/1900_7PM/ROBOT_KALMAN_PORT_2026-11-13.md`.
- Source: Rustware master 2e48c6d9 (`crates/common_robot_kalman/src/lib.rs`, `crates/robot/firmware-app/src/state_estimation.rs`, `crates/robot_protocol/config/config.toml`).
