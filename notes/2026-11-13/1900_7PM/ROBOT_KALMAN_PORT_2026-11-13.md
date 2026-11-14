# The robot's Kalman filter ported, and the 3-way pose switch (11-13, ~7:50 PM)

## Pose switch (ControllerAnalysis settings): `POSE_SOURCE`
Replaces `CAMERA_MODEL_ON`.
- `'true'` (default): the true pose, on time, no noise. Checked: identical to the pre-merge build (`Linearized MPC observer`, Dana 8 s, every digit).
- `'robot_kalman'`: every controller reads its pose from `RobotStateEstimator` (below). The camera is noisy, and late when `camera_delay_on`. Section 7.
- `'linearized'`: Henry's 1 ms observers on the 2nd core (the `Linearized MPC observer ...` options) read the camera. The other controllers read the true pose. Section 6.
- `ROBOT_KALMAN_USE_IMU` (default False, as on the robot): also fuse the IMU's accelerometer.
- `ROBOT_KALMAN_VISION_TIME`: `'capture'` (late frames replayed at their own time) or `'config'` (the robot's timing: `vision_delay_ms` −80 → arrival time, no compensation).

## The port (ControllerModels, new cells after `RobotObserverEKF`)
From Rustware master 2e48c6d9 (cloned to `Documents/THE-BOTS/Rustware`):
- `common_robot_kalman/src/lib.rs`: `RobotKalmanCore` (Kalman) and `RobotKalman`
  - 9 states, constant acceleration, jerk process noise with one gain per derivative
  - heading wrapped
  - 300 ms buffer: an out-of-order insert means a rebuild from the base, plus a rebuild every 64 incremental batches
  - waits for the first vision frame before it starts
- `firmware-app/src/state_estimation.rs`: `RobotStateEstimator`
  - speed-dependent noise (quadrature of linear and angular samples)
  - gyro; accelerometer rotated to global, with near-stop attenuation; encoders (no-slip body velocity) rotated to global
  - each sensor has a fuse flag
  - the firmware's tick order
  - the controllers read the estimate at +5 ms (`state_lookahead`)
- `config.toml`: every setting (`ROBOT_STATE_ESTIMATION_CONFIG`). The robot fuses vision, gyro and optical flow; encoders and linear IMU are off. The sim has no optical flow sensor.
- Left out: the IMU and encoder notch/low-pass filters (off or unused in the config), optical flow, the vision innovation clamp (None).
- Not ported literally: the output heading is made continuous for the controllers (the filter keeps it in ±π).

## Open loop (ModelChecks' three runs, camera 5 mm noise, 50 ms late; median / worst every 20 ms)
| Sensors | Pose | Velocity |
|---|---|---|
| vision + gyro (robot config), replayed | 19 / 70 mm | 271 / 1071 mm/s |
| same, config timing (no delay comp.) | 30 / 101 mm | 297 / 1224 mm/s |
| + linear IMU | 2.4 / 15 mm | 5.9 / 245 mm/s |
| + encoders | 79 / 282 mm | 257 / 1599 mm/s (the wheels slip in these runs) |

## Closed loop, section 7 (camera 50 ms late, noisy; RMS cm, Dana 8 s / hops / eight 0.4 m 3.4 s)
| Controller | True pose | Kalman + IMU | Kalman, no IMU |
|---|---|---|---|
| PID tuned | 17.2 / 9.5 / 14.5 | 17.5 / 9.7 / 14.8 | 31.7 / 14.3 / 22.9 |
| Simple MPC | 2.0 / 1.8 / 1.9 | 2.2 / 2.0 / 2.0 | 2.2 / 2.1 / 2.5 |
| LQR | 0.5 / 0.5 / 0.4 | 0.6 / 0.8 / 0.6 | 8.9 / 6.9 / 8.6 (heading 30–37°) |
| Linearized MPC | 0.9 / 0.8 / 1.4 | 0.7 / 0.9 / 1.4 | 0.8 / 1.3 / 1.7 |

- With the IMU, every controller is within a few mm of its true-pose result.
- Without it, the MPCs barely change. The LQR and the PID get much worse: they act hard on the velocity they difference from the late, noisy pose.
- The controllers difference the filter's pose for their velocity. They do not read the filter's velocity (that would need a hook in each controller class).
- Run time: section 7 is about 5 min (4 controllers × 3 paths, twice).
