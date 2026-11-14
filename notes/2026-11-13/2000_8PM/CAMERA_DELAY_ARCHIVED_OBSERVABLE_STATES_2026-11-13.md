# Camera delay archived; controllers read measured states only (11-13, ~8:45 PM)

Henry:
- "we're not going to use camera delay"
- "it shouldn't ever get the exact data. that was meant to be a 'perfect plant data'. We need the linearized MPC to use actual observable states."
- "use the next prediction using the last measured state … and possibly the IMU". Default: IMU on.

## Archived
`archive/camera_delay/`: the five `merged/` notebooks as they were, with the delay (README there). Taken out of `merged/`:
- `OmniRobotCamera`: on time now (a noisy frame every 20 ms). The delay parameters are removed (`camera_delay_on`, `camera_delay`, `sigma_camera_delay`). The noise draws are the same as before.
- `RobotObserver1ms`: back to the version before the replay (the 7d17ac4 parent).
- `RobotObserver1msIMU` and `RobotObserverEKF`: no history, no replay; a frame corrects at once.
- `MPCControllerLinearized`: no `camera_replay`; `estimator_update(measured_State, camera_pose)`.
- `RobotStateEstimator`: no `vision_time`. Vision is stamped as the robot's config.toml says (arrival). Matt's 300 ms buffer is kept as he wrote it.
- ControllerAnalysis: the three "... no replay" controllers, and section 6's camera-delay cell.

## Pose source (ControllerAnalysis settings)
- `POSE_SOURCE = 'robot_kalman'` (default): every controller reads its pose from the robot's Kalman filter. Between frames the filter predicts from the last measured state (constant acceleration), corrected by the gyro, and by the IMU's accelerometer when `USE_IMU` (default True).
- `'linearized'`: the same, except the linearized MPC starts from its own 1 ms observer on the linearized model (built with `imu=USE_IMU`).
- `'true'`: perfect plant data, a reference only.
- **Estimator update:** except with `'true'`, the 1 ms readings handed to any estimator have their pose set to NaN. The estimators get the pose only from the camera frames.
- Checked: `pose_source='true'` still gives the pre-merge result to every digit. ModelChecks runs.
- What the linearized MPC reads by default (`'pose'` mode): the Kalman pose; velocity from Dana's 20 ms difference of that pose; measured currents and wheel speeds; bristles from the torque balance.
