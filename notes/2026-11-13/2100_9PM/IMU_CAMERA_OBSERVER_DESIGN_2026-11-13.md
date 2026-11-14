# IMU + camera observer: design (11-13, ~9:45 PM; not built yet)

Henry: the observer uses no linearized model. It estimates the state from the measurements: the IMU every 1 ms and the camera every 20 ms. The linearized MPC needs a 1 ms observer, but the class that gives it the pose must estimate it, never take the true state. No EKF / Kalman (archived: `archive/ekf_linearized/`, `archive/robot_kalman/`).

## How it works
Two estimates of where the robot is, combined, then checked against the camera.

**Every 1 ms**
1. **Prediction (always):** move on linearly from the last estimate: position += velocity × 1 ms (heading += turn rate × 1 ms). With no IMU, this is all there is between camera frames.
2. **IMU guess (when the robot has one, `imu_on`):** the velocity changed by the measured acceleration rotated with the heading, velocity += 1 ms × R(heading) × accelerometer; the turn rate from the gyro; the position that follows from them.
3. **Combine:** estimate = prediction + `imu_trust` × (IMU guess − prediction).
   - `imu_trust` = 0: ignore the IMU (linear prediction only)
   - `imu_trust` = 1: trust the IMU fully
   - in between: a blend; the noisier the IMU (its noise is `imu_vibration_gain` × the load vibration, and `sigma_gyro`), the lower it should be

**Every 20 ms (camera frame: the pose in `plant.get_observable_state`, the true pose plus the camera noise, inside the plant: Dana, no separate camera call)**
- The camera's pose is a third, absolute guess. The estimate moves toward it by `camera_trust_pose`, and the velocity is corrected by `camera_trust_velocity` × (frame − estimate) / 20 ms (as in Henry's `RobotObserver1ms.correct_camera`).
- This is what stops the 1 ms estimates from drifting.

## Controller parameters (the observer's settings, not the plant's)
| Parameter | What it sets | Range |
|---|---|---|
| `imu_trust` | how much the IMU's guess counts against the linear prediction (the noisy IMU data) | 0 (ignore) to 1 (trust fully) |
| `camera_trust_pose` | how far the pose moves toward each camera frame | 0 to 1 |
| `camera_trust_velocity` | how much of the camera's position error goes into the velocity | 0 to 1 |

Values: not chosen yet. Either Henry's observer's (camera 1.0 / 0.5; IMU 1.0), or tuned on training paths, the same way for every setting.

## What the linearized MPC gets from it (its 20 states)
- pose and global velocity: from this observer
- lagged acceleration: the last accelerometer reading (with the IMU), or 0 / the model's value without it (to decide)
- motor currents and wheel speeds: measured directly
- bristle states z: from `BristleTorqueEstimator` (each wheel's torque balance; no linearization)

## Open
- the parameter values (above)
- how it plugs into the linearized MPC: a new subclass (e.g. `MPCControllerLinearizedIMU`) or a new `state_estimate` mode
- the lagged acceleration without the IMU

## Built (11-14)
`ImuCameraObserver` in `merged/ControllerModels.ipynb` (with the linearized model), as designed above. The linearized MPC uses it with `state_estimate='observer'`: pose and velocity from the observer; currents and wheel speeds measured; bristles from the torque balance; the lagged acceleration from those forces (the model's value, with or without the IMU). Dana's runner feeds it every plant step (`estimator_update`, the camera pose at the last sub-step). `RobotObserver1ms` (the linearized-model observer) was removed.

Defaults (not tuned): `imu_trust` 1.0, `camera_trust_pose` 1.0, `camera_trust_velocity` 0.5. First checks (open loop, against the true state, every 20 ms; velocity RMS):
- IMU on, exact camera: 0.6 mm/s (Dana's 20 ms pose difference: 25 to 35 mm/s)
- IMU off, exact camera: 75 to 86 mm/s, worse than the pose difference: the velocity is held between frames and only half corrected per frame
- 2 mm / 0.5 deg camera noise: 76 mm/s with the IMU (pose difference 190); the pose follows the camera fully (camera_trust_pose 1.0)
So the trust values need tuning (on training paths), per case (IMU on / off, camera noise).
Closed loop, Dana's section 1: linearized MPC 0.82 cm RMS with the observer, 0.84 with the camera pose and the pose difference.
