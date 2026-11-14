# Camera delay (not used: Henry, "we're not going to use camera delay")

Only the delay pieces, as they were on 11-13:
- `camera_with_delay.py`: the camera handing each frame over `camera_delay` (50 ms) late.
- `observers_with_replay.py`: Henry's 1 ms observers (and the EKF) replaying a late frame from a 0.3 s history; `MPCControllerLinearized` then had a `camera_replay` option and `estimator_update(..., camera_pose, camera_age)`, with "... no replay" entries in the controller list.
- `section6_camera_cases.py`: the analysis cell comparing on time / late / late replayed.
- The robot's Kalman filter also had a capture-time option for late frames (`vision_time='capture'`); the port itself is in `archive/robot_kalman/`.

Results with the delay: `notes/2026-11-13/1900_7PM/KALMAN_DELAY_MERGE_2026-11-13.md`.
