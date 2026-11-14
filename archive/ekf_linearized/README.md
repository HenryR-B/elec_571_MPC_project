# EKF on the linearized model (archived 2026-11-14)

Henry's `RobotObserverEKF` (commit 7d17ac4) and his test of it against the hand-gain IMU observer. Not used: Henry, "no ekf. that is kalman." The observer should use only the IMU and the camera, no linearized model. Results with it: notes/2026-11-13/1900_7PM/KALMAN_DELAY_MERGE_2026-11-13.md. It needs `MPCControllerLinearized`'s `ekf` option (see archive/imu_linearized_observer/changes.md, history).
