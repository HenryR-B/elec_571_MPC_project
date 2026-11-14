# Henry's test of the hand-gain IMU observer against the EKF (ControllerAnalysis section 6, 7d17ac4). Archived 11-14.


# ==============================================================================================================
# Hand-gain IMU observer against the EKF
# ==============================================================================================================
if POSE_SOURCE == 'linearized':     # section 6: Henry's linearized observers (settings)
    # The hand-gain IMU observer against the Kalman filter on the model
    ekf_controllers = ['Linearized MPC observer IMU', 'Linearized MPC EKF']
    ekf_results = run_and_report("Hand-gain observer against the Kalman filter", observer_paths, ekf_controllers)
    plot_paths(ekf_results, observer_paths, ekf_controllers)
    plot_errors(ekf_results, observer_paths, ekf_controllers)
