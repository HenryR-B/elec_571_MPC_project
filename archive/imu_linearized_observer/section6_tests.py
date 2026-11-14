# ControllerAnalysis section 6 (Henry's observer comparisons, from his ControllerAnalysis_Linearized.ipynb, 7d17ac4).
# New cells of merged/ControllerAnalysis.ipynb; they ran with POSE_SOURCE = 'linearized'.


# ==============================================================================================================
# Markdown
# ==============================================================================================================
## 6. Henry's linearized observers: without and with the IMU, and the Kalman filter on the model (Henry, 11-12)

Ran only with `POSE_SOURCE = 'linearized'` (the 3-way setting, deleted 11-14). From Henry's `ControllerAnalysis_Linearized.ipynb` (commit 7d17ac4), on his paths: Dana's 8 s and 4 s, the quick hops and the two 0.4 m eights. The observers take the camera frames (every 20 ms, on time) and the 1 ms readings, never the plant's pose. (The camera delay cases are in `scratch/camera_delay/`.)

* **Without and with the IMU:** `RobotObserver1ms` against `RobotObserver1msIMU` (the accelerometer sets the lagged acceleration and the velocity, the gyro the yaw rate).

Caveats (11-12 notes): these observers predict with the linearized model, an exact copy of the plant's (perfect model), and re-linearize every 1 ms. (The robot's own Kalman filter is in `scratch/robot_kalman/`.)


# ==============================================================================================================
# Observer without and with the IMU
# ==============================================================================================================
if POSE_SOURCE == 'linearized':     # section 6: Henry's linearized observers (settings)
    # Henry's observer test paths: Dana's benchmark (8 s, 4 s), the quick hops and the two 0.4 m eights
    observer_generator = PathGeneratorWithHold(dt=DT_CTRL)
    observer_small_area = {'x_min': -0.5, 'x_max': 0.5, 'y_min': -0.5, 'y_max': 0.5, 'theta_min': -np.pi / 2, 'theta_max': np.pi / 2}
    observer_paths = {
        "Dana 8 s": observer_generator.generate_waypoints_sequence([np.zeros(3), np.array([3.0, 3.0, np.pi / 4])], segment_duration=8.0, segment_hold=HOLD_DURATION),
        "Dana 4 s (too fast)": observer_generator.generate_waypoints_sequence([np.zeros(3), np.array([3.0, 3.0, np.pi / 4])], segment_duration=4.0, segment_hold=HOLD_DURATION),
        "Quick hops (1.25 s)": observer_generator.generate_random_sequence(num_segments=6, workspace_bounds=observer_small_area, segment_duration=1.25, segment_hold=HOLD_DURATION, random_seed=7),
        "Figure eight 0.4 m (3.4 s)": add_eight_markers(figure_eight_path(0.4, 3.4), 0.4, markers_per_lap=7),
        "Figure eight 0.4 m (2.5 s)": add_eight_markers(figure_eight_path(0.4, 2.5), 0.4, markers_per_lap=7),
    }

    # The 1 ms observer without and with the IMU
    imu_controllers = ['Linearized MPC observer', 'Linearized MPC observer IMU']
    imu_results = run_and_report("Observer without and with the IMU (camera on time)", observer_paths, imu_controllers)
    plot_paths(imu_results, observer_paths, imu_controllers)
    plot_errors(imu_results, observer_paths, imu_controllers)
