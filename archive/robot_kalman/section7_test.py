# ControllerAnalysis section 7 (archived 2026-11-13): every controller reading the robot's Kalman filter vs the exact pose.
# Needs the merged ControllerAnalysis setup cells and a runner with pose_source='robot_kalman'.

if POSE_SOURCE == 'robot_kalman':     # section 7: the robot's Kalman filter (settings)
    kalman_generator = PathGeneratorWithHold(dt=DT_CTRL)
    kalman_small_area = {'x_min': -0.5, 'x_max': 0.5, 'y_min': -0.5, 'y_max': 0.5, 'theta_min': -np.pi / 2, 'theta_max': np.pi / 2}
    kalman_paths = {
        "Dana 8 s": kalman_generator.generate_waypoints_sequence([np.zeros(3), np.array([3.0, 3.0, np.pi / 4])], segment_duration=8.0, segment_hold=HOLD_DURATION),
        "Quick hops (1.25 s)": kalman_generator.generate_random_sequence(num_segments=6, workspace_bounds=kalman_small_area, segment_duration=1.25, segment_hold=HOLD_DURATION, random_seed=7),
        "Figure eight 0.4 m (3.4 s)": add_eight_markers(figure_eight_path(0.4, 3.4), 0.4, markers_per_lap=7),
    }
    kalman_controllers = ['PID tuned', 'Simple MPC', 'LQR', 'Linearized MPC']
    true_pose_results = run_and_report("Perfect plant data (the exact pose)", kalman_paths, kalman_controllers, pose_source='true')
    kalman_results = run_and_report("The robot's Kalman filter (noisy camera)", kalman_paths, kalman_controllers,
                                    pose_source='robot_kalman')
    plot_paths(kalman_results, kalman_paths, kalman_controllers)
    plot_errors(kalman_results, kalman_paths, kalman_controllers)
