# ==============================================================================================================
# merged-executed-md
# ==============================================================================================================
## 5. Waypoint-executed (Henry): who gets there first

The same controllers with `WaypointExecution`: the controller drives the reference's progress instead of a fixed timeline. At each waypoint the reference waits until the robot has settled there (3 cm, 1 cm/s, 10°; no hold), then starts the next segment. On the figure eights it waits at each of the 7 markers per lap until the robot is within 3 cm (no need to stop), then carries on along the eight. Until then the controller sees the reference only up to that point. Every run bottoms out at the scheduled end plus `WAYPOINT_TIME_ALLOWANCE`: a slow controller still tracks the path but does not reach what it has not reached by then.

Compare with sections 1 to 3 (Dana's fixed time: who is better in the same time).


# ==============================================================================================================
# merged-executed-code
# ==============================================================================================================
executed = WaypointExecution(time_allowance=WAYPOINT_TIME_ALLOWANCE)
executed_paths = {
    "Dana's waypoints (no hold)": PathGeneratorCurvedWithHold(dt=DT_CTRL).generate_waypoints_sequence(WAYPOINTS, segment_duration=5.0, segment_hold=0.0),
    "Dana 8 s (no hold)": path_generator.generate_waypoints_sequence([np.zeros(3), np.array([3.0, 3.0, np.pi / 4])], segment_duration=8.0, segment_hold=0.0),
    "Dana 4 s (too fast, no hold)": path_generator.generate_waypoints_sequence([np.zeros(3), np.array([3.0, 3.0, np.pi / 4])], segment_duration=4.0, segment_hold=0.0),
    "Figure eight 0.4 m (3.4 s)": add_eight_markers(figure_eight_path(0.4, 3.4), 0.4, markers_per_lap=7),
    "Figure eight 0.4 m (2.5 s)": add_eight_markers(figure_eight_path(0.4, 2.5), 0.4, markers_per_lap=7),
    "Figure eight 0.3 m (2.2 s)": add_eight_markers(figure_eight_path(0.3, 2.2), 0.3, markers_per_lap=7),
}
executed_controllers = list(WAYPOINT_CONTROLLERS)     # the same controllers in every test
executed_results = run_and_report("Waypoint-executed (Henry)", executed_paths, executed_controllers, executor=executed)
plot_paths(executed_results, executed_paths, executed_controllers)
plot_errors(executed_results, executed_paths, executed_controllers)
plot_waypoint_timing(executed_results, executed_paths, executed_controllers)
plot_waypoint_closeups(executed_results, executed_paths, executed_controllers)
plot_marker_times(executed_results, executed_paths, executed_controllers)
