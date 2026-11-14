# ==============================================================================================================
# merged-benchmark-md
# ==============================================================================================================
## 2. Dana's benchmark

Her first path: (0, 0, 0) to (3, 3, π/4) in 8 s and in 4 s, now with a hold at the end. The 4 s path needs more wheel speed than 12 V gives, so every controller falls behind on it.


# ==============================================================================================================
# merged-benchmark-code
# ==============================================================================================================
path_generator = PathGeneratorWithHold(dt=DT_CTRL)
benchmark_paths = {
    "Dana 8 s": path_generator.generate_waypoints_sequence([np.zeros(3), np.array([3.0, 3.0, np.pi / 4])], segment_duration=8.0, segment_hold=HOLD_DURATION),
    "Dana 4 s (too fast)": path_generator.generate_waypoints_sequence([np.zeros(3), np.array([3.0, 3.0, np.pi / 4])], segment_duration=4.0, segment_hold=HOLD_DURATION),
}
benchmark_controllers = list(WAYPOINT_CONTROLLERS)     # the same controllers in every test
benchmark_results = run_and_report("Dana's benchmark", benchmark_paths, benchmark_controllers)
plot_paths(benchmark_results, benchmark_paths, benchmark_controllers, columns=2)
plot_errors(benchmark_results, benchmark_paths, benchmark_controllers, columns=2)
plot_effort(benchmark_results, benchmark_paths, benchmark_controllers, columns=2)
plot_waypoint_timing(benchmark_results, benchmark_paths, benchmark_controllers, columns=2)
plot_waypoint_closeups(benchmark_results, benchmark_paths, benchmark_controllers)


# ==============================================================================================================
# merged-motions-md
# ==============================================================================================================
## 3. Test motions

Henry's attainable paths, from Dana's generator with a hold at each waypoint: her arena (random waypoints, 10 s segments) and her quadrant waypoints (12 s segments), quick hops in a ±0.5 m area (1.25 s segments), and Henry's continuous figure eights (two laps; the lap time in brackets). On the eights, 7 markers per lap equally spaced along the eight are a visual check only (the path does not stop or wait at them): the plots show where each controller was as the reference passed each marker, and how far off it was.

The waypoint paths are waypoint-executed: the path waits at each waypoint until that controller has settled, so each controller has its own timeline.

The peak wheel speed each path needs is a path check, in `ModelChecks.ipynb`. For this motor (Pololu) the limit is top speed, not grip: the 0.4 m eight at 3.4 s per lap needs about 9.3 V of back-EMF of the 12 V (inside the limit); at 2.5 s it needs 12.6 V (past it, so it should fall behind on the fast parts). The 0.3 m eight at 2.2 s is the tight one (about 5 m/s²).

"Perp" and "Lag" split the error into the distance from the track and the distance behind along it (RMS). "Settled at" gives the time it settled at each waypoint, on its own timeline; on the eights, how far it was from the markers as the reference passed them (mean and worst).


# ==============================================================================================================
# merged-motions-code
# ==============================================================================================================
arena_half = 9.0 / 2.0
arena_margin = 0.5
workspace_9m = {'x_min': -arena_half + arena_margin, 'x_max': arena_half - arena_margin,
                'y_min': -arena_half + arena_margin, 'y_max': arena_half - arena_margin, 'theta_min': -np.pi, 'theta_max': np.pi}
quadrant_waypoints = [np.array([0.0, 0.0, 0.0]), np.array([3.2, 2.8, np.radians(90.0)]), np.array([-2.5, 3.5, np.radians(180.0)]),
                      np.array([-3.5, -2.8, np.radians(-90.0)]), np.array([2.8, -3.2, 0.0])]
small_area = {'x_min': -0.5, 'x_max': 0.5, 'y_min': -0.5, 'y_max': 0.5, 'theta_min': -np.pi / 2, 'theta_max': np.pi / 2}

motion_paths = {
    "Random (10 s segments)": path_generator.generate_random_sequence(num_segments=4, workspace_bounds=workspace_9m, segment_duration=10.0, segment_hold=HOLD_DURATION, random_seed=42),
    "Quadrants (12 s segments)": path_generator.generate_waypoints_sequence(quadrant_waypoints, segment_duration=12.0, segment_hold=HOLD_DURATION),
    "Quick hops (1.25 s)": path_generator.generate_random_sequence(num_segments=6, workspace_bounds=small_area, segment_duration=1.25, segment_hold=HOLD_DURATION, random_seed=7),
    "Figure eight 0.4 m (3.4 s)": add_eight_markers(figure_eight_path(0.4, 3.4), 0.4, markers_per_lap=7),
    "Figure eight 0.4 m (2.5 s)": add_eight_markers(figure_eight_path(0.4, 2.5), 0.4, markers_per_lap=7),
    "Figure eight 0.3 m (2.2 s)": add_eight_markers(figure_eight_path(0.3, 2.2), 0.3, markers_per_lap=7),
}
motion_controllers = list(WAYPOINT_CONTROLLERS)     # the same controllers in every test
motion_results = run_and_report("Test motions", motion_paths, motion_controllers)
plot_paths(motion_results, motion_paths, motion_controllers)
plot_errors(motion_results, motion_paths, motion_controllers)
plot_effort(motion_results, motion_paths, motion_controllers)
plot_waypoint_timing(motion_results, motion_paths, motion_controllers)
plot_waypoint_closeups(motion_results, motion_paths, motion_controllers)
plot_marker_misses(motion_results, motion_paths, motion_controllers)


# ==============================================================================================================
# merged-turn-md
# ==============================================================================================================
## 4. Dana's turning test

Move from (0, 0, 0) to (3, 3, π/2) while turning, and shorten the time; also a half turn, (3, 3, π). Each ends with a hold.

Shorter times need more wheel speed (the turn adds to it). The peak wheel speed each case needs, as a share of the top speed at 12 V (about 47 rad/s; from the path check in `ModelChecks.ipynb`). Above 100% the robot cannot keep up:

| Move | 8 s | 5 s | 4 s |
|---|---|---|---|
| to (3, 3, π/2) | 59% | 95% | 118% (too fast) |
| to (3, 3, π) | 75% | 120% (too fast) | 150% (too fast) |

One row of plots per move: 8 s, 5 s and 4 s.

`'Simple MPC no vel'` is the simple MPC with no velocity term in the stage cost (Dana's idea).


# ==============================================================================================================
# merged-turn-code
# ==============================================================================================================
turn_cases = [(np.pi / 2, 8.0), (np.pi / 2, 5.0), (np.pi / 2, 4.0), (np.pi, 8.0), (np.pi, 5.0), (np.pi, 4.0)]   # one row each: 8 s, 5 s, 4 s
turn_paths = {f"to (3, 3, {'pi/2' if theta_end < 2 else 'pi'}) in {duration:.0f} s":
              path_generator.generate_waypoints_sequence([np.zeros(3), np.array([3.0, 3.0, theta_end])], segment_duration=duration, segment_hold=HOLD_DURATION)
              for theta_end, duration in turn_cases}
turn_controllers = list(WAYPOINT_CONTROLLERS) + ['Simple MPC no vel']     # the same controllers, plus Dana's no-velocity-cost MPC
turn_results = run_and_report("Turning test", turn_paths, turn_controllers)
plot_paths(turn_results, turn_paths, turn_controllers, columns=3)
plot_effort(turn_results, turn_paths, turn_controllers, columns=3)
plot_waypoint_timing(turn_results, turn_paths, turn_controllers, columns=3)
plot_waypoint_closeups(turn_results, turn_paths, turn_controllers)
