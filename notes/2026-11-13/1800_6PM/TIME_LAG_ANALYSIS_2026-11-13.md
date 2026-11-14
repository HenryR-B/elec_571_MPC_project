# Time-lag analysis (Henry, 11-13, late)

New file: `merged/ControllerAnalysis_TimeLag.ipynb`. `merged/ControllerAnalysis.ipynb` is unchanged (it is the saved copy: Dana's fixed time and the earlier waypoint-executed run).

## The idea
- Henry's point: under fixed time, a controller that lags gets pulled toward a reference that has moved on, so it also looks inaccurate. Only a reference that slows down for it shows "lagging but accurate".
- The reference is the path. It plays at the path's pace, but never more than `LEAD_LIMIT` = 0.4 s of path ahead of the robot's point on the path (nearest path sample, searched from 10 samples behind the robot's last point up to the reference, so it does not jump loops).
- A time rule, not a distance rule: a PID's lag is about 1/gain in time (0.33 s) at any speed, so a 0.4 s lead does not cap its speed. A 5 cm leash capped it at about 0.15 m/s.
- Waypoints are only where the path stops (velocity zero). The reference waits there until the robot has settled (3 cm, 1 cm/s, 10°). A move, a turn or a figure eight has one waypoint, its end. Dana's waypoint path has three. No markers.
- Figure eights: new `figure_eight_stop_path` that starts and ends at rest at the origin (half-cosine ramp of 0.6 s), so its end is a waypoint.
- Two grades, kept apart: off the path (perpendicular, cm, time ignored) and heading against the path at the robot's point; time lost = settled at the last waypoint minus the scheduled end. The run ends at 2x the scheduled time (did not finish).
- Each shape at a normal and a cut pace; the grade plot is an arrow per controller from normal to cut, time lost (x) against off-path RMS (y).
- Code: `TimeLagExecution` (subclass of `WaypointExecution`), `run_time_lag`, `run_and_grade`, `plot_grade`, `plot_off_path_and_time`. The setup cells are copied unchanged from ControllerAnalysis.

## Full run (6 controllers, 10 paths, 297 s, one run at a time)
Off-path RMS (cm) / time lost (s), normal -> cut:

| Shape | PID tuned | Simple MPC | LQR | DOB MPC | Coulomb MPC | Linearized MPC |
|---|---|---|---|---|---|---|
| Dana's waypoints 5 -> 3.5 s | 0.98/7.5 -> 1.96/6.8 | 0.59/7.7 -> 4.98/2.9 | 0.24/1.7 -> 3.83/1.2 | 0.34/0.0 -> 4.59/1.4 | = Simple | 0.18/-0.1 -> 1.30/1.4 |
| Move 8 -> 4 s | 0.09/1.6 -> 4.11/3.1 | 0.76/0.3 -> 3.95/2.2 | 0.39/-0.1 -> 4.64/1.6 | 0.09/-0.1 -> 3.89/1.9 | = Simple | 0.05/-0.1 -> 2.42/1.4 |
| Turn 8 -> 5 s | 0.16/2.6 -> 1.10/4.7 | 0.67/0.1 -> 2.74/0.6 | 0.30/-0.2 -> 1.32/0.2 | 0.29/-0.2 -> 2.74/0.2 | = Simple | 0.05/-0.2 -> 3.22/0.3 |
| Eight 0.4 m 3.4 -> 2.5 s | 3.91/3.0 -> 6.43/2.9 | 1.55/0.1 -> 3.19/0.8 | 0.25/0.0 -> 4.03/0.7 | 0.24/0.1 -> 2.91/0.7 | ~ Simple | 0.51/0.0 -> 1.81/0.6 |
| Eight 0.3 m 3.0 -> 2.2 s | 3.60/1.7 -> 5.71/2.0 | 1.43/0.0 -> 2.05/0.1 | 0.24/0.0 -> 1.55/0.5 | 0.25/0.1 -> 1.30/0.3 | ~ Simple | 0.47/0.0 -> 1.34/0.1 |

## Notes on the results
- The linearized MPC holds up best when the time is cut on 4 of 5 shapes; not on the 5 s turn (3.2 cm, LQR 1.3 cm).
- LQR is the most accurate at the normal pace on the eights, and falls off most when cut (16 cm max on the 2.5 s eight and on Dana's 3.5 s waypoints).
- PID tuned cuts inside the eights even at the slow pace (3.6 to 3.9 cm): it chases a point about 0.4 s ahead with no velocity feedforward, so on a curve it cuts in. That is the PID's own behaviour, not the reference running away.
- Simple and Coulomb MPC lose 7.7 s on Dana's 5 s waypoints: about 4 s to settle at WP2 (inside 3 cm, 1 cm/s, 10°). Not investigated. Time lost includes settling at the waypoints.
- PID tuned reaches the plant's 15 A on the 4 s move and 5 s turn (13.6 to 15 A); the MPCs and LQR keep to 7.4 A. The PID does not know the controllers' limit.
- Still the perfect-model and perfect-pose case (11-12 notes): the linearized MPC knows the plant's parameters exactly.
