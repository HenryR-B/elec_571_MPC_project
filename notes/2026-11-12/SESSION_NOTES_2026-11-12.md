# Session notes, 2026-11-12

All changes are uncommitted (Henry commits). Each code change is also in `notes/2026-11-11/CHANGES_FROM_DANA_2026-11-11.md`.
Scripts from today are in this folder.

## The big finding at the end of the day (start here tomorrow)

**The model-based observers and the linearized MPC use an exact copy of the plant's model.** `OmniRobotLinearizedModel` is
built from the same `simulation_parameters` as the plant (same friction, bristles, motor, geometry). So:
- the 1 ms observers (`RobotObserver1ms`, `RobotObserver1msIMU`, `RobotObserverEKF`) predict with the plant's own equations,
  and their good results are largely because they "know" the plant exactly;
- `'mpc_lin'` likewise knows the true friction; part of its edge over Dana's MPC may be that.
Results from these are an upper bound (perfect model). Henry was rightly annoyed: he thought the results were earned.

**The observers also re-linearize every 1 ms** (`predict` calls `get_substep_matrices`, a Jacobian each step): 5x more
often than the MPC (every 5 ms), not realistic on the robot, and it ties the observer to the linearized MPC's model, so
Dana's MPC cannot share it cleanly. Henry's view: the observer should be like Matt's (no robot model).

**My mistake:** asked to base the observer on Matt's filter, I built `RobotObserverEKF` on the MPC's physics model
instead, without asking. See memory `copy-the-reference-faithfully`.

### What each piece does now

| Piece | Rate | Model / Jacobians |
|---|---|---|
| Plant `OmniRobotSimulationODESolve` (+ `...IMU`, `OmniRobotCamera`) | 1 ms | the true robot, nonlinear; no Jacobians |
| Dana's MPC `'mpc'` | 20 ms | 10 states, her A, B formulas at predicted headings; no Jacobians |
| Linearized MPC `'mpc_lin'` | 20 ms | 20 states, Jacobians every 5 ms along the last plan, d every 1 ms |
| x0 for `'mpc'`, `'mpc_lin'`, `'pid_tuned'` (the main tests) | 20 ms | current pose, NO delay or noise; velocity = 20 ms pose difference |
| `RobotObserver1ms`, `...IMU`, `RobotObserverEKF` | 1 ms | linearized model, Jacobian EVERY 1 ms |
| Matt's robot filter (Rustware) | per measurement | 9-state kinematics, no robot model, no Jacobians |

Also: the camera delay and noise only reach the observer options; the main benchmark still sees a perfect camera.

### Proposed next steps (ask Henry before each)
1. **Fair test:** plant with its own parameters (friction, rolling resistance, motor constants off 10-20%), controllers and
   observers keep the nominal ones. Shows how much of the linearized MPC's and the observers' gains are real.
2. **Matt-style observer (faithful port), as a new class:** 9-state kinematic KF [x, vx, ax, y, vy, ay, theta, omega, alpha],
   his jerk process noise and config.toml noise, vision/IMU/gyro/encoder updates, 300 ms timestamp buffer with rebuild.
   The linearized MPC's extra states: currents and wheel speeds measured, a_past = IMU, z = torque balance
   (`BristleTorqueEstimator`, no linearization). Serves PID and both MPCs. Open questions for Henry: confirm this fill-in;
   optical flow (our sim has none) leave out or add.
3. Then every controller reads that one observer, with the camera delay and noise applied to all of them.

## Done today

### Linearized model: what is exact and what is approximated (answered)
- `a_past` is not physics: the plant keeps the previous 1 ms step's acceleration to avoid an algebraic loop (exact
  one-step shift). The model turns it into a 1 ms lag; backward Euler with h = tau gives `a_past_next = (a_past + a_next)/2`,
  not the plant's shift. Load transfer matters a lot (Henry: already tested); choice between `a_past` and a direct load
  solve (4x4 linear solve, forces are proportional to F_n) is open. With an IMU, `a_past` is measured.
- Trust ranking: I and w (measured, near-linear rows) > pose (measured 20 ms, exact row, integrates v) > z, a_past
  (unmeasured) > v (unmeasured, nonlinear row, integrates; z ~ v_s/d couples v errors into z and back).

### Solver DLLs
Smart App Control still On, but the block has cleared; cvxpy 1.9.3 + OSQP 1.1.3 on Python 3.14 work, closed loop runs.

### IMU (PlantModels.ipynb, cell `7b52cdf3`)
- `OmniRobotSimulationODESolveIMU` (subclass, same motion as the parent, checked) and `OmniRobotObservableStateIMU`.
- Accelerometer = plant's `a_local_past[0:2]` + `imu_vibration_gain * pinv(LT) w_F` (the same draw as the load vibration,
  so they rise and fall together; LT already contains m*h/wheelbase, no extra 1/m). Gyro = yaw rate + `sigma_gyro`.
- The plant keeps each step's load noise in `self.last_load_vibration` (one added line).
- Params: `imu_vibration_gain` 1.0 (gives ~0.1 m/s^2; Rustware uses 0.05, so ~0.5 would match), `sigma_gyro` 0.002.

### Camera (PlantModels.ipynb, cell `311c95cf`)
- `OmniRobotCamera`: a frame every `camera_period` 0.02 s; `camera_delay_on` (False), `camera_delay` 0.05 s,
  `sigma_camera_delay` 0; pose noise `sigma_camera_position` 0.005 m and `sigma_camera_heading` 0.005 rad.
- Noise values are Rustware's (master, `crates/robot_protocol/config/config.toml`): the robot Kalman filter's assumed vision
  noise at rest, a tuning value, not a measurement (heading grows with speed there, not modelled). Henry: use Rustware's
  values, not our own estimates (memory `use-rustware-values`). The firmware also measures vision variance live
  (`firmware-app/src/state_estimation.rs`), so robot logs might give a measured value.
- Henry's kick data: spread at a fixed pulse 0.03-0.05 m/s slow, 0.08-0.1 fast; >50% from the robot, so vision speed std
  < 0.02-0.035 / < 0.07 m/s. Cannot give position noise (ball speed is the ball Kalman's output).

### Observers (ControllerModels_Estimator.ipynb) - see the big finding above
- `RobotObserver1msIMU` (new cell): accel sets a_past and the velocity (IMU integral), gyro sets yaw rate; gains 1.
- Camera replay added to `RobotObserver1ms` (and the IMU one): `update(..., pose_age)`, split into `advance`, `remember`,
  `correct_camera_late`; `camera_replay=True`, 0.3 s history. On-time frames behave as before (checked: `'mpc_lin_obs'`
  identical to before, to the final position).
- `RobotObserverEKF` (cell `ca2b33c0`): Kalman gains, Rustware noise, replay with covariance; Q tuned on training runs
  (`ekf_tune.py`). NOT realistic (Jacobian every 1 ms, ~12 ms per replay in Python; ~0.25 ms normal step).
- `MPCControllerLinearized` takes `imu`, `ekf`, `camera_replay`; `estimator_update(measured_State, camera_pose, camera_age)`.

### Results (all with the perfect-model caveat above)
- Open loop, delay off: IMU cuts worst velocity error 11-16 -> 1.4 mm/s, yaw rate ~15x.
- Late camera (noiseless): no replay pose 44-53 mm worst, velocity 210-290 mm/s; IMU + replay 0.1 mm, <3 mm/s.
  Closed loop eights: replayed = perfect camera; no replay 2x worse on the 2.5 s eight (`camera_replay_check.py`).
- With 5 mm camera noise the hand gains (1.0/0.5) give ~180 mm/s velocity error; 0.1/0.01 gives 6.5/19 mm/s
  (defaults NOT changed). EKF: velocity 0.3-3.6 mm/s median, pose median 4.4 mm on 2 of 3 runs (open question).
  Closed loop, noisy late camera: EKF 7.3 cm vs hand default 17.8 cm on the 2.5 s eight; Dana 4 s unreliable (saturated).

### Constraint use (`constraint_activity_check.py`)
Only the 12 V limit is ever active; current (20 A) and slew (5 V) ~never. 0% on Dana 8 s, hops, 3.4 s eight; ~50% Dana 4 s,
15-20% 2.5 s eight. So on paths within the limits the MPC is an unconstrained finite-horizon LQR with preview. Henry: then
why MPC over infinite LQR? Fair test: Riccati gain + clipping vs MPC on the saturated paths. Real cases for MPC:
a total-current limit (Henry: ~15-20 A for the whole robot; our Pololu peaks 0.7-1.7 A per motor, a BLDC may reach it;
sum|I| <= 15 is convex) and a grip limit.

### Other answers
- Reusing Dana's `compute_optimal_u0`: only its solve block (~20 lines) is shareable; splitting her method into
  get_state_estimate / set_model / solve_qp would change her class, so ask Dana first. Not done.

## Notebook cells added (not run inside the notebooks; same runs done in scripts)
`ControllerAnalysis_Linearized.ipynb`, last cells: IMU vs no IMU; camera perfect / late no replay / late replayed;
hand-gain observer vs EKF. The runner now builds the IMU plant and the camera for every run.

## Scripts in this folder
`held_loads_test.py`, `imu_observer_check.py`, `camera_replay_check.py`, `constraint_activity_check.py`, `ekf_tune.py`
(run from `dana-mpc-copy-with-estimator`).
