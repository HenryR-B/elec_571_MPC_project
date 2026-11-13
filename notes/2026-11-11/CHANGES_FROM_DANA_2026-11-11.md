# Changes from Dana's files to the copy, as of 2026-11-11

Dana's originals are in `dana-models/Project Design Folder/`; the changed copies are in `dana-mpc-copy-with-estimator/`. Each change is one line, with the commit it came in (from `git log`). Only code and markdown are compared, not cell outputs.

`PathGeneration.ipynb` is unchanged from Dana's.

## PlantModels.ipynb

- The ODESolve plant's code is Henry's merged plant: each wheel's motor, speed and bristle equations are solved together by elimination for all four wheels, repeated until the slip settles (e61421d).
- The plant's geometry is rewritten with named wheel angles, a `cross2d` helper for the yaw levers, and the load split and pitch/roll gains written as lever balances, with the same values as Dana's (e61421d).
- `self.H` is kept as an alias of `H_drive`, because Dana's runner uses `plant.H` (e61421d).
- The plant stores `self.p = params`, where Dana's copies it (`params.copy()`), so later changes to the parameter dict reach the plant (e61421d).
- Gravity is a simulation parameter, `'g': 9.81`, read by both plants (`params['g']`, `self.p['g']`), where Dana's file hard-codes 9.81 inside each plant (11-11, not yet committed).
- The ODESolve plant's slip flag (`state.S`) uses the `slip_band` parameter (0.005 m/s) its docstring names, where Dana's line hard-codes 0.05 (11-11, not yet committed).
- The simulation parameters are Henry's robot constants (e61421d).
- The explicit plant's bristle damping (σ₁) and viscous term (σ₂) are removed, so it uses the same friction law as the ODESolve plant (e61421d).
- The explicit plant is created with 100 µm bristles wherever it is built, because 2 µm is too stiff for it, and its markdown says so (e61421d).
- The explicit plant advances the global velocity and derives the local one, so a robot that turns while moving follows the right path (16aecc0).
- `roller_friction` is switched on (beb877e).
- Added an IMU: a new cell after the ODESolve plant with `OmniRobotObservableStateIMU` and `OmniRobotSimulationODESolveIMU` (a subclass, physics unchanged). Its observable state adds the accelerometer (the plant's `a_local_past[0:2]` plus the step's load vibration seen as an acceleration, `imu_vibration_gain · pinv(LT) w_F`) and the gyro (yaw rate plus `sigma_gyro` noise from its own random generator). The ODESolve plant keeps each step's load noise in `self.last_load_vibration` (one added line, plus its zero start); new parameters `'imu_vibration_gain': 1.0` and `'sigma_gyro': 0.002` (placeholder) (11-12, not yet committed).
- The markdown has Henry's merged plant's spec, flowchart and method list, plus a note that "ODESolve" is backward Euler, not an ODE solver (e61421d).
- A multi-stage sequence test and a friction validation section are added, and the existing tests' plots are reformatted (e61421d).
- Added a camera: a new cell after the IMU cell with `OmniRobotCamera`, a pose frame every `camera_period` (0.02 s), handed over `camera_delay` (0.05 s) late when `camera_delay_on` (False), the delay spread by `sigma_camera_delay` (0: fixed), from its own random generator. New parameters `'camera_period'`, `'camera_delay_on'`, `'camera_delay'`, `'sigma_camera_delay'` (11-12, not yet committed).
- The camera adds white noise to each frame's pose: new parameters `'sigma_camera_position': 0.005` (m) and `'sigma_camera_heading': 0.005` (rad), the vision noise the team's robot Kalman filter uses at rest (Rustware `robot_protocol/config/config.toml`) (11-12, not yet committed).

## ControllerModels_Estimator.ipynb

- Reset to an exact copy of Dana's `ControllerModels.ipynb` (0088ab5).
- Dana's simple MPC's velocity state is global instead of local (A_c blocks I, R(θ)A_vi, A_iv R(θ)ᵀ), which adds the turning term her model was missing (79cd5f4).
- Dana's `PoseVelocityEstimator` returns the global velocity rather than rotating it into the body frame (79cd5f4).
- Dana's MPC builds its starting state with the global velocity, and its slip diagnostic rotates that into the body frame first (79cd5f4).
- Dana's rotations (MPC model, PID command) use the plant's `_rotation`, with the same maths as before (79cd5f4).
- After 79cd5f4, none of Dana's classes changed again; everything since is new classes.
- Added `OmniRobotLinearizedModel`: a 20-state linear model of the plant's equations (pose, global velocity, currents, wheel speeds, bristles, lagged acceleration) (c91bd40 → eb1e35e).
- The model linearizes every 5 ms with batched finite differences, holds the bristle relax rate `d` in the Jacobians but refreshes it every 1 ms, and chains 20 backward-Euler 1 ms sub-steps into each 20 ms (A_d, B_d, c_d) (83cbe72).
- The model's wheel loads use the plant's previous-1-ms acceleration, kept as two extra states (83cbe72).
- Added `BristleTorqueEstimator`: the bristle state z from each wheel's torque balance, used by both the MPC and the observer (eb1e35e).
- Added `RobotObserver1ms`: a 1 ms observer that predicts with the model and corrects with wheel speeds and currents every 1 ms and the camera pose every 20 ms (83cbe72).
- Added `MPCControllerLinearized`: a subclass of Dana's MPC that reuses her optimization problem unchanged (one extra state fixed at 1 carries the offset c_j), linearizes along its last plan, and takes its starting state from `'pose'`, `'observer'` or `'true'` (c91bd40 → eb1e35e).
- After a failed solve, the linearized MPC starts OSQP cold (83cbe72).
- The linearized MPC's step 3 (setting A_j, B_j in Dana's problem) became its own method, `set_model_parameters`, with no change in behaviour (11-11, not yet committed).
- Added `MPCControllerLinearizedRiccati`: the linearized MPC with a Riccati terminal cost. Dana's fixed terminal weights are set to zero, and the weight on the pose and velocity error at the last step comes from the discrete Riccati equation of that step's model, recomputed every 20 ms (11-11, not yet committed).
- Added `FastPoseVelocityEstimator` and `MPCControllerSimple1msEstimator`: Dana's pose estimator run every 1 ms, for her simple MPC (c91bd40).
- Each new class has a markdown cell with a Subfunction box, in the same style as Dana's classes (eb1e35e).
- The header cell gets a **Controllers** line and lists the three added controllers (3-5); Dana's MPC entry gets two lines naming its prediction model and starting-state estimator. Each controller lists its own model and estimator under it. Dana's text is unchanged except two names, changed to the actual class names: `PIDCascadeController` → `CascadedPIDController`, `MPCControllerSimpleModel` → `MPCControllerSimple` (11-12, not yet committed).
- `OmniRobotLinearizedModel.get_roller_factor_batch` renamed `get_roller_friction_per_load_batch` (its returned value `roller_friction_per_load`), in the code, the docstring and the Subfunction box. No change in behaviour (11-12, not yet committed).
- `OmniRobotLinearizedModel` readability: in `get_bristle_decay_batch` and `get_wheel_loads_batch` each input is named on its own line before it is used (no nested calls or inline state slices); `get_rotation_matrix` now calls `get_rotation_matrix_batch` with one heading, as `get_wheel_loads` calls `get_wheel_loads_batch` (it used the plant's `_rotation`; same matrix); `get_rotation_matrix_batch` writes R(theta) out as a matrix with `np.cos(theta)`/`np.sin(theta)` instead of filling entries from `c`, `s` (checked identical). No change in behaviour (11-12, not yet committed).
- Added `RobotObserver1msIMU` (new cell after `RobotObserver1ms`, a subclass): every 1 ms the accelerometer sets the lagged acceleration and the velocity (last step's velocity + dt R(theta) a_imu), the gyro the yaw rate, each with a gain (default 1) (11-12, not yet committed).
- `MPCControllerLinearized` takes `imu=False`; with `imu=True` its observer is `RobotObserver1msIMU`. Its `estimator_update` takes the camera pose (`camera_pose`, None between frames) instead of a `camera_frame` flag, so a late frame can be passed in (11-12, not yet committed).
- `RobotObserver1ms` replays late camera frames (`camera_replay=True`, history 0.3 s): `update` takes `pose_age`, and is split into `advance` (predict + sensors), `remember` (readings, estimate, bristle-estimator memory per step) and `correct_camera_late` (correct the kept estimate at the frame's step, advance again to now). On-time frames behave as before. `RobotObserver1msIMU` follows the same split. `MPCControllerLinearized` takes `camera_replay` and its `estimator_update` takes `camera_age` (11-12, not yet committed).
- Added `RobotObserverEKF` (new cell after `RobotObserver1msIMU`; the old observers are kept): a Kalman filter on the 20-state linearized model, built like the robot's `RobotKalman` (Rustware `common_robot_kalman`): Kalman updates for wheel speeds, currents, accelerometer (a_past), gyro (yaw rate) every 1 ms and the camera pose per frame, noise from the robot's config.toml where it has one, late frames replayed from a 0.3 s history. Process noise tuned on training runs. `MPCControllerLinearized` takes `ekf=False` (11-12, not yet committed).

## ControllerAnalysis_Estimator.ipynb

- Reset to an exact copy of Dana's `ControllerAnalysis.ipynb` (0088ab5).
- Loads `ControllerModels_Estimator.ipynb` instead of `ControllerModels.ipynb` (79cd5f4).
- The MPC gets the global velocity reference; the local one is now only logged, using the plant's rotation (79cd5f4).
- Bryson's-rule weights replace Dana's hand-picked MPC weights, with a light voltage cost and the main cost on voltage changes (db9a404; renamed in beb877e).
- The explicit plant is created with 100 µm bristles (db9a404).
- A markdown note and code comment state the sensing assumption: the pose is given every step with no delay, unlike the real 60 Hz camera that is about 50 ms late (db9a404).
- Added an `'mpc_no_vel'` option: Dana's MPC with no velocity cost (beb877e).
- Added a test-motions cell (random paths, quadrants, quick hops, figure eights) with an off-path measure and a wheel-speed check (beb877e).
- Added Dana's turning test, (0,0,0) to (3,3,π/2) and (3,3,π) (beb877e).
- Added an `'mpc_1ms'` option and a comparison cell for Dana's MPC with the 1 ms velocity estimate (c91bd40).
- The "Sliding (%)" columns of the added cells count a wheel as sliding above the plant's `slip_band`, read from the parameters (11-11, not yet committed).

## ControllerAnalysis_Linearized.ipynb (new file)

- A fast loader and a copy of Dana's runner, with `'mpc_lin'`, `'mpc_lin_obs'` and `'mpc_lin_true'` added; for the observer option the runner feeds the observer every plant step (c91bd40 → eb1e35e).
- Its Bryson block has a separate 5° heading tolerance, which only this file has (83cbe72).
- Model checks: step 1 (f against the plant), step 2 (Jacobians against hand-worked entries), step 3 (0.4 s predictions against the plant and against Dana's model) (c91bd40 → 83cbe72).
- Closed-loop tests: Dana's benchmark, the test motions (quick hops, 0.4 m figure eight at 3 s and at 3.4 s per lap) and her turning test (c91bd40 → 83cbe72).
- The path plots have heading arrows, and each position-error plot has a heading-error plot under it (83cbe72).
- An observer test: its estimate against the plant's true state, compared with Dana's 20 ms pose difference (83cbe72).
- Its "Sliding (%)" column counts a wheel as sliding above the plant's `slip_band`, read from the parameters (11-11, not yet committed).
- The acceptable voltage `max_V` (the MPCs' light cost on the voltage itself) comes from one rule: the voltage that holds a 40 rad/s wheel speed and pushes at the grip limit, `Kb·40 + R·μs(Mr·g/4)·r/Kt`. That gives 11.7 V on the Pololu (it was 6 V) and 2.7 V on the BLDC (it was 6.4 V). Only this file changed; `ControllerAnalysis_Estimator.ipynb` still uses 6 V (11-11, not yet committed).
- The test-motion figure eights are set at this motor's limit: 2.8 s per lap (inside it) and 2.5 s (past it), in place of 3 s and 3.4 s. The Pololu's limit is top speed (back-EMF against the 12 V supply), not grip; the markdown above the cell gives the numbers. The BLDC copy keeps 3.4 s and 3 s, because its limit is grip (11-11, not yet committed).
- Added `'mpc_lin_riccati'` to the runner. The three test cells now run `'pid_tuned'`, `'mpc'`, `'mpc_lin'` and `'mpc_lin_riccati'`; `'pid'` and `'mpc_lin_obs'` are left out until the camera delay is added (they stay in the runner) (11-11, not yet committed).
- The runner builds the IMU plant (`OmniRobotSimulationODESolveIMU`, same motion as before) and a camera (`OmniRobotCamera`); the observer options get the pose from the camera, so `camera_delay_on` delays it for them only. Added `'mpc_lin_obs_imu'` (the observer with the IMU). With the delay off, `'mpc_lin_obs'` gives the same result as before, to the final position (11-12, not yet committed).
- Added a last cell comparing `'mpc_lin_obs'` and `'mpc_lin_obs_imu'` on the benchmark and test-motion paths, with `camera_delay_on` set in the cell (11-12, not yet committed).
- Runner options `'mpc_lin_obs_noreplay'` and `'mpc_lin_obs_imu_noreplay'` (late frames used as if on time); the observer options pass the frame's age (`camera.last_frame_delay`). Added a last cell comparing the IMU observer with a perfect camera, late frames with no replay, and late frames replayed (11-12, not yet committed).
- Runner options `'mpc_lin_ekf'` and `'mpc_lin_ekf_noreplay'`; a last cell compares `'mpc_lin_obs_imu'` and `'mpc_lin_ekf'` with the noisy, 50 ms late camera (11-12, not yet committed).

## Kept as they are

- Plant limits and controller limits stay separate: `V_max` (the supply) and `I_max` (the motor driver) are simulation parameters, while `voltage_limit`, `current_limit` and `delta_v_max` are each controller's own settings, passed in the runner.
- Dana's `OmniRobotNominalModel` copies `V_max` and `I_max` from the parameters but never uses them (her MPC's constraints use `voltage_limit` and `current_limit`).
- Dana's explicit plant never sets its slip flags (`state.S` stays `False`); nothing in the runs reads them.
- The ODESolve plant does not clip the motor current at `I_max`; the explicit plant does.
