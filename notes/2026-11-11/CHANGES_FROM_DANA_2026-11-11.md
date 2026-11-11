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
- The markdown has Henry's merged plant's spec, flowchart and method list, plus a note that "ODESolve" is backward Euler, not an ODE solver (e61421d).
- A multi-stage sequence test and a friction validation section are added, and the existing tests' plots are reformatted (e61421d).

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
- Added `FastPoseVelocityEstimator` and `MPCControllerSimple1msEstimator`: Dana's pose estimator run every 1 ms, for her simple MPC (c91bd40).
- Each new class has a markdown cell with a Subfunction box, in the same style as Dana's classes (eb1e35e).

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

## Kept as they are

- Plant limits and controller limits stay separate: `V_max` (the supply) and `I_max` (the motor driver) are simulation parameters, while `voltage_limit`, `current_limit` and `delta_v_max` are each controller's own settings, passed in the runner.
- Dana's `OmniRobotNominalModel` copies `V_max` and `I_max` from the parameters but never uses them (her MPC's constraints use `voltage_limit` and `current_limit`).
- Dana's explicit plant never sets its slip flags (`state.S` stays `False`); nothing in the runs reads them.
- The ODESolve plant does not clip the motor current at `I_max`; the explicit plant does.
