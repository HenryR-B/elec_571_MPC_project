# Current status (read this first, every session)

Last updated: 2026-11-14 (early morning, the 11-13 session). Keep this file short; details go in the dated folders in `notes/`.

## Where the work is
- Active files: `merged/` (PlantModels, ControllerModels, ControllerAnalysis, PathGeneration). It was reset to Dana's files (commit bf208dd); changes come back **one topic at a time, as Henry picks them**. Commit 489a3c3 has the IMU, global velocity, noise settings.
- `scratch_before/` (the 11-13 merge vs Dana's) and `scratch/` (work after it): untracked, each split into `models/` and `analysis/`, each topic with a README / `changes.md` (before and after). Delete a piece only after a full diff shows it is applied or filed elsewhere.
- Archived 11-14 (`archive/README.md`): `camera_delay/`, `robot_kalman/` (Rustware Kalman port), `imu_linearized_observer/`, `ekf_linearized/`. No Kalman / EKF.
- Session notes: `notes/2026-11-13/` (hour folders `1400_2PM` ... `2100_9PM`). Observer design: `2100_9PM/IMU_CAMERA_OBSERVER_DESIGN_2026-11-13.md`.
- Henry commits himself.

## In merged/ now
- **Plant** (`OmniRobotSimulationODESolve`, one class): `get_observable_state()` returns the camera pose (true + `sigma_camera_*`, 0 now), wheel speeds, currents, voltages, and with `imu_on` (True) the accelerometer and gyro. `sigma_V` 0.15, `sigma_mu` 0.03 (Dana's 2.0 / 0.1). Explicit plant: no IMU or camera noise.
- **Global velocity state** in every model; reference velocity `VEL_REF` (global) in Dana's runner.
- Dana's deadband (`ENABLE_DEADBAND=True`); it reads the plant's exact pose and speed (flag).
- **Uncommitted (11-14):**
  - Henry's readability edits to `OmniRobotLinearizedModel` / `MPCControllerLinearized` (batch rotation etc.).
  - `RiccatiTerminalCost` class cell; all five MPCs default `terminal_cost='riccati'`.
  - Total current limit: plant `I_total_max` 15 A (Dana-form per-wheel re-solve with the currents fixed); MPC constraint 7.4 A; 7.4 A clip in LQR and cascaded PID (`limit_total_current`).
- Section 1 RMS (cm): PID 14.46, Simple 0.87, LQR 1.07, DOB 0.86, Coulomb 4.74 (no arrival, double count), lin henry 0.92.
- The unit circle cell after section 1 fails (`run_multi_waypoint_benchmark()` called with 2 args); likely already in Dana's file, not checked.

## Cost functions
All five MPCs share the defaults (pose 100/100/60, vel 15/15/8, voltage 0.015, slew 0.008, K 8, dV 5 V). LQR has no slew cost (`scratch_before/models/lqr_slew_gain`). Simple MPC names its terminal weights `p_multiply`/`p_pose_fin` vs `p_pose` in the others.

## Next (Henry picks; give lists, don't take charge)
1. Full diff, then delete the applied scratch pieces (`henry_linearized_mpc`, `riccati_terminal_cost`, `total_current_limit` except the LQR slew-mixed hunks).
2. Remaining models scratch: `dana_mpc_fixes` (DOB, Coulomb double count, LuGre damping), `lqr_slew_gain`, `henry_plant` (probably not needed), `model_checks`.
3. Analysis scratch: `runner_and_settings`, `dana_sections`, `henry_tests`, `waypoint_executed`; `scratch/analysis/straight_dana_paths` (likely delete), `time_lag` (archive?).
4. IMU + camera observer (design note; open: gains, how it plugs in, lagged acceleration without IMU).
5. Grading proposal: `notes/2026-11-13/1800_6PM/GRADING_PROPOSAL_2026-11-13.md`.

## Environment
- Run notebooks headless with the scratchpad `run_nb.py` approach (IPython shell, Agg backend); `jupyter nbconvert` is not installed. One simulation at a time.
