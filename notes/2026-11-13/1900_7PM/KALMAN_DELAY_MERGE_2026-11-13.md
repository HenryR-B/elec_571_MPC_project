# Kalman / camera-delay work merged into `merged/` (11-13, evening)

Henry's commit 7d17ac4 ("kalman delay replay etc. not fully done", his 11-12 work in `dana-mpc-copy-with-estimator/`) had features the 11-13 merge left out. They are now in `merged/`. Each cell was merged 3-way (`git merge-file`: the commit's parent as base, `merged/` as ours, 7d17ac4 as theirs).

## PlantModels.ipynb
- `OmniRobotSimulationODESolve` (cell `ce831979`): keeps each step's load noise in `self.last_load_vibration` (two lines, Henry's). Clean merge with the 15 A total-current change. The load noise is still drawn once per step.
- New cells (Henry's, unchanged):
  - `7b52cdf3`: `OmniRobotSimulationODESolveIMU` and `OmniRobotObservableStateIMU` (accelerometer, gyro)
  - `311c95cf`: `OmniRobotCamera` (frame every 20 ms, Rustware's vision noise, late when `camera_delay_on`)
- Parameters (cell `f56abec6`): `imu_vibration_gain`, `camera_period`, `camera_delay_on` (False), `camera_delay` 0.05 s, `sigma_camera_position` 5 mm, `sigma_camera_heading` 0.005 rad, `sigma_camera_delay` 0, `sigma_gyro` 0.002.

## ControllerModels.ipynb
- `RobotObserver1ms` (cell `cd5012b4`): the late-camera replay (`advance`, `remember`, `correct_camera_late`; `camera_replay=True`, 0.3 s history). The merged cell was the commit's base, so Henry's version as is.
- New cells (Henry's, unchanged):
  - `cad39837`: `RobotObserver1msIMU`
  - `ca2b33c0`: `RobotObserverEKF`
- `MPCControllerLinearized` (cell `96fdc523`): Henry's `imu`, `ekf`, `camera_replay` options and the new `estimator_update(measured_State, camera_pose, camera_age)`, together with the merged `terminal_cost` (one conflict: both added arguments; both kept).
- Intro markdown: one line for the observer options. The rest of the commit's intro edit was already covered by the merged intro.
- `OmniRobotLinearizedModel`: the commit's readability edits were already in (taken on 11-13), so no change.

## ControllerAnalysis.ipynb
- The runner:
  - builds the IMU plant (`OmniRobotSimulationODESolveIMU`; checked: the same motion as the plain plant, difference 0.0 over 3 s of random voltages) and the camera
  - the observer options read the pose from the camera (noisy, late when `camera_delay_on`); the other controllers still read the true pose with no delay
  - `run_simulation(..., params=None)` and `run_and_report(..., params=None)` take the plant's and the camera's parameters (default `SIMULATION_PARAMATERS`)
- New controllers: `Linearized MPC observer no replay`, `... observer IMU`, `... observer IMU no replay`, `... EKF`, `... EKF no replay`.
- New section 6, from Henry's three comparison cells, on his paths (Dana 8 s and 4 s, quick hops, the two 0.4 m eights):
  - without and with the IMU (camera on time)
  - camera on time, late with no replay, late replayed
  - hand-gain IMU observer against the EKF, camera 50 ms late
- Change in behaviour: `Linearized MPC observer` now gets the noisy camera (5 mm), as in Henry's commit. Before, it got the true pose at the end of each 20 ms step. It is not in any test list except section 6.

## On/off switch (Henry, 11-13 late): `CAMERA_MODEL_ON = False` for now
- In ControllerAnalysis's settings.
- **False:** the observer options get the true pose at the end of each 20 ms step, on time, no noise, exactly as before the merge, and section 6 is skipped. Checked: `Linearized MPC observer` on Dana 8 s gives the same final pose, RMS and voltages as the committed (pre-merge) `merged/`, to every digit.
- **True:** they read `OmniRobotCamera` (noisy; late when `camera_delay_on`), and section 6 runs.
- The other controllers always read the true pose.
- Henry: the late-camera filter should use only measured quantities (like Matt's kinematic `RobotKalman`), nothing from the linearization, so it is fair to every controller. The merged observers (`RobotObserver1ms`, `...IMU`, `RobotObserverEKF`) predict with the linearized model, so they are not that. A faithful port of Matt's filter needs the Rustware source, which is not on this PC.

## Checks run
- ModelChecks: all cells, 0 errors, 16 s. Same numbers as 11-13: 0.4 s predictions 0.2 / 0.7 / 5.4 mm median.
- ControllerAnalysis section 6 (with the setup cells): 0 errors, 584 s.
- ControllerAnalysis sections 1 to 5 and Dana's unit circle (separate run, one at a time): 0 errors, 879 s.

Off path (cm) / RMS (cm), section 6:

| Path | on time, no IMU | on time, IMU | late, no replay | late, replayed | EKF (late, replayed) |
|---|---|---|---|---|---|
| Dana 8 s | 0.3 / 1.1 | 0.3 / 0.9 | 0.4 / 3.9 | 0.3 / 0.9 | 0.1 / 1.0 |
| Quick hops | 0.5 / 0.9 | 0.6 / 0.7 | 3.0 / 2.1 | 0.5 / 0.7 | 0.6 / 0.7 |
| Eight 0.4 m, 3.4 s | 1.1 / 1.5 | 0.9 / 1.3 | 2.4 / 3.2 | 0.9 / 1.3 | 0.8 / 1.3 |
| Eight 0.4 m, 2.5 s | 6.4 / 4.0 | 7.8 / 4.3 | 11.2 / 8.1 | 7.7 / 4.8 | 5.6 / 3.7 |

- The replay brings a 50 ms late camera back to the on-time result. Without it, the 2.5 s eight goes from 7.7 to 11.2 cm off path, and the hops from 0.5 to 3.0 cm.
- The EKF is a little better than the hand-gain IMU observer on the 2.5 s eight (5.6 vs 7.7 cm), about the same elsewhere.
- Settling: on Dana 8 s, the hand-gain observers never settle (X: the robot stays above 1 cm/s). The EKF settles at 7.82 s. This fits the 11-12 finding that the hand gains pass the 5 mm camera noise into the velocity.
- The 11-12 numbers (EKF 7.3 cm vs hand 17.8 cm on the 2.5 s eight) came from Henry's older runner (other weights, no current limit). They are not comparable to these.

## Not merged
- The 11-12 scripts stay in `notes/2026-11-12/`.
- `ControllerAnalysis_TimeLag.ipynb` has its own copy of the old runner: its observer options would fail with the new `estimator_update`. It does not use them.

## Caveats (11-12 notes)
- These observers predict with an exact copy of the plant's model and re-linearize every 1 ms (not realistic on the robot).
- The EKF was built on the MPC's model, not on Matt's kinematic filter (not what Henry asked for on 11-12). Merged as Henry pushed it.
