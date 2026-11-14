# The IMU and Henry's linearized observers (commit 7d17ac4, merged 11-13)

Henry's 11-12 work from `dana-mpc-copy-with-estimator/`, as it was merged into `merged/` on 11-13 (with the camera on time):
- (The IMU is in merged/PlantModels now, 11-14, inside OmniRobotSimulationODESolve's get_observable_state, with the `imu_on` switch; no separate IMU class.)
- `observers.py`: `RobotObserver1msIMU`, the 1 ms observer on the linearized model with the IMU (the EKF is archived: archive/ekf_linearized/).
- `section6_tests.py`: the analysis cells that compared them.
- `changes.md`: the edits to existing cells (plant, parameters, `MPCControllerLinearized`, controller list, runner).

Uses the camera (merged/PlantModels since 11-14): the observers take its frame once per control step.
