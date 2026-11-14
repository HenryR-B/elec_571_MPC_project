# Archive

Material that is no longer part of the working model, sorted by what it is. The working model is
`RobotSimulationOpenLoopModel_HenryConstants.ipynb` at the repository root (Dana's discrete-time model with
Henry's constants); its change log is `dana-models/open_loop_model_changes.md`.

| Folder | What it is |
|---|---|
| `old_project/` | The original project: the RK4 Williams omni-wheel model (`williams_model.py`) with its README, physics write-up (`docs/`), test, figures, `Project.pdf`, and the wheel-configuration and roller-drag scripts. |
| `williams/` | The later Williams-based simulator modules: actuator and motor models, PID controller, integrator, robot dynamics and simulation. |
| `continuous_time/` | The continuous-time models, integrated with `solve_ivp` / Radau: Henry's full models `robot_model_voltage.ipynb` and `robot_model_current.ipynb` (LuGre, load transfer), the earlier `RobotSimulationOpenLoopModel_ClaudeReference.ipynb`, and the plans written for them (friction model, wheel geometry and contact, wheel-slip workflow, the model comparison with Dana's). |
| `bldc_current_loop/` | The current-command input and the Simovr (Tinymovr) current-loop model that were removed from the working model on 2026-10-27, kept for the BLDC model planned for December: `bldc_current_loop_notes.md` (the BLDC-as-DC notes, the firmware and Rustware constants, the removed code) and the current-loop scripts. |
| `BLDC/` | The BLDC motor in the same structure as `dana-mpc-copy-with-estimator/` (2026-11-11): `voltage_scaled/` (the five notebooks with the BLDC motor and current limits, voltage input, for comparison with the Pololu copy) and `current_loop_plan/` (the plan for the current-commanded version). |
| `ekf_linearized/` | Henry's EKF on the linearized model (7d17ac4) and its test. Not used: no Kalman filter, the observer uses only the IMU and the camera (2026-11-14). |
| `camera_delay/` | The camera handing frames over 50 ms late, and the observers replaying late frames (11-13). Not used: Henry, "we're not going to use camera delay". |
| `robot_kalman/` | The team robot's Kalman state estimator ported from Rustware, and its closed-loop test (11-13). Not used: no Kalman filter. |
| `imu_linearized_observer/` | Henry's 1 ms observer on the linearized model with the IMU (`RobotObserver1msIMU`, 7d17ac4), its `imu` option in `MPCControllerLinearized` and its tests. Not used: the observer should use no linearized model, only the IMU and the camera (2026-11-14). |

**Running the archived notebooks and scripts:** they import `robot_constants.py` from the repository root. The
notebooks expect it next to them, so open them with the repository root as the working directory (or add the root
to `sys.path`). The scripts in `bldc_current_loop/` add the root themselves.
