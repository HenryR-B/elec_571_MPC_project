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

**Running the archived notebooks and scripts:** they import `robot_constants.py` from the repository root. The
notebooks expect it next to them, so open them with the repository root as the working directory (or add the root
to `sys.path`). The scripts in `bldc_current_loop/` add the root themselves.
