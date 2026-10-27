# Tests: motor driver, LuGre friction and plant stepping

Standalone scripts from the 2026-10-27 investigation. They use `robot_constants.py` and do not change any notebook. Run them from the repository root, one at a time.

| Script | What it tests | Run |
|---|---|---|
| `motor_only2.py` | One motor and the Simovr/Tinymovr current loop, simulated as the firmware runs it (discrete PI every 50 µs). No floor and no friction: locked rotor and lifted wheel, at current-loop bandwidths 500, 2000 and 6000. | `python tests/motor_only2.py motor_only.png` |
| `wheel_bench.py` | One wheel carrying a quarter of the robot's mass, driven by the current loop, with LuGre friction at a fixed 50 µs step. Sweeps bristle deflection, damping ratio and damping fade. Checks grip below the Coulomb breakaway command, the breakaway peak, sliding at 107 %, and the sliding force against μ_d(v)·N. | `python tests/wheel_bench.py` |
| `wheel_bench_check.py` | The candidates from `wheel_bench.py` at wheel loads of 3 to 11 N, with the 50 µs step compared against a 1 µs reference. | `python tests/wheel_bench_check.py` (slow, a few minutes) |
| `reversal_impulse.py` | How long the force peaks at a slip reversal stay above μ_s·N, and how much they change the robot's speed. | `python tests/reversal_impulse.py` |
| `robot_fixed50.py` | The whole robot at a fixed 50 µs step: current loop, LuGre, Dana's chassis and pose updates, and the load transfer with a 1 ms lag. Times each scenario against real time. | `python tests/robot_fixed50.py` |
| `driver_continuous_check.py` | The continuous-time current loop used inside the implicit solver, against the exact discrete firmware loop (motor only). | `python tests/driver_continuous_check.py` |
| `compare_solver_versions.py` | Two versions of the working notebook on Dana's manoeuvres (noise off): end speed, path, heading, run time and the newer version's solver steps. | `python tests/compare_solver_versions.py OLD.ipynb NEW.ipynb` |
| `lugre_constants_robot.py` | Two LuGre constant sets in the full implicit model (2 µm / ζ 1 / fade 5 mm/s vs 30 µm / ζ 3 / fade 50 mm/s) on Dana's manoeuvres and a slow current ramp: paths, where the wheels first slip, solver effort. | `python tests/lugre_constants_robot.py RobotSimulationOpenLoopModel_HenryConstants.ipynb` |
| `frame_fix_check.py` | The working notebook's model: a robot coasting at 1 m/s while spinning at 3 rad/s, with friction off. Correct physics is a straight line (2 m in 2 s). | `python tests/frame_fix_check.py RobotSimulationOpenLoopModel_HenryConstants.ipynb` |
| `strafe_5a_forces.py` | The 5 A strafe on the 30/45 layout: world-frame impulse from each wheel over time, to see what drives the path. | `python tests/strafe_5a_forces.py RobotSimulationOpenLoopModel_HenryConstants.ipynb` |
| `strafe_5a_causes.py` | The same strafe in the normal model, without slipping (μ = 5), and without load transfer. Above about 4 A the rear-right wheel loses grip to roll load transfer and the robot spins out. | `python tests/strafe_5a_causes.py RobotSimulationOpenLoopModel_HenryConstants.ipynb` |
| `strafe_history.py` | The 30/45 strafe at 2 and 5 through the model's history (Dana's original, her model with Henry's constants, after the axis swap, the current model): end position, heading, slipping. | `python tests/strafe_history.py LABEL=NOTEBOOK ...` |
| `design_toggle_compare.py` | `MECHANICAL_DESIGN` `'built'` (CAD CoG 10.2 mm forward, CAD height) vs `'proper'` (CoG at the robot's centre, Dana's lower height), on 2 A and 5 A strafes and a 5 A forward run. | `python tests/design_toggle_compare.py RobotSimulationOpenLoopModel_HenryConstants.ipynb` |
| `strafe_grip_margin.py` | The 30/45 strafe for the four combinations of CoG position (10.2 mm forward / centred) and CoG height (61 / 32 mm): each wheel's load and grip limit μ_s·N against the drive force its current asks for, when each wheel first slips at 5 A, and the lowest strafe current that makes a wheel slip. | `python tests/strafe_grip_margin.py RobotSimulationOpenLoopModel_HenryConstants.ipynb` |
| `strafe_wheel_forces_body.py` | The 30/45 5 A strafe, `'built'` vs `'proper'`: each wheel's floor force over the first 50 ms, and the net force and yaw moment in the robot's frame. Shows why one slipping wheel (proper) pushes the robot further backwards than two (built). | `python tests/strafe_wheel_forces_body.py RobotSimulationOpenLoopModel_HenryConstants.ipynb` |
| `stribeck_sensitivity.py` | The 30/45 5 A strafe (`'proper'`) against the unmeasured Stribeck drop: the rear-right wheel's slip, friction and load over the first 100 ms, then the end of the run for a 5× slower drop, a smaller drop (μ_k 0.75) and no drop (μ_k = μ_s). | `python tests/stribeck_sensitivity.py RobotSimulationOpenLoopModel_HenryConstants.ipynb` |
| `voltage_equivalent_tests.py` | Dana's voltage tests on the current-input model: each step the test commands the current her voltage would drive at the present wheel speed, I = clip((V − K_b·ω)/R, ±17.3 A), against the flat-current tests now in the notebook. Both designs, 30/45. | `python tests/voltage_equivalent_tests.py RobotSimulationOpenLoopModel_HenryConstants.ipynb` |
| `frame_check.py` | Dana's original class (`dana-models/`): a robot coasting while it spins, with zero floor force, drives a circle instead of a straight line. | `python tests/frame_check.py` |

## Results (2026-10-27)

- **Current loop:** follows its command with bandwidth `I_bw`. At the default 2000, it reaches 90 % of a step in 0.8 ms with no overshoot.
  - The voltage limit caps a free wheel at about 507 rad/s.
  - Four motors stalled at 20 A draw about 23 A from the battery.
- **LuGre with δ = 30 µm, ζ = 3 and the damping fading above 50 mm/s:**
  - grips at 25–93 % of the breakaway command for wheel loads of 3 to 11 N;
  - breaks away at about 0.98 μ_s·N and slides at exactly μ_d(v)·N;
  - the reversal force peaks last at most 0.5 ms and change the robot's speed by at most 1.2 mm/s.
- **Constant damping (no fade)** gives force spikes of 1.4–4.2 μ_s·N, and it caused current spikes of up to 87 A in a continuous-solver prototype.
- **A fixed 50 µs step** matches the 1 µs reference to within mm/s, and the whole robot runs at about real time in plain Python.
