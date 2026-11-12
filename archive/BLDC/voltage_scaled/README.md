# BLDC, voltage-scaled comparison

A copy of `dana-mpc-copy-with-estimator/` (as of 2026-11-11) with the robot's own motor: the 5010 360 KV BLDC as a DC-equivalent motor (`robot_constants.py`, `archive/bldc_current_loop/bldc_current_loop_notes.md`). The controllers still command **voltage**, as in the Pololu copy, so the two compare like for like. The proper BLDC model, where the controller commands current, is planned in `../current_loop_plan/`.

Same five notebooks, same classes, same COG (`cog_y` = 0) and friction. The cell outputs from the Pololu copy were cleared. Every change is marked `BLDC:` in the code; a note at the top of each notebook says so.

## Changes from `dana-mpc-copy-with-estimator/`

### PlantModels.ipynb

| Parameter | Pololu 37D 30:1 | BLDC 5010 360 KV | Source |
|---|---|---|---|
| `R` | 2.18 Ω | 0.24 Ω (line-to-line) | `MOTOR_RESISTANCE` |
| `Lm` | 2.3 mH | 0.133 mH (line-to-line) | `MOTOR_INDUCTANCE` |
| `Kt`, `Kb` | 0.255 | 60 / (2π·360) = 0.0265 | `MOTOR_KT` |
| `Jw` | 1.8e-4 kg·m² (geared, reflected) | 1.66e-5 kg·m² (direct drive) | `WHEEL_INERTIA` |
| `V_max` | 12 V | 13.44 V (0.8 PWM limit × 16.8 V) | `PWM_LIMIT`, `MAX_MOTOR_VOLTAGE` |
| `I_max` | 20 A (not enforced by the ODESolve plant) | 17.3 A per motor (Simovr Iq 20 A as DC-equivalent current) | `MOTOR_CURRENT_LIMIT` |
| `V_bus` | (none) | 16.8 V | `MAX_MOTOR_VOLTAGE` |
| `I_bus_max` | (none) | 20 A for the whole robot (battery) | Henry |

- **New plant method `apply_driver_limits`** (ODESolve plant, called in `step` after the supply clip): the drivers' current limits, as a limit on each motor's voltage. Each motor current stays within `I_max`; if the battery current Σ V·i / V_bus would pass `I_bus_max`, all four motor currents are scaled down by the same factor. The current is taken as its steady value with the step's wheel speed, i = (V − Kb·ω)/R, because it settles in about L/R = 0.55 ms, under one 1 ms step. `state.V_actual` is the voltage after these limits.
- **The explicit plant** gets the new parameters only. It clips each motor current at `I_max` (Dana's code) and has no battery limit.
- **The demo and test cells in PlantModels** still use their Pololu-scale voltages (3-12 V). On the BLDC these hit the driver limits; they were left as they are.

### ControllerAnalysis_Estimator.ipynb and ControllerAnalysis_Linearized.ipynb (the runners)

The controllers' voltage settings ask for **the same force at the wheels as the Pololu settings**. One amp gives 8.5 N on the Pololu (geared) and 0.89 N on the BLDC (direct drive), so the same force takes 9.6 times the current; with the BLDC's 9 times lower resistance (V = R·i) that is a factor (0.24 / 2.18) × (0.255 / 0.0265) = **1.06 on the Pololu's voltage numbers**. (A first version used the same currents instead, a factor 0.11: that allowed the BLDC about a tenth of the force, and the robot lagged 9-44 cm while using at most 6 A of the 17.3 A available.) **To be replaced by a current controller** (`../current_loop_plan/`), where these become current or force settings directly.

| Setting | Pololu copy | BLDC copy (same force) |
|---|---|---|
| MPC acceptable voltage `max_V` | 6 V (2.75 A) | 6.4 V (26 A) |
| MPC acceptable voltage change `max_dV` | 0.5 V per step (0.23 A) | 0.53 V per step (2.2 A) |
| MPC slew limit `delta_v_max` | 5 V per step | 5.3 V per step |
| PID wheel PI gains | kp 0.15, ki 0.8 | × 1.06 (same force per speed error) |
| Voltage limit (PID, MPC) | 12 V | 13.44 V (0.8 PWM limit × 16.8 V battery) |
| MPC current limit | 20 A | 17.3 A (the driver) |

The controllers do not know the battery limit; only the plant enforces it.

**`MPCControllerLinearizedRiccati`** (2026-11-11, the same class as in the Pololu copy): the linearized MPC with a Riccati terminal cost, recomputed every 20 ms from the last horizon step's model. It is `'mpc_lin_riccati'` in the Linearized runner, with the same BLDC limits as `'mpc_lin'`. As in the Pololu copy, that notebook's three test cells now run `'pid_tuned'`, `'mpc'`, `'mpc_lin'` and `'mpc_lin_riccati'`; `'pid'` and `'mpc_lin_obs'` stay in the runner.

**`'pid_tuned'`** (both runners and the tests): Dana's PID for **this motor** (2026-11-11). From the BLDC search (separate training paths, stay on track with a 50 % gain margin): position kp 3, ki 0.1 (Dana's; the search had 0 to 0.05), kd 0.2; wheel PI kp 0.05, ki 2 (Pololu-equivalent, x1.06 same force); a_max 2. alpha_max 15 and k_theta 15 are an educated adjustment (the search found 30 and 25, which slid 77 % of the time on the quick hops); not re-verified. On the held-out paths the search's version was about 40 % more accurate than Dana's PID. The Pololu-tuned gains blew up on every BLDC test path: the direct-drive wheel has about 10 times less inertia and cannot take their wheel integral gain of 20.
