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

The controllers' voltage settings mean **the same currents as the Pololu settings** (V = R·i, a factor R_BLDC / R_Pololu = 0.24 / 2.18), as Henry chose. **To be replaced by a current controller** (`../current_loop_plan/`), where these become current settings directly.

| Setting | Pololu copy | BLDC copy |
|---|---|---|
| MPC acceptable voltage `max_V` | 6 V | 6 V × 0.24/2.18 = 0.66 V (2.75 A) |
| MPC acceptable voltage change `max_dV` | 0.5 V per step | 0.055 V per step (0.23 A) |
| MPC slew limit `delta_v_max` | 5 V per step | 0.55 V per step (2.3 A) |
| PID wheel PI gains | kp 0.15, ki 0.8 | × 0.24/2.18 (same current per speed error) |
| Voltage limit (PID, MPC) | 12 V | 13.44 V (the supply) |
| MPC current limit | 20 A | 17.3 A (the driver) |

The controllers do not know the battery limit; only the plant enforces it.

### ControllerAnalysis_Linearized.ipynb (the checks)

The step 1-3 checks and the observer test use BLDC voltages that keep every current under the driver limit (|V − Kb·ω|/R ≤ 12.5 A) but still make the wheels slide (12.5 A × 0.89 N/A ≈ 11 N per wheel, against about 5-7 N of grip): 1.5 V strafe (6 V and 10 V on the Pololu), 1 V side-to-side (4 V), random ±1.5 V steps (±8 V).

## Checks (2026-11-11)

- Step 1: the model's rates match the plant's to round-off; loads exact.
- Step 2: the Jacobians match the hand-worked entries.
- Step 3, 0.4 s position error, median / worst: strafe 1.7 / 4.8 mm, side-to-side 0.1 / 10.8 mm, random steps 1.9 / 7.8 mm (Dana's simple model: 45-120 mm median).
- Observer: velocity error 0.2-0.5 mm/s median.
