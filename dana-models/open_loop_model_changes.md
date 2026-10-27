# Changes to Dana's open-loop model

**Working copy:** `RobotSimulationOpenLoopModel_HenryConstants.ipynb`
**Branch:** `open-loop-model-robot-constants`
**Dana's original (unchanged):** `dana-models/RobotSimulationOpenLoopModel.ipynb`

Each change is made only when Henry asks for it, one at a time. Each entry says what changed, why, where, and how it was checked. The equations in the notebook's specification cell are updated with the code, in Dana's layout.

**Order:** the entries are ordered by how much they change the model, smallest first, not by date. Each heading gives its commit, or says it is not committed yet. In time order they were made 0, 1, 8, 4, 7, 5, 3, 2, 6.

---

## 0. Starting point: Dana's model with Henry's constants (commit `6f9f0b6`)

**What:** an exact copy of Dana's notebook. The only change is that the values in `simulation_parameters` come from `robot_constants.py`. Every other cell is identical to her original.

| Parameter | Dana's value | Now | From |
|---|---|---|---|
| `Mr` | 5.0 kg | 2.698 kg | `ROBOT_MASS` |
| `Jzz` | 0.15 kg·m² | 0.007726 kg·m² | `ROBOT_INERTIA` |
| `Rb` | 0.20 m | 0.0795 m | `WHEEL_ROBOT_DIST` |
| `h` | 0.08 m | 0.0611 m | `COG_Z_MM` |
| `r` | 0.04 m | 0.0299 m | `WHEEL_RADIUS` |
| `Jw` | 0.0005 kg·m² | 1.6576e-5 kg·m² | `WHEEL_INERTIA` |
| `alpha_f`, `alpha_b` | 45°, 45° | 30°, 45° | `WHEEL_ALPHA_DEG` |
| `R`, `Lm` | 1.5 Ω, 3 mH | 0.24 Ω, 0.133 mH | `MOTOR_RESISTANCE`, `MOTOR_INDUCTANCE` |
| `Kt`, `Kb` | 0.12, 0.12 | 0.0265, 0.0265 | `MOTOR_KT` (Kb = Kt in SI units) |
| `Crr` | 0.015 | 0.025 | `WHEEL_ROLLING_RESISTANCE_COEFF` |
| `mu_s`, `mu_k` | 0.60, 0.45 | 0.90, 0.422 | `MU_D_STAT`, `MU_D_DYN` |
| `sigma_V`, `sigma_mu` | 0.15 N, 0.03 | unchanged | no measured values yet |

The previous, larger version (with several unrequested fixes) is kept as `RobotSimulationOpenLoopModel_ClaudeReference.ipynb`, for reference only.

---

## 1. Henry's axes: x right, y forward (commit `e97da15`)

**What:**
- Body frame: $+x_r$ right, $+y_r$ forward.
- Wheel angles: measured counter-clockwise from $+x_r$. Wheel 1 front-right (quadrant I), 2 front-left (II), 3 rear-left (III), 4 rear-right (IV).
- At heading $\theta = 0$ the robot faces world $+Y$.
- Dana's symbols are kept: $\theta$ heading, $\dot\theta$ yaw rate, $\omega$ wheel speeds.

**Why:** Henry's convention, agreed with Dana. Her text described x forward and y left, but her $H$ matrix already worked in x right, y forward. The text and the code disagreed.

**Changed:**
- Text and labels: the frame diagram, the test descriptions, the parameter comments and the state comments.
  - Test 1 drives along $+y_r$.
  - Test 2 strafes right, along $+x_r$.
  - The multi-stage side test strafes left first, along $-x_r$.
- **Normal-force split** (spec Step 5.2 and `compute_dynamic_normal_forces`). It was written for x forward, so x and y are swapped. The formula has the same shape:

  Before:
  $$F_{n,1} = \frac{M_r g x_b}{2L} - \frac{M_r a_x h}{2L} - \frac{y_f}{\Sigma Y^2}\frac{M_r a_y h x_b}{L}, \qquad L = x_f + x_b$$

  After:
  $$F_{n,1} = \frac{M_r g y_b}{2L} - \frac{M_r a_y h}{2L} - \frac{x_f}{\Sigma X^2}\frac{M_r a_x h y_b}{L}, \qquad L = y_f + y_b, \quad \Sigma X^2 = x_f^2 + x_b^2$$

  (Wheels 2–4 follow the same pattern.) So the front/rear distances now come from $\sin\alpha$, pitch from the forward acceleration $a_y$, and roll from the sideways acceleration $a_x$.

**Unchanged:** $H$, the mass matrix, and the rotation into the world frame. These were already correct in these axes.

**Checked:**
- At rest: 7.75 N on each front wheel and 5.48 N on each rear wheel. Before the swap it was 5.95 / 7.28 N, with the rear wrongly heavier.
- Accelerating forward moves load onto the rear wheels.
- Accelerating right moves load onto the left wheels.
- The four loads always sum to the robot's weight, 26.467 N.

---

## 2. Chassis velocity integrated in the world frame (rotating-frame fix, not committed yet)

**What:** the chassis velocity is now integrated in the global (room) frame, and the local velocity is derived from it. Before, Dana's model integrated the local (body-frame) velocity directly.

**Why:** this was a bug, and a high-priority one.
- **The physics:** Newton's law holds in a fixed frame. The local axes turn with the robot, so in local axes the velocity also changes just because the axes rotate:

  $$\frac{d\mathbf v_{local}}{dt} = \mathbf a_{local} - \boldsymbol\omega \times \mathbf v_{local}$$

  Dana's update, d**v**_local/dt = **a**_local, leaves out the −ω × v term. A turning robot carries its velocity around with it, as if it were on rails.
- **The symptom:** a robot coasting at 1 m/s while spinning at 3 rad/s, with no force on it, drives a circle of radius v/ω = 0.33 m instead of a straight line (`tests/frame_check.py` on Dana's original class).
- **Why it matters:** every motion that translates and turns at the same time was wrong, and that is what the MPC will command.
- **The root fix, as Henry asked:** integrate the velocity where Newton's law is simple, the world frame. This doesn't add a correction term to the local-frame equations.

**Equations (spec Steps 5.4 and 5.5, in Dana's layout):**

$$\frac{d\dot x_g}{dt} = a_{x,r}\cos\theta - a_{y,r}\sin\theta, \qquad \frac{d\dot y_g}{dt} = a_{x,r}\sin\theta + a_{y,r}\cos\theta, \qquad \frac{d\dot\theta}{dt} = \alpha_r$$

$$\frac{dx}{dt} = \dot x_g, \qquad \frac{dy}{dt} = \dot y_g, \qquad \frac{d\theta}{dt} = \dot\theta$$

$$\dot x_r = \dot x_g\cos\theta + \dot y_g\sin\theta, \qquad \dot y_r = -\dot x_g\sin\theta + \dot y_g\cos\theta \quad \text{(derived, for } \mathbf H\text{)}$$

- **Accelerations:** a_r = M⁻¹HᵀF, as before. The forces act along body directions because the wheels are fixed to the chassis, so they are rotated by θ into the room frame.
- **No ω × v term appears anywhere.** The local velocities are only computed, never integrated.
- **The mass matrix** diag(M_r, M_r, J_zz) is the same in both frames.

**Code changes (cell `ebf0a37d`):**
- The solver's velocity block now holds `v_global`, the integrated state.
- `advance_chassis_kinetics(F_floor, theta)` returns the local acceleration and the global acceleration R(θ)·a_local.
- New `global_to_local(v_global, theta)` gives R(θ)ᵀ·v_global, the local velocities that H and the slip speeds use.
- `step` stores `v_global` and derives `v_local`.
- The load transfer still uses the local accelerations, which is correct: pitch and roll depend on body-frame acceleration.

**Text:** spec section 2 (which velocity is integrated), Steps 5.4, 5.5 and 5.7 (the state vector now holds the global velocities), section 4, item 6, the flowchart and the architecture note.

**Checked:**
- **Coasting while spinning, no force** (`tests/frame_fix_check.py`, friction and rolling resistance off, 1 m/s forward at 3 rad/s for 2 s):
  - before: 0.094 m from the start, inside a circle reaching 0.67 m sideways;
  - after: **2.000 m in a straight line, 0.0 mm sideways**.
- **Dana's manoeuvres, before vs after** (`tests/compare_solver_versions.py`, noise off):

| Test | Change |
|---|---|
| 2 A forward, 5 A forward, 4 A forward / back / coast | identical (0.0 mm): no rotation, so both frames agree |
| 2 A strafe (yaws to about 60°) | path differs by 0.54 m; end heading 51.4° vs 61.4° |
| 2 A rotation | the spinning robot's drift speed falls from 0.38 to 0.04 m/s |

- **Run time:** unchanged.

---

## 3. Wheel-layout figure instead of the ASCII frame diagram (not committed yet)

**What:** Dana's ASCII "Spatial Reference Frame Logic" diagram (cell `6c52e30a`) is replaced. The figure lives in a new code cell, `layout01`, after the plant configuration cell. It shows:
- the global frame, with the heading θ;
- the local frame, with each wheel's position, angle and drive direction;
- the α_f and α_b angles.

**Why:**
- Henry asked for it.
- ASCII can't show the real angles, because a character cell is about twice as tall as it is wide. In Dana's diagram the 30° and 45° lines looked like about 45° and 60°.
- The figure is drawn on equal axes from the model's own parameters, so its angles are true, and it updates if α_f, α_b or R_b change.

**How:**
- **Wheel positions:** R_b·(cos β_i, sin β_i), with β = [α_f, 180° − α_f, 180° + α_b, 360° − α_b].
- **Drive arrows:** the rows of the plant's own **H**, so the figure shows what the model actually uses.
- **Source:** it is the figure from `RobotSimulationOpenLoopModel_ClaudeReference.ipynb` (cell `hc000004`), relabelled with Dana's symbols (θ for heading).

**Text:** cell `6c52e30a` keeps Dana's heading and her wheel notes as text:
- the global and local frames;
- θ = 0 facing +Y_global;
- the wheel angle rule;
- the drive vectors as the rows of H;
- the front and rear stance notes.

**Checked:** the figure draws without errors. The front lines are at 30° and the rear lines at 45° from +X_robot, and the arrows match H. No model code changed, so the test outputs from the change 5 run still apply.

---

## 4. Motor constants from Rustware and the Simovr firmware (`robot_constants.py`, not committed yet)

**What:** the motor-driver values in `robot_constants.py` now come from the robot's own software and the motor-driver firmware, instead of placeholders.

| Constant | Before | Now | Source |
|---|---|---|---|
| `VEL_P_GAIN` | 0.1 | 0.196 A per rad/s | Rustware `VEL_P_GAIN` = 0.00015 A per (tick/s), × 8192/(2π) ticks per rad |
| `VEL_I_GAIN` | 1 | 19.6 A per rad | Rustware `VEL_I_GAIN` = 0.015 A per tick, × 8192/(2π) |
| `IQ_LIMIT` | 5 A | 20 A (FOC q-axis current) | Rustware `IQ_LIMIT` |
| `MOTOR_CURRENT_LIMIT` | — | 17.3 A | `IQ_LIMIT` × √3/2: the same limit as DC-equivalent current (see change 7) |
| `ENCODER_TICKS_PER_REV` | — | 8192 | firmware `SENSOR_COMMON_RES_TICKS`; Rustware scales wheel speeds with the same 8192 |
| `PWM_FREQ_HZ` | — | 20000 | firmware `config.h`: rate of the current and velocity loops |
| `CURRENT_BANDWIDTH` | — | 2000 (used as rad/s, about 320 Hz) | firmware default; Rustware does not change it |
| `CURRENT_P_GAIN`, `CURRENT_I_GAIN` | — | bandwidth × L, bandwidth × R | firmware `controller_update_I_gains` |
| `PWM_LIMIT` | — | 0.8 | firmware `config.h` |
| `INTEGRATOR_DECAY` | — | 0.995 per period while limited | firmware `config.h` |
| `CURRENT_FILTER_GAIN` | — | 0.3 per period | firmware default `I_k` |

**Why:**
- Rustware's gains are in the firmware's units, encoder ticks. The firmware's velocity loop computes `Iq_setpoint = p_gain × (error in tick/s) + integrator`, with `integrator += i_gain × (error in tick/s) × dt`, so the units are A per (tick/s) and A per tick.
- The models use rad/s, so the gains are converted with 8192 ticks per revolution. The wheels are direct drive.
- The old values 0.1 and 1 were placeholders. The real proportional gain is about 2× larger and the integral gain about 20× larger.

**Motor R and L stay line-to-line** (0.24 Ω, 0.133 mH, from a SimpleFOC forum measurement of a 5010 360 KV motor). They go with K_t = K_b = 60/(2π·KV): a DC-equivalent motor model. The note in the file explains why they are not halved. Phase values belong to the FOC frame, where K_t ≈ 8.27/KV, and mixing phase R with this K_t would make the motor √2 stronger than measured.

**Effect on Henry's `robot_model_voltage` and `robot_model_current`:**
- Their velocity servo now uses the real gains and the 20 A limit.
- It runs once per 10 ms, which is unstable with these gains while a wheel slips: the proportional loop on the rim alone has a rate of about 313 /s. It needs a faster servo step before those notebooks are rerun.

---

## 5. One implicit solver for the whole model (Radau each step, not committed yet)

**What:** every state is now integrated together over each 1 ms step by the implicit adaptive solver Radau (SciPy `solve_ivp`). Before, the model updated the current, chassis and pose with Forward Euler, and used Radau only for the wheels and bristles. The states are:
- motor currents;
- driver estimates and integrators;
- wheel speeds;
- bristle loads;
- chassis velocities;
- pose;
- the lagged accelerations behind the load transfer.

The current commands and the disturbance draws are held over the step.

**Why:** Henry asked for an implicit solver instead of Euler, so the model does not lose information between steps. Before this change, within each step:
- the back-EMF used the step-start wheel speed;
- the torque used the step-start current;
- the chassis advanced with one averaged force;
- the pose advanced with one Euler update at the step-start heading.

Now every coupling acts continuously, inside the solver.

**Equations (spec section 5, now rates):** each step gives the rate of change of its states, in Dana's layout. A new Step 5.7 describes the integration.
- **Driver (Step 5.1):** written as a continuous loop. The firmware runs it at 20 kHz, about 30× faster than its 2000 rad/s bandwidth.
  - The per-period filter gain κ and decay γ become rates: λ_f = −ln(1−κ)·f_pwm and λ_γ = −ln(γ)·f_pwm.
  - The anti-windup switch is smoothed over ε_V = 0.05 V (`anti_windup_width`), so the solver does not stall on it.
- **Load transfer (Step 5.2):** uses lagged accelerations ã_x, ã_y that follow the chassis accelerations with a time constant of one step (τ = dt). This is the continuous counterpart of Dana's "previous step's acceleration", and it keeps the loads free of an algebraic loop.
- **Chassis and pose (Steps 5.4–5.5):** dv/dt = M⁻¹Hᵀ F and dx_g/dt = R(θ)·v, Dana's equations as rates.
- **Outputs:** T_load, the applied voltage and the battery current are step averages, from integrals carried by the solver.

**Code changes (cell `ebf0a37d`):** Dana's method names stay, but each now returns rates:
- `compute_electrical_subsystem`
- `compute_dynamic_normal_forces`
- `compute_contact_friction_interface`
- `advance_chassis_kinetics`

A new `rates` combines them. `step` draws the disturbances, calls Radau once, and stores the solver's internal step sizes in `last_steps` and its evaluation count in `last_nfev`, so the step size can be tracked over time. The parameter cell gains `anti_windup_width`.

**Checked:**
- **Continuous driver vs the exact discrete firmware loop** (`tests/driver_continuous_check.py`, motor only): median current difference 0.0003 A, largest 0.25 A in the first 50 µs of a step. Radau also handles the voltage limit and the current reversal.
- **Whole model vs the previous version** (Dana's manoeuvres, noise off):

| Test | End speed, new vs before | Path difference | Heading, new vs before | Solver steps per 1 ms (median) |
|---|---|---|---|---|
| 2 A forward | 2.764 vs 2.764 m/s | 0.4 mm | 0 vs 0° | 2 |
| 2 A strafe | 2.142 vs 2.141 m/s | 2.8 mm | +61.4 vs +61.7° | 2 |
| 2 A rotation | 0.381 vs 0.382 m/s | 0.2 mm | +3999.5 vs +4004.8° | 3 |
| 5 A forward | 5.220 vs 5.217 m/s | 2.8 mm | 0 vs 0° | 3 |
| 4 A forward / back / coast | 0.497 vs 0.502 m/s | 8.4 mm | 0 vs 0° | 3 |

  The solver takes 16–23 evaluations per 1 ms step. It uses smaller internal steps only at events, such as a wheel breaking loose or a current reversing.
- **Run time:** about 2× the previous version, roughly 4× slower than real time for these tests.
- **Full notebook:** runs without errors in 455 s (211 s before).

**Left as it is (for Henry and Dana):**
- **Run time.** Telling Radau which states affect which (its Jacobian sparsity) would cut the evaluations needed to build each Jacobian.
- **The LuGre constants stay as in change 8** (δ = 2 µm, ζ = 1, fade at 5 mm/s), decided by Henry; see the decision note under change 8.

---

## 6. Mechanical-design toggle and the centre-of-mass position (not committed yet)

**What:** a switch, `MECHANICAL_DESIGN`, in the parameter cell (`21f5c2d7`), and a new parameter `cog_y`: the centre of mass (CoG) position forward of the base centre. The chassis motion is now measured about the CoG, so the yaw levers in $H$ and the front/rear load split are taken from it.

| | `'built'` (default) | `'proper'` |
|---|---|---|
| `cog_y` | +10.2 mm (CAD, `COG_Y_MM`) | 0 (the robot's centre) |
| `Jzz` | 0.007444 kg·m², CAD yaw inertia about the CoG (`IZZ_COG_KG_MM2`) | the same |
| `h` | 61.1 mm (CAD, `COG_Z_MM`) | 31.8 mm: Dana's proportion, h = 0.33 × wheelbase (her 0.08 / (0.20·(sin 30° + sin 45°))); ours is 0.64 |
| Yaw levers, front / rear | 74.4 / 86.7 mm | 79.5 / 79.5 mm (= R_b) |
| Static loads, front / rear wheel | 9.16 / 4.07 N | 7.75 / 5.48 N |

Mass, wheels, motors and friction are the same in both.

**Why:**
- Henry asked for a toggle between our robot as built and a better-proportioned design. "Proper" is defined by symmetry and loading, not by absolute values: the CoG over the robot's centre, and a CoG at Dana's (lower) relative height. Henry chose the robot's centre over the centroid of the wheel contacts. With the 30/45 layout the centroid sits 8.2 mm behind the centre; it would equalise the static loads but make the yaw levers unequal (83.6 vs 73.7 mm).
- The built robot's CoG is not at its centre: CAD puts it 10.2 mm forward (and 0.6 mm sideways, which is ignored). Dana's model assumed it was at the centre.

**The inertia:** the CAD value, 7443.681 kg·mm², is already about the CoG. Because the model now measures the motion about the CoG, with the diagonal mass matrix diag(M_r, M_r, J_zz), J_zz is that value directly, with no parallel-axis term. `ROBOT_INERTIA` (0.007726 kg·m²) is the same inertia moved to the base centre with the parallel-axis theorem; it belongs with a model that measures about the base centre and has the coupled mass matrix (Henry's main model), so this model no longer uses it. Before this change, the built design used 0.007726 with the CoG at the centre.

**Equations (spec section 3 and Step 5.2, in Dana's layout):**

Each wheel's yaw lever about the CoG is $R_b - y_{cg}\sin\beta_i$:

$$\mathbf{H} = \begin{bmatrix} -\sin\alpha_f & \cos\alpha_f & R_b - y_{cg}\sin\alpha_f \\ -\sin\alpha_f & -\cos\alpha_f & R_b - y_{cg}\sin\alpha_f \\ \sin\alpha_b & -\cos\alpha_b & R_b + y_{cg}\sin\alpha_b \\ \sin\alpha_b & \cos\alpha_b & R_b + y_{cg}\sin\alpha_b \end{bmatrix}$$

The load split uses the axle distances from the CoG, $y_{f,cg} = y_f - y_{cg}$ and $y_{b,cg} = y_b + y_{cg}$, in place of $y_f$ and $y_b$:

$$F_{n,1} = \frac{M_r g\, y_{b,cg}}{2L} - \frac{M_r a_y h}{2L} - \frac{x_f}{\Sigma X^2}\frac{M_r a_x h\, y_{b,cg}}{L}$$

(wheels 2–4 the same way). $L = y_f + y_b$ is unchanged, and the four loads still sum to the robot's weight.

**Code changes:**
- Class cell `ebf0a37d`: `__init__` sets `self.y_f_cg` and `self.y_b_cg`; the third column of $H$ uses `Rb - c*sin(af)` and `Rb + c*sin(ab)`; `compute_dynamic_normal_forces` uses the CoG distances.
- Parameter cell `21f5c2d7`: `MECHANICAL_DESIGN`, the `_DESIGNS` table, `'cog_y'` in `simulation_parameters`, and the design name in the printed report. The comment above it explains both designs and why there is no parallel-axis term.

**Text:** spec section 1 (lists $y_{cg}$), section 3 (the levers, and $J_{zz}$ about the CoG), Step 5.2 (the CoG distances), and cell `e6b1eaeb` (what the switch does).

**Checked** (noise off, 1.5 s; `tests/design_toggle_compare.py`):
- **cog_y = 0 reproduces the previous model exactly** (0.0 mm on all of Dana's manoeuvres, `tests/compare_solver_versions.py`).

| Test | built | proper |
|---|---|---|
| 45/45, 2 A strafe | yaws: +335°, 6.2 rad/s at the end | straight: 1.87 m, 0° |
| 30/45, 2 A strafe | spins: +275°, 5.6 rad/s, 0.68 m/s | heading +53°, 2.05 m/s (as with the CoG at the centre) |
| 30/45, 5 A strafe | spins out, both rear wheels slipping | spins out, rear-right wheel slipping |
| 30/45, 5 A forward | no slip, 7.33 m/s | no slip, 7.33 m/s |

- **The built robot yaws on a strafe even with the symmetric 45/45 layout.** That is the physics of a CoG ahead of the centre: with equal currents, the front and rear sideways forces act on unequal levers about the CoG, so they leave a net yaw moment. The open-loop real robot would do the same; the controller has to correct it.
- **The extra front load helps going forward:** at 5 A forward the front wheels no longer slip (they slipped the whole run with the CoG at the centre, at 5.22 m/s).
- **The 5 A strafe spins out in both designs.** Halving the CoG height does not stop the rear-right wheel losing grip, so this is not just a matter of the built robot's proportions.
- **Full notebook (built):** runs without errors in 638 s.

---

## 7. Current-command input through the Simovr current loop (not committed yet)

**What:** the model's input is now a current command per motor, not a voltage. As on the robot, the Simovr motor driver (Tinymovr firmware) turns the current error into the voltage it applies, and Dana's electrical equation then sets the actual current. A voltage command is still accepted, applied straight to the motor terminals as in Dana's model, and Dana's tests use it.

**Why:** Henry asked for the more realistic input. The robot's drivers are current-controlled (FOC); the controller sends current, and the voltage follows. Decided with Henry:
- current command input;
- the firmware's default current-loop bandwidth;
- 20 A per motor.

**Equations (spec Step 5.1, in Dana's layout).** Each 1 ms step holds n = dt·f_pwm = 20 driver periods, with the wheel speeds held at step k, as Dana's back-EMF already does. Each period, in the firmware's order:

$$i_{set} = \text{clip}(i_{cmd}, \pm I_{max}), \qquad \hat i \leftarrow \hat i + \kappa (i - \hat i), \qquad e = i_{set} - \hat i$$

$$x_I \leftarrow x_I + T_{pwm} K_i e, \qquad V = \text{clip}(K_p e + x_I, \pm \eta V_{bus}), \qquad K_p = \omega_c L_m, \; K_i = \omega_c R$$

$$i \leftarrow i + \frac{T_{pwm}}{L_m}\left(V - R i - K_b \omega_k\right)$$

- While V is limited, the integrator is multiplied by γ each period.
- The torque is still K_t·I_k, Dana's convention.
- The applied voltage V_a and the battery current I_bus = Σ V·i / V_bus are reported as averages over the step.

**Code changes:**
- **Cell `ebf0a37d`:**
  - `OmniMotorState`: new states `I_est` and `x_I`, and new outputs `V_applied` and `I_bus`.
  - `__init__`: the gains `Kp_I` and `Ki_I`, and `n_pwm`, with a check that dt is a whole number of driver periods.
  - `compute_electrical_subsystem(state, I_cmd)`: rewritten as the driver loop above.
  - `step(state, I_cmd)`: carries the new states and outputs forward.
- **Parameter cell:** the driver settings, taken from `robot_constants.py`.

**Current limit: 17.3 A, not 20 A.** The Simovr limits its FOC q-axis current to 20 A (`IQ_LIMIT`). This model's current is DC-equivalent, so the clamp is `MOTOR_CURRENT_LIMIT` = (√3/2) × 20 = 17.3 A. Two independent checks give the same factor:
- **Equal torque:** K_t,FOC = (√3/2)·K_t,DC (8.27/KV vs 9.55/KV).
- **Equal copper heat:** R_LL·i² = 1.5·R_phase·I_q² = 0.75·R_LL·I_q².

Dana's tests command voltages, which bypass the driver, so this limit does not apply to them.

**Voltage input, for Dana's tests.** `step(state, V_cmd=V)` applies V straight to the motor terminals, limited to 0.8 × V_bus, with the driver bypassed and its states held; `step(state, I_cmd=I)` goes through the driver. Exactly one must be given.
- **Why:** Dana's tests command voltages, and a voltage is not a current: under voltage drive the current falls as the wheel speeds up (back-EMF), so a slipping wheel loses torque, while a current command keeps pushing. Comparing her voltage tests with current commands of the same numbers compares different tests (Henry).
- **First version, corrected:** the tests had been converted to current commands with the same numbers (2 A, 5 A, ±4 A). That was wrong; they are back on Dana's voltages, 2 V, 5 V and ±4 V. Her test cells are restored exactly, except that each call is `step(state, V_cmd=...)`; her test text is restored with the axis changes of change 1 kept.
- **Unchanged for the controller:** the current command is still the input the controller uses. With a current command, the results are identical to before (0.0 mm).
- **Checked:** with a voltage command the driver applies exactly the commanded voltage (±5.000 V) and its states stay at zero. The full notebook runs without errors in 324 s (`'proper'` design). On our motor (R = 0.24 Ω) Dana's voltages are strong: in the 45/45 2 V forward test the front wheels slip until about 0.4 s and then grip again as the back-EMF cuts their current; the 30/45 2 V and 5 V strafes spin (about −19 and −29 rad/s at 1.5 s).

**Text changes:**
- Spec sections 1 and 2: driver parameters, states and outputs, and the two inputs.
- Spec section 4: flow items 1 and 2.
- Spec Step 5.1: rewritten, with the voltage-command bypass.
- The flowchart's inputs and electrical box, the architecture note and the configuration note. The test descriptions are Dana's again (voltage).

**Checked:**

*Driver on its own:* see `tests/motor_only2.py`. At the default bandwidth the current reaches 90% of a step in 0.8 ms with no overshoot.

*Link to Dana's voltage model:* Dana's voltage-driven model was run with her test voltages, and the currents it produced were fed as commands to this model.
- **2 V tests** (forward, strafe, rotation): the driver applies Dana's voltage to a median of 0.003–0.009 V. The robot paths agree to within 1.5 mm forward and 0.1 mm in rotation, and to within 72 mm in the spinning strafe.
- **5 V forward and 4 V multi-stage:** the currents match exactly and the robot motion agrees to within 4–25 mm, but the applied voltage sits about 0.6–1 V lower. The wheels slide there.
  - Dana's 1 ms forward-Euler current update overshoots on its first step: at 5 V it jumps to (Δt/L)·V = 37.6 A, where the real circuit reaches about 17 A.
  - The sliding wheels keep that extra spin, about 40 rad/s, because sliding friction does not depend on wheel speed. That speed difference is the 1 V of back-EMF.

*Full notebook:* runs without errors in 211 s.

**Left as it is (for Henry and Dana):**
- **The 0.8 × V_bus voltage limit** is the firmware's `PWM_LIMIT` (largest modulation, 80%). The firmware gives no reason. The usual one is leaving time in each PWM period for low-side current sensing and dead time. How it maps onto this DC-equivalent model depends on the firmware's modulation scaling, which was not confirmed.
- **The driver used the wheel speed from step k for the back-EMF**, Dana's convention, so it lagged the wheel by up to 1 ms. Change 5 removes this.
- **The friction is unchanged:** it is still the LuGre setup of change 8.

---

## 8. LuGre floor friction and smoothed rolling resistance (commit `41bd17d`)

**What:** Dana's stick/slip switch is replaced by Henry's LuGre friction model, with his parameters from `robot_model_voltage.ipynb` on `main`. Rolling resistance is smoothed through zero wheel speed.

**Why:**
- With the real constants, Dana's friction chattered between grip and slip at every step. A wheel re-gripped as soon as its requested force fell below $\mu_s N$, and its speed then snapped to the rolling speed, even while it was spinning the wrong way.
- In LuGre the force comes from a bristle state, which is integrated instead of switched. So the force is continuous through zero slip, and a wheel only regains grip when its slip actually returns to zero.
- `sign(ω)` in the rolling resistance chattered the same way when a wheel coasted to a stop.

### Equations

**Before** (Dana):
$$F_i = \begin{cases} F_{req,i} = \dfrac{T_{m,i} - T_{roll,i}}{r} & |F_{req,i}| \le \mu_s F_{n,i} \text{ (stick; wheel speed snapped to } v_i / r) \\ \text{sign}(F_{req,i})\,\mu_k F_{n,i} & \text{otherwise (slip)} \end{cases}, \qquad T_{roll,i} = \text{sign}(\omega_i)\,C_{rr} F_{n,i} r$$

**After:**

Slip and rolling resistance:
$$v_{s,i} = (H\dot{\mathbf X}_r)_i - r\,\omega_i \qquad T_{roll,i} = C_{rr} F_{n,i}\, r \tanh(\omega_i / \varepsilon)$$

Stribeck curve and bristle dynamics:
$$\mu_{d,i} = \mu_k + (\mu_s - \mu_k)\,e^{-\alpha_d |v_{s,i}|} \qquad \dot z_i = \frac{1}{\delta}\left(v_{s,i} - \frac{\mu_s}{\mu_{d,i}}\sqrt{v_{s,i}^2 + v_c^2}\; z_i\right)$$

Bristle damping and floor force:
$$\sigma_{1,i} = 2\zeta\sqrt{\frac{\mu_s F_{n,i}}{\delta}\cdot\frac{J_w}{r^2}}\; e^{-(v_{s,i}/v_\sigma)^2} \qquad F_i = -\left(\mu_s F_{n,i}\, z_i + \sigma_{1,i}\,\delta\,\dot z_i + \sigma_2 v_{s,i}\right)$$

Wheel dynamics, the same for every wheel (no stick/slip branches, no speed snap):
$$J_w\,\dot\omega_i = T_{m,i} - T_{roll,i} - r F_i$$

How to read it:
- $v_{s,i}$ is the contact-point slip along the drive direction. A wheel spinning faster than the ground has $v_s < 0$ and pushes the robot along $+d_i$, matching Dana's sign convention.
- $z_i$ is the bristle deflection as a fraction of $\delta$. A gripping wheel holds up to $\mu_s F_n$ (at $z = 1$). In steady sliding $F_i = -\text{sign}(v_s)\,\mu_d F_n$.

### Parameters (added to `simulation_parameters`)

| Key | Value | Meaning | Source |
|---|---|---|---|
| `alpha_d` | 5 s/m | Stribeck decay $\mu_s \to \mu_k$ | `rc.ALPHA_D` |
| `lugre_deflection` | 2e-6 m | bristle deflection $\delta$ at $\mu_s N$ | `LUGRE_PRESLIDING_DEFLECTION` (main) |
| `lugre_damping_ratio` | 1.0 | $\zeta$, damping as a fraction of critical, against the rim inertia | `LUGRE_DAMPING_RATIO` (main) |
| `lugre_damping_slip` | 5e-3 m/s | $v_\sigma$, damping fades out above this slip | `LUGRE_DAMPING_SLIP` (main) |
| `lugre_viscous` | 0 N·s/m | $\sigma_2$ | `LUGRE_VISCOUS_COEFF` (main) |
| `lugre_creep` | 1e-4 m/s | $v_c$, smooths $\lvert v_s \rvert$ at zero slip | `LUGRE_CREEP_SLIP` (main) |
| `slip_band` | 5e-3 m/s | slip flag threshold (diagnostic only) | `rc.DRIVE_BREAKAWAY_BAND` |
| `roll_smoothing` | 0.5 rad/s | $\varepsilon$ in $\tanh(\omega/\varepsilon)$ | `rc.WHEEL_LOSS_SMOOTHING_SPEED` |
| `rtol` | 1e-4 | implicit solver tolerance | `LUGRE_RTOL` (main) |

None of the LuGre values are measured; they are Henry's tuned choices (see `friction_model_plan.md`, section 7).

### Integration

At $\delta$ = 2 µm the bristles are very stiff: about $3.5\cdot10^6$ N/m against the rim mass $J_w/r^2 \approx 0.019$ kg, which is about $1.4\cdot10^4$ rad/s. Dana's 1 ms Forward Euler step would be unstable. As Henry chose, within each 1 ms step:
- The wheel speeds, bristle loads and chassis velocity are integrated together with an implicit solver (`scipy.integrate.solve_ivp`, Radau, rtol 1e-4).
- The motor torques, normal forces and friction coefficients are held at their step-$k$ values.

Dana's 1 ms discrete map is otherwise unchanged:
- The current, normal loads and noise are still updated once per step.
- `advance_chassis_kinetics` is unchanged. It receives the floor force averaged over the step, so its Forward Euler update gives exactly the solver's chassis velocity.

### Code changes (cell `ebf0a37d`)

- `OmniMotorState`: new state `self.z` (bristle loads, 4).
- `__init__`: `self.m_rim = Jw / r**2`.
- New `lugre_contact(slip, z, F_n, mu_s, mu_k)`: returns the floor force and $\dot z$.
- `compute_contact_friction_interface`:
  - Rewritten: the same friction noise $w_\mu$, then the implicit integration over the step.
  - Returns the step-average `F_floor` and `T_load`, the slip flags, and `omega_next`, `z_next`.
- `step`: the Condition A/B wheel-speed block is replaced by storing `omega_next` and `z_next`.
- `from scipy.integrate import solve_ivp`.

### Text changes

- Spec section 1: new parameters.
- Section 2: new state $\mathbf Z_k$.
- Section 4: flow items 4 and 8.
- Step 5.3: rewritten for LuGre.
- Step 5.4: the integration over the step, and the average force.
- Step 5.6: one torque balance for every wheel.
- Phase 3: disturbances in the new law.
- Flowchart: the contact box; the stick/slip decision diamond is removed.
- Architecture notes and the configuration note.

### Checked (Test-1 pattern with Henry's constants, 30/45, no noise)

**2 V forward for 1 s, then reverse for 0.5 s:**

| | Dana's friction | LuGre |
|---|---|---|
| Slip-flag changes per wheel | 302 / 302 / 73 / 73 | 3 / 3 / 3 / 3 |
| Speed at 1 s | 2.60 m/s | 2.54 m/s |
| Speed after braking 0.5 s | 0.008 m/s | 0.81 m/s |

Why the braking differs:
- When braking starts, the voltage flips, and back-EMF adds to it. The current jumps to about −16 A, so each front wheel asks for about 14.6 N against a grip limit of about 9.6 N. The wheels break loose.
- With LuGre they keep sliding backwards at about −26 rad/s, at $\mu_k N$ (about 4.8 N per front wheel), and the robot brakes at about 3.5 m/s².
- Dana's model re-grips every few steps at close to $\mu_s N$, so it braked at about 5.2 m/s². That is the chatter this change removes.

**Multi-stage 4 V test:**
- The wheel-speed plots no longer show ±100 rad/s spikes.
- The wheels slide through the whole drive phase at close to free speed, and the robot accelerates at about 3.2 m/s² (sliding limit). With Dana's friction it reached 4.2 m/s instead of 3.2 m/s.
- The coast phase decays smoothly.

**Full notebook:** runs without errors in 206 s. A single 1.5 s run takes about 5.7 s, against 0.1 s with Dana's friction.

### Left as it is (for Henry and Dana to decide)

- **Two of Dana's descriptions no longer match the results:**
  - the "Transient Speed Chattering" note in the multi-stage test description;
  - the plot title "Wheel Angular Velocities (Stick-Slip Spikes)".
  
  LuGre doesn't produce those spikes.
- **The noise ($w_F$, $w_\mu$) is still inside the model class.** The agreed plan is to move disturbances to the simulation layer.
- **Run time.** A faster option is still possible if needed, for example a fixed-step implicit update.

**Later tests (2026-10-27, `tests/`):** these LuGre constants (δ = 2 µm, ζ = 1, damping fading above 5 mm/s) are stiff. A softer set was also tested (δ = 30 µm, ζ = 3, fade at 50 mm/s).

**Decision (Henry, 2026-10-27): keep δ = 2 µm.**
- With the implicit solver (change 5), both sets cost the same to solve and give the same robot behaviour (`tests/lugre_constants_robot.py`): paths within 3 mm, and the front wheels break away at 4.38 A vs 4.35 A on a slow current ramp.
- 2 µm is the closest to Coulomb–Stribeck friction through slip reversals: the force changes sign 10–32 mm/s past zero slip, against 32–95 mm/s at 20 µm.
- It reproduces most closely the friction law of Henry's main model. That is the Coulomb/Stribeck "Williams kinetic law" (the Williams-style force law in `williams_model.py`), with μ_d(v) = μ_k + (μ_s − μ_k)·e^(−α_d |v|) from `MU_D_STAT`, `MU_D_DYN` and `ALPHA_D`. LuGre's steady sliding uses this same curve for every δ. The curve was checked against the code, not against the Williams paper itself.
- The real contact's deflection is unmeasured.
