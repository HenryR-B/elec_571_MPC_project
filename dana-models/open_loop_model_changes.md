# Changes to Dana's open-loop model

**Working copy:** `RobotSimulationOpenLoopModel_HenryConstants.ipynb`
**Draft rewrite:** `RobotSimulationOpenLoopModel_HenryStyle.ipynb`, the same model in plain functions and named constants (entry 10)
**Branch:** `open-loop-model-robot-constants`
**Dana's original (unchanged):** `dana-models/RobotSimulationOpenLoopModel.ipynb`
**Dana's v3 (unchanged):** `dana-models/robot-simulation-open-loop-v3.ipynb`, merged into the working copy on 2026-10-28 (entry 8)

Each change is made only when Henry asks for it, one at a time. Each entry says what changed, why, where, and how it was checked. The equations in the notebook's specification cell are updated with the code, in Dana's layout.

**Notes in the code:** the model class, parameter and figure cells carry `[CHANGE n]` comments, where n is the entry below, each saying what Dana's original did. Code with no tag is hers. The test cells carry no tags: they are exactly the cells of her v3 (entry 8).

**Step labels:** the code and the flowchart use Dana's original "Step 5.x" names. Since the merge (entry 8) the spec cell uses her v3 layout, so: Step 5.1 is section 5 item 1; Step 5.2 is section 4; Step 5.3 is section 5 items 2–6; Step 5.6 is item 7; Step 5.8 is item 8; Steps 5.4 and 5.5 are item 9.

**Order:** the entries are ordered by how much they change the model, smallest first, not by date. Each heading gives its commit, or says it is not committed yet. In time order they were made 0, 1, 7, 4, 5, 3, 2, 6, 8, 9, 10. Entry 10 changes how the code is organised, not the model, so it comes last.

**Removed again:** for a while the model took a current command per motor, through a model of the Simovr (Tinymovr firmware) current loop. Henry and Dana then agreed the project uses a voltage input, like a PWM-driven DC motor, with the controller choosing the voltage, so the current input and the driver were removed (2026-10-27). The model's input is Dana's terminal voltage again, and the tests are her v3 tests at her v3 voltages (entry 8). Everything worked out for the current loop, the firmware values and the removed code are kept in `archive/bldc_current_loop/bldc_current_loop_notes.md`, for the BLDC model planned for December.

**Checks in entries 2, 5, 6 and 7** were mostly run with the earlier BLDC motor constants (and entry 2 while the model took current commands, in amps), so their numbers are kept as a record, not as today's results. Entry 5 and entry 8 give the results with the current Pololu constants.

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

Since entry 8 the motor (`R`, `Lm`, `Kt`, `Kb`) and `Jw` are Dana's Pololu values, and `Jzz` is the CAD inertia about the CoG (entry 6).

The previous, larger version (with several unrequested fixes) is kept as `archive/continuous_time/RobotSimulationOpenLoopModel_ClaudeReference.ipynb`, for reference only.

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
- **Normal-force split** (spec section 4 and `compute_dynamic_normal_forces`). It was written for x forward, so x and y are swapped. The formula has the same shape:

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

## 2. Chassis velocity integrated in the world frame (rotating-frame fix, commit `da3f01b`; final form `0557b8d`)

**What:** the chassis velocity is now integrated in the global (room) frame, and the local velocity is derived from it. Before, Dana's model integrated the local (body-frame) velocity directly.

**Why:** this was a bug, and a high-priority one.
- **The physics:** Newton's law holds in a fixed frame. The local axes turn with the robot, so in local axes the velocity also changes just because the axes rotate:

  $$\frac{d\mathbf v_{local}}{dt} = \mathbf a_{local} - \boldsymbol\omega \times \mathbf v_{local}$$

  Dana's update, d**v**_local/dt = **a**_local, leaves out the −ω × v term. A turning robot carries its velocity around with it, as if it were on rails.
- **The symptom:** a robot coasting at 1 m/s while spinning at 3 rad/s, with no force on it, drives a circle of radius v/ω = 0.33 m instead of a straight line (`tests/frame_check.py` on Dana's original class).
- **Why it matters:** every motion that translates and turns at the same time was wrong, and that is what the MPC will command.
- **The root fix, as Henry asked:** integrate the velocity where Newton's law is simple, the world frame. This doesn't add a correction term to the local-frame equations.

**Equations (spec section 5, item 9, there in vector form):**

$$\dot x_{g,k+1} = \dot x_{g,k} + dt\,(a_{x,r}\cos\theta_k - a_{y,r}\sin\theta_k), \qquad \dot y_{g,k+1} = \dot y_{g,k} + dt\,(a_{x,r}\sin\theta_k + a_{y,r}\cos\theta_k), \qquad \dot\theta_{k+1} = \dot\theta_k + dt\,\alpha_r$$

$$x_{k+1} = x_k + dt\,\dot x_{g,k+1}, \qquad y_{k+1} = y_k + dt\,\dot y_{g,k+1}, \qquad \theta_{k+1} = \theta_k + dt\,\dot\theta_{k+1}$$

$$\dot x_r = \dot x_g\cos\theta + \dot y_g\sin\theta, \qquad \dot y_r = -\dot x_g\sin\theta + \dot y_g\cos\theta \quad \text{(derived, for } \mathbf H\text{)}$$

- **Accelerations:** a_r = M⁻¹HᵀF, as before. The forces act along body directions because the wheels are fixed to the chassis, so they are rotated by θ into the room frame.
- **No ω × v term appears anywhere.** The local velocities are only computed, never advanced themselves.
- **The mass matrix** diag(M_r, M_r, J_zz) is the same in both frames.

**Code changes (cell `ebf0a37d`):**
- The velocity state is `v_global`, advanced in the world frame.
- `advance_chassis_kinetics` rotates the local acceleration into the world frame with R(θ) and advances `v_global`, then the pose.
- New `global_to_local(v_global, theta)` gives R(θ)ᵀ·v_global, the local velocities that H and the slip speeds use.
- `step` stores `v_global` and derives `v_local`.
- The load transfer still uses the local accelerations, which is correct: pitch and roll depend on body-frame acceleration.

**Text:** spec section 2 (which velocity is advanced), section 5 item 9, the flowchart and the architecture note.

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

## 3. Wheel-layout figure, below Dana's ASCII frame diagram (commit `da3f01b`; text `0557b8d`)

**What:** a new code cell, `layout01`, after the plant configuration cell, draws the wheel layout. It shows:
- the global frame, with the heading θ;
- the local frame, with each wheel's position, angle and drive direction;
- the α_f and α_b angles.

Dana's ASCII frame diagram (cell `6c52e30a`) is there too. The figure first replaced it; since the merge (entry 8) the cell is her v3 diagram, in Henry's axes, with one added sentence pointing to the figure.

**Why:**
- Henry asked for it.
- ASCII can't show the real angles, because a character cell is about twice as tall as it is wide. In Dana's original diagram the 30° and 45° lines looked like about 45° and 60°.
- The figure is drawn on equal axes from the model's own parameters, so its angles are true, and it updates if α_f, α_b or R_b change.

**How:**
- **Wheel positions:** R_b·(cos β_i, sin β_i), with β = [α_f, 180° − α_f, 180° + α_b, 360° − α_b].
- **Drive arrows:** the rows of the plant's own **H**, so the figure shows what the model actually uses.
- **Source:** it is the figure from `archive/continuous_time/RobotSimulationOpenLoopModel_ClaudeReference.ipynb` (cell `hc000004`), relabelled with Dana's symbols (θ for heading).

**Checked:** the figure draws without errors. The front lines are at 30° and the rear lines at 45° from +X_robot, and the arrows match H.

---

## 4. Motor constants (commit `0557b8d`)

**What the model uses:** since entry 8, Dana's Pololu 37D 30:1 gearmotor, typed into the notebook's parameter cell: R = 2.18 Ω, $L_m$ = 2.3 mH, $K_t = K_b$ = 0.255 (at the wheel, so the gear ratio is included), and the reflected rotor inertia $J_w$ = 0.00018 kg·m². The robot's geometry, mass, CoG and floor friction still come from `robot_constants.py`. (Henry, 2026-10-28: later, all constants will move into the notebook.)

**Our BLDC motor stays in `robot_constants.py`, not used by this model:** `MOTOR_RESISTANCE`, `MOTOR_INDUCTANCE` and `MOTOR_KT`, a DC-equivalent model of the robot's 5010 360 KV BLDC, kept for the BLDC model planned for December. R and L there are line-to-line (0.24 Ω, 0.133 mH, from a SimpleFOC forum measurement of a 5010 360 KV motor), going with K_t = K_b = 60/(2π·KV); the note in the file explains why they are not halved. The Rustware velocity gains and the Simovr firmware's current-loop values, with their sources, are in `archive/bldc_current_loop/bldc_current_loop_notes.md`.

---

## 5. Backward Euler for the motor current, wheel speed and LuGre bristles (commit `0557b8d`)

**What:** Dana's model is a discrete-time map at $dt$ = 1 ms, with Forward Euler for every state. Her discrete step stays. The motor current, wheel speed and LuGre bristle state of each wheel now use **backward (implicit) Euler**: $\frac{dx}{dt} \approx \frac{x_{k+1} - x_k}{dt}$, with the right-hand side at step $k+1$.
- With the Stribeck coefficient and the bristle damping fixed, the new current, the new bristle state and the floor force are all linear in the new wheel speed. So each wheel's torque balance is one linear equation for $\omega_{k+1}$.
- The Stribeck coefficient and the damping depend on the new slip, so the solve is repeated with them updated at the new slip. It stops when the slip changes by less than `implicit_tol` = 0.1 mm/s, or after `implicit_max_iter` = 50 solves.
- **Unchanged from Dana:** the chassis velocity and pose use her Forward Euler, in her order (velocity from the new floor forces, then position from the new velocity), in the world frame of change 2. The load transfer uses the previous step's acceleration, and the load torque is the step's own value, $T_{load} = F_{k+1} r + T_{roll,k}$.

**Why:** Forward Euler at 1 ms doesn't work on the wheel and its bristles:
- **The LuGre bristles** (change 7). At δ = 2 µm they are stiff: about $3\cdot10^6$ N/m against a wheel's 201 g rim-equivalent mass (the Pololu $J_w$), which rings at about 3,800 rad/s. That is ω·dt ≈ 3.8 at 1 ms, and an explicit step needs under 2. Dana's v3 updates the wheel explicitly, and its wheel speed chatters (entry 8).
- **The motor current does not need it with the current motor.** With the Pololu, $L_m/R$ = 1.05 ms, and Forward Euler is stable on the current alone (a factor of +0.05 per step). The current stays in the solve because the back-EMF couples it to the wheel. When this was chosen, the BLDC constants had $L_m/R$ = 0.55 ms, where Forward Euler flips the current's sign each step (a factor of −0.8).

Backward Euler is stable at any step size, so Dana's 1 ms step can stay. Dana also asked for a discrete LuGre, for runtime with a 40–50 Hz controller.

**Equations:** spec section 5:
- **Item 1:** the backward-Euler current, $i_{k+1} = \left(\frac{L_m}{dt} i_k + V_k - K_b \omega_{k+1}\right) / \left(\frac{L_m}{dt} + R\right)$.
- **Item 2:** the slip $v_{s,k+1} = v_{i,k} - r\,\omega_{k+1}$.
- **Items 5–6:** the backward-Euler bristles and floor force.
- **Item 7:** the backward-Euler wheel torque balance and the repeated solve.
- **Item 9:** Dana's Forward Euler chassis and pose update, in the world frame.
- **Section 4:** the normal loads from the previous step's accelerations.

**Code changes (cell `ebf0a37d`):**
- **`compute_electrical_subsystem(I, V_cmd)`:** returns the new current as $c_I + k_I\,\omega_{k+1}$.
- **`lugre_coefficients`** (was `lugre_contact`): the Stribeck coefficient, static limit and bristle damping at a given slip speed.
- **`compute_contact_friction_interface(...)`:** returns the new bristle state and floor force as linear forms in $\omega_{k+1}$, and the rolling resistance.
- **`advance_chassis_kinetics(F_floor, state)`:** the new accelerations, global velocities and pose, in Dana's order.
- **`step`:**
  - solves each wheel's balance, repeating until the slip settles, and keeps the count in `last_iterations`;
  - then advances the chassis;
  - sets the slip flags from the slip the bristles saw over the step.
- **`compute_dynamic_normal_forces(a_past, w_F)`:** takes the previous accelerations and the noise as arguments. Dana read them from the state and drew $w_F$ inside it.
- **Noise:** $w_F$ and $w_\mu$ are drawn once at the top of `step`, in her order.
- **Parameter cell:** `implicit_tol` and `implicit_max_iter` added.

**Text changes:** spec sections 2, 4 and 5, the flowchart's electrical, contact and re-coupling boxes, and the architecture note.

**Checked with the current Pololu constants** against the same model at a 10× finer step (dt = 0.1 ms), on the notebook's tests (noise off; re-run 2026-11-05):

| Test | Path difference, 1 ms vs 0.1 ms | End heading, 0.1 ms vs 1 ms | Solves per step, median / max |
|---|---|---|---|
| All 1 V tests, 45/45 and 30/45 | 0–0.2 mm | within 0.1° | 1 / 4–6 |
| 30/45, 10 V forward | 0.6 mm | 0 vs 0° | 2 / 11 |
| 30/45, 10 V strafe | 2.0 mm | −100.1 vs −101.2° | 2 / 13 |
| 30/45, 10 V rotation | 0.0 mm | +374.0 vs +374.3° | 1 / 12 |
| 30/45, ±4 V fwd/back/coast | 1.5 mm | 0 vs 0° | 1 / 19 |
| 30/45, ±4 V side-to-side | 14.7 mm | −48.8 vs −47.4° | 2 / 18 |
| 30/45, ±4 V rotation | 0.0 mm | +16.8 vs +17.2° | 1 / 18 |

Time slipping per wheel is the same at both steps, to within 1 % on one wheel in the side-to-side. All 12 tests take about 1 s at 1 ms.

**Checked earlier with the old BLDC constants**, on Dana's original voltages, same method:

| Test | Path difference, 1 ms vs 0.1 ms | End heading, 0.1 ms vs 1 ms | Time slipping per wheel | Run time at 1 ms |
|---|---|---|---|---|
| 45/45 2 V forward | 2.6 mm | 0 vs 0° | identical | 0.22 s |
| 45/45 2 V strafe | 2.6 mm | 0 vs 0° | identical | 0.20 s |
| 45/45 2 V rotation | 0.0 mm | +2183 vs +2184° | identical | 0.11 s |
| 30/45 2 V strafe | 0.6 mm | −1171 vs −1173° | identical | 0.32 s |
| 30/45 5 V forward | 0.7 mm | 0 vs 0° | identical | 0.12 s |
| 30/45 5 V strafe | 1.1 mm | −1241 vs −1241° | identical | 0.08 s |
| 30/45 ±4 V side-to-side | 35 mm | +1085 vs +1073° | identical | 0.24 s |

- **The reference has settled:** 0.1 ms and 0.01 ms agree to within 0.1–0.3 mm.
- **The side-to-side difference is the largest** because that robot spins about three turns, which magnifies any small difference.
- **The remaining difference is the 1 ms step, not the solve tolerance.** Against a 0.01 ms reference, the 2 V forward run is 2.9 mm off at `implicit_tol` 0.1 mm/s and 3.0 mm at 0.01, 0.001 and 0.0001 mm/s; the error halves with the step (1.5 mm at 0.5 ms, 0.6 mm at 0.2 ms), as backward Euler's should. Tighter tolerances only cost solves (mean 4 → 11 per step, and over 50 at the worst steps), so the tolerance stays at 0.1 mm/s, the same as LuGre's creep slip.
- **Solves per step:** median 1–5. A wheel breaking loose takes up to about 40.
- **One solve per step is not enough.** With the Stribeck coefficient lagging a step, wheels broke loose late or not at all, and paths were off by up to 3 m. Repeating the solve until the slip settles is what makes it match.
- **Apparent slip of a gripping wheel.** Within a step, the wheel solve holds the chassis velocity of step $k$. Measured against the *new* chassis velocity, a gripping wheel therefore appears to slip by about one step's acceleration × dt: 1.6 mm/s at 1.6 m/s². The bristles don't see this, so the slip flags use the slip they saw.
- **Full notebook:** runs without errors in about 10 s.

**Left as it is (for Henry and Dana):**
- **The chassis is held during the wheel solve** (semi-implicit). That's the one-step lag in the gripping slip above. It changes the paths by a few mm.
  - **Decision (Henry, 2026-10-27): keep the two rules** — backward Euler for the current, wheel speed and bristles, Dana's Forward Euler for the chassis and pose — so the position updates stay exactly Dana's. The alternative, backward Euler for the chassis too, would solve all four wheels and the chassis together each step and change her chassis equations, for a few mm.
- **The LuGre constants stay as in change 7** (δ = 2 µm, ζ = 1, fade at 5 mm/s).
  - δ from 0.5 to 5 µm gives the same results.
  - From 10 µm up, wheels that should grip break loose, unless the damping is retuned (`tests/lugre_stiffness_choice.py`).

---

## 6. Centre-of-mass position and height as parameters (commit `0557b8d`)

**What:** two parameters in the parameter cell (`21f5c2d7`), set independently and both from `robot_constants.py`:
- `COG_Y_MM`: the centre of mass (CoG) forward of the base centre. **0 for now (centred)**; the CAD value is `rc.COG_Y_MM` = 10.2 mm.
- `COG_Z_MM`: the CoG height, the CAD value `rc.COG_Z_MM` = 61.1 mm (the height Dana uses).

`Jzz` is the CAD yaw inertia about the CoG, `rc.IZZ_COG_KG_MM2` = 0.007444 kg·m², wherever the CoG sits.

**Why:**
- The chassis motion is measured about the CoG, with the diagonal mass matrix diag($M_r$, $M_r$, $J_{zz}$), so $J_{zz}$ must be the inertia about the CoG. Dana's 0.007726 kg·m² is the same CAD inertia moved to the base centre with the parallel-axis theorem for the CAD offset; with the CoG centred, the consistent value is 0.007444 (4 % lower).
- The CoG offset changes each wheel's yaw lever and the front/rear load split, so it stays a parameter to try later (Henry: "we can arbitrarily change this later").
- Height and offset are independent: changing one never changes the other (Henry, 2026-10-28).

**Equations (spec sections 3 and 4):** each wheel's yaw lever about the CoG is $R_b - y_{cg}\sin\beta_i$:

$$\mathbf{H} = \begin{bmatrix} -\sin\alpha_f & \cos\alpha_f & R_b - y_{cg}\sin\alpha_f \\ -\sin\alpha_f & -\cos\alpha_f & R_b - y_{cg}\sin\alpha_f \\ \sin\alpha_b & -\cos\alpha_b & R_b + y_{cg}\sin\alpha_b \\ \sin\alpha_b & \cos\alpha_b & R_b + y_{cg}\sin\alpha_b \end{bmatrix}$$

and the load split uses the axle distances from the CoG, $y_{f,cg} = y_f - y_{cg}$ and $y_{b,cg} = y_b + y_{cg}$, in place of $y_f$ and $y_b$. With $y_{cg} = 0$ both reduce to Dana's equations exactly.

**Code:** `__init__` sets `y_f_cg` and `y_b_cg`; `_build_kinematics_matrix` uses the levers above; `compute_dynamic_normal_forces` uses the CoG distances.

**Earlier version, replaced:** a `MECHANICAL_DESIGN` switch between `'built'` (CAD CoG 10.2 mm forward, CAD height) and `'proper'` (CoG centred, h = 31.8 mm). Removed in the merge with Dana's v3 (entry 8).

**For reference, with the CAD offset (`COG_Y_MM` = 10.2 mm):** yaw levers 74.4 mm front and 86.7 mm rear (79.5 mm each when centred), static loads 9.16 N front and 4.07 N rear wheel (7.75 / 5.48 N when centred). On the old BLDC constants this made even the symmetric 45/45 strafe yaw, because equal sideways forces then act on unequal levers about the CoG.

---

## 7. LuGre floor friction and smoothed rolling resistance (commit `41bd17d`)

**What:** Dana's stick/slip switch is replaced by Henry's LuGre friction model, with his parameters from `robot_model_voltage.ipynb` (now in `archive/continuous_time/`). Rolling resistance is smoothed through zero wheel speed.

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

None of the LuGre values are measured; they are Henry's tuned choices (see `archive/continuous_time/friction_model_plan.md`, section 7).

### Integration

At $\delta$ = 2 µm the bristles are very stiff: about $3\cdot10^6$ N/m against the rim mass $J_w/r^2 \approx 0.20$ kg with the Pololu motor, which is about 3,800 rad/s. Dana's 1 ms Forward Euler step would be unstable on them, so they use backward Euler (change 5).

### Code changes (cell `ebf0a37d`)

- `OmniMotorState`: new state `self.z` (bristle loads, 4).
- `__init__`: `self.m_rim = Jw / r**2`.
- New `lugre_coefficients(slip, F_n, mu_s, mu_k)`: the Stribeck coefficient, static limit and bristle damping.
- `compute_contact_friction_interface`:
  - Rewritten for LuGre, with the same friction noise $w_\mu$; its backward-Euler form is in change 5.
- `step`: the Condition A/B wheel-speed block is replaced by one torque balance for every wheel.

### Text changes

- Spec sections 1 and 2: new parameters and the state $\mathbf z_k$.
- Spec section 5, items 2–7: the LuGre friction and one torque balance for every wheel.
- Flowchart: the contact box; the stick/slip decision diamond is removed.
- Architecture and configuration notes.

### Checked (Test-1 pattern with the old BLDC constants, 30/45, no noise)

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

**Full notebook:** runs without errors in about 10 s (with change 5).

### Left as it is (for Henry and Dana to decide)

- **The noise ($w_F$, $w_\mu$) is still inside the model class.** The agreed plan is to move disturbances to the simulation layer.

**Later tests (2026-10-27, `tests/`):** these LuGre constants (δ = 2 µm, ζ = 1, damping fading above 5 mm/s) are stiff. A softer set was also tested (δ = 30 µm, ζ = 3, fade at 50 mm/s).

**Decision (Henry, 2026-10-27): keep δ = 2 µm.**
- Both sets give the same robot behaviour (`tests/lugre_constants_robot.py`): paths within 3 mm, and the front wheels break away at 4.38 A vs 4.35 A on a slow current ramp.
- 2 µm is the closest to Coulomb–Stribeck friction through slip reversals: the force changes sign 10–32 mm/s past zero slip, against 32–95 mm/s at 20 µm.
- It reproduces most closely the friction law of Henry's main model. That is the Coulomb/Stribeck "Williams kinetic law" (the Williams-style force law in `archive/williams/williams_model.py`), with μ_d(v) = μ_k + (μ_s − μ_k)·e^(−α_d |v|) from `MU_D_STAT`, `MU_D_DYN` and `ALPHA_D`. LuGre's steady sliding uses this same curve for every δ. The curve was checked against the code, not against the Williams paper itself.
- The real contact's deflection is unmeasured.

---

## 8. Merged with Dana's v3 (commit `0557b8d`)

**What:** Dana's `robot-simulation-open-loop-v3.ipynb` (her original plus LuGre, an exponential bristle update, the Pololu motor, supply limits and new tests) merged into the working copy, item by item as agreed with Henry (2026-10-28). Compared side by side first, with her constants in both (`tests/compare_dana_v3.py`, noise off).

**Taken from Dana's v3:**
- **Motor:** Pololu 37D 30:1, R = 2.18 Ω, $L_m$ = 2.3 mH, $K_t = K_b$ = 0.255, $J_w$ = 0.00018 kg·m² (entry 4).
- **Supply voltage limit:** the applied voltage is clipped to ±`V_max` = 12 V and logged as `V_actual` (`OmniMotorState`, `step`).
- **Tests:** her test cells exactly. Against her original: Blocks 1–2 at 1 V for 0.5 s (was 2 V for 1.5 s), Block 3 at 10 V (was 5 V), the multi-stage tests still ±4 V, and the plot title "Wheel Angular Velocities (LuGre Continuous Response)".
- **Text:** her spec layout and wording, flowchart, architecture note, configuration note, test descriptions and frame diagram, with our equations where the model differs (below). Two corrections to her test description: forward is $+Y_r$ and the strafe goes right along $+X_r$ (entry 1), and the runs are 0.5 s, not 1.5 s.

**Kept from ours** (Dana's v3 differs):
- **Wheel–bristle step (entry 5):** her bristle state uses an exact exponential step with the slip held, and the wheel speed is then advanced with Forward Euler from that force. The stiff wheel–bristle loop therefore still runs explicitly (ω·dt ≈ 3.9 with her $J_w$): on her 1 V tests the gripping wheels' speed changes reverse direction on 55–92 % of steps and their slip reaches 62–97 mm/s. Ours solves current, wheel and bristles together with backward Euler: 0–1 % (10 % in one test) and 2–7 mm/s.
- **Chassis in the world frame (entry 2):** her chassis still advances the local velocity, so a robot that moves while it turns follows a wrong, curved path.
- **Load torque sign:** $T_{load} = r F + T_{roll}$, where F is the floor force on the robot (positive pushes it along the wheel's drive direction). Dana's v3 has $-rF + T_{roll}$, which flips F but not $T_{roll}$. Checked: driving forward at 1 V, wheel 1's floor force is positive, its motor torque +0.0179 N·m and its load torque +0.0162 N·m at 0.1 s (the difference is the torque accelerating the wheel), and both 0.0050 N·m once steady.
- **$J_{zz}$ about the CoG, and the CoG offset and height as parameters (entry 6).**
- **Constants from `robot_constants.py`** for geometry, mass, CoG and friction; the Pololu motor in the notebook (entry 4).
- **The wheel-layout figure** (entry 3), now below her ASCII frame diagram.

**Not taken: her `I_max` = 20 A current clip.** A voltage-driven brushed motor limits its own current through R: with the Pololu motor at 12 V, stall is 5.5 A, and about 11 A when reversing at full speed, which the real motor does draw. A clip in the plant would stand for a particular motor driver's protection, and none has been chosen (Henry, 2026-10-28). It can be added if a driver is picked.

**Checked:**
- **Side by side with her constants** (`tests/compare_dana_v3.py`): paths within 0.5–3.4 mm on her straight and gripping tests; 74 mm on the side-to-side and 197 mm on the 10 V strafe, which both spin (her wheel chatter and local-velocity chassis). Run time 0.9 s for all 12 tests, against 0.6 s for hers.
- **Solves per step** with the Pololu motor: median 1, at most 6 (1 V forward).
- **Supply limit:** a 30 V command is applied as 12 V.
- **Full notebook:** runs without errors in about 10 s, all 13 figures (re-run 2026-11-05; run times on this PC vary).

**Left as it is (for Henry and Dana), from her v3:**
- **Block 3's plot title says "30-45 @ 5.0V"**, but the test now runs at 10 V. The cell is hers, kept exactly.
- **Her spec dropped the disturbance section** of her original. The model still draws both noises, but the spec describes only $w_F$ (section 4), not $w_\mu$.
- **Her spec's heading says "Henry Calibrated Constants"**, while her configuration note says "estimated not-calibrated constants". Most robot values are CAD or unmeasured; the motor values are hers.

---

## 9. Roller (a-axis, "tangential") friction (not committed yet)

**What:** each wheel's rollers let it slide sideways along its axle direction $\mathbf a_i$; the roller bushings now resist that sliding with a friction force, as in Henry's `robot_model_voltage.ipynb` (`mu_a`, `axial_friction_wrench_body`). Dana's model has free rollers (no such force). The force is computed **explicitly**, from the chassis velocity at the start of the step, and acts on the chassis only. **It is off by default** (`roller_friction` = False, Henry 2026-10-29): the saved model has Dana's free rollers until the coefficients are settled.

**Why:** Henry asked for it (2026-10-29). His archived model has this friction; there it cut the peak forward acceleration from about 5 to about 4.4 m/s² (with the old BLDC constants).

**Equations (spec section 5, new item 8; item 9 is the chassis):** with $\mathbf d_i$ the drive direction (row $i$ of $\mathbf H$, first two entries), $\mathbf a_i = (d_{y,i}, -d_{x,i})$ the axle direction ($\mathbf d_i$ turned −90°; radial for tangential wheels), and $\mathbf p_i$ the wheel's contact point from the CoG:

$$v_{a,i,k} = \mathbf a_i \cdot \left( \begin{bmatrix} \dot x_{r,k} \\ \dot y_{r,k} \end{bmatrix} + \dot\theta_k \begin{bmatrix} -p_{y,i} \\ p_{x,i} \end{bmatrix} \right) \qquad \text{(axle slip of wheel } i\text{, chassis velocity of step } k\text{)}$$

$$\mu_{a,i} = \mu_{a,k} + (\mu_{a,s} - \mu_{a,k})\, e^{-\alpha_a |v_{a,i,k}|}, \qquad F_{a,i,k} = -F_{n,i,k}\, \mu_{a,i}\, \frac{2}{\pi}\arctan(k_a v_{a,i,k})$$

$$\mathbf W_{a,k} = \sum_i \begin{bmatrix} F_{a,i,k}\, \mathbf a_i \\ (\mathbf p_i \times F_{a,i,k}\, \mathbf a_i)_z \end{bmatrix}, \qquad \mathbf{\ddot X}_{r,k} = \mathbf M^{-1}\left(\mathbf H^\top \mathbf F_{k+1} + \mathbf W_{a,k}\right)$$

- $\frac{2}{\pi}\arctan(k_a v_a)$ is a smoothed sign: it goes from −1 to +1 within a few cm/s of axle slip, so the force has no jump at zero slip.
- $\mu_{a,k} > \mu_{a,s}$ in Henry's values (0.322 vs 0.2): he expects the bushings to drag more when forced to turn faster.
- The normal loads $F_{n,i,k}$ are the same as for the drive friction (Step 5.2), so the roller friction follows the load transfer and the vibration noise. The floor friction noise $w_\mu$ is not applied: this is bushing friction, not the floor.

**How the "last step" (explicit) roller force works:**
1. At the start of step $k$, the model takes the chassis velocity $\mathbf{\dot X}_{r,k}$ and each wheel's normal load $F_{n,i,k}$.
2. It computes each wheel's axle slip and roller force from them (`compute_roller_friction`), and holds that force over the whole step.
3. The wheel solve (current, wheel speed, bristles; change 5) runs without it: the roller force lies along the wheel's axle, so it cannot turn the wheel.
4. The chassis step adds the roller wrench to the drive forces and advances the velocity and pose (Dana's Forward Euler, world frame).

So the roller force always uses the velocity one step old. That is stable here because, unlike the drive-direction bristles, this force is not stiff: its steepest slope (at zero slip) is $k_a \mu_{a,s} \frac{2}{\pi} F_n \approx$ 170 N/(m/s) per wheel, which changes the chassis velocity at about 160 /s at most (computed from the 30/45 geometry and static loads), far below $1/\Delta T$ = 1000 /s.

**Parameters (`simulation_parameters`, from `robot_constants.py`):**

| Key | Value | Meaning | Source |
|---|---|---|---|
| `roller_friction` | **False** (off by default, Henry 2026-10-29, until the roller coefficients are measured) | include the roller friction (False = Dana's free rollers) | — |
| `mu_a_s` | 0.2 | roller friction coefficient at zero axle slip | `MU_A_STAT` ("TBD: measure via tilt test") |
| `mu_a_k` | 0.322 | at high axle slip | `MU_A_DYN` |
| `alpha_a` | 1 s/m | decay from `mu_a_s` to `mu_a_k` | `ALPHA_A` |
| `roller_smoothing` | 200 s/m | slope $k_a$ of the smoothed sign | `K_SMOOTH_A` |

**Code changes (cell `ebf0a37d`), each marked "(added 2026-10-29, change 9)":**
- `__init__`: the axle directions `a_hat` and the contact points from the CoG `p_wheel`.
- New `compute_roller_friction(v_local, F_n)`: the axle slips, roller forces and their wrench on the chassis; returns zeros when `roller_friction` is False.
- `advance_chassis_kinetics(F_floor, state, W_roller)`: adds the roller wrench to $\mathbf H^\top \mathbf F$.
- `step`: computes the roller wrench from the step-$k$ velocity after the normal loads, passes it to the chassis, and keeps `(F_a, v_a)` in `last_roller` for plotting.
- Parameter cell: the five parameters above; the printed report shows whether it is on.

**Text:** spec sections 1 and 5 (new item 8; the chassis item now includes $\mathbf W_{a,k}$), the flowchart (new Step 5.8 box), and the architecture note.

**Checked with the force switched on** (`tests/roller_friction_check.py`, noise off; the script turns it on itself, and was corrected on 2026-11-05 after the default went to off, which had made its "on" runs silently off; re-run, same numbers):
- **Off reproduces the model before the change exactly** (0.000000 mm on all of Dana's tests).
- **Geometry:** a pure spin gives zero axle slip and zero roller force (the axle is radial, the spin velocity tangential). Driving forward at 0.5 m/s on 30/45, the roller forces total 3.67 N straight back against the motion, with no sideways force and no moment (CoG centred).
- **Stability:** each wheel's roller force changes sign only 1–2 times in 3000 steps (at the motion reversals); while coasting the axle slip settles to exactly zero. No chatter.
- **Effect on Dana's tests** (on vs off):

| Test | Speed at the end, on vs off | Path difference |
|---|---|---|
| 45/45 and 30/45, 1 V forward / strafe | 0.10 vs 0.14–0.19 m/s (about 30–50 % slower) | 17–38 mm |
| Rotation tests | unchanged | 0–0.8 mm |
| 30/45, 10 V forward | 1.06 vs 1.47 m/s | 160 mm |
| 30/45, 10 V strafe | 0.17 vs 0.78 m/s; heading −163° vs −101° | 173 mm |
| 30/45, ±4 V fwd/back/coast | stops at 0.059 vs 0.096 m | 81 mm |
| 30/45, ±4 V side-to-side | heading +196° vs −47° | 651 mm |

- **Full notebook:** runs without errors in about 10 s, all 13 figures, with the force off (the saved outputs); the printed report shows whether it is on.

**Left as it is (for Henry and Dana):**
- **The effect is large because the coefficients are large.** $\mu_a$ = 0.2–0.32 is about half of the drive sliding friction ($\mu_k$ = 0.422), so the rollers take a large share of any sideways or diagonal push. The values are not established; see the tilt-test fit below.
- **The roller force is one step late** (explicit). If the chassis step is ever made fully coupled (all wheels and the chassis solved together), the roller force should go into that solve too.

**Later tests, nothing in the model changed (2026-10-29, `tests/friction_tilt_fit.py`):** Henry's tilt-test angles (Google sheet "with battery", three roller materials, eight robot angles), fitted with the model's 30/45 geometry and load split. Each wheel is assumed to resist up to $\mu_d N_i |\mathbf d_i \cdot \mathbf u| + \mu_a N_i |\mathbf a_i \cdot \mathbf u|$ along the downhill direction $\mathbf u$, with the wheels held and the robot sliding without turning.

| Material | Fit $\mu_{d,s}$, $\mu_{a,s}$ | rms angle error |
|---|---|---|
| Urethane A90 | 0.59 ± 0.17, 0.35 ± 0.17 | 2.9° |
| Urethane A70 | 0.68 ± 0.10, 0.44 ± 0.09 | 1.5° |
| Chloroprene A65 | 0.64 ± 0.10, 0.37 ± 0.10 | 1.6° |
| A70 with $\mu_{d,s}$ held at 0.9 / 1.0 / 1.1 | $\mu_{a,s}$ = 0.22 / 0.13 / 0.03 | 2.2° / 2.9° / 3.6° |

- **The two values trade off** (correlation about −0.97), so the data mainly fix their combination. Other assumptions move them: an elliptical limit per wheel gives 0.83, 0.57 for A70, and fits worse (3.0°).
- **Henry's sheet estimate (A70: $\mu_d$ 1.0–1.1, $\mu_a$ 0.16–0.18) uses 45/45 relations**, which do not hold on the 30/45 robot. The current `robot_constants.py` values are 0.9 and 0.2.
- **With the force on and the A70 fit's values, the robot barely moves:** 0.047 m/s at 1 V forward (0.159 with the force off), 0.018 m/s at 10 V strafe. So these are breakaway values at most; the force as modelled applies about this size at every axle speed. The rolling drag has not been measured. A coast-down test in several directions would measure it.
- **The saved model is unchanged by these tests:** none of the friction values were changed.

---

## 10. Rewrite in plain functions and named constants (draft, not committed yet)

**File:** `RobotSimulationOpenLoopModel_HenryStyle.ipynb`. The working copy above is unchanged, and stays the working copy until Henry decides otherwise.

**What:** the same equations as the working copy, organised differently. Dana's original has one class `OmniRobotSimulation`, a state class `OmniMotorState` and a parameter dictionary `simulation_parameters`. The draft has plain functions and named constants, in the style of Henry's `archive/williams/` modules, in four separate parts:

| Part (one cell each) | What it holds | Rule |
|---|---|---|
| **Model** | wheel geometry, rotations, wheel kinematics, normal loads, friction laws, chassis acceleration | physics only: no `DT`, no noise generator |
| **Timestep** | wheel solve (backward Euler), chassis update, `step` | advances the model by one `DT` |
| **Simulation** | `draw_disturbances`, `measured_outputs`, the history, `run_simulation` | runs the loop and draws the noise |
| **Controller** | `constant_voltage`, `multi_stage_voltage` | sees only `[X, Y, θ, ω₁..₄]`; the MPC goes here |

**Why:** Henry asked for code he can read and understand himself (2026-11-05). The split follows what Henry and Dana agreed on 2026-10-26: the model has no noise functions, the simulation applies the disturbances, and the controller sees only the measured outputs.

**Changes from Dana's original, besides what entries 0–9 already list:**
- **Noise:** she draws $w_F$ in `compute_dynamic_normal_forces` and $w_\mu$ in `compute_contact_friction_interface`. It is now drawn in `draw_disturbances` (Simulation), in the same order, and `step` receives the values. The noise itself is the same.
- **Wheel angles and noise levels:** her test cells overwrite them in the shared dictionary. Each test now builds its own `geometry`, and the noise levels go to `run_simulation` as inputs.
- **Outputs:** the slip flags `S` are no longer in the state; `step` returns them as `slipping`. `last_iterations` is returned as `solves`. `last_roller` is not kept.
- **Test cells:** the repeated run and plot code is now in `run_simulation`, `run_ideal_and_noisy` and `plot_results`, so each block is 10–16 lines and can run without Block 1 having run first.

**What looks different when it runs:**
- Block 3 says 10 V, which is what it applies. Her text says 5 V.
- The multi-stage figures use the same subplot titles as the other blocks.
- The time axis is `k * DT`, so it lines up with the logged samples. Her `linspace` runs up to `t_end`, about 1 ms or less away.
- The plot warnings from the multi-stage cell are gone (she sets a line style twice there).

**Names:** clear symbols are kept (`F_n`, `mu_s`, `mu_k`, `T_load`, `I`, `V`, `z`, `H`, `theta`, `omega`). Only ambiguous ones changed:

| Spec or earlier name | Draft | Why |
|---|---|---|
| `alpha_f`, `alpha_b` | `front_angle`, `rear_angle` | `alpha` is also a friction decay rate |
| `alpha_d`, `alpha_a` | `MU_D_DECAY`, `MU_A_DECAY` | the same; only the import line renames them, `robot_constants.py` is unchanged |
| `beta` | `angles` | another Greek letter for an angle |
| `a`, `sigma_1` (LuGre) | `bristle_decay`, `bristle_damping` | `a` reads as acceleration, `sigma` also means noise level |
| `sigma_V` | `sigma_load` | it is a force noise in newtons, not a voltage |
| `dt`, `Mr`, `Rb`, `r` and the other dictionary keys | `DT`, `ROBOT_MASS`, `WHEEL_ROBOT_DIST`, `WHEEL_RADIUS`, and so on | named constants, mostly from `robot_constants.py` |

**Velocities, named as in `archive/williams/williams_model.py`:** `v_center` (wheel centre, no spin), `v_center_d` (`v_center` along $\mathbf d_i$, which is $\mathbf H \mathbf{\dot X}_r$), `v_contact` (contact point, `v_center - r omega d_hat`), `v_slip_d` and `v_slip_a` (`v_contact` along $\mathbf d_i$ and $\mathbf a_i$). One function, `slip_velocity`, returns both slips. The sign of the wheel term is the opposite of Williams': here a positive $\omega$ drives the robot along $+\mathbf d_i$.

**Text:** the specification cell is unchanged except for two parameter names. The architecture and plant configuration cells describe the new functions and constants. The other text cells are the same as the working copy.

**Checked** (`tests/compare_henry_style.py`, against the working copy as saved on 2026-11-05):

| Case, all 12 test motions | Largest path difference | Largest difference in any state | Slip flags and solve counts differing |
|---|---|---|---|
| Noise off | 3.8e-12 mm | 1.5e-9 | 0 |
| Noise on | 5.2e-12 mm | 6.8e-11 | 0 |
| Noise off, rollers on | 2.0e-12 mm | 7.8e-11 | 0 |
| Noise on, rollers on | 4.5e-12 mm | 5.8e-11 | 0 |
| Noise off, CoG 10.2 mm | 2.2e-12 mm | 5.6e-11 | 0 |
| Noise on, CoG 10.2 mm, rollers on | 1.7e-12 mm | 7.2e-11 | 0 |

- **The states compared:** pose, global and local velocity, wheel speeds, currents, bristles, past acceleration, applied voltages and load torque, at every step of all 72 runs.
- **The draft's own loop:** `run_simulation` against the logging of the working copy's test cells differs by at most 4.7e-12.
- **Separation:** the script also checks that the Model cell has no `DT`, that the Model and Timestep cells have no noise generator or controller, and that the Controller cell cannot reach the plant state.
- **Full notebook:** all code cells run in order without errors or warnings, and give all 13 figures. This was run from a Python script, not yet in a notebook kernel, so the draft has no saved outputs.

**Left as it is (for Henry and Dana):**
- **It runs about 30 % slower** (plant steps of all cases: 7.3 s for the working copy, 9.5 s for the draft). `slip_velocity` works out both slips on every wheel solve, where the solve only needs `v_slip_d`. This may matter once the MPC runs the model many times.
- **`z_offset`, `z_slope`, `F_offset`, `F_slope`** still have the old style of name; `I_stalled` and `dI_domega` replaced the same style for the current.
- **`geometry` and `state` are still dictionaries.**
- **The other scripts in `tests/`** use the working copy's class, so they do not run on the draft.
- **The spec's "chassis contact speed"** (section 5, item 2) is what the draft calls `v_center_d`. The spec's wording is unchanged.
