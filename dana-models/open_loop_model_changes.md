# Changes to Dana's open-loop model

**Working copy:** `RobotSimulationOpenLoopModel_HenryConstants.ipynb`
**Branch:** `open-loop-model-robot-constants`
**Dana's original (unchanged):** `dana-models/RobotSimulationOpenLoopModel.ipynb`
**Dana's v3 (unchanged):** `dana-models/robot-simulation-open-loop-v3.ipynb`, merged into the working copy on 2026-10-28 (entry 8)

Each change is made only when Henry asks for it, one at a time. Each entry says what changed, why, where, and how it was checked. The equations in the notebook's specification cell are updated with the code, in Dana's layout.

**Order:** the entries are ordered by how much they change the model, smallest first, not by date. Each heading gives its commit, or says it is not committed yet. In time order they were made 0, 1, 7, 4, 5, 3, 2, 6, 8.

**Removed again:** for a while the model took a current command per motor, through a model of the Simovr (Tinymovr firmware) current loop. Henry and Dana then agreed the project uses a voltage input, like a PWM-driven DC motor, with the controller choosing the voltage, so the current input and the driver were removed (2026-10-27). The model's input is Dana's terminal voltage again, and her tests run on her original voltages. Everything worked out for the current loop, the firmware values and the removed code are kept in `archive/bldc_current_loop/bldc_current_loop_notes.md`, for the BLDC model planned for December.

**Checks in entries 2 and 5** were run with earlier constants (and entry 2 while the model took current commands, in amps). They are kept as they were.

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

**Text:** spec section 2 (which velocity is advanced), Steps 5.4, 5.5 and 5.7 (the state vector now holds the global velocities), section 4, item 6, the flowchart and the architecture note.

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
- **Source:** it is the figure from `archive/continuous_time/RobotSimulationOpenLoopModel_ClaudeReference.ipynb` (cell `hc000004`), relabelled with Dana's symbols (θ for heading).

**Text:** cell `6c52e30a` keeps Dana's heading and her wheel notes as text:
- the global and local frames;
- θ = 0 facing +Y_global;
- the wheel angle rule;
- the drive vectors as the rows of H;
- the front and rear stance notes.

**Checked:** the figure draws without errors. The front lines are at 30° and the rear lines at 45° from +X_robot, and the arrows match H. No model code changed, so the test outputs from the change 5 run still apply.

---

## 4. Motor constants (not committed yet)

**What the model uses:** since entry 8, Dana's Pololu 37D 30:1 gearmotor, typed into the notebook's parameter cell: R = 2.18 Ω, $L_m$ = 2.3 mH, $K_t = K_b$ = 0.255 (at the wheel, so the gear ratio is included), and the reflected rotor inertia $J_w$ = 0.00018 kg·m². The robot's geometry, mass, CoG and floor friction still come from `robot_constants.py`. (Henry, 2026-10-28: later, all constants will move into the notebook.)

**Our BLDC motor stays in `robot_constants.py`, not used by this model:** `MOTOR_RESISTANCE`, `MOTOR_INDUCTANCE` and `MOTOR_KT`, a DC-equivalent model of the robot's 5010 360 KV BLDC, kept for the BLDC model planned for December. R and L there are line-to-line (0.24 Ω, 0.133 mH, from a SimpleFOC forum measurement of a 5010 360 KV motor), going with K_t = K_b = 60/(2π·KV); the note in the file explains why they are not halved. The Rustware velocity gains and the Simovr firmware's current-loop values, with their sources, are in `archive/bldc_current_loop/bldc_current_loop_notes.md`.

---

## 5. Backward Euler for the motor current, wheel speed and LuGre bristles (not committed yet)

**What:** Dana's model is a discrete-time map at $dt$ = 1 ms, with Forward Euler for every state. Her discrete step stays. The motor current, wheel speed and LuGre bristle state of each wheel now use **backward (implicit) Euler**: $\frac{dx}{dt} \approx \frac{x_{k+1} - x_k}{dt}$, with the right-hand side at step $k+1$.
- With the Stribeck coefficient and the bristle damping fixed, the new current, the new bristle state and the floor force are all linear in the new wheel speed. So each wheel's torque balance is one linear equation for $\omega_{k+1}$.
- The Stribeck coefficient and the damping depend on the new slip, so the solve is repeated with them updated at the new slip. It stops when the slip changes by less than `implicit_tol` = 0.1 mm/s, or after `implicit_max_iter` = 50 solves.
- **Unchanged from Dana:** the chassis velocity and pose use her Forward Euler, in her order (velocity from the new floor forces, then position from the new velocity), in the world frame of change 2. The load transfer uses the previous step's acceleration, and the load torque is the step's own value, $T_{load} = F_{k+1} r + T_{roll,k}$.

**Why:** both of these are faster than a 1 ms step, so Forward Euler at 1 ms doesn't work on them:
- **The LuGre bristles** (change 7). At δ = 2 µm they are stiff: against a wheel's 18.5 g rim-equivalent mass they ring at about $1.4\cdot10^4$ rad/s. Forward Euler at 1 ms is unstable on them.
- **Our motor's current.** $L_m/R$ = 0.55 ms is shorter than the step, so Forward Euler flips the current's sign each step (a factor of −0.8).

Backward Euler is stable at any step size, so Dana's 1 ms step can stay. Dana also asked for a discrete LuGre, for runtime with a 40–50 Hz controller.

**Equations:** spec section 5, in Dana's layout:
- **Step 5.1:** the backward-Euler current, $i_{k+1} = \left(\frac{L_m}{dt} i_k + V_k - K_b \omega_{k+1}\right) / \left(\frac{L_m}{dt} + R\right)$.
- **Step 5.2:** the normal loads from $a_{x,r,k-1}, a_{y,r,k-1}$.
- **Step 5.3:** the slip $v_{s,k+1} = v_{i,k} - r\,\omega_{k+1}$, and the backward-Euler bristles and floor force.
- **Steps 5.4–5.5:** Dana's Forward Euler chassis and pose update, in the world frame.
- **Step 5.6:** the backward-Euler wheel torque balance and the repeated solve.
- **Step 5.7:** the step as a whole, $\mathbf x_{k+1} = f(\mathbf x_k, \mathbf V_k)$.

**Code changes (cell `ebf0a37d`):**
- **`compute_electrical_subsystem(I, V_cmd)`:** returns the new current as $c_I + k_I\,\omega_{k+1}$.
- **`lugre_coefficients`** (was `lugre_contact`): the Stribeck coefficient, static limit and bristle damping at a given slip speed.
- **`compute_contact_friction_interface(...)`:** returns the new bristle state and floor force as linear forms in $\omega_{k+1}$, and the rolling resistance.
- **`advance_chassis_kinetics(F_floor, state)`:** the new accelerations, global velocities and pose, in Dana's order.
- **`step`:**
  - solves each wheel's balance, repeating until the slip settles, and keeps the count in `last_iterations`;
  - then advances the chassis;
  - sets the slip flags from the slip the bristles saw over the step.
- **Parameter cell:** `implicit_tol` and `implicit_max_iter` added.

**Text changes:** spec sections 2, 4 and 5, the flowchart's electrical, contact and re-coupling boxes, and the architecture note.

**Checked** against the same model at a 10× finer step (dt = 0.1 ms), on Dana's tests at her voltages (`'proper'` design, noise off):

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
- **One solve per step is not enough.** With the Stribeck coefficient lagging a step, wheels broke loose late or not at all, and paths were off by 0.25–3 m. Repeating the solve until the slip settles is what makes it match.
- **Apparent slip of a gripping wheel.** Within a step, the wheel solve holds the chassis velocity of step $k$. Measured against the *new* chassis velocity, a gripping wheel therefore appears to slip by about one step's acceleration × dt: 1.6 mm/s at 1.6 m/s². The bristles don't see this, so the slip flags use the slip they saw.
- **Full notebook:** runs without errors in 12 s.

**Left as it is (for Henry and Dana):**
- **The chassis is held during the wheel solve** (semi-implicit). That's the one-step lag in the gripping slip above. It changes the paths by a few mm.
  - **Decision (Henry, 2026-10-27): keep the two rules** — backward Euler for the current, wheel speed and bristles, Dana's Forward Euler for the chassis and pose — so the position updates stay exactly Dana's. The alternative, backward Euler for the chassis too, would solve all four wheels and the chassis together each step and change her chassis equations, for a few mm.
- **The LuGre constants stay as in change 7** (δ = 2 µm, ζ = 1, fade at 5 mm/s).
  - δ from 0.5 to 5 µm gives the same results.
  - From 10 µm up, wheels that should grip break loose, unless the damping is retuned (`tests/lugre_stiffness_choice.py`).

---

## 6. Centre-of-mass position and height as parameters (not committed yet)

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

At $\delta$ = 2 µm the bristles are very stiff: about $3.5\cdot10^6$ N/m against the rim mass $J_w/r^2 \approx 0.019$ kg, which is about $1.4\cdot10^4$ rad/s. Dana's 1 ms Forward Euler step would be unstable on them, so they use backward Euler (change 5).

### Code changes (cell `ebf0a37d`)

- `OmniMotorState`: new state `self.z` (bristle loads, 4).
- `__init__`: `self.m_rim = Jw / r**2`.
- New `lugre_coefficients(slip, F_n, mu_s, mu_k)`: the Stribeck coefficient, static limit and bristle damping.
- `compute_contact_friction_interface`:
  - Rewritten for LuGre, with the same friction noise $w_\mu$; its backward-Euler form is in change 5.
- `step`: the Condition A/B wheel-speed block is replaced by one torque balance for every wheel.

### Text changes

- Spec section 1: new parameters.
- Section 2: new state $\mathbf Z_k$.
- Section 4: flow items 4 and 8.
- Step 5.3: rewritten for LuGre.
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

**Full notebook:** runs without errors in 12 s (with change 5). A single 1.5 s run takes 0.1–0.3 s.

### Left as it is (for Henry and Dana to decide)

- **Two of Dana's descriptions no longer match the results:**
  - the "Transient Speed Chattering" note in the multi-stage test description;
  - the plot title "Wheel Angular Velocities (Stick-Slip Spikes)".
  
  LuGre doesn't produce those spikes.
- **The noise ($w_F$, $w_\mu$) is still inside the model class.** The agreed plan is to move disturbances to the simulation layer.

**Later tests (2026-10-27, `tests/`):** these LuGre constants (δ = 2 µm, ζ = 1, damping fading above 5 mm/s) are stiff. A softer set was also tested (δ = 30 µm, ζ = 3, fade at 50 mm/s).

**Decision (Henry, 2026-10-27): keep δ = 2 µm.**
- Both sets give the same robot behaviour (`tests/lugre_constants_robot.py`): paths within 3 mm, and the front wheels break away at 4.38 A vs 4.35 A on a slow current ramp.
- 2 µm is the closest to Coulomb–Stribeck friction through slip reversals: the force changes sign 10–32 mm/s past zero slip, against 32–95 mm/s at 20 µm.
- It reproduces most closely the friction law of Henry's main model. That is the Coulomb/Stribeck "Williams kinetic law" (the Williams-style force law in `williams_model.py`), with μ_d(v) = μ_k + (μ_s − μ_k)·e^(−α_d |v|) from `MU_D_STAT`, `MU_D_DYN` and `ALPHA_D`. LuGre's steady sliding uses this same curve for every δ. The curve was checked against the code, not against the Williams paper itself.
- The real contact's deflection is unmeasured.

---

## 8. Merged with Dana's v3 (not committed yet)

**What:** Dana's `robot-simulation-open-loop-v3.ipynb` (her original plus LuGre, an exponential bristle update, the Pololu motor, supply limits and new tests) merged into the working copy, item by item as agreed with Henry (2026-10-28). Compared side by side first, with her constants in both (`tests/compare_dana_v3.py`, noise off).

**Taken from Dana's v3:**
- **Motor:** Pololu 37D 30:1, R = 2.18 Ω, $L_m$ = 2.3 mH, $K_t = K_b$ = 0.255, $J_w$ = 0.00018 kg·m² (entry 4).
- **Supply voltage limit:** the applied voltage is clipped to ±`V_max` = 12 V and logged as `V_actual` (`OmniMotorState`, `step`).
- **Tests:** her test cells exactly: Blocks 1–2 at 1 V for 0.5 s, Block 3 at 10 V ("BOOSTED VOLTAGE"), the multi-stage tests at ±4 V, and the plot title "Wheel Angular Velocities (LuGre Continuous Response)".
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
- **Full notebook:** runs without errors in 45 s, all 13 figures.
