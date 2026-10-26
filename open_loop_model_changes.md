# Changes to Dana's open-loop model

**Working copy:** `RobotSimulationOpenLoopModel_HenryConstants.ipynb`
**Branch:** `open-loop-model-robot-constants`
**Dana's original (unchanged):** `dana-models/RobotSimulationOpenLoopModel.ipynb`

Each change is made only when Henry asks for it, one at a time. Each entry says what changed, why, where, and how it was checked. The equations in the notebook's specification cell are updated with the code, in Dana's layout.

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

## 2. LuGre floor friction and smoothed rolling resistance (this commit)

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
