# Chassis load transfer, pitch, and roll: research and plan

**Status (2026-10-23, later):** Levels 1 and 2 implemented in both notebooks behind `CHASSIS_LOAD_MODEL` (`"equal"`, `"quasi_static"`, `"dynamic"`; default `"dynamic"`), with the full planar mass matrix for the CoG offset. Checks in section 4.6; results in section 6 below. Branch `chassis-load-transfer`.

## 1. Why: the model cannot slip in the drive direction

All four normal loads are fixed at $N = mg/4 = 6.62$ N. With `MU_D_STAT = 0.9` a wheel grips up to 5.96 N, but at `IQ_LIMIT` its motor can push only $K_tI_{\max}/R_w = 4.43$ N. Breaking grip would take 6.72 A. A hard-acceleration test of the voltage model (forward, forward-then-reverse, sideways, 45° diagonal, yaw, all at saturated current) used at most 79% of the static grip and never slid.

Lowering `MU_D_STAT` is not acceptable: μ_d and μ_a were fitted together from the same force measurements, so lowering one raises the other.

The missing physics is **load transfer**. The centre of mass is $h = 61.1$ mm above the floor (CAD). Accelerating forward takes about $0.86\,a$ N off each front wheel (a in m/s²; front–rear contact spacing 96 mm). At $a \approx 2$ m/s² a front wheel's grip limit falls to about the 4.43 N its motor can push, so it should start to slip. That is close to the 2.5 m/s² at which the real robot was observed to slip.

## 2. What the literature does

- **Purwin & D'Andrea (ACC 2005), a 2.7 kg four-wheel RoboCup omni robot** (h = 50 mm, l = 80 mm). Force and moment balance give six equations for seven unknowns. They close the system by assuming a rigid body on wheels that act as linear springs, which gives $n_1 + n_3 = n_2 + n_4$ (the rigid-pillar problem). The normal forces are then explicit in the wheel friction forces, e.g. $n_1 = [2h(f_2 - f_4) + lmg]/(4l)$. They assume no tipping, so $\ddot z = \ddot\phi_x = \ddot\phi_y = 0$: quasi-static, with no pitch or roll states. Friction limits are $|f_i| \le \mu n_i$.
- **arXiv 2011.01066 (rolling-contact simulation of a mobile robot):** the same closure, with each wheel load linear in its own vertical deflection, plus the centre-of-mass height and the body-frame accelerations.
- **Vehicle dynamics (general):** a rigid four-contact vehicle is statically indeterminate. The load split is determined only by compliance (tyre or suspension stiffness) or by a symmetry assumption. Full pitch, roll, and heave models (half- or full-car) carry the body's height, roll, and pitch, and their rates, as states, with spring–damper contacts. Wheel loads cannot go negative, so a wheel lifts off at zero load.
- **Duda et al. (2021):** the envelope of a double-row omniwheel is not round. Its axle rises and falls about 0.05 mm peak-to-peak, with 16 bumps per revolution. In a compliant-contact model this enters directly as contact deflection.

## 3. Two levels of model

### Level 1: quasi-static load transfer (no new states)

Assume a rigid chassis on four equally stiff contacts. Compatibility (a rigid plane on springs) makes every load linear in its contact position: $N_i = c_0 + c_x x_i + c_y y_i$. This is the general form of Purwin's $n_1 + n_3 = n_2 + n_4$, and it also holds for our layout, where the front and rear tracks differ. The three coefficients follow from three balances, with the horizontal contact forces acting $h$ below the centre of mass:

$$
\sum_i N_i = mg,\qquad
\sum_i N_i\,y_i = -h\sum_i F_{y,i},\qquad
\sum_i N_i\,x_i = -h\sum_i F_{x,i}.
$$

Here $\sum_i F_i$ is the total horizontal contact force (drive plus roller friction), which equals $m\mathbf a$ in body axes. Unilateral contact: if a load would go negative, that wheel is set to $N_i = 0$ and the remaining three are re-solved (tipping onset).

- **New states:** none.
- **Coupling:** the loads depend on the contact forces, which depend on the loads. Use the previous step's forces (one-step lag, consistent with `DT` = 10 ms), or iterate the contact solve. The lag needs to be checked against an iterated reference.
- **New parameters:** only $h$ (CAD: 61.1 mm).
- **What it captures:** load-transfer-induced slip, and the speed- and acceleration-dependent grip changes. It does not capture vibration or bounce.

### Level 2: dynamic heave, roll, and pitch (six new states)

Give each contact an effective vertical stiffness $k$ and damping $c$ (X-ring plus carpet), with the chassis rigid. New states: heave $z$ and $\dot z$, roll $\phi$ and $\dot\phi$, pitch $\vartheta$ and $\dot\vartheta$ (small angles). The contact compression of wheel $i$ is

$$
\delta_i = z_0 - z - x_i\,\phi + y_i\,\vartheta + e_i(\theta_i),\qquad
N_i = \max\big(0,\; k\,\delta_i + c\,\dot\delta_i\big),
$$

where $e_i(\theta_i)$ is that wheel's envelope profile as a function of its wheel angle (roller passing, amplitude $\varepsilon/2$ from Duda). The new equations of motion are

$$
m\ddot z = \sum_i N_i - mg,\qquad
I_{xx}\ddot\phi = -\sum_i N_i\,x_i - h\sum_i F_{y,i},\qquad
I_{yy}\ddot\vartheta = \sum_i N_i\,y_i + h\sum_i F_{x,i},
$$

with signs to be fixed against the body axes (x right, y forward, z up) during implementation. Yaw stays as it is. At small angles the roll/pitch coupling into yaw (products of inertia, $h$ times angle) is second order and is left out at first. The centre of mass does not move relative to the body; tilting moves it horizontally by only about $h\,\vartheta$ (0.5 mm per degree).

- **New states:** 6 (14 → 20 in the current plant, 18 → 24 in the voltage plant).
- **New parameters:** $h$; $I_{xx}$ and $I_{yy}$ (CAD; only $I_{zz}$ is in `robot_constants.py` now); $k$ and $c$ (unmeasured: estimate from an assumed static deflection, e.g. 0.2–0.5 mm under 6.6 N gives $k \approx 1.3$–$3.3\times10^4$ N/m, and $c$ from an assumed damping ratio); and $\varepsilon$ (Duda: 0.05 mm).
- **What it captures:** everything in Level 1, plus bounce, chatter, and lift-off emerging from the wheel geometry. The `ENABLE_FRICTION_VIBRATION` surrogate can then be retired, as the plan's "no arbitrary bump signal" requirement asks. Load transfer between wheels is then automatic.
- **Numerics:** the bounce mode is $\sqrt{4k/m} \approx 140$–$220$ rad/s (22–35 Hz), and roller passing reaches about 85 Hz at 1 m/s. Radau handles the stiffness, but the envelope must be evaluated inside the dynamics, not held per `DT`. Cost will rise; measure it before committing.

## 4. Proposed order

1. **Level 1** in the voltage model behind a switch `ENABLE_LOAD_TRANSFER` (configuration cell). Verify: (a) loads sum to $mg$ and satisfy the moment balances; (b) the hard-acceleration test now slips, and the onset acceleration is close to the ~2 m/s² estimate; (c) the open-loop suite is unchanged with the switch off. Compare the lagged and iterated load solutions.
2. Move the vibration from the friction coefficient onto the normal load, so all load effects act through $N_i$. Dana's fixed per-wheel friction offset stays on μ.
3. **Level 2** once $I_{xx}$, $I_{yy}$, and the $k$ and $c$ assumptions are agreed. Verify static equilibrium (no spurious roll or pitch for a symmetric load), timestep convergence, the energy balance, and lift-off and re-contact.

## 5. Questions before implementation

1. Can CAD give $I_{xx}$ and $I_{yy}$ (about the centre of mass), and confirm $h = 61.1$ mm with the robot at ride height?
2. For Level 2, what static deflection (or contact stiffness) of the X-ring rollers on the field surface is plausible? If unknown, start from an assumed 0.3 mm under 6.6 N and treat it as a sensitivity parameter?
3. Level 1 first, as in section 4, or go straight to Level 2?

## Sources

- Purwin, O., D'Andrea, R., "Trajectory generation for four wheeled omnidirectional vehicles," ACC 2005.
- "Application of Kalker theory of rolling contact for dynamic simulation of mobile robots," arXiv:2011.01066.
- Duda, S. et al., "Determination of the kinematic excitation originating from the irregular envelope of an omnidirectional wheel," Sensors 21 (2021) 6931.
- Vehicle load-distribution background: ScienceDirect topic overview "Independent suspension"; IntechOpen chapter on long combination vehicle stability.

## 6. Implementation and results (2026-10-23)

**Answers to section 5.**
- Q1: CAD gives $I_{xx}$ = 6398.534 and $I_{yy}$ = 6120.905 kg·mm², about the CoG; $h$ = 61.146 mm. The CoG is at (0.597, 10.206) mm from the robot centre P: the CAD origin is on the ground, centred in x, and P is at y = 24.523 mm from it (Henry's sketch). The tensor is about the CoG, not the CAD origin: about the origin, $L_{xx}$ would be at least $m(y^2+z^2)$ = 13,341 kg·mm², but CAD reports 6,398.
- Q2: no measurement is possible. Assumed 0.3 mm static deflection ($k$ = 2.2e4 N/m) and heave damping ratio 0.2 (`robot_constants.py`).
- Q3: both levels, behind one switch, so they can be compared.

**Changes.**
- Full planar mass matrix about P, with the CoG offset c: $M_b$ = [[m, 0, −m c_y], [0, m, m c_x], [−m c_y, m c_x, I_P]], and the matching velocity terms. $I_P$ = `ROBOT_INERTIA` (parallel axis), unchanged.
- `"quasi_static"`: loads and contact forces solved together by fixed-point iteration inside every dynamics evaluation (not the one-step lag).
- `"dynamic"`: six states [z, pitch, roll, rates] appended after all other states (indices 14–19 in the current plant, 18–23 in the voltage plant). Unilateral spring-damper contacts, optional roller-envelope excitation (`ENABLE_WHEEL_ENVELOPE`, off by default).
- Energy-balance checks in sections 4.1/4.5 now use the full mass matrix with `load_model="equal"`.

**Results (section 4.6, voltage plant).**
- Regression: with c = 0 and `"equal"`, the patched plant reproduces the old one exactly (difference 0.0) in both motor modes.
- Static loads are strongly unequal: front 9.23 / 9.09 N, rear 4.02 / 4.13 N. The front contacts are at y = +39.7 mm and the rear at −56.2 mm, and the CoG is 10.2 mm ahead of P. The rear grip limit at rest (0.9 × 4.0 = 3.6 N) is below the motor's 4.43 N.
- Free spin with friction off: the CoG moves on a straight line (error 4e-14 m) while P circles it at |c| = 10.2 mm, confirming the mass matrix.
- Rest: no attitude drift (7e-14). Equilibrium pitch −0.14 deg (nose down).
- Hard forward step (3 m/s request, current-limited): `"equal"` and `"quasi_static"` do not slide (max 71% and 81% of grip). `"dynamic"` slides on all four wheels from t = 0.02 s and keeps sliding (wheelspin): the current reaches the limit in under 1 ms, but load moves to the rear only as fast as the chassis pitches (~5 ms), and once sliding, μ_k N ≈ 3 N < 4.43 N.
- That outcome depends on the assumed contact damping, not the stiffness: wheelspin for ζ ≤ 0.5, no slide at ζ = 1.0, for deflections 0.1–1.0 mm.
- Timestep convergence (DT vs DT/2) through the hard step: 0.03 mm. Drop test from 1 mm: 9 ms airborne, re-contact, settles. Envelope on at 1 m/s: load ripple about 2.4–2.7 N peak to peak.

**Open.**
- Contact damping ratio: it decides whether a current step spins the wheels. Pick a value, or keep 0.2 as a stress case for the controllers.
- Does CAD +y point to the front (dribbler side)? Assumed.
- The rear wheels' grip at rest (3.6 N) is below the motor force, so a rear wheel can spin at standstill under full current if load has not yet transferred.

## 7. Tabled (2026-10-23): status and how to continue

Henry tabled the dynamic model to work on `main`. The static, CoG-based wheel loads (and the Level 1 load transfer) move to `main` behind a switch, because the uneven loading alone may decide whether the robot slips. This branch keeps the dynamic (Level 2) model.

**Where it stopped.**
- Both notebooks on this branch contain the full mass matrix, `CHASSIS_LOAD_MODEL` (`"equal"`, `"quasi_static"`, `"dynamic"`) and section 4.6. They are **not verified**. The voltage-notebook run stopped at the first sanity check (`plantv04`): with `"dynamic"`, a 1e6 rad/s request on every wheel blows up on the third step (currents ~1e5 A, pitch ~80 rad).
- Diagnosis so far: the equations are not unstable at the state before the blow-up (largest Jacobian eigenvalue +0.19 1/s). Radau (max step 1e-3 or 1e-4) and LSODA exceed 200,000 function calls on that step. BDF integrates the same step in 1.3 s with sensible results. The trigger is the rear wheels sliding (static rear load ~4 N, grip 3.6 N < motor force 4.43 N) while the contact loads oscillate.
- Section 4.6 passed in a scratch run before the notebooks were patched (results in section 6). The robot_model_current notebook has not been run.

**How to continue (in order).**
1. Rebase this branch onto `main` once `main` has the load switch, so the shared code (mass matrix, CoG settings, load functions) exists once.
2. Integrator: test BDF as `PLANT_INTEGRATOR` for the dynamic model against Radau on the open-loop suite (results and run time). If BDF is not enough, find the stiff term at the slide onset: the K_SMOOTH_D slope of the sliding law times the oscillating load, or the grip/slide switching inside the dynamics.
3. Re-run the sanity checks, then both notebooks top to bottom, one at a time.
4. Decide the contact damping ratio. It decides whether a hard current step spins the wheels (section 6).
5. Confirm with CAD: the wheel angles (front 30 deg, rear 45 deg, all on r = 79.485 mm) and that CAD +y points to the front. The static load split depends directly on both.
6. Then plan step 2: move the vibration from the friction coefficient onto the normal load via the roller envelope, and retire `ENABLE_FRICTION_VIBRATION`.

## 8. Resumed on main's model (2026-10-23, later)

`main` now has the static and quasi-static wheel loads from the CoG (`WHEEL_LOAD_MODEL`), the full planar mass matrix, separate wheel positions and directions (`WHEEL_POSITION_DEG`, `WHEEL_ALPHA_DEG`), no wheel-angle states, the load vibration (on the normal loads), and a nominal-model PID. This branch merged `main` (taking `main`'s model files) and re-implemented the dynamic model on top as `WHEEL_LOAD_MODEL = "dynamic"` (default on this branch):

- Six attitude states after all others: indices 10-15 in the current plant, 14-19 in the voltage plant (after the winding currents). Loads from unilateral spring-damper contacts, evaluated at every dynamics call; the load vibration scales them as for the other models.
- The roller-envelope excitation is not ported: it needed the wheel-angle states, which `main` removed. The stochastic load vibration covers it for now. If the envelope comes back, it needs a roller phase integrated inside the step (about 5 rad per 10 ms step at 1 m/s).
- The Radau blow-up from section 7 is gone: it came from the grip/slide solve releasing several wheels at once (fixed on `main`: one wheel at a time). The 1e6 rad/s sanity check passes in 2 s with the dynamic loads.
- The energy-balance checks (sections 4.1, 4.5) run with `"quasi_static"` when the configuration is `"dynamic"`; section 4.9 checks the attitude.

**Section 4.9 results (scratch run, voltage plant, CAD CoG, zeta = 0.2):** rest attitude constant to 5e-16 (equilibrium pitch -0.14 deg); timestep convergence 0.01 mm; drop from 1 mm: 9 ms airborne, settles; quasi-static loads from the dynamic run's forces within 0.30 N after 0.1 s.
- Hard forward step: dynamic slides all four wheels (zeta = 0.2), "quasi_static" only the rear ones (its one-step lag leaves the rear at the static 4 N on the first step). zeta = 0.5: one front wheel; zeta = 1.0: nothing slides.
- Hard sideways (+x) step: both models spin at about 20 rad/s with the rear wheels sliding.

**Open:** the contact damping ratio (it still decides whether a current step spins the wheels); whether the one-step lag of `"quasi_static"` is acceptable or the dynamic model should become the reference.
