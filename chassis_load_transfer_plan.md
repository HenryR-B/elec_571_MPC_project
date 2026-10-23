# Chassis load transfer, pitch, and roll: research and plan

**Status (2026-10-23):** research only, nothing implemented. Branch `chassis-load-transfer`. Target model: `robot_model_voltage.ipynb`.

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
