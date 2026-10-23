# Omniwheel Geometry and Contact Dynamics: Working Plan

**Status (2026-10-23):** Phase 0 and Phase 1 done on branch `plant-open-loop-verification`. Priority set by Henry: open-loop plant verification (Dana's request) comes before disturbance/geometry work. Phases 2–4 remain blocked on the questions in section 4.

## 1. Goal

Model how the physical geometry of an omniwheel affects the robot's simulated motion. The robot should not behave like a generic planar platform: roller transitions and wheel-ground contact should be able to produce physically motivated irregularities, slip, and chassis bumping.

The main outcomes of interest are:
- Changes in planar motion and d-axis slip.
- Changes in wheel contact forces, motor current, and wheel-speed oscillations.
- Chassis vertical motion, pitch, and roll caused by contact geometry.
- Changes in yaw and planar trajectory caused by the resulting contact-force distribution.

Roller angular velocity is not a priority and should not be added as an independent state unless evidence shows it is needed.

## 2. Confirmed requirements and constraints

- **Rigid chassis:** Treat the chassis as perfectly rigid. Do not add frame-flex states or chassis-flex compliance.
- **Chassis motion:** Retain existing planar position and yaw dynamics, and add vertical translation, roll, and pitch if the chosen model can support them consistently.
- **Vertical dynamics:** Track vertical position and vertical velocity. Compute vertical acceleration from the net vertical contact force and gravity; acceleration is derived from the equations of motion, not an independent state.
- **Geometry-driven effects:** Use wheel phase, derived from the existing wheel-angle state, to determine the wheel's geometry-dependent contact/support behaviour.
- **Contact coupling:** Contact forces must feed back into chassis motion. Normal loads must affect available tangential traction and slip, and tangential forces must contribute to planar force and yaw moment.
- **Unilateral contact:** The ground can push the wheel but cannot pull it down. Contact loss is possible only if the chosen contact model and geometry support it.
- **Compliance location:** If compliance is included, it represents effective X-ring/roller-ground contact compliance, not chassis flexibility.
- **No arbitrary bump signal:** Do not merely multiply drive force by an arbitrary periodic factor or prescribe a vertical oscillation. Any bumping should emerge from the geometry/contact model and rigid-body dynamics.
- **No unnecessary roller states:** Start without independent roller angular-velocity states.
- **Transverse friction:** Keep transverse friction enabled in the model unless the user explicitly changes this requirement.
- **Slip diagnosis:** Investigate why d-axis slip is reported as zero while wheel speed, force, and current oscillate. Check whether rolling-mode constraints make slip zero by construction and whether the reported signal is raw slip or post-contact/mask force.
- **No assumptions:** Do not choose unprovided physical parameters, conventions, model complexity, or notebook targets silently. Ask for clarification or explicitly mark a parameter as an unresolved estimate.
- **Validation:** Test timestep convergence and distinguish verified hardware data from hypotheses or fitted parameters.
- **Workflow:** Keep a written Markdown plan and progress checklist, inspect the actual working notebook before edits, save local checkpoints, make small verifiable changes, and report accurately what is planned, edited, tested, locally saved, and committed.

## 3. Planned work sequence

### Phase 0: Establish the source of truth
- [x] Identify the exact notebook(s) to modify and create. (`rolling_only_matrix_trajectory_mpc.ipynb`, active plant `WheelSlipRobotSimulator`; Dana's `robot22102026D.ipynb` is a separate plant with its own geometry and is used only as the template for the test suite.)
- [x] Inspect the current notebook and its workflow notes.
- [x] Inventory the current state vector, wheel-angle integration, slip equations, rolling/sliding mode logic, contact-force equations, normal-load model, friction model, actuator model, and tests.
- [x] Save an untouched local copy and a separate working copy. (`main` is untouched; the work is on the branch.)
- [x] Save this plan and maintain a dated progress log.

**Exit condition:** The exact source notebook and current equations are identified; no edits have been made on an uncertain file.

### Phase 1: Diagnose the existing zero d-axis slip
- [x] Trace the full calculation of d-axis slip velocity from state variables to logged output.
- [x] Identify whether the rolling mode imposes a no-slip kinematic constraint. (Yes: rolling sets theta_ddot = -H^T qdot / R_w, i.e. d(slip)/dt = 0, so slip stays at round-off.)
- [x] Plot/compare wheel angular velocity, chassis velocity projected onto the wheel's driven direction, raw d-axis slip, contact/rolling mode, required rolling force, static-friction limit, and applied force.
- [x] Check whether roller-gap/contact masking changes the applied force without changing the raw slip signal. (The mask never fired: double degree conversion.)
- [x] Add or correct logging only after identifying the intended definition and sign convention.

**Exit condition:** Explain, with code evidence, whether zero slip is physically predicted or imposed by implementation.

### Phase 2: Derive the geometry-driven contact representation
- [ ] Establish the actual wheel/roller/X-ring geometry available from CAD or measurements.
- [ ] Decide how wheel phase maps to effective support height/contact locations.
- [ ] Define the contact compression and compression-rate sign conventions.
- [ ] Determine whether the geometry needs a continuous profile, discrete contact transitions, or another representation.
- [ ] Keep unknown geometry and material parameters explicit.

**Exit condition:** A documented geometry-to-contact calculation with its inputs, assumptions, and unresolved parameters stated.

### Phase 3: Define and implement the reduced contact model
- [ ] Select a normal-contact law only after stiffness/damping and geometry questions are resolved.
- [ ] Enforce unilateral normal force and define how lift-off/re-contact are handled.
- [ ] Compute the normal forces from contact geometry and rigid-body state.
- [ ] Couple normal forces into vertical, roll, and pitch dynamics.
- [ ] Couple normal-load changes into the existing traction/slip calculation.
- [ ] Feed all tangential and normal contact forces into the appropriate net forces and moments, including yaw.
- [ ] Preserve transverse friction in the standard model.
- [ ] Keep the existing planar model available as a baseline for comparison.

**Exit condition:** A mechanically consistent model, with no arbitrary periodic force injection standing in for contact geometry.

### Phase 4: Integrate the rigid-body dynamics
- [ ] Define the state vector and coordinate conventions.
- [ ] Derive vertical acceleration from contact forces and gravity.
- [ ] Derive roll/pitch angular accelerations from contact moments and the chosen inertia representation.
- [ ] Couple the new states to existing x/y/yaw motion and wheel dynamics.
- [ ] Use the existing integration approach where valid; check whether contact stiffness requires a smaller timestep or event handling.
- [ ] Do not add chassis-flex or independent roller-spin states without evidence they are necessary.

**Exit condition:** State derivatives and force/moment balance are documented and pass basic physical checks.

### Phase 5: Validation and comparisons
- [ ] Zero-input/static equilibrium test.
- [ ] Single-wheel phase/contact-profile test.
- [ ] Symmetric wheel-contact test, checking that symmetry does not create spurious roll/pitch.
- [ ] Uneven/asymmetric phase test, checking expected force/moment direction.
- [ ] Contact-loss/re-contact test if lift-off is enabled.
- [ ] Forward, lateral, yaw, and combined-path tests.
- [ ] Compare planar baseline against geometry-driven contact model.
- [ ] Inspect d-axis slip, transverse slip/force, wheel speed, normal loads, current, yaw, pitch, roll, and vertical motion.
- [ ] Run timestep-convergence checks.
- [ ] Separate measured parameters, literature-supported values, fitted parameters, and exploratory hypotheses.

**Exit condition:** Results are reproducible, key signals are interpretable, and numerical artefacts are distinguished from physical predictions.

### Phase 6: Checkpoint and delivery
- [ ] Save the working notebook and test outputs locally at each completed phase.
- [ ] Keep the plan and progress log updated.
- [ ] Review the final diff and execute the relevant tests.
- [ ] Commit only after the local version is saved and verified.
- [ ] Verify the remote commit independently; never report a commit based only on a submitted request.

## 4. Questions that must be answered before implementation

1. **Notebook target:** Which exact notebook is the source of truth for this work: `rolling_only_matrix_trajectory_mpc.ipynb`, `dana-models/robot22102026D.ipynb`, or another file? Should the attitude/contact version be a separate notebook?
2. **Geometry source:** Do we have a CAD-derived wheel/roller/X-ring profile or measured dimensions to use, or should Phase 2 first identify what data is missing? I will not invent a support-height profile from roller count alone.
3. **X-ring representation:** Do you want the X-rings represented as effective contact stiffness/damping, or do you have material/deflection measurements that should determine the law? If unknown, should implementation pause at a parameterized model until those values are measured?
4. **Ground contact:** Should the ground itself be treated as rigid, with compliance assigned only to the X-ring/roller assembly, or should effective ground compliance also be included?
5. **Lift-off:** Should the initial model permit wheels to lose contact when normal force reaches zero, or should we first use a continuously contacting compliant model and add lift-off only after validating the geometry?
6. **Rigid-body orientation:** Is small-angle roll/pitch acceptable for the initial model, or do you want full 3D orientation/rotation equations from the beginning? The latter requires an explicit coordinate and inertia convention.
7. **Centre of mass and inertia:** Are the robot's centre-of-mass height and full chassis inertia tensor available from CAD? If not, should these remain explicit unknown parameters rather than estimated silently?
8. **Scope of the first deliverable:** Should the first deliverable be diagnosis of the zero d-axis slip only, followed by contact-model implementation, or do you want both completed in the same notebook change once the equations are understood?
9. **Friction convention:** I understand transverse friction must remain enabled. Should its current coefficient/law remain unchanged initially so the contact-geometry effect can be isolated, or is the existing transverse-friction law itself part of the investigation?
10. **Parameter calibration:** Do you have measured bump/vertical-motion, wheel-speed, current, or chassis IMU data that the model should match? If not, should validation initially be limited to physical consistency and sensitivity analysis, without claiming hardware agreement?

## 5. Progress log

- [x] Requirements from the supplied discussion transcribed into this plan.
- [ ] Persistent memory entry confirmed.
- [x] Exact notebook and current implementation inspected.
- [ ] Clarification questions answered.
- [x] Local source and working checkpoints created.
- [x] Implementation started (Phase 1 only).
- 2026-10-23: Reproduced Dana's finding with an open-loop current input: after a yaw spin-up and I_q = 0, omega stays at 3.000 rad/s and kinetic energy is conserved exactly. Cause: r_i is parallel to a_hat_i, so pure yaw makes zero axial roller slip, and the plant has no other loss (no rolling resistance, no bearing drag, and no torque at I_q = 0 in current mode).
- 2026-10-23: Found three bugs: (1) `DRIVE_SLIP_STICK_TOLERANCE` was unused, so rolling mode froze any existing slip (a wheel at +20 rad/s on a stationary robot stayed at +20 rad/s, 'rolling', with zero force); (2) `wheel_gap_mask` converted degrees twice and never fired; (3) the 2-D `np.cross` fails on NumPy >= 2.5.
- 2026-10-23: Changes: stick only when |v_d| <= tolerance; gap units fixed and the mask put behind `ENABLE_ROLLER_GAP_MASK = False`; `wheel_loss_torque()` added with UNMEASURED coefficients (default 0) in `robot_constants.py`; open-loop verification section 4.1 added (zero input, forward, lateral, yaw coast, slip recovery, mirror symmetry, timestep convergence). All checks pass. The checks were also run with placeholder loss values (not committed) to confirm that the loss path stops yaw.
- Open item: measure C_rr / b_w with a yaw coast-down test. Until then, a yaw coast in the plant never decays.
- 2026-10-23 (later): Replaced the current-input checks in section 4.1 with Dana's suite from `robot22102026D.ipynb` (sections 6, 7, 7.1): body-twist command -> inverse kinematics -> wheel-speed PI servo (`VEL_P_GAIN`, `VEL_I_GAIN`, `IQ_LIMIT`) -> plant. Same commands, durations, assertions, and plots. Added `WheelSlipRobotSimulator.enable_friction` for her friction-ablation check. All of her checks pass.
- 2026-10-23 (later): Yaw diagnosis (section 4.5). The rolling d-force equation is correct: an energy balance (dE/dt = motor + a-axis + d-sliding power) holds to 1e-9. Yaw is *created* because every drive force has the same lever +L, so yaw moment = L * sum(F_d). Lateral commands give unequal servo forces (|d_x| = 0.5 front vs 0.707 rear), so sum(F_d) != 0. Near-Coulomb a-axis friction (`K_SMOOTH_A` = 200) resists translation while yaw has zero resistance, so a lateral command spins the robot first (-0.4 rad/s within 0.2 s). Yaw is *never removed* in current mode: a-axis forces are radial (r x a = 0), gripping does no work, wheels cannot slide (4.43 N < 5.96 N), and wheel losses are 0.
- Open question (Henry): `K_SMOOTH_A` = 200 gives most of the a-axis friction at about 1 cm/s slip, whereas Henry described transverse friction as starting low and rising roughly linearly with slip. Not changed.
- 2026-10-23 (later): Verified the rolling matrix form two ways: an independent 11x11 solve of chassis + wheel + grip equations agrees with `_contact_wrenches` to 3e-13 over 200 random states, and our M_eff = M_b + (J_w/R^2) H H^T matches the SJTU four-wheel model (SPIE 6794, eq. 10). Every no-slip model reviewed (Kalmar-Nagy et al., SJTU, Conceicao et al. 2008) gets yaw damping only from wheel-side terms: back-EMF, drivetrain viscous drag, or lumped rotational friction. None gets it from contact friction while rolling.
- 2026-10-23 (later): Added `VoltageDrivenSlipSimulator`: voltage input, winding currents as states (L di/dt = V - R i - K_e theta_dot), and driver bus-voltage (`MAX_MOTOR_VOLTAGE`) and current (`IQ_LIMIT`) limits. Dana's suite now runs in voltage mode by default (`OPEN_LOOP_MOTOR_MODE`) and passes. V_m = 0 stops the yaw with a 99 ms time constant; holding i = 0 still coasts. Windup check: with the current limit active and anti-windup off, a 1.5 m/s step overshoots 56% (27% with anti-windup on). PID and MPC still use the current-input plant.
- Open item: measure b_w (wheel spun in the air, motor open-circuit) and C_rr (robot spun in place at i = 0). Reference only: Conceicao et al. give roughly b_w = 4.7e-4 N m s/rad and C_rr = 0.07 on a geared 2.34 kg robot.
- 2026-10-23 (later): No measurements are possible, so the wheel losses are ASSUMED: C_rr = 0.02 (carpet rolling resistance 0.014-0.020, RESNA 2018) and b_w = 2e-4 N m s/rad (Dana's WHEEL_VISCOUS_DRAG). They act while rolling and are separate from MU_D_* / MU_A_*, which apply once a contact slides. A free yaw coast from 3 rad/s now stops in about 0.57 s in both motor models (0.24 s with V_m = 0).
- 2026-10-23 (later): Dana's plant, with zero motor torque and her drag set to 0, also coasts (3.000 -> 2.999 rad/s in 2 s). Her default braking comes from her viscous drag term. Her transverse friction stops yaw only because her front axles are not radial: positions on a +/-45 deg grid with 30 deg front drive directions. Open question for the CAD: radial axles (this notebook) or Dana's layout?
- 2026-10-23 (later): Equal-current check. In this notebook's convention (every d_i is a CCW tangent) equal commands spin the robot, and the [-,+,+,-] pattern drives straight with zero yaw, in both motor models. Dana mirrors her left and right drive directions, so equal commands cancel in yaw for her. Both conventions are self-consistent.
