# Plan: the BLDC with a current controller, as on the robot

**Why.** On the robot the controller does not set a voltage. It sends each Simovr (Tinymovr) driver a current command; the driver's internal current loop sets the voltage, and the motor's real current comes out of that, the same way the current comes out of the voltage in our present model. The voltage-scaled copy (`../voltage_scaled/`) keeps the voltage input so it compares with the Pololu copy; this plan replaces it.

**What already exists.**
- `archive/bldc_current_loop/bldc_current_loop_notes.md`: the driver model built on 10-27 and then removed: its equations (Step 5.1, the driver version), the removed code, and the checks.
  - The firmware's loop each PWM period: current filter, PI on the current error, PWM voltage limit, integrator decay while limited.
  - A continuous version for an implicit solver, checked against the exact discrete loop (median 0.0003 A apart).
- `archive/bldc_current_loop/*.py`: the driver and motor-only scripts.
- `robot_constants.py`: every driver value (`CURRENT_BANDWIDTH` 2000 rad/s, `CURRENT_P_GAIN` = bandwidth × L, `CURRENT_I_GAIN` = bandwidth × R, `PWM_FREQ_HZ` 20 kHz, `PWM_LIMIT` 0.8, `INTEGRATOR_DECAY`, `CURRENT_FILTER_GAIN`, `IQ_LIMIT` 20 A → 17.3 A DC-equivalent) and Rustware's velocity-loop gains (`VEL_P_GAIN` 0.196 A per rad/s, `VEL_I_GAIN` 19.6 A per rad).
- `../voltage_scaled/`: the BLDC motor values and the battery limit, already in the plant.

## Steps

1. **Plant: current command in.** `step(state, I_cmd)`: each 1 ms step runs the driver loop (20 PWM periods, as built on 10-27, or its continuous version inside the backward-Euler solve), which sets the motor voltage; the motor equation gives the real current. Keep the per-motor limit (the driver clamps the command at 17.3 A) and the battery limit (Σ V·i / V_bus ≤ 20 A; how the real robot meets it is still to be decided: the drivers do not know it).
2. **Model (`OmniRobotLinearizedModel`): current command in.** The current loop's bandwidth (2000 rad/s, about 0.5 ms) is faster than a 1 ms step, so the simplest model is i = i_cmd, with the voltage it needs as an output, V = R·i + Kb·ω (+ L di/dt). If that is not close enough, add the driver's states (filtered current, integrator).
3. **MPC: currents as inputs.**
   - The current limit becomes a plain input bound, |i_cmd| ≤ 17.3 A.
   - The voltage headroom becomes a linear constraint, |R·i + Kb·ω| ≤ 13.44 V.
   - The battery limit Σ V·i ≤ 336 W is not linear; linearize it along the plan or use a conservative per-motor share.
   - The Bryson costs become costs on current and current change, which replaces the "same force" voltage scaling of the voltage-scaled copy.
4. **PID: the robot's own velocity loop.** Dana's wheel PI outputs current instead of voltage, or uses Rustware's gains (0.196 A per rad/s, 19.6 A per rad) directly.
5. **Observer:** the voltage input to its prediction becomes the current command (or the measured current); the measured currents and wheel speeds stay as they are.
6. **Tests:** the same checks and closed-loop paths as the voltage-scaled copy, then compare the three: Pololu, BLDC voltage-scaled, BLDC current-controlled.

## Open questions (Henry and Dana)

- How detailed the driver model in the plant should be (the 20 kHz loop, its continuous version, or ideal current tracking).
- Where the 20 A battery limit is enforced on the real robot, and so how to model it.
- The COG: kept centred for now, to compare with the Pololu copy; the CAD value (10.2 mm forward) later.
