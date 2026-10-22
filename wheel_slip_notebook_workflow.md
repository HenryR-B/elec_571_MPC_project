# Workflow: Add Wheel Slip to the MPC Notebook

## Goal

Extend `rolling_only_matrix_trajectory_mpc.ipynb` so wheel angular speeds can differ dynamically from the no-slip rolling speeds, and wheel-ground contact forces respond to the resulting slip. Use Henry's existing geometry, sign conventions, and slip/friction functions as the source of truth. Treat Dana's `robot2` model as a comparison for coupled wheel/chassis dynamics, not as the source of geometry or friction equations.

## Reference files

- `rolling_only_matrix_trajectory_mpc.ipynb` — notebook to update when implementation is requested.
- `williams_model.py` — project wheel frame, contact velocity, d/a slip, friction coefficients, and wheel-force definitions.
- `robot_constants.py` — project dimensions, mass/inertia, wheel radius/inertia, motor constants, and friction parameters.
- `dana-models/robot2.ipynb` and `dana-models/robot2_dynamics_motor_friction_lqr_plan_comparison_pid_explained.ipynb` — comparison material for force/wheel/chassis coupling only.

## Workflow

1. **Map the active notebook path.** Identify the simulator class, controller mode, dynamics, and plotting/diagnostic cells that actually run. The notebook contains multiple plant/model sections, so locate active definitions before editing and avoid changing dormant examples accidentally.

2. **Freeze the project's conventions.** Take wheel order, positions, drive/axle directions, robot origin, and parameter values from the project's own definitions. Trace the signs of wheel angular speed, d/a slip, wheel force on the chassis, and contact torque on the wheel. Do not substitute Dana's dimensions or wheel orientations.

3. **Make wheel speeds independent dynamic states.** Keep the four actual wheel angular velocities as states. Remove the no-slip constraint from the drive direction wherever the slip-capable plant is used. The existing effective-mass matrix includes wheel inertia after imposing that constraint, so do not retain it unchanged while also integrating wheel inertia independently; that would count wheel inertia twice.

4. **Use the existing slip definitions.** Evaluate the actual contact-point velocity from robot translation, yaw, wheel location, and actual wheel speed. Reuse or faithfully call the project's `slip_velocities` / `all_wheel_forces` definitions so d-axis drive slip and a-axis passive-roller slip retain their current signs and friction laws. Avoid inventing a second slip convention inside the notebook.

5. **Couple contact force, wheel torque, and chassis wrench.** Apply the existing wheel-force law to the slip components. Apply each contact force to the chassis and its matching reaction torque to that wheel with consistent signs. Derive the coupled planar translation/yaw and four wheel-rotation equations, including the body-frame rotating-coordinate terms where applicable.

6. **Keep actuator and parameter assumptions explicit.** Use the project's motor torque/current constants, current limits, wheel inertia, and normal-load assumptions. Keep provisional friction parameters labeled as provisional. Do not copy Dana's speed-error torque approximation or independent wheel-speed filter into the plant.

7. **Integrate the coupled state consistently.** Advance chassis and wheel states together using one plant derivative, rather than updating wheel speed from one force sample and then advancing the chassis with a different force sample. Preserve the notebook's pose/world-frame conventions and controller interface where they remain physically compatible.

8. **Reconnect constraints and controllers.** Review the plant's traction/current constraints, linearization, and MPC predictions against the slip-capable dynamics. Identify any controller components that still assume no-slip rolling; update them only where needed for a coherent notebook run, and label approximations in the MPC prediction model.

9. **Add diagnostic outputs.** Show per-wheel actual speed, no-slip reference speed, d/a slip, d/a force, motor torque/current, and torque/force balance residuals. Keep existing trajectory and safety plots where they remain meaningful.

10. **Review physical invariants and regression behavior.** Check zero-motion behavior, zero d-slip at the project's no-slip wheel speed, nonzero d-slip after perturbing a wheel speed, expected a-slip under lateral motion, force direction and bounds, equal-and-opposite wheel/chassis contact effects, torque balance, and timestep sensitivity. Compare rolling-only results in low-slip conditions to the previous baseline.

11. **Document what changed and what remains approximate.** Summarize the equations, sign conventions, changed notebook sections, validation performed, and limitations such as provisional friction coefficients, equal wheel loads, simplified motor electrical dynamics, or locally linear MPC predictions.

## Completion criteria

- The notebook uses the project's own geometry and existing slip/friction definitions.
- Wheel speeds evolve from motor torque and contact reaction torque, independently of the no-slip kinematic speed.
- The chassis and wheels receive dynamically consistent contact forces and torques.
- Existing controller/constraint logic is either compatible with the new plant or clearly identified as an approximation.
- Slip and force diagnostics make the new behavior inspectable.
- The updated notebook's results and remaining assumptions are summarized for review.

## Scope boundary

This document is a plan only. It does not authorize or perform notebook, simulator, or model changes.
