# Plan and results, 2026-11-10

Read this first next session (Henry commits himself).

## 1. Where things stand

All the work is in `dana-mpc-copy-with-estimator/`. Dana's folder `dana-models/` is not touched.

| File | State |
|---|---|
| `ControllerModels_Estimator.ipynb` | Uncommitted: `OmniRobotLinearizedModel`, `MPCControllerLinearized`, `RobotObserver1ms` (and the older `FastPoseVelocityEstimator`, `MPCControllerSimple1msEstimator`) |
| `ControllerAnalysis_Linearized.ipynb` | Untracked: the linearized model's checks (Steps 1-3) and closed-loop tests |
| `ControllerAnalysis_Estimator.ipynb` | Uncommitted additions (runner branches `mpc_no_vel`, `mpc_1ms`, turning test, 1 ms comparison) |

`scratch/` was deleted on 11-10: the 1 ms friction estimator and the 1 ms pose-differencing version were both replaced by `RobotObserver1ms`.

**The linearized model now (`OmniRobotLinearizedModel`):**
- **20 states:** pose, global velocity, currents, wheel speeds, bristles z, and the lagged acceleration `a_past` (the plant's `a_local_past`).
- **Loads lagged 1 ms, as in the plant:** F_n = F_static + LT a_past. The same-step 4x4 load solve is gone; it caused growing modes of 300-700 /s.
- **d held in the Jacobians, refreshed every 1 ms:** Jacobians every 5 ms, d worked out again from the state at every 1 ms point. 20 backward-Euler sub-steps per 20 ms step.
- **Step 1 check:** the model's loads match the plant's exactly (gap 1.8e-15 N).
- **Step 3 check, 0.4 s prediction, median / worst:** strafe 0.2 / 2.2 mm, side-to-side 0.7 / 12.9 mm, random 8 V steps 5.4 / 25.9 mm.

**The MPC (`MPCControllerLinearized`):** 20 sub-steps by default. After a failed solve, the next solve starts cold (a warm start from a failed attempt kept OSQP stuck for 99 steps).

**Weights:** in the Linearized analysis runner only, the heading tolerance is its own `max_heading_err = 5 deg` (it was max_pos_err / wheelbase_radius, about 36 deg, which let heading drift 20 deg).

**Closed loop, `mpc_lin_obs`, about 9.5-10.5 ms per step:**

| Path | RMS | Max | Heading | Sliding |
|---|---|---|---|---|
| Dana 8 s | 1.3 cm | 2.0 cm | 0.2 deg | 0 % |
| Dana 4 s (not attainable) | 30.5 cm | 56 cm | 9 deg | 42 % |
| Quick hops | 1.7 cm | 2.8 cm | 2.3 deg | 2.5 % |
| Eight 0.4 m, 3 s per lap | 2.0 cm | 2.6 cm | 2.6 deg | 10 % |
| Eight 0.4 m, 3.4 s per lap (edge of sliding) | 1.8 cm | 2.5 cm | 1.2 deg | 0 % |
| Turns to pi/2 and pi | 2.1-2.2 cm | 3.3-3.6 cm | 0.5-0.9 deg | 0 % |

For comparison on the hops, the eights and the pi turn (RMS): PID 25-56 cm, Dana's simple MPC (with Bryson weights) 3-6 cm.

**Why `mpc_lin` and `mpc_lin_obs` give the same results:** the plant's `get_observable_state` hands over the exact pose, wheel speeds and currents, with no noise and no delay. The two controllers differ only in their starting state, and with perfect sensors both starting states are nearly right. The observer has nothing to fix. **That is what the next session is for (section 2).**

**Re-learned this session (it was already in `notes/2026-11-09`, section 3.4-3.5):**
- Differentiating d lets the plan run away while sliding hard. Holding it is needed.
- The same-step load transfer flutters. The plant avoids it with the last step's acceleration.
- The z bound (|z| <= 0.93) was the 11-09 fix for the runaway. Henry does not want it for now. (A fully sliding wheel sits at z = mu_d/mu_s, about 0.44-0.9, so the bound caps the force at the grip peak; it does not forbid sliding.)

## 2. Next session (2026-11-11): a delayed camera, so the observer matters

**Goal:** show that the 1 ms observer helps the linearized MPC when the camera pose is late, as on the real robot (camera about 60 Hz, about 50 ms late).

**Why it should help:** with a 50 ms delay, `mpc_lin` plans from a pose 50 ms old: about 5 cm behind on the figure eight at 1 m/s. The observer can carry the pose forward through the delay with the 1 ms wheel speeds and currents.

Steps (ask Henry before each):
1. **Delay in the runner, not the plant.** In the Linearized analysis runner, keep a queue of past poses. The controller gets the pose from D ms ago, every camera period. Wheel speeds and currents stay every 1 ms with no delay. Settings: delay D (0, 20, 50 ms) and camera period (20 ms, or 60 Hz = 16.7 ms).
2. **`mpc_lin` with the delay:** its measured pose is the delayed one. This is the baseline that should get worse.
3. **Observer, simple version first:** apply the late pose to the current estimate as if it were fresh. Expected: better than `mpc_lin`, still biased.
4. **Observer, replay version:** keep a 1 ms history of the estimate and the voltages. When the pose for time t - D arrives, correct the estimate at t - D, then predict forward again to now (50 steps of the 1 ms model for a 50 ms delay). Time it; it runs on its own core.
5. **Tests:** both figure eights, quick hops, Dana 8 s. Controllers `pid`, `mpc`, `mpc_lin`, `mpc_lin_obs`. Delays 0, 20, 50 ms. Expect `mpc_lin` to lose several cm at 50 ms and `mpc_lin_obs` much less.
6. **Then:** camera noise (a few mm, about 0.5 deg), and a Kalman gain for the observer in place of the hand-set gains.

**Questions for Henry before starting:**
- Delay and camera rate to use (50 ms and 60 Hz?).
- Should Dana's simple MPC (`mpc`) also get the delayed pose? It should, for a fair comparison.
- Any delay or noise on the wheel speeds and currents, or keep them perfect?

## 3. Other open items (ask Henry before each)

- **Dana 4 s, one infeasible step per run:** the plan runs away while the robot is pinned at 12 V (z planned out to 200+). Ideas that are not a z constraint: linearize along a rollout of the model instead of carrying the state along the last plan's maps, or clip the linearization points to |z| <= 1 (where we linearize, not a constraint on the plan).
- **Remaining growing modes, 40-140 /s:** Henry suspects the steep friction curve. Test by scaling the bristle stiffness with the lagged loads in place.
- **Heading in the starting state:** fixed on 11-11. `get_state_estimate` now unwraps the estimate's own heading (it used to replace it with the camera's, which matters once the camera is delayed).
- **Heading tolerance in `ControllerAnalysis_Estimator.ipynb`:** only the Linearized runner has the 5 deg `max_heading_err`. Henry's call.
- **Plant load clipping:** when a wheel lifts, clipping it at 0 does not move its load to the others (seen once: 28.9 N against 26.5 N weight). Henry: rare, not the problem. Note only.
- **Later:** the terminal (Riccati) cost; retuning weights for the linearized MPC.

## 4. This session's changes, for reference

- **Figures:** heading arrows on the X-Y plots, heading error under each position-error plot.
- **Heading weight:** separate 5 deg tolerance (heading error on the pi turn 20 deg to 0.9 deg, position unchanged).
- **Benchmark:** `mpc_lin_true` removed from the plots (still in the runner as a test).
- **Test motions:** quick hops, eight 0.4 m at 3 s and at 3.4 s per lap (the edge between rolling and sliding). The 0.3 m eights were removed.
- **Model:** lagged loads (20 states); d held in the Jacobians and refreshed every 1 ms; 20 sub-steps; notes in the class's markdown cell explain each change.
- **MPC:** cold solve after a failed one.
- **Dana-style rewrite (same numbers, checked):** `get_...` method names (`get_state_derivative`, `get_continuous_matrices`, `get_discrete_matrices`, `get_discrete_matrices_along_plan`, ...), "Subfunction N" docstrings, one markdown cell per class with the Subfunction box. The torque-balance bristle estimate is one class, `BristleTorqueEstimator`, used by both the MPC and `RobotObserver1ms` (`reset`/`update`/`estimate`, like Dana's estimators). The MPC takes `state_estimate='pose' | 'observer' | 'true'`, is fed by `estimator_update(obs, camera_frame)`, and uses Dana's `initializeOptimizationProblem` unchanged (one extra state fixed at 1 carries the offset c_j).
- **Analysis intro:** the speed line now says about 9-10 ms per step (it said 60 ms, from the first version).
- **Backups** of the notebooks before each model change are in the session scratchpad only (`models_before_lagged`, `models_before_d1ms`, and the matching analysis files); they will not survive the session.
