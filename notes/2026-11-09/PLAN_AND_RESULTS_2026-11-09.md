# Plan and results, 2026-11-09

Read this first next session. Nothing here is committed (Henry commits himself).

## 1. Where things stand

**Main result:** a new MPC model, `JP3`, in [MPCModel_Jacobian.ipynb](../../MPCModel_Jacobian.ipynb).
- **The model:** the plant's own equations, linearized with exact Jacobians three times per 20 ms step.
- **The bound:** a friction bound |z| <= z_max at 2 standard deviations of the friction noise.
- **Accuracy:** the best controller on almost every path in Dana's test suite, using only the measured pose, wheel speeds and currents.
- **Time:** median solve 3-7 ms, worst under 15 ms, inside the 20 ms budget.
- **The notebook:** runs top to bottom in about 1.5 min, reading Dana's notebooks straight from `dana-models/Project Design Folder.zip` (nothing extracted or changed).

Files changed today (all uncommitted):

| File | What changed |
|---|---|
| `RobotSimulationOpenLoopModel_Merged.ipynb` | Plotting and style only; the plant's maths is unchanged, every change was checked to give identical numbers. See section 4. |
| `MPCModel_Bristle.ipynb` | `BristleModel` rewritten in Henry's style (identical numbers); step-by-step derivative comments in `linear_model`; plant names made explicit; prediction plot cell added. |
| `MPCObserver_Bristle.ipynb` | Henry's own edits. **He commented out `VELOCITY = slice(3, 6)`, so the cell crashes until it is put back.** |
| `MPCModel_Jacobian.ipynb` | New: the Jacobian models, the generic MPC, Dana's suite, the results. |
| `Figures/friction_vs_slip.png` | New: the friction figure. |
| `notes/2026-11-09/` | This file, plus raw logs and scripts in `raw/`. |

## 2. Plan: next steps (ask Henry before each)

1. **Decide on JP3 as "the" MPC.** If yes, move it into `RobotSimulationMPC.ipynb`, or give Dana the adapter class (`JacobianMPCForDana` in the notebook) to paste into her `ControllerAnalysis`.
2. **Estimator.** JP3 currently estimates velocity from the model's prediction plus the pose error, and z from each wheel's torque balance with the measured current. The observer notebook (1 ms Kalman, `MPCObserver_Bristle`) is unfinished. Part 4 onwards was never built, and Henry's VELOCITY edit breaks it. Decide: finish the observer, or keep the simple estimator. Dana thinks the observer runs at 20 ms.
3. **Decisions waiting on Henry and Dana:**
   - **Motor.** The plant uses Dana's Pololu 37D 30:1 at 12 V: K_t = K_b = 0.255, R = 2.18, so 47 rad/s at most. `robot_constants.py` has a 5010 360 KV brushless at 16.8 V: K_t = 0.0265, R = 0.24, L = 0.133 mH, about 630 rad/s if direct drive. Henry said 47 rad/s "should not be the limit". Switching is a plant change.
   - **Roll gain error.** The plant's roll load transfer gives 3.4% too much roll moment with the 30-45 layout (exact for 45-45). The fix:

     ```python
     self.roll_gain_front = (Mr * h / wheelbase) * rear_y / (2 * front_x)
     self.roll_gain_rear = (Mr * h / wheelbase) * front_y / (2 * rear_x)
     ```

     A plant change, and Dana's load model.
   - **Friction noise.** It's white noise redrawn every 1 ms. That's why sliding friction looks scattered and why disturbances barely affect a rolling wheel. A low-pass or held noise would make disturbance tests visible. A plant change.
   - **Large σ_μ guard.** σ_μ above about 0.3 makes the plant return NaN (μ_k clipped to 0, then μ_s/μ_d divides by zero). Keep σ_μ <= μ_k/3 (0.14 for ours, 0.1 for Dana's). A guard would be a plant change.
   - **The z bound.** 2σ is a design choice. Re-check it if σ_μ changes.
4. **Paper material.**
   - The friction figure. Coulomb (arctan) is a function of slip; our plant's friction comes from the bristle state, and slip is a result of it.
   - The JP3 results table. Why holding d and F_n is needed. Sub-step accuracy. The friction bound as a trust region.
5. **Tell Dana:**
   - Her runner compares the pose after the plant step with the reference from before it. That adds speed x 20 ms (about 2 cm at 1 m/s) to every controller's error.
   - Her `path_time_fast = 8.0` and `path_time_slow = 4.0` are swapped in name.
   - Her `sigma_mu = 0.1` with `mu_k = 0.3` sits at the NaN edge.
6. **Left over from earlier sessions:**
   - Approval to delete `RobotSimulationCurrentLoop.ipynb` and `tests/RobotSimulationOpenLoopModel_Merged_elimination.ipynb`.
   - Renames in `MPCModel_GripLimit` (`H`, `H_a`, `T_load`, `k_wheel`, `M_wheels`, `S`). `MPCModel_Bristle` was renamed today, the grip model was not.
   - Whether to move `predict()` out of `BristleModel` into the check cell. It's only used by the check.
   - Fix the other notebooks' `exec` loading so the editor sees the names (done for `MPCModel_Bristle` only).

## 3. Results

### 3.1 Dana's test suite: final candidates (her plant, `model_choice = 2`, her parameters)

Max position error in cm, measured against the reference one step later. "meas" uses only pose, ω and I; "true" gets the plant's state.

| Path | PID | Dana's MPC | Held bristle MPC | **JP3 meas** | JP3 true |
|---|---|---|---|---|---|
| Dana 8 s, (3, 3, π/4) | 61.4 | 5.8 | 0.0 | **0.0** | 0.0 |
| Dana 4 s | 136.0 | 65.0 | 46.3 | **16.7** | 16.7 |
| short 1.2 s, (0.5, 0.5) | 47.1 | 4.6 | 0.7 | **0.2** | 0.1 |
| short 1.0 s | 50.6 | 13.7 | 8.9 | **2.7** | 1.9 |
| eight 0.5 m, 4 s | 52.9 | 4.4 | 1.6 | **1.0** | 1.0 |
| eight 0.4 m, 3 s | 47.1 | 15.5 | 45.7 | **4.9** | 3.4 |
| eight 0.4 m, 2.5 s | 49.0 | 50.6 | 70.1 | 54.3 | 50.5 |
| eight 0.3 m, 2 s | 37.8 | 35.8 | 52.6 | **34.6** | 33.0 |

Worst solve, in ms:

| Path | Held | JP3 meas | JP3 true |
|---|---|---|---|
| Dana 8 s | 6.6 | 5.7 | 5.6 |
| Dana 4 s | 11.1 | 7.3 | 6.7 |
| short 1.2 s | 9.7 | 8.8 | 8.6 |
| short 1.0 s | 14.4 | 13.2 | 14.5 |
| eight 0.5 m, 4 s | 12.2 | 13.4 | 13.2 |
| eight 0.4 m, 3 s | 19.0 | 13.0 | 13.5 |
| eight 0.4 m, 2.5 s | 20.9 | 12.7 | 12.9 |
| eight 0.3 m, 2 s | 21.0 | 12.8 | 12.3 |

**With our parameters** (rollers on, σ_μ = 0.03; Dana's extra keys kept), max error in cm:

| Path | PID | Dana's MPC | Held | JP3 meas | JP3 true |
|---|---|---|---|---|---|
| Dana 4 s | 142.5 | 74.3 | 54.2 | 23.3 | 23.2 |
| short 1.0 s | 51.4 | 7.5 | 2.1 | 0.8 | 0.8 |
| eight 0.4 m, 3 s | 48.4 | 5.2 | 0.9 | 0.8 | 0.9 |
| eight 0.3 m, 2 s | 38.6 | 11.8 | 17.9 | 15.0 | 13.4 |

### 3.2 Why the paths are hard

- **Wheel speed.** At 12 V with no load, a wheel tops out at 47 rad/s (12 V / K_b), which is 1.4 m/s at the rim. Dana's 4 s path needs 63 rad/s, so 34% of it is impossible for any controller. Her 8 s path needs 31.5 rad/s, which is fine.
- **Grip.** The eights at 2.5 s and 2 s need more acceleration than the grip gives, so every controller ends up 30-55 cm off.
- **PID only *looks* best in X-Y plots.** On the 4 s path it stays near the line (10 cm off it) but falls 136 cm behind schedule. Dana's MPC cuts 34 cm sideways. The bristle MPC is 10 cm off the line and 45 cm behind.

### 3.3 Model accuracy: 0.4 s prediction against the plant

Our parameters, noise off; median (90th percentile) position error in mm.

| Model | 10 V strafe | 4 V side-to-side | random 8 V |
|---|---|---|---|
| Held 100 µm | 1.3 (18) | 3.1 (61) | 42 (103) |
| Held 2 µm | 1.3 (18) | 3.3 (63) | 44 (147) |
| Jacobian, 1 sub-step (exact matrix exponential) | 0.0 (20) | 4.4 (61) | 45 (130) |
| Jacobian, 1 sub-step (backward Euler) | 1.2 (30) | 5.7 (63) | 43 (106) |
| Jacobian, 3 sub-steps (backward Euler) | 0.4 (13) | 2.3 (44) | 27 (89) |
| Jacobian, 5 sub-steps (backward Euler) | 0.2 (8) | 1.5 (33) | 18 (75) |
| Jacobian, 10 sub-steps (backward Euler) | 0.1 (4) | 1.0 (16) | 8 (56) |

- **Relinearizing every 1 ms** gives 0.2 (3.6) mm on side-to-side.
- **Conclusion:** the remaining error comes from holding over 20 ms, not from what is held.
- **Load transfer matters while sliding.** Without it in the held model: strafe 1.3 → 11.1 mm and random 42 → 54 mm. Gripping cases are unchanged.
- **Acceleration as a delayed state** (the plant's way) in the 20 ms MPC model: slightly worse, and it blew up in 1 of 35 predictions. The loads flip front to back and grow each step. Solving the acceleration within the step, as the models do, is better.

### 3.4 Why some values are held, not differentiated

- **d = (μ_s/μ_d)·√(v_s² + v_c²)** has a corner at zero slip (|v_s| in μ_d, on a v_c = 0.1 mm/s scale). Its Jacobian at grip gave eigenvalues up to +390 /s and z exploded to about 500.
- **The same-step load transfer.** Differentiating it gives a lopsided wheel-to-wheel stiffness, an unstable flutter at about 5127 rad/s (816 Hz). Holding F_n removes it at every state tested. Settling the current does not (still +327 /s), so the inductance was not the cause.
- **The plant doesn't show this flutter.** Its loads use last step's acceleration, and its backward Euler damps anything that fast.

### 3.5 What was tried in the suite (round 1: every variant, z bound at 3σ = 0.57 with Dana's σ_μ)

Full log: `raw/suite_round1_all_variants.txt`.
- **Without the friction bound,** all Jacobian models (J1, J2, J4, JP5, JP10) slide 85-95% of the time on hard paths and track worse than the held model. A linear model says more slip gives more force.
- **J2 with the bound** is the best on hard paths: Dana 4 s 17.5 cm, short 1.0 s 7.2 cm.
- **The true Stribeck slope of d while sliding** (J2s, threshold 2 mm/s) doesn't help on its own (78% sliding). Only the bound fixes it.
- **The slip-friction model, F(v_slip)** (S2, SP5, SP10, v_eps 5 mm/s): good on easy paths (0.1 cm), but 60-95% sliding on hard paths and up to 575 cm off.
- **The size of the bound** (JP5), max error in cm:

  | Bound | short 1.0 s | eight 0.4 m, 3 s | eight 0.3 m, 2 s | Dana 4 s |
  |---|---|---|---|---|
  | 3σ (0.57) | 6.8 | 21.2 | 38.4 | 17.5 |
  | **2σ (0.71)** | **2.3** | **4.1** | **33.2** | **16.7** |
  | 1σ | 4.0 | 3.9 | 31.9 | 16.3 |
  | 0σ | 8.8 | 7.2 | 60.6 | 17.5 |

  2σ is the best overall.
- **The OSQP iteration cap** (JP3, the hardest eights): cap 800 gives 7 of 250 solves over 20 ms on the 2 s eight. These repeat on the same steps each run, so they come from the QP, not jitter. Cap 300 gives a worst of 11-18 ms with the same or better accuracy.
- **The first, cold solve** was about 25 ms with a 5-sub-step rollout. A one-sub-step rollout followed by the batch brings it to 13-14 ms.

### 3.6 The held bristle MPC in Dana's benchmark (first run, before the Jacobian work)

- **8 s path:** max error 2.0 cm, against her MPC's 3.95. Those 2 cm were *entirely* the runner's 20 ms logging offset: aligned, it's 0.02 cm. Neither true state, noise off, nor tuning changed it.
- **4 s path:** 43.3 cm, against her MPC's 62.5 and PID's 132.8.

### 3.7 Solve-time jitter on this laptop

- Timing the identical model build 2000 times gave a median of 3.8 ms but occasional 45-65 ms. That's Windows/Python scheduling.
- Slow solves land on different steps in identical runs, which is how to tell jitter from a hard QP.
- The JP5 run with a 42 ms median was disturbed by machine load; the rerun gave 6.7 ms.
- Count only repeatable slow solves.
- 2 µm against 100 µm bristle makes no difference to solve time, because the exact matrix exponential handles stiffness. The model build (about 4 ms) is the larger part of the time; the QP is about 2 ms.

## 4. Other work today (merged notebook, plotting and style only)

- **Friction validation cells** at the end: Coulomb arctan with "=", and the ±μ_d curves. The plant is labelled "bristle model: −F/F_n = μ_s z, z a state (the slip is a result)". The figure is saved to `Figures/friction_vs_slip.png`.
  - The plant's points sit on the Stribeck curve (median gap 5e-6). The 10 of about 12,000 off-curve points are reversal transients in the first 1-2 steps after a ±12 V flip.
  - They're real frictional lag: they don't shrink with dt = 0.1 ms or a 1e-7 tolerance. Filtering z was rejected; Henry agreed it's fine.
- **Test figures:**
  - **X-Y plots:** scaled per plot in robot widths (2·Rb = 0.159 m), X and Y separately, at least 4 units. The 1 V tests use quarter widths.
  - **A third row in each grid:** drive and axial friction F/F_n on ±1, logged with `FrictionLoggedRobot`, a subclass that recomputes the axial forces and checks them against the plant's roller wrench.
  - **Grip flags** on the drive plot: +1 rolling, −1 sliding, from `S_status`.
- **`cross2d(a, b)`:** a static method (a and b are 2-vectors or N x 2), used for the yaw levers.
- **Wheel angles:** one named line per wheel.
- **The wheel-load block** rewritten for readability, with shares and plain-word pitch and roll comments.
- **Confirmed while answering Henry's questions:**
  - The mass matrix is diagonal about the CoG, even with a forward CoG; positions are measured from the CoG.
  - `a_hat` being radial assumes tangential wheels.
  - In the plant, a_past is a discrete state (a one-step memory). In the MPC models it isn't (solved within the step).
- **Noise:**
  - σ_V and σ_μ only move the friction *limit*, so they show only while sliding.
  - A rolling wheel needs σ_V of 2-3 N before it slips at all at 1 V.
  - Henry's test with σ_μ = 1 produced NaN from the first step, so the plots showed only the ideal run.
- **Notes for the observer:**
  - The friction force F is observable from I and ω; z is F scaled by an assumed μ_s·F_n.
  - Dana's version estimates F directly, as a disturbance observer.
  - The MPC needs x_k from an estimator because it solves one QP and doesn't step.

## 5. Raw files in `raw/`

- `suite_round1_all_variants.txt` and `suite_round1.json`: 14 controllers x 8 paths.
- `suite_round2_final_candidates.txt` and `suite_round2_and_extras.json`: the final candidates, plus the bound and estimator tests.
- `mpc_models.py`, `suite.py`, `exact_mpc.py`: the development code. Its paths point at the old scratchpad: the suite expects Dana's zip extracted next to it. The notebook is the runnable version.
- `build_notebook.py`: generates `MPCModel_Jacobian.ipynb`. Edit the notebook directly from now on.
- `run_with_bristle.py`: the first held bristle MPC run in Dana's benchmark.
- `dana_benchmark_with_bristle.png`, `nb_plot.png`: figures from those runs.
