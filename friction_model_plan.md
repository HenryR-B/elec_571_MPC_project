# Drive-axis friction model: current state and LuGre plan

Status (2026-10-24):
- The speed-ups in section 5 are applied.
- LuGre is implemented as `DRIVE_SLIDING_LAW = "lugre"`, with section 4.15 comparing it with the switch model. Results are in section 6.
- The default law is `"lugre"` since 2026-10-24 (section 7); `DRIVE_SLIDING_LAW = "switch"` turns it off.

Scope: the drive (d) axis friction between each omni wheel and the floor in `robot_model_current.ipynb` and `robot_model_voltage.ipynb`. The roller (a) axis friction is unchanged throughout.

---

## 1. What exists now (switch model, `DRIVE_SLIDING_LAW = "switch"`)

### 1.1 Grip (rolling) wheels: no friction curve at all
Every evaluation of the plant equations starts with all four wheels gripping. `_solve_contact` (cell `9676641c`) solves the robot-level equations, the 3x3 system for body x, y and yaw:

    (M_body + H diag(blend * J_w / R_w^2) H^T) q_ddot = H (known wheel forces) + roller wrench + external wrench - velocity terms

From that solution, it computes the force each gripping wheel needs from the floor:

    F_required = -(J_w / R_w^2) (H^T q_ddot) - tau_wheel / R_w + F_restore,   F_restore = -k v_d,   k = DRIVE_GRIP_SLIP_STIFFNESS = 180 N/(m/s)

If a wheel needs more than its grip limit `mu_s N`, the most overloaded wheel is released and the system is solved again: one wheel at a time, at most 5 solves. Each solve is exact; there is no iteration toward a tolerance.

- Grip is an algebraic constraint with a cap, and the slip of a gripping wheel is held at zero.
- The restoring term `-k v_d` removes leftover slip. It acts on the rim inertia `J_w / R_w^2 = 18.5 g`, a 0.1 ms time constant.

### 1.2 Sliding wheels: Stribeck curve times a sign function
    F_slide = -N mu_d(|v|) * shape(v)
    mu_d(v) = mu_k + (mu_s - mu_k) exp(-ALPHA_D |v|)
    mu_s = MU_D_STAT = 0.9 (not measured: "TBD tilt test"),  mu_k = MU_D_DYN = 0.422 (assumed),  ALPHA_D = 5 s/m

`shape(v)` depends on `DRIVE_SLIDING_LAW`:

| Law | shape(v) | At 1 / 10 / 50 mm/s | Note |
|---|---|---|---|
| `"switch"` (default) | tanh(v / 1 mm/s) | 0.76 / 1.00 / 1.00 | plus the transition band below |
| `"continuous"` | tanh(v / 1 mm/s) | same | no band |
| `"atan"` (original) | (2/pi) atan(200 v) | 0.13 / 0.70 / 0.94 | zero at zero slip |

### 1.3 Transition band (Karnopp 1985 / Leine et al. 1998)
`DRIVE_BREAKAWAY_BAND = 5 mm/s`. A wheel released while its slip is below 5 mm/s gets `sign(F_required) * mu_d(|v|) N` instead of the curve, i.e. the static limit in the direction the robot needs. **It is skipped when the blend is on** (see 1.4).

### 1.4 Grip/slide blend
`DRIVE_MODE_BLEND_TIME = 0.033 s`, about 5 cm of travel at 1.5 m/s. Each wheel has a weight state lambda_i, which is 1 when rolling and 0 when sliding:

    F_i = lambda_i * (rolling part, capped at mu_s N) + (1 - lambda_i) * F_slide
    d(lambda_i)/dt = (target_i - lambda_i) / tau

State layout:
- current plant: `[x, y, phi, vx, vy, omega, theta_dot_1..4, lambda_1..4]` (14 states);
- voltage plant: `[10 states, i_1..4, lambda_1..4]` (18 states).

### 1.5 Validation that exists (sections 4.10 to 4.14; `RUN_PLANT_CHECKS = True` runs them)

| Section | What it checks |
|---|---|
| 4.10 | The three laws; force at breakaway is at least 1.00 of the Stribeck force; robot-level breakaway/re-grip cycles |
| 4.11 | Leine stick-slip benchmark against an exact event-driven reference (switch law period -0.01%) |
| 4.12 | Stress cases: slow current ramp, vibration with stop/go, fast reversals, tight tolerances |
| 4.13 | Aggressive manoeuvres |
| 4.14 | Transitions while the robot moves at 1.5 to 3 m/s, instant switch vs blend |

The force plots overlay four lines:
- the applied force;
- the force that would be needed if every wheel gripped (`force_to_roll`);
- `mu_s N`;
- `mu_k N`.

---

## 2. What we found (problems)

1. **Slip is unreliable near zero.** `v_d` is the difference of two nearly equal speeds, the rim speed `R_w theta_dot` and the ground speed, both about 1.5 m/s. The solver evaluates the equations at trial states inside each step that are far less accurate than the final tolerance. Anything decided from the sign of `v_d` near zero flips between those trial states: a steep tanh, `sign(v)`, or which side of a threshold the slip is on. So does `sign(F_required)`, which comes from the same states.

2. **Mode decisions inside the solver make it slow.**
   - Grip/slide and the band direction are discrete decisions taken at every evaluation, so the equations jump while the solver is converging.
   - Measured: steps with 359 to 1974 evaluations at transitions, and one step with 42,479 evaluations when the band was applied in blend mode.
   - Widening the band to 0.15 m/s with atan made the 4.14 section unable to finish in 400 s.

3. **The blend skips the band.** With the blend on, a sliding wheel whose slip reverses gets the raw curve through zero. That is acceptable for tanh (about ±3 mm/s, crossed in tens of microseconds) but it is the reason the band was wanted.

4. **How long a wheel slides is set mostly by the friction parameters, not by demand.** 4.14 at speed, rear wheels, share of sliding samples:

   | Case | Robot needs more than mu_s N | Need between mu_k N and mu_s N | Need below mu_k N |
   |---|---|---|---|
   | ALPHA_D = 10 | 1-5% | 53-57% | 42-43% |
   | ALPHA_D = 5 | 2-6% | 66-85% | 9-32% |

   Breakaway is real demand. A sliding wheel then only re-grips when its slip returns to zero, and while sliding it gets `mu_k N`, so the gap between `mu_s` and `mu_k` sets how long the slide lasts. Both values are unmeasured.

5. **Run time.** A profile of 1.6 s of 4.14 took 17 ms of wall time per 1 ms step (inflated about 1.5x by the profiler).
   - There are 26 evaluations per step. About 15 of them only re-estimate the Jacobian, because every step is a fresh `solve_ivp` call.
   - Each evaluation costs about 0.5 ms, mostly NumPy overhead on 4-element arrays.
   - Friction transitions add only about 7% to the solver's own evaluations.
   - Pending fixes are in `pending_patches/` (section 5).

---

## 3. Options considered

| Option | Fixes 2.1/2.2? | Cost | Verdict |
|---|---|---|---|
| Keep switch + tanh (current) | Partly. Fine in practice, but the solver struggles at transitions | none | baseline |
| atan + blend, band at 5 mm/s | Same as current; atan is gentler on the solver | config change only | quick fallback; 4.10 already passes with atan |
| atan + band at 0.15 m/s | **No.** Makes 2.2 much worse | tried | rejected |
| Decide modes once per step from the accepted state (Moreau-Jean-style time stepping) | Yes. Equations smooth within a step; transitions up to 1 ms late | medium: restructure `_solve_contact` / `discrete_map` | good alternative if LuGre disagrees with expectations |
| **LuGre (bristle state per wheel)** | **Yes.** No discrete decisions, the force comes from an integrated state, and noise in v only nudges z | larger: 4 new states, new contact solve | **proposed**, as a switchable law |

---

## 4. LuGre plan

### 4.1 Model (Canudas de Wit et al. 1995; tyre version: Canudas de Wit et al. 2003)
Per wheel i, with drive slip `v = v_d,i`:

    dz/dt = v - sigma0 |v| z / g(v)
    F_i   = -(sigma0 z + sigma1 dz/dt + sigma2 v)
    g(v)  = mu_d(|v|) N_i * friction_scale_i     (our existing Stribeck curve, unchanged)

- **Steady sliding (constant v):** `z_ss = sign(v) g(v) / sigma0`, so `F = -sign(v) g(v) - sigma2 v`. With `sigma2 = 0` this is **exactly our current sliding force** for |v| beyond the regularisation width.
- **Steady grip (constant force below the limit):** dz/dt = 0 needs `v (1 - sigma0 z sign(v) / g) = 0`. With `|z| < g / sigma0` that means **v = 0**: zero steady slip, as in the current grip model.
- **Breakaway:** the force can build up to `sigma0 |z|max = g(0) = mu_s N`, the same limit as now. The bristles deflect (presliding) before the wheel truly slides.
- **Near zero slip:** v enters dz/dt continuously (v and |v|), so the force has no discontinuity, the sign of v is never needed, and slip noise only changes z by about (noise x step time).

### 4.2 Parameters (none measured; chosen from physics, then checked for sensitivity)

| Term | Meaning | Choice | Value (rear wheel, N ≈ 4 N) |
|---|---|---|---|
| sigma0 | contact (bristle) stiffness | `mu_s N / delta_pre`, with presliding deflection `delta_pre` = 0.5 mm (sweep 0.1 to 2 mm) | about 7200 N/m |
| sigma1 | bristle damping | about critical against the rim mass: `2 sqrt(sigma0 J_w/R_w^2)` | about 23 N·s/m |
| sigma2 | viscous friction | 0 (no data) | 0 |
| g(v) | Stribeck curve | existing `mu_s`, `mu_k`, `ALPHA_D` | unchanged |

Both sigma0 and sigma1 scale with N, so wheels with different loads behave consistently. They should follow the load model and the load vibration, i.e. be recomputed from N each step.

Solver load: sigma0 on the rim mass gives about 100 Hz; the sigma1 damping time is about 0.8 ms; when sliding at 1 m/s, the z relaxation rate is `sigma0 |v| / g` ≈ 2000 s^-1 (0.5 ms). All of these are smooth, so Radau (implicit) handles them without tiny steps. It should be much gentler than the tanh slope or the mode switching.

### 4.3 Code changes

1. **Config cell (`549569af`):**
   - add `"lugre"` to `DRIVE_SLIDING_LAW`;
   - add `LUGRE_PRESLIDING_DEFLECTION = 0.5e-3`, `LUGRE_DAMPING_RATIO = 1.0`, `LUGRE_SIGMA2 = 0.0`.
2. **`robot_constants.py`:** document the parameters as unmeasured, with the reasoning in 4.2.
3. **States:** append 4 bristle states z_i.
   - current: `[10, lambda(4), z(4)]` = 18;
   - voltage: `[10, i(4), lambda(4), z(4)]` = 22.
   - Alternative: when the law is `"lugre"`, reuse the lambda slots, since the blend is not used. Decision needed: appending keeps every index stable and is safer.
   - Update `reference_state`, the state-shape test (`c4e5f6a7`), the voltage plant's `__init__` and its derivative assembly, and `OpenLoopPlant` (`n_states`).
4. **`_solve_contact` (`9676641c`) for `"lugre"`:**
   - no grip/slide loop, no band, no blend;
   - each F_i comes directly from (z_i, v_i, N_i), so the robot-level equation is solved once (3x3) with all drive forces known;
   - `theta_ddot = (tau - R_w F)/J_w` as now;
   - add `dz/dt` to the derivative.
5. **Diagnostics:**
   - `rolling_mask` needs a definition for plots: "gripping" = `|sigma0 z| < 0.99 g(v)` and `|v|` < 5 mm/s;
   - `force_to_roll` stays as is;
   - report presliding deflection z.
6. **Controllers:**
   - PID: nothing changes; it sees the same interface.
   - MPC: `linear_model` finite-differences the plant, so it adapts, but its prediction model gains states. Check the dimensions in `ConvexMPCController`.
7. **Branch `chassis-load-transfer`:** port after `main` is validated (its attitude states add another index shift).

### 4.4 Will LuGre predict friction like our static / dynamic / Coulomb model?
It does by construction in the regimes we have data or assumptions for, and differs only in the transitions.

| Regime | Switch model (now) | LuGre | Expected difference |
|---|---|---|---|
| Steady sliding at v | `mu_d(|v|) N` | `g(v) + sigma2 v` = `mu_d(|v|) N` | none (sigma2 = 0) |
| Steady grip below the limit | exact zero slip, F = need | zero slip after presliding z = F/sigma0 | none in force; position offset of at most delta_pre (0.5 mm) per wheel |
| Breakaway force, slow ramp | `mu_s N` | `mu_s N` | small |
| Breakaway force, fast ramp | `mu_s N` (capped) | can **exceed** `mu_s N` briefly through the `sigma1 dz/dt` term | a known LuGre feature (rate-dependent breakaway); bounded by sigma1; must be measured in 4.12 |
| Time to break away | instant when the need exceeds the limit | after the bristles deflect: delta_pre at the slip rate, microseconds to a millisecond | small |
| Re-grip | at slip = 0, force steps by ~1 N within 1 ms | z relaxes smoothly | smoother, possibly slightly earlier |
| Oscillating force below the limit | no motion | slow presliding **drift** (a known LuGre artefact; Dupont et al. 2002 elasto-plastic model fixes it) | check in the vibration test; usually negligible |
| Slip reversal while sliding | force crosses zero over about ±3 mm/s | force follows z, no zero dwell | smoother |

Main risk: **sigma1 overshoot at fast breakaway** and **presliding drift**. Both are bounded and measurable. If either proves significant, the elasto-plastic variant (Dupont 2002) or the per-step mode decision (option in section 3) are the fallbacks.

### 4.5 Validation plan (compare LuGre with the switch model, same scenarios)

1. **Curve check (4.10 extension):** steady sliding force vs v for both models; require agreement within 1% for |v| ≥ 20 mm/s.
2. **Slow current ramp (4.12):** breakaway force within 2% of `mu_s N`; same slide duration within 10%.
3. **Fast reversals (4.12):** peak force above `mu_s N`, reported, with an acceptance bound to set, e.g. at most 5%.
4. **Leine benchmark (4.11):** stick-slip period vs the exact reference; the switch model gets -0.01%, LuGre should be within 1 to 2%.
5. **Manoeuvres and at speed (4.13 / 4.14):**
   - final position and heading differences;
   - sliding samples per wheel;
   - peak yaw rate, compared with the switch model plus blend.
6. **Sensitivity:**
   - delta_pre in {0.1, 0.5, 2} mm and damping ratio in {0.5, 1, 2};
   - robot-level results should change little;
   - if they do, the transitions matter and need measurement (see 4.6).
7. **Run time and solver effort:** evaluations per step, the largest step, and wall time for each section.

### 4.6 What would let us measure instead of assume
- **Tilt test (planned):** `mu_s` for the drive axis.
- **Constant-current slide test** (push the robot sideways at a fixed motor current and log the wheel encoder vs the robot's ground speed): `mu_k` and `ALPHA_D` from the slip speed.
- **Presliding:** a slow force ramp on a locked wheel with a dial gauge or encoder gives delta_pre, and hence sigma0. Otherwise keep it as an assumption with the sensitivity result.

---

## 5. Speed-ups and other patches (scripts in `pending_patches/`)

Applied on 2026-10-24 (commit `fc67351`):
- **A:** results identical bit for bit, 1.5x faster.
- **B:** at rtol 1e-5, 2.1x faster in total on the current plant and 3.0x on the voltage plant, and closer to a 1e-8 reference than before.
- **Tolerances:** rtol 1e-4 and 1e-3 gave no further speed (one solver step per 1 ms is already the cap), so the tolerance stays at 1e-5.

The descriptions below are from before they were applied.

- **`patch_speed_a.py`**: removes repeated work with identical results expected:
  - cached wheel geometry;
  - 2-D cross products;
  - single current saturation;
  - fixed diagnostic matrix;
  - no redundant copies;
  - reuse of the PID feasibility check's integration in the same step.

  Verify with `compare_a.py` (bit-for-bit comparison and timing).
- **`patch_speed_b.py`**: reuses the Jacobian across steps (`PLANT_REUSE_JACOBIAN`), and puts the tolerances in config as `PLANT_RTOL` / `PLANT_ATOL`. Measure with `tol_study.py` at rtol 1e-3, 1e-4 and 1e-5 against a 1e-8 reference.
- **`patch_band_blend.py`**: band in blend mode. **Do not apply**: it caused the 42,479-evaluation step (section 2.2).
- **`profile_step.py`**: cProfile of the plant on the 4.14 scenario.

The patch scripts edit the notebooks by cell id and assert on the exact code they replace, so they fail loudly if the code has changed since.

Suggested order:
1. Speed-up A, verified identical.
2. Speed-up B and the tolerance choice.
3. LuGre as `"lugre"`, then the comparisons in 4.5.
4. Decide the default law.
5. Port to `chassis-load-transfer`.

---

## 6. Results: LuGre vs the switch model (section 4.15, 2026-10-24)

Setup:
- Current model, real CoG, `"static"` loads, 1 ms step, no disturbances, open loop.
- LuGre with presliding deflection 0.5 mm, damping ratio 1, sigma2 = 0.
- Three models: **switch** (default, with the 33 ms blend), **switch, no blend** (plain static/kinetic Coulomb-Stribeck), and **lugre**.
- Figures and printed output: `Figures/notebooks/current_model/2026-10-24_LuGre-vs-switch-section-4.15/`.

### 6.1 Friction level: LuGre computes the friction we expect
In steady sliding (slip above 20 mm/s, one way, for 0.1 s), the error against the Stribeck curve `mu_d(|v|) N` is:

| Model | Median | 95th percentile |
|---|---|---|
| LuGre | 0.00-0.03% of mu_s N | 0.02-1.7% |
| Switch, no blend | 0.00% | 0.00% |
| Switch with blend | 0.00-0.17% | up to 1.7% |

The F/N against slip plot shows why the blended switch strays: during the 33 ms blend after each breakaway it sits **above** the curve, up to mu_s at slips of 1-2 m/s. LuGre and the no-blend switch both stay on the curve. **The model that departs from Coulomb-Stribeck friction is the blend, not LuGre.**

### 6.2 Breakaway: LuGre breaks loose below mu_s N under fast loading
The bristles need slip to deflect, and while the wheel slips the Stribeck curve has already dropped. Peak force as a fraction of mu_s N:

| Scenario | LuGre |
|---|---|
| Slow ramp | 0.957 |
| Sideways | 0.875 |
| At speed | 0.887 |
| Fast reversals | **1.076** |

The reversal overshoot comes from the sigma1 dz/dt term. This is LuGre's known rate-dependent breakaway. It approaches mu_s N as the presliding deflection shrinks: 0.967 at 0.05 mm.

### 6.3 Robot level: LuGre matches the plain Coulomb-Stribeck model
End-of-run difference, LuGre against each switch variant:

| Scenario | LuGre vs switch, no blend | LuGre vs switch with blend |
|---|---|---|
| Fast reversals | 1 mm, -0.1 deg | 9 mm, -0.5 deg |
| Sideways 2 m/s, then stop | 0 mm, +0.2 deg (of a -671 deg spin) | 23 mm, -17 deg |
| Slow current ramp | 326 mm, +3.3 deg | 970 mm, +11 deg |
| Transitions at speed | 422 mm, +15 deg | 1862 mm, -25 deg |

The blend reduces sliding and yaw (peak yaw at speed: 1.27 rad/s with the blend, 3.59 without, 4.72 with LuGre).

### 6.4 Sensitivity to the unmeasured bristle parameters (transitions at speed)
- **Presliding deflection matters.**
  - At 0.05-0.1 mm, LuGre's slip and yaw traces lie almost on the no-blend switch model. So LuGre converges to Coulomb-Stribeck as the bristles stiffen.
  - At 0.5-2 mm, the second breakaway slides much longer: back-left slip reaches about 1.9 m/s instead of 1.0 m/s.
- **The damping ratio (0.5-2) barely matters.**

### 6.5 Solver
| | Switch | LuGre |
|---|---|---|
| Typical step | 15 evaluations | 15 evaluations |
| Worst step | 155-1102 evaluations | 58-219 evaluations |
| Wall time | about the same | about the same |

LuGre has no switching spikes.

### 6.6 What this means
1. LuGre does **not** compute friction differently from our static/kinetic model where it matters: sliding follows the same curve, and with stiff bristles the robot moves like the plain switch model.
2. The real modelling choice is **how breakaway and re-grip happen**:
   - instant (the switch model);
   - delayed by the 33 ms blend, an assumption with no physical parameter;
   - set by a presliding deflection (LuGre), a physical quantity that could be measured.
3. **Open questions:**
   - Which default to use: "switch" with the blend, "switch" with no blend, or "lugre"?
   - What presliding deflection? A measurement would settle the at-speed differences in 6.4. Until then, 0.05-0.1 mm keeps LuGre close to the Coulomb-Stribeck model, at slightly more solver effort (max 115-143 evaluations per step instead of 63).

---

## 7. LuGre tuned to the friction curve (sections 4.15-4.16, 2026-10-24)

Section 6 used a 0.5 mm presliding deflection with constant damping. That version strayed from the curve after slip reversals. These are the constants and fixes that make it follow the curve.

### 7.1 Why 0.5 mm strayed
- **At zero slip** LuGre holds its bristle load: F = -sigma0 z, not zero.
- **After the slip reverses**, the bristles must slide 2-3 deflections before the force changes sign. Meanwhile the slip accelerates at up to about 400 m/s^2. At 0.5 mm the force therefore kept the wrong sign up to about 1 m/s of slip.
- **A constant sigma1** added sigma1 dz/dt while the bristles reloaded, overshooting mu_s N by up to 16%.

### 7.2 Constants (single-wheel bench sweep, then the robot)

| Constant | Value | Reason |
|---|---|---|
| `LUGRE_PRESLIDING_DEFLECTION` | 2 um (sigma0 = mu_s N / 2 um, about 1.8e6 N/m on a rear wheel) | See the trade-off table below |
| sigma1 | critical damping against the rim inertia, x exp(-(v / 5 mm/s)^2) | Damps presliding only; no overshoot. A 20 mm/s band brought the overshoot back |
| sigma2 (`LUGRE_VISCOUS_COEFF`) | 0 | The Stribeck curve already holds the velocity dependence |
| `LUGRE_CREEP_SLIP` | 0.1 mm/s; \|v\| in dz/dt -> sqrt(v^2 + creep^2) | Removes the kink at zero slip that made the solver crawl. Keeps the static limit; adds 0.13 mm/s of creep at 80% of mu_s N |
| `LUGRE_RTOL` | 1e-4 (the switch model stays at 1e-5) | Solver tolerance with "lugre" |

Deflection trade-off on the wheel bench:

| Deflection | Force held as the slip reverses | Sign change after | Error beyond 100 mm/s | Robot run time vs switch |
|---|---|---|---|---|
| 0.1 um | 0.98-0.99 mu_s N | 3 mm/s | 0.0% | about 6x |
| **2 um** | **0.92-0.93** | **23-28 mm/s** | **0.9%** | **about 2x** |
| 5 um | 0.89-0.90 | 37-44 mm/s | 2.0% | |
| 20 um | 0.82 | 74-81 mm/s | 48% | |

Things tried and rejected:
- **A v tanh(v/eps) smoothing** of \|v\| broke the static limit: a wheel carried 1.07 mu_s N without slipping.
- **A fixed-step Rosenbrock (ROS2) integrator** was fast (3.5 s against 9 s for the switch model), but its strongly damped large steps suppressed the micro-slip that starts breakaway, so wheels stuck above mu_s N.
- **Starting each step from the largest previous step size** did not speed up the switch model; an earlier comparison claiming it did was wrong.

### 7.3 Results (section 4.16: all checks pass)
**Wheel bench:**
- Breakaway peak 0.974-0.994 mu_s N.
- 0.92-0.93 mu_s N held as the slip reverses; sign change 23-28 mm/s later.
- Largest curve error beyond 100 mm/s: 0.86%.

**Robot level, six scenarios.** Largest curve error beyond 100 mm/s: switch without blend 0.000%, LuGre 0.54% (99th percentile at most 0.13%). LuGre against the no-blend switch model:

| Scenario | End position | End heading | Largest yaw-rate difference | Run time, LuGre vs switch |
|---|---|---|---|---|
| Slow current ramp | 26 mm | -0.35 deg | 0.15 rad/s | 51 s vs 20 s |
| Fast reversals | 0 mm | -0.03 deg | | 14 s vs 13 s |
| Forward 2.5 m/s | 2 mm | +0.44 deg | | 22 s vs 14 s |
| Sideways 2 m/s | 1 mm | -0.55 deg (of a -671 deg spin) | | 25 s vs 14 s |
| Diagonal | 0 mm | +0.03 deg | | 26 s vs 17 s |
| At speed | 37 mm | -0.91 deg | | 40 s vs 17 s |

- Rear-wheel sliding times match to within 2 ms.
- Overall LuGre takes about 1.9x the switch model's run time (it was 30x before the fixes).
- **Smoothness:** LuGre's force is continuous, but at these slip accelerations a reversal still completes within a fraction of a millisecond. The largest change between 1 ms samples is about the same as the switch model's (0.3-1.6 mu_s N; both are driven by the step commands in these tests). A larger deflection makes the transitions slower (20 um: sign change about 75 mm/s after a reversal, robot within 118 mm / 3 deg at speed) at the cost of curve accuracy.

### 7.4 Conclusion
With these constants LuGre behaves like the Coulomb-Stribeck switch model without the blend, at robot level and on the curve, while keeping a continuous force that never drops to zero at a slip reversal. `"lugre"` is now the default (about twice the run time of `"switch"`). With it the full check suite passes on both notebooks, after two switch-specific checks were adjusted:
- 4.4 `toe02` allows 1e-3 instead of 1e-4 between chassis and wheel kinematics while gripping, because LuGre wheels creep slightly under load.
- 4.14 always runs the switch law, because it tests that law's blend.

### 7.5 Implementation check (2026-10-26)
`tests/verify_lugre.py` checks the LuGre code in a notebook independently of the comparison sections. On both notebooks every check passes:
- **Equations:** `lugre_contact` matches dz/dt = v - sigma0 |v| z / g(v) and F = -(sigma0 z + sigma1 dz/dt + sigma2 v) in physical units, to rounding.
- **Steady sliding:** on the Stribeck curve within 0.005% of mu_s N, always opposing the slip.
- **Static limit:** the bristle load never exceeds mu_d/mu_s, so the force never exceeds the limit.
- **Zero slip:** the force is the held bristle load.
- **Lifted wheel:** carries no force.
- **Dissipation:** friction does negative net work over slip cycles.
- **State slots:** in both plants the bristle states (current 14-17, voltage 18-21) feed the force and receive the rates; blend weights are inert under LuGre and bristles frozen under the switch law.
- **Creep:** under a very slow push the bristles relax as they load (load 0.71 at 0.1 mm/s of slip), so holding a load takes a small creep slip, as designed.
