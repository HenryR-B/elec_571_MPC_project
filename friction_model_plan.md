# Drive-axis friction model: current state and LuGre plan

Status: plan only. The model on `main` at commit `aac68af` is the switch model described in section 1. Nothing in section 4 is implemented yet.

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

## 5. Other pending work (written, not tested; scripts in `pending_patches/`)

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
