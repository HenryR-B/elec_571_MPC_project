# Disturbance-observer MPC (DOB MPC): formulation, the bug, the fix (11-14)

Checked by reading `merged/ControllerModels.ipynb` and running the observer and the closed loop (one run at a time). Not changed in `merged/` yet: the fix waits for Henry and Dana.

## 1. The formulation as coded

**Model** (`OmniRobotNominalModel`; the DOB MPC uses this one, not `OmniRobotNominalModelWithDisturbance`)
- State $x = [p;\ v;\ i]$: pose $p = [X, Y, \theta]$, global velocity $v$, motor currents $i$ (4). Input $u = V$ (4 voltages).
- $H$ (4x3): each wheel's drive direction and yaw lever. $M = \mathrm{diag}(M_r, M_r, J_{zz})$.
- $A_{vi} = \frac{K_t}{r} M^{-1} H^\top$ (current to acceleration), $A_{iv} = -\frac{K_b}{L_m r} H$ (back-EMF), $A_{ii} = -\frac{R}{L_m} I_4$, $B = \frac{1}{L_m} I_4$ on the current rows.
- Continuous, global frame:
  $$\dot p = v, \qquad \dot v = R(\theta)\, A_{vi}\, i, \qquad \frac{di}{dt} = A_{iv} R(\theta)^\top v + A_{ii}\, i + \frac{V}{L_m}$$
- Discrete: exact zero-order hold ($\exp$ of the block matrix) at each predicted heading $\theta_j$, giving $A_{d,j}, B_{d,j}$.

**Observer** (`ChassisDisturbanceObserver.update`), every 20 ms
- $L = \mathrm{diag}(L_t, L_t, L_r)$, $L_t = 1 - e^{-15 \cdot 0.02} = 0.259$, $L_r = 1 - e^{-20 \cdot 0.02} = 0.330$.
- $v_k$: the pose-difference velocity (camera pose, 20 ms window, global frame).
  $$v_{pred} = v_{k-1} + \Delta t\,(a_{nom} + M^{-1}\hat d), \qquad \hat d \leftarrow \mathrm{clip}\!\left(\hat d + L\,M\,\frac{v_k - v_{pred}}{\Delta t},\ \pm 25\right)$$
- $a_{nom} = -D_{damp} v + B_{force} u$ **only if the model has `D_damp`. It does not, so $a_{nom} = 0$.**
- $\hat d = [F_x, F_y, M_z]$, global frame (N, N, N m).

**MPC prediction**: $x_{j+1} = A_{d,j} x_j + B_{d,j} u_j + [0;\ M^{-1}\hat d\,\Delta t;\ 0]$, with $\hat d$ held constant over the horizon.

## 2. The bug (confirmed)

With $a_{nom} = 0$ the update is $\hat d_k = (1-L)\hat d_{k-1} + L\,M\,(v_k - v_{k-1})/\Delta t$: a low-pass filter of **mass x measured acceleration**, i.e. the *whole* force on the chassis (motor force + disturbance). The MPC already has the motor force ($R(\theta) A_{vi} i$), so it is counted twice.

Open-loop check (forward push; Y axis, N):

| moment | true unmodelled force | model's motor force | $\hat d$ as coded | $\hat d$ with the fix |
|---|---|---|---|---|
| speeding up | -3.42 | +14.78 | **+6.74** (wrong sign) | -0.29 |
| cruising | -0.53 | +0.53 | +0.03 (misses the drag) | **-0.53** |
| braking | +0.94 | -8.40 | -1.03 | -0.58 |
| RMS error | | | **2.48** | **0.63** |

(True unmodelled force in this plant = rolling drag + wheel spin-up inertia; the nominal model has neither.)

## 3. The fix

Predict the acceleration the model gives the measured currents, averaged over the last step:
$$a_{nom,k} = R(\bar\theta)\, A_{vi}\, \bar i, \qquad \bar i = \tfrac12 (i_{k-1} + i_k), \quad \bar\theta = \tfrac12(\theta_{k-1} + \theta_k)$$
Everything else unchanged. Then $\hat d$ is only what the model leaves out. Code: the observer stores the last currents and heading instead of `u_past`, and the MPC passes `obs.I` and the measured heading (`scratch_before/models/dana_mpc_fixes/changes.md` has exactly this). Averaging the currents was the best of the three simple choices (average 0.60 N RMS error, previous 0.86, current 0.81).

## 4. Other findings

- Frames and signs are consistent (all global).
- Stable: with the fix the error dynamics have eigenvalues 0.741 (x, y) and 0.670 (yaw).
- The +-25 clip uses one number for N and N m: 25 N m in yaw can never bind. Not hit in any run.
- `u_prev` is passed but unused; measured currents are the right input.
- With 5 mm camera noise (now 0) $\hat d$ would be very noisy (about 14 N std).
- `OmniRobotNominalModelWithDisturbance.__init__` still sets an unrotated `self.E_c` that nothing reads (leftover).

## 5. Closed loop (Dana's section 1, her weights, DOB MPC only)

| DOB MPC | section-1 RMS (cm) | same-instant RMS (cm) | RMS during holds (cm) |
|---|---|---|---|
| $\hat d$ forced to 0 | 0.654 | 0.564 | 0.22 |
| as coded | 0.648 | 0.967 | 0.52 |
| with the fix | 0.849 | **0.336** | **0.08** |

**The section-1 RMS compares the position after the step with the reference at the start of the step** (`log_pos` is logged after the 20 plant substeps; `pos_err` uses `POS_REF[k]`). At 0.97 m/s that is up to 1.9 cm, so the metric rewards lagging, for every controller. Compared at the same instant, the fix is clearly better.
