# BLDC and Simovr current-loop notes (for the December BLDC model)

**Kept from:** `dana-models/open_loop_model_changes.md` and `RobotSimulationOpenLoopModel_HenryConstants.ipynb`, on 2026-10-27.

**Why these are here:** the open-loop model briefly took a current command per motor, passed through a model of the Simovr (Tinymovr firmware) current loop. Henry and Dana then agreed that the project uses a **voltage input**, like a PWM-driven DC motor, with the controller choosing the voltage. The current input and the driver were removed from the model. These notes keep what was worked out, for the proper BLDC model planned for December.

**Still in `robot_constants.py`:** every firmware and Rustware value below (velocity gains, `IQ_LIMIT`, `MOTOR_CURRENT_LIMIT`, `CURRENT_BANDWIDTH`, `PWM_LIMIT`, `INTEGRATOR_DECAY`, `CURRENT_FILTER_GAIN`, `PWM_FREQ_HZ`, `ENCODER_TICKS_PER_REV`) and the R/L note. The open-loop model uses only `MOTOR_RESISTANCE`, `MOTOR_INDUCTANCE` and `MOTOR_KT`.

---

## Our BLDC as a DC motor

- **The DC-equivalent model is valid** for our motor: V = R·i + L·di/dt + K_b·ω, T = K_t·i, with line-to-line (phase-to-phase) R and L and K_t = K_b = 60/(2π·KV) = 0.0265 for 360 KV. This is the model of a BLDC driven six-step, where two phases conduct in series, so the circuit sees the line-to-line values. A voltage command on it is the PWM-averaged voltage across the conducting pair.
- **Line-to-line is phase-to-phase.** Both mean the value measured between two motor leads. The per-phase (line-to-neutral) values are half the resistance and inductance, and go with the FOC-frame torque constant K_t,FOC ≈ 8.27/KV = (√3/2)·K_t,DC.
- **What the DC model leaves out:** commutation and torque ripple, and the FOC dq-frame dynamics. Neither matters at a 1 ms step.
- **Our motor in a voltage model:** R = 0.24 Ω makes the current large. At stall, 2 V gives 8.3 A and the full 0.8 × 16 V = 12.8 V would give 53 A, where the real driver stops at 20 A Iq (17.3 A DC-equivalent). A voltage controller needs the current constraint |V − K_b·ω| ≤ R·17.3 A = 4.15 V.
- **Electrical time constant** L/R = 0.55 ms, shorter than a 1 ms step: an explicit 1 ms Euler current update oscillates (factor 1 − dt·R/L = −0.8 per step). An implicit solver handles it, or a controller model can take the current as instant, i = (V − K_b·ω)/R.
- **Open-loop current drive vs voltage drive:** under a constant current command a slipping wheel keeps its torque and runs away to near free speed in about 0.1 s (the wheel's rim-equivalent mass is only 18.5 g). Under voltage drive, the back-EMF cuts the current as the wheel spins up. On the real robot, Rustware's velocity loop plays that role.

### Motor constants from Rustware and the Simovr firmware (`robot_constants.py`, not committed yet)

**What:** the motor-driver values in `robot_constants.py` now come from the robot's own software and the motor-driver firmware, instead of placeholders.

| Constant | Before | Now | Source |
|---|---|---|---|
| `VEL_P_GAIN` | 0.1 | 0.196 A per rad/s | Rustware `VEL_P_GAIN` = 0.00015 A per (tick/s), × 8192/(2π) ticks per rad |
| `VEL_I_GAIN` | 1 | 19.6 A per rad | Rustware `VEL_I_GAIN` = 0.015 A per tick, × 8192/(2π) |
| `IQ_LIMIT` | 5 A | 20 A (FOC q-axis current) | Rustware `IQ_LIMIT` |
| `MOTOR_CURRENT_LIMIT` | — | 17.3 A | `IQ_LIMIT` × √3/2: the same limit as DC-equivalent current (see the current-input section below) |
| `ENCODER_TICKS_PER_REV` | — | 8192 | firmware `SENSOR_COMMON_RES_TICKS`; Rustware scales wheel speeds with the same 8192 |
| `PWM_FREQ_HZ` | — | 20000 | firmware `config.h`: rate of the current and velocity loops |
| `CURRENT_BANDWIDTH` | — | 2000 (used as rad/s, about 320 Hz) | firmware default; Rustware does not change it |
| `CURRENT_P_GAIN`, `CURRENT_I_GAIN` | — | bandwidth × L, bandwidth × R | firmware `controller_update_I_gains` |
| `PWM_LIMIT` | — | 0.8 | firmware `config.h` |
| `INTEGRATOR_DECAY` | — | 0.995 per period while limited | firmware `config.h` |
| `CURRENT_FILTER_GAIN` | — | 0.3 per period | firmware default `I_k` |

**Why:**
- Rustware's gains are in the firmware's units, encoder ticks. The firmware's velocity loop computes `Iq_setpoint = p_gain × (error in tick/s) + integrator`, with `integrator += i_gain × (error in tick/s) × dt`, so the units are A per (tick/s) and A per tick.
- The models use rad/s, so the gains are converted with 8192 ticks per revolution. The wheels are direct drive.
- The old values 0.1 and 1 were placeholders. The real proportional gain is about 2× larger and the integral gain about 20× larger.

**Motor R and L stay line-to-line** (0.24 Ω, 0.133 mH, from a SimpleFOC forum measurement of a 5010 360 KV motor). They go with K_t = K_b = 60/(2π·KV): a DC-equivalent motor model. The note in the file explains why they are not halved. Phase values belong to the FOC frame, where K_t ≈ 8.27/KV, and mixing phase R with this K_t would make the motor √2 stronger than measured.

**Effect on Henry's `robot_model_voltage` and `robot_model_current`:**
- Their velocity servo now uses the real gains and the 20 A limit.
- It runs once per 10 ms, which is unstable with these gains while a wheel slips: the proportional loop on the rim alone has a rate of about 313 /s. It needs a faster servo step before those notebooks are rerun.

### Driver as a continuous loop, inside the implicit solver

- **Driver (Step 5.1):** written as a continuous loop. The firmware runs it at 20 kHz, about 30× faster than its 2000 rad/s bandwidth.
  - The per-period filter gain κ and decay γ become rates: λ_f = −ln(1−κ)·f_pwm and λ_γ = −ln(γ)·f_pwm.
  - The anti-windup switch is smoothed over ε_V = 0.05 V (`anti_windup_width`), so the solver does not stall on it.

- **Continuous driver vs the exact discrete firmware loop** (`driver_continuous_check.py` in this folder, motor only): median current difference 0.0003 A, largest 0.25 A in the first 50 µs of a step. Radau also handles the voltage limit and the current reversal.

### The current-command input as it was built

**What:** the model's input is now a current command per motor, not a voltage. As on the robot, the Simovr motor driver (Tinymovr firmware) turns the current error into the voltage it applies, and Dana's electrical equation then sets the actual current. A voltage command is still accepted, applied straight to the motor terminals as in Dana's model, and Dana's tests use it.

**Why:** Henry asked for the more realistic input. The robot's drivers are current-controlled (FOC); the controller sends current, and the voltage follows. Decided with Henry:
- current command input;
- the firmware's default current-loop bandwidth;
- 20 A per motor.

**Equations (spec Step 5.1, in Dana's layout).** Each 1 ms step holds n = dt·f_pwm = 20 driver periods, with the wheel speeds held at step k, as Dana's back-EMF already does. Each period, in the firmware's order:

$$i_{set} = \text{clip}(i_{cmd}, \pm I_{max}), \qquad \hat i \leftarrow \hat i + \kappa (i - \hat i), \qquad e = i_{set} - \hat i$$

$$x_I \leftarrow x_I + T_{pwm} K_i e, \qquad V = \text{clip}(K_p e + x_I, \pm \eta V_{bus}), \qquad K_p = \omega_c L_m, \; K_i = \omega_c R$$

$$i \leftarrow i + \frac{T_{pwm}}{L_m}\left(V - R i - K_b \omega_k\right)$$

- While V is limited, the integrator is multiplied by γ each period.
- The torque is still K_t·I_k, Dana's convention.
- The applied voltage V_a and the battery current I_bus = Σ V·i / V_bus are reported as averages over the step.

**Code changes:**
- **Cell `ebf0a37d`:**
  - `OmniMotorState`: new states `I_est` and `x_I`, and new outputs `V_applied` and `I_bus`.
  - `__init__`: the gains `Kp_I` and `Ki_I`, and `n_pwm`, with a check that dt is a whole number of driver periods.
  - `compute_electrical_subsystem(state, I_cmd)`: rewritten as the driver loop above.
  - `step(state, I_cmd)`: carries the new states and outputs forward.
- **Parameter cell:** the driver settings, taken from `robot_constants.py`.

**Current limit: 17.3 A, not 20 A.** The Simovr limits its FOC q-axis current to 20 A (`IQ_LIMIT`). This model's current is DC-equivalent, so the clamp is `MOTOR_CURRENT_LIMIT` = (√3/2) × 20 = 17.3 A. Two independent checks give the same factor:
- **Equal torque:** K_t,FOC = (√3/2)·K_t,DC (8.27/KV vs 9.55/KV).
- **Equal copper heat:** R_LL·i² = 1.5·R_phase·I_q² = 0.75·R_LL·I_q².

Dana's tests command voltages, which bypass the driver, so this limit does not apply to them.

**Voltage input, for Dana's tests.** `step(state, V_cmd=V)` applies V straight to the motor terminals, limited to 0.8 × V_bus, with the driver bypassed and its states held; `step(state, I_cmd=I)` goes through the driver. Exactly one must be given.
- **Why:** Dana's tests command voltages, and a voltage is not a current: under voltage drive the current falls as the wheel speeds up (back-EMF), so a slipping wheel loses torque, while a current command keeps pushing. Comparing her voltage tests with current commands of the same numbers compares different tests (Henry).
- **First version, corrected:** the tests had been converted to current commands with the same numbers (2 A, 5 A, ±4 A). That was wrong; they are back on Dana's voltages, 2 V, 5 V and ±4 V. Her test cells are restored exactly, except that each call is `step(state, V_cmd=...)`; her test text is restored with the axis changes of change 1 kept.
- **Unchanged for the controller:** the current command is still the input the controller uses. With a current command, the results are identical to before (0.0 mm).
- **Checked:** with a voltage command the driver applies exactly the commanded voltage (±5.000 V) and its states stay at zero. The full notebook runs without errors in 324 s (`'proper'` design). On our motor (R = 0.24 Ω) Dana's voltages are strong: in the 45/45 2 V forward test the front wheels slip until about 0.4 s and then grip again as the back-EMF cuts their current; the 30/45 2 V and 5 V strafes spin (about −19 and −29 rad/s at 1.5 s).

**Text changes:**
- Spec sections 1 and 2: driver parameters, states and outputs, and the two inputs.
- Spec section 4: flow items 1 and 2.
- Spec Step 5.1: rewritten, with the voltage-command bypass.
- The flowchart's inputs and electrical box, the architecture note and the configuration note. The test descriptions are Dana's again (voltage).

**Checked:**

*Driver on its own:* see `motor_only2.py` in this folder. At the default bandwidth the current reaches 90% of a step in 0.8 ms with no overshoot.

*Link to Dana's voltage model:* Dana's voltage-driven model was run with her test voltages, and the currents it produced were fed as commands to this model.
- **2 V tests** (forward, strafe, rotation): the driver applies Dana's voltage to a median of 0.003–0.009 V. The robot paths agree to within 1.5 mm forward and 0.1 mm in rotation, and to within 72 mm in the spinning strafe.
- **5 V forward and 4 V multi-stage:** the currents match exactly and the robot motion agrees to within 4–25 mm, but the applied voltage sits about 0.6–1 V lower. The wheels slide there.
  - Dana's 1 ms forward-Euler current update overshoots on its first step: at 5 V it jumps to (Δt/L)·V = 37.6 A, where the real circuit reaches about 17 A.
  - The sliding wheels keep that extra spin, about 40 rad/s, because sliding friction does not depend on wheel speed. That speed difference is the 1 V of back-EMF.

*Full notebook:* runs without errors in 211 s.

**Left as it is (for Henry and Dana):**
- **The 0.8 × V_bus voltage limit** is the firmware's `PWM_LIMIT` (largest modulation, 80%). The firmware gives no reason. The usual one is leaving time in each PWM period for low-side current sensing and dead time. How it maps onto this DC-equivalent model depends on the firmware's modulation scaling, which was not confirmed.
- **The driver used the wheel speed from step k for the back-EMF**, Dana's convention, so it lagged the wheel by up to 1 ms. The implicit solver (change 5 of the change log) removed this lag.
- **The friction is unchanged:** it is still the LuGre setup of the change log.

---

## Removed code (from the notebook, before the voltage input was added)

### `OmniRobotSimulation.__init__`: driver gains and rates

```python
        # Simovr current loop: gains set from its bandwidth as the firmware does (controller_update_I_gains),
        # and its per-period filter and anti-windup decay written as rates (they act every 1/pwm_freq)
        self.Kp_I = self.p['I_bw'] * self.p['Lm']
        self.Ki_I = self.p['I_bw'] * self.p['R']
        self.V_max = self.p['pwm_limit'] * self.p['V_bus']
        self.filter_rate = -np.log(1.0 - self.p['current_filter_gain']) * self.p['pwm_freq']
        self.decay_rate = -np.log(self.p['integrator_decay']) * self.p['pwm_freq']
```

The state also carried `I_est` (driver current estimates, A) and `x_I` (current-loop integrators, V), and the outputs `V_applied` (step-average applied voltage) and `I_bus` (step-average battery current).

### `compute_electrical_subsystem` with the driver

```python
    def compute_electrical_subsystem(self, I, I_est, x_I, omega, I_cmd):
        """
        Calculates Step 5.1: the Simovr (Tinymovr firmware) current loop and the motor circuit, as rates.
        The driver filters its current measurement, runs a PI on the error to the clamped command and limits
        the voltage to pwm_limit * V_bus; while limited, its integrator decays (anti-windup). The switch is
        smoothed over anti_windup_width volts so the solver sees no jump.

        Returns:
            T_m: drive torques (Nm), I_dot (A/s), I_est_dot (A/s), x_I_dot (V/s), V: applied voltages (V)
        """
        Lm = self.p['Lm']
        R = self.p['R']
        Kb = self.p['Kb']
        Kt = self.p['Kt']

        # Driver: current estimate, error to the clamped command, PI, voltage limit
        I_est_dot = self.filter_rate * (I - I_est)
        e = np.clip(I_cmd, -self.p['Iq_limit'], self.p['Iq_limit']) - I_est
        V_unlimited = self.Kp_I * e + x_I
        V = np.clip(V_unlimited, -self.V_max, self.V_max)
        limited = 0.5 * (1.0 + np.tanh((np.abs(V_unlimited) - self.V_max) / self.p['anti_windup_width']))
        x_I_dot = self.Ki_I * e - limited * self.decay_rate * x_I

        # Motor circuit: di/dt = (V - R*i - Kb*omega) / Lm
        I_dot = (V - R * I - Kb * omega) / Lm

        # Gross electromagnetic drive torque
        T_m = Kt * I
        return T_m, I_dot, I_est_dot, x_I_dot, V
```

### `rates` (driver states and step integrals)

```python
    def rates(self, t, y, I_cmd, w_F, mu_s, mu_k):
        """Rates of change of every state, with the commands and disturbance draws held over the step."""
        r = self.p['r']
        Jw = self.p['Jw']
        I, I_est, x_I, omega, z = y[self._I], y[self._IE], y[self._XI], y[self._W], y[self._Z]
        v_global, pos, a_lag = y[self._V], y[self._POS], y[self._AL]
        v_local = self.global_to_local(v_global, pos[2])

        # Step 5.1: Simovr current loop and motor circuits
        T_m, I_dot, I_est_dot, x_I_dot, V = self.compute_electrical_subsystem(I, I_est, x_I, omega, I_cmd)

        # Step 5.2: Normal loads from the lagged accelerations
        F_n = self.compute_dynamic_normal_forces(a_lag, w_F)

        # Step 5.3: Rolling resistance and LuGre floor forces
        F_floor, z_dot, T_roll = self.compute_contact_friction_interface(omega, z, v_local, F_n, mu_s, mu_k)

        # Step 5.6: Wheel torque balance
        omega_dot = (T_m - T_roll - r * F_floor) / Jw

        # Step 5.4 & 5.5: Chassis accelerations, integrated in the global frame
        a_local, a_global = self.advance_chassis_kinetics(F_floor, pos[2])
        pos_dot = v_global

        # Load transfer follows the accelerations with a lag of one step
        a_lag_dot = (a_local[:2] - a_lag) / self.load_lag

        I_bus = np.sum(V * I) / self.p['V_bus']
        return np.concatenate((I_dot, I_est_dot, x_I_dot, omega_dot, z_dot, a_global, pos_dot, a_lag_dot,
                               F_floor, T_roll, V, [I_bus]))
```

### Parameter block

```python
    # Simovr Motor Driver: current loop (firmware values, see robot_constants.py)
    'I_bw': rc.CURRENT_BANDWIDTH,        # Current-loop bandwidth, firmware default (used as rad/s in the gains)
    'Iq_limit': rc.MOTOR_CURRENT_LIMIT,  # Current command limit per motor: the Simovr's 20 A Iq as DC-equivalent current, 17.3 A
    'V_bus': rc.MAX_MOTOR_VOLTAGE,       # Battery (bus) voltage (V)
    'pwm_limit': rc.PWM_LIMIT,           # Largest applied voltage as a fraction of the bus voltage
    'pwm_freq': rc.PWM_FREQ_HZ,          # Driver loop rate (Hz)
    'integrator_decay': rc.INTEGRATOR_DECAY,        # Integrator decay per period while the voltage is limited
    'current_filter_gain': rc.CURRENT_FILTER_GAIN,  # Current-estimate filter gain per period
    'anti_windup_width': 0.05,           # Width of the smoothed anti-windup switch (V); numerical, for the solver
```

### Spec Step 5.1 (the driver version)

#### Step 5.1: Discrete Electrical Motor Current System (Simovr Current Loop)
The input to each motor is a current command $i_{cmd,i,k}$. As on the robot, the Simovr motor driver (Tinymovr firmware) turns the error between the command and its measured current into the voltage it applies, and the motor's electrical equation then sets the actual current. The firmware runs this loop every PWM period $T_{pwm} = 1 / f_{pwm}$ (50 µs), far faster than the loop's bandwidth, so it is written here as a continuous loop.

The driver clamps each command to its current limit:
$$
i_{set,i} = \text{clip}\left(i_{cmd,i,k}, \ -I_{max}, \ I_{max}\right)
$$

It filters its current measurement (the firmware's $\kappa$ per period, as a rate):
$$
\frac{d\hat{i}_i}{dt} = \lambda_f \cdot \left(i_i - \hat{i}_i\right), \qquad \lambda_f = -\ln(1 - \kappa) \cdot f_{pwm}
$$

and runs a PI controller on the error, with gains set from the current-loop bandwidth $\omega_c$:
$$
K_p = \omega_c \cdot L_m, \qquad K_i = \omega_c \cdot R
$$
$$
e_i = i_{set,i} - \hat{i}_i
$$
$$
V_i = \text{clip}\left(K_p \cdot e_i + x_{I,i}, \ -\eta V_{bus}, \ \eta V_{bus}\right)
$$
$$
\frac{dx_{I,i}}{dt} = K_i \cdot e_i - s_i \cdot \lambda_\gamma \cdot x_{I,i}, \qquad \lambda_\gamma = -\ln(\gamma) \cdot f_{pwm}
$$

While the voltage is limited, the integrator decays (the firmware's $\gamma$ per period, as a rate). The switch $s_i$ goes from 0 to 1 as the unlimited voltage passes the limit, smoothed over a width $\varepsilon_V$ so the solver sees no jump:
$$
s_i = \frac{1}{2}\left(1 + \tanh\left(\frac{|K_p e_i + x_{I,i}| - \eta V_{bus}}{\varepsilon_V}\right)\right)
$$

The continuous electrical differential equation for a motor is $V(t) = R \cdot i(t) + L_m \frac{di(t)}{dt} + K_b \cdot \omega(t)$. Isolating the current derivative yields:
$$
\frac{di_i}{dt} = \frac{1}{L_m}\left(V_i - R \cdot i_i - K_b \cdot \omega_i\right)
$$

The instantaneous gross electromagnetic torque vector produced by the armatures maps directly from the current:

$$
\mathbf{T}_{m} = \begin{bmatrix} T_{m,1} \\ T_{m,2} \\ T_{m,3} \\ T_{m,4} \end{bmatrix} = K_t \begin{bmatrix} i_{1} \\ i_{2} \\ i_{3} \\ i_{4} \end{bmatrix}
$$

The applied voltage and the battery current are reported as averages over the step:
$$
V_{a,i,k} = \frac{1}{dt} \int_{t_k}^{t_{k+1}} V_i \, dt
$$
$$
I_{bus,k} = \frac{1}{dt} \int_{t_k}^{t_{k+1}} \frac{\sum_i V_i \cdot i_i}{V_{bus}} \, dt
$$

### Spec definitions that went with it

* $V_{bus}, \eta$: Battery (bus) voltage (V) and the driver's modulation limit (largest applied voltage as a fraction of $V_{bus}$)
* $I_{max}$: Driver current command limit per motor (A)
* $\omega_c, f_{pwm}, \kappa, \gamma, \varepsilon_V$: Driver current-loop bandwidth (rad/s), loop rate (Hz), current-estimate filter gain and integrator decay per period while the voltage is limited, and the width of the smoothed anti-windup switch (V)
* $\mathbf{I}_{cmd,k} = [i_{cmd,1,k}, i_{cmd,2,k}, i_{cmd,3,k}, i_{cmd,4,k}]^\top$: Motor current commands sent to each wheel's driver (Amperes).
* $\hat{\mathbf{I}}_k = [\hat{i}_{1,k}, \dots, \hat{i}_{4,k}]^\top$, $\mathbf{X}_{I,k} = [x_{I,1,k}, \dots, x_{I,4,k}]^\top$: Driver current estimates (Amperes) and current-loop integrators (Volts).

---

## Related tests

In this folder:

- `motor_only2.py`: one motor and the discrete firmware current loop at 50 µs, locked rotor and lifted wheel, at bandwidths 500, 2000 and 6000.
- `driver_continuous_check.py`: the continuous driver against the exact discrete firmware loop.
- `voltage_equivalent_tests.py`: current commands that reproduce a voltage drive, I = (V − K_b·ω)/R; they matched the true voltage command to the millimetre. (Written for the current-input model, which no longer exists.)

In `tests/`:
- `robot_fixed50.py`, `stribeck_sensitivity.py`, `strafe_grip_margin.py`, `strafe_wheel_forces_body.py`, `design_toggle_compare.py`: written for the current-input model (their commands are amps).
