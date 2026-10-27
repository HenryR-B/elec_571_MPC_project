"""Continuous-time Simovr current loop (for an implicit solver) vs the exact discrete firmware loop, motor only.

Continuous form, integrated with Radau:
  i_est' = f_filt (i - i_est),                 f_filt  = -ln(1 - CURRENT_FILTER_GAIN) * PWM_FREQ_HZ
  e = clip(i_cmd, +-limit) - i_est,  u = Kp e + x_I,  V = clip(u, +-PWM_LIMIT V_bus)
  x_I' = Ki e - s(u) f_decay x_I,              f_decay = -ln(INTEGRATOR_DECAY) * PWM_FREQ_HZ,
                                               s(u) = smooth 0..1 switch around |u| = V_max (width EPS_V)
  L i' = V - R i - Kb w,   J w' = Kt i (lifted wheel) or w = 0 (locked rotor)
Discrete form: the firmware loop every 50 us, motor solved exactly between ticks (as in motor_only2.py).
"""
import sys
from pathlib import Path
import numpy as np
from scipy.integrate import solve_ivp
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import robot_constants as rc
import motor_only2 as disc

R, L, Kt, J = rc.MOTOR_RESISTANCE, rc.MOTOR_INDUCTANCE, rc.MOTOR_KT, rc.WHEEL_INERTIA
Kb, V_bus = Kt, rc.MAX_MOTOR_VOLTAGE
Kp, Ki = rc.CURRENT_P_GAIN, rc.CURRENT_I_GAIN
V_MAX = rc.PWM_LIMIT * V_bus
F_FILT = -np.log(1 - rc.CURRENT_FILTER_GAIN) * rc.PWM_FREQ_HZ
F_DECAY = -np.log(rc.INTEGRATOR_DECAY) * rc.PWM_FREQ_HZ
EPS_V = 0.05
LIMIT = 20.0      # same clamp as motor_only2.py, for a like-for-like comparison


def continuous(i_cmd_fn, locked, t_end, n_out):
    def f(t, y):
        i, i_est, x_I, w = y
        e = np.clip(i_cmd_fn(t), -LIMIT, LIMIT) - i_est
        u = Kp * e + x_I
        V = np.clip(u, -V_MAX, V_MAX)
        s = 0.5 * (1 + np.tanh((abs(u) - V_MAX) / EPS_V))
        return [(V - R * i - Kb * w) / L, F_FILT * (i - i_est), Ki * e - s * F_DECAY * x_I, 0.0 if locked else Kt * i / J]
    t_eval = np.linspace(t_end / n_out, t_end, n_out)
    sol = solve_ivp(f, (0, t_end), [0, 0, 0, 0], method="Radau", rtol=1e-6, atol=1e-8, t_eval=t_eval)
    return sol.t, sol.y[0], sol.y[3], sol


print("Discrete firmware loop (exact) vs continuous loop in Radau, I_bw = 2000\n")
for name, fn, locked, T in (("locked rotor, 5 A then 20 A", lambda t: 5.0 if t < 0.01 else 20.0, True, 0.02),
                            ("lifted wheel, +5 A then -5 A", lambda t: 5.0 if t < 0.3 else -5.0, False, 0.6)):
    td, i_d, V_d, w_d, _ = disc.simulate(rc.CURRENT_BANDWIDTH, fn, locked, T)
    tc, i_c, w_c, sol = continuous(fn, locked, T, len(td))
    print(f"{name:30}: largest current difference {np.abs(i_c - i_d).max():.3f} A "
          f"(median {np.median(np.abs(i_c - i_d)):.4f} A), largest wheel-speed difference {np.abs(w_c - w_d).max():.2f} rad/s, "
          f"solver {sol.nfev} evaluations, status {sol.status}")
