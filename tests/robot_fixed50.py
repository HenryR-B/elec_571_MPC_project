"""Whole robot, one fixed step h = 50 us (the driver's period), Dana's form x_{k+1} = x_k + h * rate_k for every state
except the bristle relaxation (one-line implicit update). Current-commanded (Simovr/Tinymovr driver, I_bw 2000).
Dana's chassis and pose updates, the swapped load split, load transfer lagged by 1 ms, LuGre 30 um / zeta 3 / fade 50 mm/s,
smoothed rolling resistance. Noise off. Times each scenario against real time."""
import sys, time
import numpy as np
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent.parent))
import robot_constants as rc

h = 5e-5
Mr, Jzz, Rb, hcg, r, Jw = rc.ROBOT_MASS, rc.ROBOT_INERTIA, rc.WHEEL_ROBOT_DIST, rc.COG_Z_MM * 1e-3, rc.WHEEL_RADIUS, rc.WHEEL_INERTIA
R, L, Kt = rc.MOTOR_RESISTANCE, rc.MOTOR_INDUCTANCE, rc.MOTOR_KT
Kb, V_bus, g = Kt, rc.MAX_MOTOR_VOLTAGE, 9.81
mu_s, mu_k, alpha_d, vc, Crr, eps = rc.MU_D_STAT, rc.MU_D_DYN, rc.ALPHA_D, 1e-4, rc.WHEEL_ROLLING_RESISTANCE_COEFF, rc.WHEEL_LOSS_SMOOTHING_SPEED
delta, zeta, fade, tau_load = 3e-5, 3.0, 5e-2, 1e-3
I_bw, I_LIMIT, VMAX = 2000.0, 20.0, 0.8 * V_bus
Kp, Ki = I_bw * L, I_bw * R
m_rim = Jw / r**2

af, ab = np.radians(rc.WHEEL_ALPHA_DEG[0]), np.radians(360.0 - rc.WHEEL_ALPHA_DEG[3])
H = np.array([[-np.sin(af), np.cos(af), Rb], [-np.sin(af), -np.cos(af), Rb], [np.sin(ab), -np.cos(ab), Rb], [np.sin(ab), np.cos(ab), Rb]])
Minv = np.diag([1 / Mr, 1 / Mr, 1 / Jzz])
x_f, x_b, y_f, y_b = Rb * np.cos(af), Rb * np.cos(ab), Rb * np.sin(af), Rb * np.sin(ab)
Lw, SX2 = y_f + y_b, x_f**2 + x_b**2


def loads(ax, ay):
    front, rear = Mr * g * y_b / (2 * Lw), Mr * g * y_f / (2 * Lw)
    dy = Mr * ay * hcg / (2 * Lw)
    lat = Mr * ax * hcg / Lw
    dxf, dxr = x_f / SX2 * lat * y_b, x_b / SX2 * lat * y_f
    return np.maximum(0.0, np.array([front - dy - dxf, front - dy + dxf, rear + dy + dxr, rear + dy - dxr]))


def simulate(cmd_fn, T):
    i = np.zeros(4); i_f = np.zeros(4); x_I = np.zeros(4); w = np.zeros(4); z = np.zeros(4)
    v = np.zeros(3); pos = np.zeros(3); a_lag = np.zeros(2)
    n = int(round(T / h)); log = []
    I_pk = Ibus_pk = 0.0; slip_time = np.zeros(4)
    for k in range(n):
        t = k * h
        # driver (firmware order)
        i_f += 0.3 * (i - i_f)
        e = np.clip(cmd_fn(t), -I_LIMIT, I_LIMIT) - i_f
        x_I += e * h * Ki
        V = Kp * e + x_I
        lim = np.abs(V) > VMAX
        V = np.clip(V, -VMAX, VMAX)
        x_I[lim] *= 0.995
        # floor
        Fn = loads(a_lag[0], a_lag[1])
        slip = H @ v - r * w
        md = mu_k + (mu_s - mu_k) * np.exp(-alpha_d * np.abs(slip))
        a_rel = (mu_s / md) * np.sqrt(slip**2 + vc**2) / delta
        zdot = slip / delta - a_rel * z
        s1 = 2 * zeta * np.sqrt(mu_s * Fn / delta * m_rim) * np.exp(-(slip / fade) ** 2)
        F = -(mu_s * Fn * z + s1 * delta * zdot)
        T_roll = Crr * Fn * r * np.tanh(w / eps)
        # Euler updates
        acc = Minv @ (H.T @ F)
        i = i + h * (V - R * i - Kb * w) / L
        w = w + h * (Kt * i - T_roll - r * F) / Jw
        v = v + h * acc
        c, s = np.cos(pos[2]), np.sin(pos[2])
        pos = pos + h * np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])
        a_lag = a_lag + h * (acc[:2] - a_lag) / tau_load
        slip_new = H @ v - r * w
        md_new = mu_k + (mu_s - mu_k) * np.exp(-alpha_d * np.abs(slip_new))
        z = (z + h * slip_new / delta) / (1 + h * (mu_s / md_new) * np.sqrt(slip_new**2 + vc**2) / delta)
        I_pk = max(I_pk, np.abs(i).max()); Ibus_pk = max(Ibus_pk, float(np.sum(V * i)) / V_bus)
        slip_time += np.abs(slip_new) > 5e-3
        if k % 200 == 199:
            log.append((t + h, *v, *pos))
    return np.array(log), I_pk, Ibus_pk, slip_time * h / T


FWD, STR, ROT = np.array([1, -1, -1, 1.0]), np.array([-1, -1, 1, 1.0]), np.ones(4)
SCEN = [("2 A forward", lambda t: 2.0 * FWD), ("5 A forward", lambda t: 5.0 * FWD), ("5 A strafe", lambda t: 5.0 * STR),
        ("5 A rotation", lambda t: 5.0 * ROT), ("10 A fwd, reverse at 1 s", lambda t: 10.0 * FWD * (1 if t < 1 else -1))]
for name, fn in SCEN:
    t0 = time.time()
    log, I_pk, Ibus_pk, slip_frac = simulate(fn, 1.5)
    wall = time.time() - t0
    vend = log[-1, 1:4]
    print(f"{name:26}: {wall:5.1f} s wall for 1.5 s ({wall / 1.5:.1f}x real time) | end speed {np.hypot(vend[0], vend[1]):.2f} m/s, "
          f"yaw rate {vend[2]:+.2f} rad/s, heading {np.degrees(log[-1, 6]):+.0f} deg, end ({log[-1, 4]:+.2f}, {log[-1, 5]:+.2f}) m | "
          f"peak motor current {I_pk:.1f} A, peak battery current {Ibus_pk:.1f} A, time slipping per wheel {np.round(100 * slip_frac)}%", flush=True)
