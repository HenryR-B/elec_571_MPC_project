"""Single wheel + quarter-robot mass on a floor patch, driven by the Simovr/Tinymovr current loop.

Fixed-step, discrete-time (Dana's form x_{k+1} = x_k + h * rate_k) at the driver's own period h = 50 us:
  driver (firmware, 20 kHz):  i_f += 0.3 (i - i_f); e = clip(i_cmd, +-20) - i_f; x_I += e h Ki; V = Kp e + x_I;
                              if |V| > 0.8 V_bus: V = +-0.8 V_bus, x_I *= 0.995.   Kp = I_bw L, Ki = I_bw R, I_bw = 2000
  motor:      i += h (V - R i - Kb w) / L
  LuGre:      slip v = v_c - r w;  mu_d = mu_k + (mu_s - mu_k) exp(-alpha_d |v|);  a = (mu_s / mu_d) sqrt(v^2 + vc^2) / delta
              z_dot = v / delta - a z;  F = -(mu_s N z + sigma1 delta z_dot),  sigma1 = 2 zeta sqrt(mu_s N / delta * m_rim) * fade(v)
  wheel:      w += h (Kt i - r F) / J        (F acts on the robot along +d; the reaction on the rim is -r F)
  chassis:    v_c += h F / m_c
  bristles:   z = (z + h v_new / delta) / (1 + h a_new)    (implicit relaxation: the one exception to plain Euler)
Reference: the same equations with the motor/wheel/bristle/chassis update sub-stepped 50x (1 us) within each driver period.
"""
import sys
import numpy as np
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent.parent))
import robot_constants as rc

R, L, Kt, J, r = rc.MOTOR_RESISTANCE, rc.MOTOR_INDUCTANCE, rc.MOTOR_KT, rc.WHEEL_INERTIA, rc.WHEEL_RADIUS
Kb, V_bus = Kt, rc.MAX_MOTOR_VOLTAGE
mu_s, mu_k, alpha_d, vc = rc.MU_D_STAT, rc.MU_D_DYN, rc.ALPHA_D, 1e-4
N = 7.75                                  # front-wheel static load (N)
m_c = rc.ROBOT_MASS / 4                   # robot mass carried by this wheel along its drive direction (kg)
m_rim = J / r**2
I_bw, I_LIMIT, VMAX, T = 2000.0, 20.0, 0.8 * V_bus, 5e-5
Kp, Ki = I_bw * L, I_bw * R
# Coulomb: grip holds while the floor force needed, (Kt i / r) m_c / (m_c + m_rim), is within mu_s N
I_BREAK = mu_s * N * r * (m_c + m_rim) / (Kt * m_c)


def run(delta, zeta, fade, i_cmd_fn, t_end, sub=1):
    s1_0 = 2 * zeta * np.sqrt(mu_s * N / delta * m_rim)
    i = i_f = x_I = w = v_c = z = 0.0
    h = T / sub
    n = int(round(t_end / T))
    out = np.empty((n, 6))
    for k in range(n):
        t = k * T
        # driver, once per PWM period
        i_f += 0.3 * (i - i_f)
        e = min(max(i_cmd_fn(t), -I_LIMIT), I_LIMIT) - i_f
        x_I += e * T * Ki
        V = Kp * e + x_I
        if abs(V) > VMAX:
            V = VMAX if V > 0 else -VMAX
            x_I *= 0.995
        Fsum = 0.0
        for _ in range(sub):
            v = v_c - r * w
            md = mu_k + (mu_s - mu_k) * np.exp(-alpha_d * abs(v))
            a = (mu_s / md) * np.sqrt(v * v + vc * vc) / delta
            zdot = v / delta - a * z
            s1 = s1_0 * (np.exp(-(v / fade) ** 2) if fade < 1e8 else 1.0)
            F = -(mu_s * N * z + s1 * delta * zdot)
            i += h * (V - R * i - Kb * w) / L
            w += h * (Kt * i - r * F) / J
            v_c += h * F / m_c
            v_new = v_c - r * w
            md_new = mu_k + (mu_s - mu_k) * np.exp(-alpha_d * abs(v_new))
            a_new = (mu_s / md_new) * np.sqrt(v_new * v_new + vc * vc) / delta
            z = (z + h * v_new / delta) / (1 + h * a_new)
            Fsum += F
            if not np.isfinite(F) or abs(F) > 1e4:
                return out[:k + 1], False
        out[k] = (t + T, i, w, v_c, Fsum / sub, v_c - r * w)
    return out, True


def summarize(delta, zeta, fade, sub=1):
    res = {}
    # static: steps that Coulomb says must grip
    grip_ok, acc_err = True, 0.0
    for frac in (0.25, 0.5, 0.75, 0.93):
        out, ok = run(delta, zeta, fade, lambda t, c=frac * I_BREAK: c, 0.15, sub)
        if not ok:
            return None
        slip_end = abs(out[-1, 5])
        a_meas = (out[-1, 3] - out[len(out) // 2, 3]) / (out[-1, 0] - out[len(out) // 2, 0])
        a_coul = Kt * out[-1, 1] / r / (m_c + m_rim)
        grip_ok &= slip_end < 5e-3
        acc_err = max(acc_err, abs(a_meas / a_coul - 1))
    res["grip"], res["acc_err"] = grip_ok, acc_err
    # breakaway: slow ramp of the command; force peak in units of mu_s N
    out, ok = run(delta, zeta, fade, lambda t: I_BREAK * 1.3 * t / 0.5, 0.5, sub)
    if not ok:
        return None
    res["ramp_peak"] = np.abs(out[:, 4]).max() / (mu_s * N)
    # just above breakaway: a step to 107% must slide
    out, ok = run(delta, zeta, fade, lambda t: 1.07 * I_BREAK, 0.15, sub)
    res["slides_107"] = ok and abs(out[-1, 5]) > 5e-3
    res["step_peak_107"] = np.abs(out[:, 4]).max() / (mu_s * N) if ok else np.nan
    # sliding: 15 A for 0.15 s, then -15 A for 0.15 s (slip reverses through zero)
    out, ok = run(delta, zeta, fade, lambda t: 15.0 if t < 0.15 else -15.0, 0.3, sub)
    if not ok:
        return None
    mid = (out[:, 0] > 0.10) & (out[:, 0] < 0.15)
    v_mid = out[mid, 5]
    F_coul = -np.sign(v_mid) * (mu_k + (mu_s - mu_k) * np.exp(-alpha_d * np.abs(v_mid))) * N
    res["slide_err"] = np.abs(out[mid, 4] - F_coul).max() / (mu_s * N)
    res["slide_peak"] = np.abs(out[:, 4]).max() / (mu_s * N)
    res["I_peak"] = np.abs(out[:, 1]).max()
    res["traj"] = out
    return res


if __name__ == "__main__":
    print(f"Coulomb breakaway command: {I_BREAK:.2f} A (mu_s N = {mu_s * N:.2f} N, quarter mass {m_c:.3f} kg, rim mass {1e3 * m_rim:.1f} g)\n")
    for delta in (2e-5, 3e-5, 5e-5, 1e-4):
        for zeta in (1.0, 2.0, 3.0):
            for fade in (5e-3, 5e-2, 1e9):
                r50 = summarize(delta, zeta, fade)
                fs = "none" if fade > 1e8 else f"{1e3 * fade:g} mm/s"
                if r50 is None:
                    print(f"delta {1e6 * delta:4.0f} um zeta {zeta:.0f} fade {fs:9}: UNSTABLE at 50 us"); continue
                good = (r50["grip"] and r50["acc_err"] < 0.01 and 0.95 <= r50["ramp_peak"] <= 1.02 and r50["slides_107"]
                        and r50["step_peak_107"] <= 1.05 and r50["slide_err"] < 0.02 and r50["slide_peak"] <= 1.05)
                print(f"delta {1e6 * delta:4.0f} um zeta {zeta:.0f} fade {fs:9}: grips below breakaway {str(r50['grip']):5} "
                      f"(accel error {100 * r50['acc_err']:.2f}%), ramp breakaway {r50['ramp_peak']:.3f}, slides at 107% {str(r50['slides_107']):5} "
                      f"(peak {r50['step_peak_107']:.3f}), sliding error {100 * r50['slide_err']:.2f}%, peak force {r50['slide_peak']:.2f} mu_s N, "
                      f"peak current {r50['I_peak']:.1f} A{'  <-- meets all' if good else ''}", flush=True)
