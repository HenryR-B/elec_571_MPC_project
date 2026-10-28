"""One motor + Simovr/Tinymovr current loop, run exactly as the firmware does: a discrete loop every 50 us
(20 kHz PWM), with the motor solved exactly over each period while the voltage is held. No floor, no friction.

Each period (firmware CLControlStep order):
  i_f += 0.3 (i - i_f)                      current estimate (I_k = 0.3)
  e = clip(i_cmd, +-I_limit) - i_f
  x_I += e * T * Ki                          Ki = I_bw R
  V = Kp e + x_I                             Kp = I_bw L
  if |V| > 0.8 V_bus: V = +-0.8 V_bus, x_I *= 0.995
Motor over the period (exact, linear):  L i' = V - R i - Kb w,  J w' = Kt i  (lifted)  or  w = 0 (locked)
"""
import sys
import numpy as np
from scipy.linalg import expm
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[2]))   # repository root
import robot_constants as rc

R, L, Kt = rc.MOTOR_RESISTANCE, rc.MOTOR_INDUCTANCE, rc.MOTOR_KT
Kb, J, V_bus = Kt, rc.WHEEL_INERTIA, rc.MAX_MOTOR_VOLTAGE
I_LIMIT, PWM_LIMIT, T = 20.0, 0.8, 1.0 / 20000


def discretize(locked):
    """Exact zero-order-hold discretization of x = [i, w] with input V over one period T."""
    A = np.array([[-R / L, 0.0 if locked else -Kb / L], [0.0 if locked else Kt / J, 0.0]])
    B = np.array([[1.0 / L], [0.0]])
    M = expm(np.block([[A, B], [np.zeros((1, 3))]]) * T)
    return M[:2, :2], M[:2, 2]


def simulate(I_bw, i_cmd_fn, locked, t_end):
    Ad, Bd = discretize(locked)
    Kp, Ki, Vmax = I_bw * L, I_bw * R, PWM_LIMIT * V_bus
    n = int(round(t_end / T))
    x, i_f, x_I = np.zeros(2), 0.0, 0.0
    out = np.zeros((n, 5))
    for k in range(n):
        t = k * T
        i_f += 0.3 * (x[0] - i_f)
        e = np.clip(i_cmd_fn(t), -I_LIMIT, I_LIMIT) - i_f
        x_I += e * T * Ki
        V = Kp * e + x_I
        if abs(V) > Vmax:
            V = np.sign(V) * Vmax
            x_I *= 0.995
        x = Ad @ x + Bd * V
        out[k] = (t + T, x[0], V, x[1], V * x[0] / V_bus)
    return out.T


def rise(t, i, target):
    k = np.argmax(np.abs(i) >= 0.9 * abs(target))
    return t[k] if abs(i[k]) >= 0.9 * abs(target) else np.nan


if __name__ == "__main__":
    fig, ax = plt.subplots(3, 2, figsize=(13, 10))
    print(f"R {R} ohm, L {1e3 * L:.3f} mH (open-loop L/R {1e3 * L / R:.2f} ms), Kt = Kb {Kt:.4f}, J {J:.2e} kg m^2, "
          f"voltage limit {PWM_LIMIT * V_bus:.2f} V, no-load speed at the limit {PWM_LIMIT * V_bus / Kb:.0f} rad/s\n")
    for I_bw, style in ((500, ":"), (2000, "-"), (6000, "--")):
        t, i, V, w, Ib = simulate(I_bw, lambda t: 5.0 if t < 0.01 else 20.0, True, 0.02)
        k5 = np.searchsorted(t, 0.0099)
        print(f"I_bw {I_bw:5d} | locked rotor: 90% rise to 5 A in {1e3 * rise(t, i, 5):.2f} ms (1/I_bw = {1e3 / I_bw:.2f} ms), "
              f"overshoot {max(0.0, i[:k5].max() - 5):.2f} A; holding 20 A: {i[-1]:.2f} A at {V[-1]:.2f} V, battery {Ib[-1]:.1f} A")
        ax[0, 0].plot(t * 1e3, i, style, label=f"I_bw {I_bw}"); ax[1, 0].plot(t * 1e3, V, style, label=f"I_bw {I_bw}")
        ax[2, 0].plot(t * 1e3, Ib, style, label=f"I_bw {I_bw}")

        t, i, V, w, Ib = simulate(I_bw, lambda t: 5.0 if t < 0.3 else -5.0, False, 0.6)
        k = np.searchsorted(t, 0.3)
        top = (PWM_LIMIT * V_bus - R * 5) / Kb
        after = i[k:]
        print(f"{'':10} | lifted wheel, +5 A then -5 A at 0.3 s: speed at 0.3 s {w[k]:.0f} rad/s (5 A until the voltage limits "
              f"it at {top:.0f}), current just before reversal {i[k - 1]:.2f} A; after reversal peak {after.min():.1f} A, "
              f"settles {after[-1]:.2f} A; peak battery current {Ib.max():.1f} A (regen min {Ib.min():.1f} A)")
        ax[0, 1].plot(t, i, style, label=f"I_bw {I_bw}"); ax[1, 1].plot(t, V, style, label=f"I_bw {I_bw}")
        ax[2, 1].plot(t, w, style, label=f"I_bw {I_bw}")

    ax[0, 0].set(title="Locked rotor: current (A); command 5 A, then 20 A at 10 ms", xlabel="ms")
    ax[1, 0].set(title="Locked rotor: applied voltage (V)", xlabel="ms")
    ax[2, 0].set(title="Locked rotor: battery current (A)", xlabel="ms")
    ax[0, 1].set(title="Lifted wheel: current (A); command +5 A, then -5 A at 0.3 s", xlabel="s")
    ax[1, 1].set(title="Lifted wheel: applied voltage (V); limit +-13.44 V", xlabel="s")
    ax[2, 1].set(title="Lifted wheel: wheel speed (rad/s)", xlabel="s")
    for a in ax.flat:
        a.grid(True, alpha=0.3); a.legend(fontsize=8)
    plt.tight_layout(); plt.savefig(sys.argv[1], dpi=100)
