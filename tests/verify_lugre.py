"""Independent checks of the LuGre drive friction in a model notebook (DRIVE_SLIDING_LAW = "lugre").

Checks the code against the LuGre equations in physical units, steady sliding on the Stribeck
curve, the static limit, the held force at zero slip, a lifted wheel, dissipation over slip cycles,
and, inside the notebook's plants, which state slots feed the force and receive the rates.
Usage, from the repository root: python tests/verify_lugre.py robot_model_current.ipynb
(or robot_model_voltage.ipynb). Exits with status 1 if a check fails.
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # robot_constants

import numpy as np
from scipy.integrate import solve_ivp

d = {c.get('id'): ''.join(c['source']) for c in json.load(open(sys.argv[1], encoding='utf-8'))['cells']}
setup = ['549569af', '766627f5', 'f26c71d2', 'dc169fed', '72c90d84', 'b7053b69', '9676641c']
if 'vplant02' in d:
    setup.append('vplant02')
G = {}
exec('\n\n'.join(d[k] for k in setup) + '\n\nclass ConvexMPCController: pass\n\n' + d['plantv02'], G)
rc, mu_d, lugre_contact = G['rc'], G['mu_d'], G['lugre_contact']
delta, creep = G['LUGRE_PRESLIDING_DEFLECTION'], G['LUGRE_CREEP_SLIP']
print(f"law {G['DRIVE_SLIDING_LAW']}, deflection {1e6 * delta:g} um, creep {1e3 * creep:g} mm/s, damping slip "
      f"{1e3 * G['LUGRE_DAMPING_SLIP']:g} mm/s, sigma2 {G['LUGRE_VISCOUS_COEFF']}")
N, M_RIM = 4.0, rc.WHEEL_INERTIA / rc.WHEEL_RADIUS**2
LIMIT = rc.MU_D_STAT * N
ok = True


def check(name, condition, detail):
    global ok
    ok &= bool(condition)
    print(f"  {'PASS' if condition else 'FAIL'}  {name}: {detail}")


# 1. Physical-units form: z = delta s, sigma0 = mu_s N / delta, g = mu_d N.
v, s = np.array([0.37, -0.02, 1e-3]), np.array([0.4, -0.8, 0.95])
force, s_rate = lugre_contact(v, s, np.full(3, LIMIT), M_RIM)
z, sigma0, g = delta * s, LIMIT / delta, mu_d(v) * N
z_rate = v - sigma0 * np.sqrt(v**2 + creep**2) * z / g
sigma1 = 2 * G['LUGRE_DAMPING_RATIO'] * np.sqrt(sigma0 * M_RIM) * np.exp(-(v / G['LUGRE_DAMPING_SLIP'])**2)
check("matches dz/dt = v - sigma0 |v| z / g(v)", np.allclose(delta * s_rate, z_rate, rtol=1e-12, atol=1e-15), f"max diff {np.max(np.abs(delta * s_rate - z_rate)):.1e} m/s")
check("matches F = -(sigma0 z + sigma1 dz/dt + sigma2 v)", np.allclose(force, -(sigma0 * z + sigma1 * z_rate + G['LUGRE_VISCOUS_COEFF'] * v), rtol=1e-12),
      f"max diff {np.max(np.abs(force + sigma0 * z + sigma1 * z_rate)):.1e} N")

# 2. Steady sliding: ds/dt = 0 gives the Stribeck curve.
speeds = np.array([-3.0, -0.5, -0.05, -0.01, 0.01, 0.05, 0.5, 3.0])
s_ss = speeds / ((rc.MU_D_STAT / mu_d(speeds)) * np.sqrt(speeds**2 + creep**2))
force_ss, rate_ss = lugre_contact(speeds, s_ss, np.full(speeds.size, LIMIT), M_RIM)
curve = -np.sign(speeds) * mu_d(speeds) * N
check("steady sliding is on the Stribeck curve", np.all(np.abs(rate_ss) < 1e-9) and np.max(np.abs(force_ss - curve)) / LIMIT < 1e-3,
      f"largest difference {100 * np.max(np.abs(force_ss - curve)) / LIMIT:.4f}% of mu_s N (creep term), opposes the slip: {np.all(force_ss * speeds < 0)}")

# 3. Static limit: pushing the bristles one way never takes the load past mu_d/mu_s (<= 1).
for v_push in (1e-6, 1e-4, 1e-2, 1.0):
    sol = solve_ivp(lambda t, y: [lugre_contact(np.array([v_push]), y, np.array([LIMIT]), M_RIM)[1][0]],
                    (0, 50 * delta / v_push), [0.0], method="Radau", rtol=1e-9, atol=1e-12)
    peak = sol.y[0].max()
    check(f"static limit at {v_push * 1e3:g} mm/s of slip", peak <= mu_d(v_push) / rc.MU_D_STAT + 1e-9,
          f"bristle load reaches {peak:.6f} (limit mu_d/mu_s = {mu_d(v_push) / rc.MU_D_STAT:.6f})")

# 4. At zero slip the force is the held bristle load, not zero.
force0, _ = lugre_contact(np.array([0.0, 0.0]), np.array([0.9, -0.5]), np.full(2, LIMIT), M_RIM)
check("force at zero slip is the held load", np.allclose(np.abs(force0), np.array([0.9, 0.5]) * LIMIT, rtol=0.05),
      f"{np.round(force0 / LIMIT, 3)} of mu_s N for loads [0.9, -0.5] (damping part from creep relaxation included)")

# 5. A lifted wheel carries nothing.
force_lift, _ = lugre_contact(np.array([0.3, 0.0]), np.array([0.5, 0.9]), np.zeros(2), M_RIM)
check("lifted wheel (N = 0) carries no force", np.all(force_lift == 0.0), f"{force_lift}")

# 6. Dissipation: over a closed slip cycle the friction does net negative work.
def cycle(t, y):
    slip = 0.2 * np.sin(2 * np.pi * 5 * t)
    f, rate = lugre_contact(np.array([slip]), np.array([y[0]]), np.array([LIMIT]), M_RIM)
    return [rate[0], f[0] * slip]
sol = solve_ivp(cycle, (0, 1.0), [0.0, 0.0], method="Radau", rtol=1e-8, atol=1e-12)
check("friction dissipates over slip cycles", sol.y[1, -1] < 0, f"net work {sol.y[1, -1]:.3f} J over 5 cycles")

# 7. Inside the plant: the bristle slots feed the force and receive the rates (this notebook's plants).
classes = [G['WheelSlipRobotSimulator']] + ([G['VoltageDrivenSlipSimulator']] if 'VoltageDrivenSlipSimulator' in G else [])
for cls in classes:
    plant = cls(dt=1e-3, load_model="static")
    state = plant.state.copy()
    b = plant.bristle_start
    state[3] = 0.8                                    # chassis moving: the wheels slip
    state[b:b + 4] = [0.2, -0.4, 0.6, -0.8]
    G['DRIVE_SLIDING_LAW'] = "lugre"
    derivative, force_used, _, _ = plant.physical_dynamics(state, np.zeros(4), np.zeros(3))
    diag = plant._force_selection_diagnostics
    expected_force, expected_rate = lugre_contact(diag["wheel_slip_d"], state[b:b + 4], diag["traction_limits"], M_RIM)
    check(f"{cls.__name__}: force and rates use slots {b}-{b + 3}",
          np.allclose(force_used, expected_force) and np.allclose(derivative[b:b + 4], expected_rate),
          f"state size {state.size}, blend slots {plant.blend_start}-{plant.blend_start + 3} rates {derivative[plant.blend_start:plant.blend_start + 4]}")
    G['DRIVE_SLIDING_LAW'] = "switch"
    derivative = plant.physical_dynamics(state, np.zeros(4), np.zeros(3))[0]
    check(f"{cls.__name__}: bristles frozen under the switch law", np.all(derivative[b:b + 4] == 0.0), f"{derivative[b:b + 4]}")
    G['DRIVE_SLIDING_LAW'] = "lugre"

print("ALL PASS" if ok else "SOME CHECKS FAILED")
sys.exit(0 if ok else 1)
