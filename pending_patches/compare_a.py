"""Run the 4.14 scenario slice through two notebook versions; compare every logged value and time it.

Usage: python compare_a.py OLD_NOTEBOOK NEW_NOTEBOOK [SECONDS]
"""
import json
import sys
import time

import numpy as np

KEY_T = [0.0, 1.0, 1.1, 1.3, 1.4, 2.0, 2.1, 2.3, 2.4, 3.5]
KEY_FORWARD = [0.0, 1.5, 1.5, 1.5, 1.5, 1.5, 3.0, 3.0, 1.5, 1.5]
KEY_LEFT = [0.0, 0.0, -1.5, -1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
duration = float(sys.argv[3]) if len(sys.argv) > 3 else 1.6


def load(path):
    d = {c.get('id'): ''.join(c['source']) for c in json.load(open(path, encoding='utf-8'))['cells']}
    setup = ['549569af', '766627f5', 'f26c71d2', 'dc169fed', '72c90d84', 'b7053b69', '9676641c']
    if 'vplant02' in d:
        setup.append('vplant02')
    G = {}
    exec('\n\n'.join(d[k] for k in setup) + '\n\nclass ConvexMPCController: pass\n\n' + d['plantv02'], G)
    return G


def run(G):
    with G['cad_cog']():
        sim = G['OpenLoopPlant'](dt=1e-3, load_model="static")
    rows = []
    t0 = time.perf_counter()
    for t in np.arange(0.0, duration, 1e-3):
        command = G['body_command'](forward=float(np.interp(t, KEY_T, KEY_FORWARD)), left=float(np.interp(t, KEY_T, KEY_LEFT)))
        out = sim.step(G['inverse_kinematics'](command))
        rows.append(np.r_[sim.state, out["ground_forces"], out["slip_speeds"], out["rolling"], out["force_needed"]])
    return np.array(rows), time.perf_counter() - t0, sim.state.size


old_G, new_G = load(sys.argv[1]), load(sys.argv[2])
old, old_time, n = run(old_G)
new, new_time, _ = run(new_G)
print(f"{len(old)} steps: old {old_time:.1f} s, new {new_time:.1f} s, speed-up {old_time / new_time:.2f}x")
print(f"identical bit for bit: {np.array_equal(old, new)}; largest difference: states {np.max(np.abs(old[:, :n] - new[:, :n])):.3e}, "
      f"forces {np.max(np.abs(old[:, n:n + 4] - new[:, n:n + 4])):.3e} N, grip/slide modes differ in {int(np.sum(old[:, n + 8:n + 12] != new[:, n + 8:n + 12]))} samples")

# Step cache: a repeated integration of the same step returns the same state as a fresh one.
plant = new_G['WheelSlipRobotSimulator'](dt=1e-3)
plant.update_contact_disturbances()
state = plant.state.copy(); state[3] = 1.0; request = np.array([3.0, -3.0, 3.0, -3.0])
first = plant.discrete_map(state, request, np.zeros(3))
cached = plant.discrete_map(state, request, np.zeros(3))
plant._step_cache = {}
fresh = plant.discrete_map(state, request, np.zeros(3))
# With PLANT_REUSE_JACOBIAN the fresh integration starts from the stored Jacobian, so it can differ
# within the tolerance; the reused result must equal the first one exactly.
print(f"step cache: reused result equals the first integration: {np.array_equal(cached, first)}; "
      f"a fresh integration differs by at most {np.max(np.abs(fresh - first)):.1e}")
