"""Solver settings vs a tight reference on the 4.14 at-speed scenario: time, effort, accuracy.

Usage: python tol_study.py NOTEBOOK [SECONDS]
"""
import json
import sys
import time

import numpy as np

KEY_T = [0.0, 1.0, 1.1, 1.3, 1.4, 2.0, 2.1, 2.3, 2.4, 3.5]
KEY_FORWARD = [0.0, 1.5, 1.5, 1.5, 1.5, 1.5, 3.0, 3.0, 1.5, 1.5]
KEY_LEFT = [0.0, 0.0, -1.5, -1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 3.5

d = {c.get('id'): ''.join(c['source']) for c in json.load(open(sys.argv[1], encoding='utf-8'))['cells']}
setup = ['549569af', '766627f5', 'f26c71d2', 'dc169fed', '72c90d84', 'b7053b69', '9676641c']
if 'vplant02' in d:
    setup.append('vplant02')
G = {}
exec('\n\n'.join(d[k] for k in setup) + '\n\nclass ConvexMPCController: pass\n\n' + d['plantv02'], G)
plant_class = G['WheelSlipRobotSimulator']
original = plant_class.physical_dynamics
evaluations = [0]


def counted(self, *args, **kwargs):
    evaluations[0] += 1
    return original(self, *args, **kwargs)


plant_class.physical_dynamics = counted


def run(rtol, atol, reuse):
    G['PLANT_REUSE_JACOBIAN'] = reuse
    with G['cad_cog']():
        sim = G['OpenLoopPlant'](dt=1e-3, load_model="static")
    sim.plant.integrator_rtol, sim.plant.integrator_atol = rtol, atol
    rows = []
    evaluations[0] = 0
    t0 = time.perf_counter()
    for t in np.arange(0.0, duration, 1e-3):
        command = G['body_command'](forward=float(np.interp(t, KEY_T, KEY_FORWARD)), left=float(np.interp(t, KEY_T, KEY_LEFT)))
        out = sim.step(G['inverse_kinematics'](command))
        rows.append(np.r_[sim.state[:3], out["slip_speeds"], out["ground_forces"], out["rolling"]])
    return np.array(rows), time.perf_counter() - t0, evaluations[0] / len(rows)


reference, ref_time, ref_evals = run(1e-8, 1e-10, False)
print(f"reference rtol 1e-8, fresh Jacobian: {ref_time:.0f} s, {ref_evals:.1f} evaluations per step")
print(f"{'setting':34s} {'time':>7s} {'evals/step':>10s} {'position':>10s} {'heading':>9s} {'slip':>10s} {'force':>8s} {'mode samples':>12s}")
for rtol, atol, reuse in ((1e-5, 1e-7, False), (1e-5, 1e-7, True), (1e-4, 1e-6, True), (1e-3, 1e-5, True)):
    rows, elapsed, evals = run(rtol, atol, reuse)
    diff = np.abs(rows - reference)
    label = f"rtol {rtol:.0e}, {'reused' if reuse else 'fresh'} Jacobian"
    print(f"{label:34s} {elapsed:6.0f}s {evals:10.1f} {1e3 * diff[:, :2].max():8.3f}mm {np.degrees(diff[:, 2].max()):7.3f}deg "
          f"{1e3 * diff[:, 3:7].max():7.2f}mm/s {diff[:, 7:11].max():7.3f}N {int(diff[:, 11:15].sum()):12d}", flush=True)
