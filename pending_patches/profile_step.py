"""Profile the open-loop plant on the 4.14 scenario (first 1.6 s: ramp, sideways burst, re-grip).

Usage: python profile_step.py NOTEBOOK [SECONDS]
Counts every right-hand-side evaluation, including the ones Radau uses for its finite-difference
Jacobian (solve_ivp's nfev does not count those).
"""
import cProfile
import json
import pstats
import sys
import time

import numpy as np

d = {c.get('id'): ''.join(c['source']) for c in json.load(open(sys.argv[1], encoding='utf-8'))['cells']}
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 1.6
setup = ['549569af', '766627f5', 'f26c71d2', 'dc169fed', '72c90d84', 'b7053b69', '9676641c']
if 'vplant02' in d:
    setup.append('vplant02')
G = {}
exec('\n\n'.join(d[k] for k in setup) + '\n\nclass ConvexMPCController: pass\n\n' + d['plantv02'], G)
print("integrator", G['PLANT_INTEGRATOR'], "ALPHA_D", G['rc'].ALPHA_D)

KEY_T = [0.0, 1.0, 1.1, 1.3, 1.4, 2.0, 2.1, 2.3, 2.4, 3.5]
KEY_FORWARD = [0.0, 1.5, 1.5, 1.5, 1.5, 1.5, 3.0, 3.0, 1.5, 1.5]
KEY_LEFT = [0.0, 0.0, -1.5, -1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
bc, ik = G['body_command'], G['inverse_kinematics']

with G['cad_cog']():
    sim = G['OpenLoopPlant'](dt=1e-3, load_model="static")
plant = sim.plant
calls = {"rhs": 0, "steps": 0}
original = type(plant).physical_dynamics


def counted(self, *args, **kwargs):
    calls["rhs"] += 1
    return original(self, *args, **kwargs)


type(plant).physical_dynamics = counted
nfev = []


def run():
    for t in np.arange(0.0, duration, 1e-3):
        command = bc(forward=float(np.interp(t, KEY_T, KEY_FORWARD)), left=float(np.interp(t, KEY_T, KEY_LEFT)))
        sim.step(ik(command))
        calls["steps"] += 1
        nfev.append(plant.last_nfev)


profiler = cProfile.Profile()
t0 = time.perf_counter()
profiler.runcall(run)
wall = time.perf_counter() - t0
print(f"{calls['steps']} steps in {wall:.1f} s = {1e3 * wall / calls['steps']:.2f} ms per 1 ms step; "
      f"RHS evaluations {calls['rhs']} = {calls['rhs'] / calls['steps']:.1f} per step "
      f"(solve_ivp nfev median {np.median(nfev):.0f}, total {sum(nfev)}); "
      f"{1e6 * wall / calls['rhs']:.0f} us per RHS evaluation incl. overhead")
stats = pstats.Stats(profiler)
stats.sort_stats("cumulative").print_stats(28)
stats.sort_stats("tottime").print_stats(22)
