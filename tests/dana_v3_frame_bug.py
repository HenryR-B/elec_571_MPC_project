"""Where the rotating-frame bug shows in Dana's v3 tests: her model as written (it advances the LOCAL chassis velocity)
against her model with only the chassis update changed to advance the GLOBAL velocity (our change 2). Everything else
is hers: constants, friction, motor, tests. Noise off. Also the plain check: a robot coasting at 1 m/s while spinning
at 3 rad/s, friction off, should travel 2 m in a straight line in 2 s.
Usage: python tests/dana_v3_frame_bug.py dana-models/robot-simulation-open-loop-v3.ipynb"""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
d = {c.get("id"): "".join(c["source"]) for c in json.load(open(sys.argv[1], encoding="utf-8"))["cells"]}
g = {}
exec(d["ebf0a37d"], g)
exec(d["21f5c2d7"].split("# 2. Instantiate")[0], g)
Dana, State = g["OmniRobotSimulation"], g["OmniMotorState"]


class DanaWorldFrame(Dana):
    """Dana's v3 with only advance_chassis_kinetics changed: the global velocity is advanced, the local one derived."""
    def advance_chassis_kinetics(self, F_floor, state):
        dt, th = self.dt, state.pos_global[2]
        a_local = self.M_inv @ (self.H.T @ F_floor)
        c, s = np.cos(th), np.sin(th)
        v_global_next = state.v_global + dt * np.array([c * a_local[0] - s * a_local[1], s * a_local[0] + c * a_local[1], a_local[2]])
        pos_next = state.pos_global + dt * v_global_next
        c1, s1 = np.cos(pos_next[2]), np.sin(pos_next[2])
        nxt = State()
        nxt.v_global, nxt.pos_global, nxt.a_local_past = v_global_next, pos_next, a_local
        nxt.v_local = np.array([c1 * v_global_next[0] + s1 * v_global_next[1], -s1 * v_global_next[0] + c1 * v_global_next[1], v_global_next[2]])
        return nxt


p0 = dict(g["simulation_parameters"], sigma_V=0.0, sigma_mu=0.0)
FWD, STR, ROT = np.array([1, -1, -1, 1.0]), np.array([-1, -1, 1, 1.0]), np.ones(4)
A30 = dict(alpha_f=np.radians(30.0), alpha_b=np.radians(45.0)); A45 = dict(alpha_f=np.radians(45.0), alpha_b=np.radians(45.0))
TESTS = [("Block 1 45/45 1 V forward", A45, lambda t: 1.0 * FWD, 0.5), ("Block 1 45/45 1 V strafe", A45, lambda t: 1.0 * STR, 0.5),
         ("Block 1 45/45 1 V rotation", A45, lambda t: 1.0 * ROT, 0.5),
         ("Block 2 30/45 1 V forward", A30, lambda t: 1.0 * FWD, 0.5), ("Block 2 30/45 1 V strafe", A30, lambda t: 1.0 * STR, 0.5),
         ("Block 2 30/45 1 V rotation", A30, lambda t: 1.0 * ROT, 0.5),
         ("Block 3 30/45 10 V forward", A30, lambda t: 10.0 * FWD, 0.5), ("Block 3 30/45 10 V strafe", A30, lambda t: 10.0 * STR, 0.5),
         ("Block 3 30/45 10 V rotation", A30, lambda t: 10.0 * ROT, 0.5),
         ("Multi 30/45 +-4 V fwd/back/coast", A30, lambda t: 4.0 * FWD * (1 if t < 1 else (-1 if t < 2 else 0)), 3.0),
         ("Multi 30/45 +-4 V side-to-side", A30, lambda t: 4.0 * -STR * (1 if t < 1 else (-1 if t < 2 else 0)), 3.0),
         ("Multi 30/45 +-4 V rotation", A30, lambda t: 4.0 * ROT * (1 if t < 1 else (-1 if t < 2 else 0)), 3.0)]


def run(cls, p, cmd, T):
    sim, s = cls(p), State()
    n = int(round(T / p["dt"])); P = np.zeros((n, 3)); yaw = 0.0
    for k in range(n):
        s, _, _ = sim.step(s, cmd(k * p["dt"])); P[k] = s.pos_global; yaw = max(yaw, abs(s.v_global[2]))
    return P, yaw


print(f"{'test':34} | {'as written: end (x, y), heading':34} | {'world frame: end (x, y), heading':34} | path diff | peak yaw rate")
for name, geo, cmd, T in TESTS:
    p = dict(p0, **geo)
    P0, y0 = run(Dana, p, cmd, T)
    P1, _ = run(DanaWorldFrame, p, cmd, T)
    diff = 1e3 * np.max(np.hypot(*(P0[:, :2] - P1[:, :2]).T))
    print(f"{name:34} | ({P0[-1, 0]:+.3f}, {P0[-1, 1]:+.3f}) {np.degrees(P0[-1, 2]):+8.1f}° | ({P1[-1, 0]:+.3f}, {P1[-1, 1]:+.3f}) {np.degrees(P1[-1, 2]):+8.1f}° "
          f"| {diff:6.1f} mm | {y0:5.2f} rad/s", flush=True)

# coasting while spinning, no friction, no motor
print("\nCoasting at 1 m/s forward while spinning at 3 rad/s for 2 s (friction and rolling resistance off, no voltage):")
pc = dict(p0, mu_s=1e-9, mu_k=1e-9, Crr=0.0)   # negligible friction (her LuGre divides by mu_s)
for cls, lab in ((Dana, "as written "), (DanaWorldFrame, "world frame")):
    sim, s = cls(pc), State()
    s.v_local = np.array([0.0, 1.0, 3.0]); s.v_global = np.array([0.0, 1.0, 3.0])
    s.omega = (sim.H @ s.v_local) / pc["r"]
    for k in range(2000):
        s, _, _ = sim.step(s, np.zeros(4))
    print(f"   {lab}: end ({s.pos_global[0]:+.3f}, {s.pos_global[1]:+.3f}) m, {np.hypot(*s.pos_global[:2]):.3f} m from the start")
