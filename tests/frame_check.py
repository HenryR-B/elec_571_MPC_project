"""Dana's original class, unmodified: coast with zero floor force while spinning."""
import json
import numpy as np
nb = json.load(open("dana-models/RobotSimulationOpenLoopModel.ipynb", encoding="utf-8"))
d = {c.get("id"): "".join(c["source"]) for c in nb["cells"]}
g = {}
exec(d["ebf0a37d"], g)
exec(d["21f5c2d7"].split("# 2. Instantiate")[0], g)          # Dana's own parameters
sim = g["OmniRobotSimulation"](g["simulation_parameters"])
s = g["OmniMotorState"]()
s.v_local = np.array([0.0, 1.0, 3.0])     # 1 m/s along +y_r, spinning at 3 rad/s
path = []
for k in range(int(2.0 / sim.dt)):
    s = sim.advance_chassis_kinetics(np.zeros(4), s)    # no floor force at all
    path.append(s.pos_global[:2].copy())
path = np.array(path)
speed = np.linalg.norm(np.diff(path, axis=0), axis=1) / sim.dt
print(f"world speed stays {speed.min():.3f}-{speed.max():.3f} m/s")
print(f"after 2 s: position {path[-1].round(3)} m, distance from start {np.linalg.norm(path[-1]):.3f} m (straight line would be 2.000 m)")
print(f"farthest from start: {np.linalg.norm(path, axis=1).max():.3f} m  -> circle of radius v/omega = {1.0 / 3.0:.3f} m")
