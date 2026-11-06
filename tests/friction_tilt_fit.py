"""Fit the static drive (d) and roller (a) friction coefficients to Henry's tilt-test data (Google sheet "with battery",
2026-10-29), using the model's own geometry (30/45 layout, H, axle directions) and load split.

The tilt test: the robot sits on a tilted surface at robot angle theta; the slope is raised until it slides at angle phi;
mu_eff = tan(phi). Model of the break-away, for pure sliding straight downhill (no rotation):
    each wheel resists along its drive direction d_i up to mu_d * N_i (wheel held: motors unpowered, 30:1 gearbox) and
    along its axle a_i up to mu_a * N_i (rollers), independently, so the most it can resist along the downhill unit
    vector u is N_i * (mu_d |d_i . u| + mu_a |a_i . u|);
    the robot starts to slide when  M g sin(phi) = sum_i N_i(phi) (mu_d |d_i . u| + mu_a |a_i . u|),
    with N_i(phi) the model's static split of M g cos(phi) plus the downhill load transfer of M g sin(phi) at height h.
Assumed: theta = 0 means the robot's forward (+y_r) points downhill, theta measured counter-clockwise.
Usage: python tests/friction_tilt_fit.py RobotSimulationOpenLoopModel_HenryConstants.ipynb"""
import json, sys
from pathlib import Path
import numpy as np
from scipy.optimize import brentq, least_squares
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
d = {c.get("id"): "".join(c["source"]) for c in json.load(open(sys.argv[1], encoding="utf-8"))["cells"]}
g = {}
exec(d["ebf0a37d"], g)
exec(d["21f5c2d7"].split("# 2. Instantiate")[0], g)
p = dict(g["simulation_parameters"], sigma_V=0.0, sigma_mu=0.0)
sim = g["OmniRobotSimulation"](p)
M, G = p["Mr"], 9.81

# Henry's tilt angles at which the robot slid (degrees), robot angles 0..315 deg (sheet "with battery", rows 4-11)
THETA = np.radians([0, 45, 90, 135, 180, 225, 270, 315])
TILT = {"Urethane A90": [32, 30, 33, 30, 35, 32, 29, 25],
        "Urethane A70": [39, 33, 36, 33, 38, 32, 39, 31],
        "Chloroprene A65": [35, 31, 34, 31, 36, 31, 32, 29]}


def loads(phi, u):
    """Model's load split on the slope: static part with g cos(phi); load transfer as if accelerating uphill at g sin(phi)."""
    a_equiv = -G * np.sin(phi) * u                 # body-frame 'acceleration' that gives the downhill load transfer
    p_cos = dict(p); sim_c = g["OmniRobotSimulation"](p_cos)
    Fn = sim_c.compute_dynamic_normal_forces(a_equiv, np.zeros(4))
    static = sim_c.compute_dynamic_normal_forces(np.zeros(2), np.zeros(4))
    return static * np.cos(phi) + (Fn - static)


def slide_angle(theta, mu_d, mu_a):
    u = np.array([-np.sin(theta), np.cos(theta)])   # downhill direction in body axes (theta = 0: forward downhill)
    dproj, aproj = np.abs(sim.H[:, :2] @ u), np.abs(sim.a_hat @ u)
    f = lambda phi: M * G * np.sin(phi) - np.sum(loads(phi, u) * (mu_d * dproj + mu_a * aproj))
    return brentq(f, 1e-4, np.radians(80.0))


print("Projections of the downhill direction on each wheel's drive (|d.u|) and axle (|a.u|) direction, 30/45 layout:")
for th in THETA:
    u = np.array([-np.sin(th), np.cos(th)])
    print(f"   theta {np.degrees(th):5.0f} deg: |d.u| {np.round(np.abs(sim.H[:, :2] @ u), 3)}  |a.u| {np.round(np.abs(sim.a_hat @ u), 3)}")

print("\nLeast-squares fit of (mu_d,s, mu_a,s) to the slide angles:")
fits = {}
for name, tilt in TILT.items():
    tilt = np.radians(tilt)
    res = least_squares(lambda x: [slide_angle(th, x[0], x[1]) - t for th, t in zip(THETA, tilt)], x0=[0.9, 0.2],
                        bounds=([0.0, 0.0], [3.0, 3.0]))
    J = res.jac; cov = np.linalg.inv(J.T @ J) * (res.fun @ res.fun) / (len(tilt) - 2)
    sd = np.sqrt(np.diag(cov))
    fits[name] = res.x
    pred = [np.degrees(slide_angle(th, *res.x)) for th in THETA]
    print(f"   {name:16}: mu_d,s = {res.x[0]:.3f} ± {sd[0]:.3f}, mu_a,s = {res.x[1]:.3f} ± {sd[1]:.3f} | rms angle error {np.degrees(np.sqrt(np.mean(res.fun**2))):.1f} deg")
    print(f"      measured  {np.round(np.degrees(tilt)).astype(int)}\n      predicted {np.round(pred, 1)}")

print("\nmu_a,s implied for a fixed mu_d,s (least squares on the angles), A70:")
tilt = np.radians(TILT["Urethane A70"])
for md in (0.9, 1.0, 1.1):
    r = least_squares(lambda x: [slide_angle(th, md, x[0]) - t for th, t in zip(THETA, tilt)], x0=[0.2], bounds=([0.0], [3.0]))
    print(f"   mu_d,s = {md:.1f}: mu_a,s = {r.x[0]:.3f}, rms angle error {np.degrees(np.sqrt(np.mean(r.fun**2))):.1f} deg")
print("\nCurrent robot_constants: MU_D_STAT 0.9, MU_A_STAT 0.2; Henry's sheet estimate (A70): mu_d 1.0-1.1, mu_a 0.16-0.18")
