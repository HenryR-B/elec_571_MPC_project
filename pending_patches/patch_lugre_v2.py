"""LuGre tuned to follow the Stribeck curve away from zero slip, plus the strict comparison (section 4.16).

1. lugre_contact(): the LuGre force and bristle rate, shared by the plant and the wheel bench.
   The bristle state is stored as a fraction of LUGRE_PRESLIDING_DEFLECTION (the "bristle load"),
   so the solver tolerance means the same at any deflection.
2. Constants from the wheel bench: deflection 0.1 um, damping only near zero slip
   (exp(-(v / 5 mm/s)^2)), sigma2 = 0.
3. Comparison helpers (law_run, scenarios, curve error) move to the open-loop harness cell.
4. Section 4.15 uses them; its sensitivity sweep moves to the new deflection range.
5. New section 4.16: switch model without the blend and LuGre against the Stribeck curve, on a
   single-wheel bench and at robot level, with assertions.
Usage: python patch_lugre_v2.py NOTEBOOK...   (after patch_lugre and patch_lugre_section)
"""
import json
import re
import sys


def rep(s, old, new, count=1):
    assert s.count(old) == count, (s.count(old), old[:80])
    return s.replace(old, new)


def lines(text):
    parts = text.rstrip('\n').split('\n')
    return [p + '\n' for p in parts[:-1]] + [parts[-1]]


def wrap(source):
    compile(source, '<cell>', 'exec')
    body = '\n'.join(('    ' + line) if line.strip() else '' for line in source.rstrip('\n').split('\n'))
    wrapped = f'if RUN_PLANT_CHECKS:\n{body}\nelse:\n    print("Plant check skipped (RUN_PLANT_CHECKS = False).")'
    compile(wrapped, '<wrapped>', 'exec')
    return wrapped


def config(s):
    return rep(s, '''# LuGre drive friction (DRIVE_SLIDING_LAW = "lugre"). None of these is measured; the reasoning is in
# friction_model_plan.md, section 4.2. For each wheel, with slip v and g(v) = mu_d(|v|) N:
#   dz/dt = v - sigma0 |v| z / g(v),   F = -(sigma0 z + sigma1 dz/dt + sigma2 v)
# LUGRE_PRESLIDING_DEFLECTION: bristle deflection z at the static limit mu_s N [m]; sets the
#   stiffness sigma0 = mu_s N / deflection (about 7200 N/m for a rear wheel at 0.5 mm).
# LUGRE_DAMPING_RATIO: sigma1 as a fraction of critical damping of sigma0 against the rim inertia
#   J_w / R_w^2 (18.5 g).
# LUGRE_VISCOUS_COEFF: sigma2 [N s/m]. 0 keeps steady sliding equal to mu_d(|v|) N.
LUGRE_PRESLIDING_DEFLECTION = 0.5e-3
LUGRE_DAMPING_RATIO = 1.0
LUGRE_VISCOUS_COEFF = 0.0
''', '''# LuGre drive friction (DRIVE_SLIDING_LAW = "lugre"; lugre_contact). For each wheel, with slip v,
# g(v) = mu_d(|v|) N and bristle deflection z:
#   dz/dt = v - sigma0 |v| z / g(v),   F = -(sigma0 z + sigma1(v) dz/dt + sigma2 v)
# None is measured. They are chosen so the force never drops toward zero when the slip reverses,
# follows the Stribeck curve approximately once the slip has left zero, and stays affordable to
# simulate (section 4.16; friction_model_plan.md, section 7):
# LUGRE_PRESLIDING_DEFLECTION: z at the static limit mu_s N [m]; sigma0 = mu_s N / deflection. At zero
#   slip the bristles hold their load; after the slip reverses they slide 2-3 deflections before the
#   force changes sign. Wheel bench at 2 um: about 0.93 mu_s N held as the slip reverses, sign change
#   22-26 mm/s later, within 1% of the curve beyond 100 mm/s, breakaway 0.97-1.00 mu_s N. Stiffer is
#   closer to the curve and slower to simulate (0.1 um: sign change within 3 mm/s, about 6x the run
#   time of the switch model); 0.5 mm had the wrong sign up to about 1 m/s after a reversal.
# LUGRE_DAMPING_RATIO, LUGRE_DAMPING_SLIP: sigma1(v) = ratio x critical damping of sigma0 against the
#   rim inertia J_w / R_w^2, times exp(-(v / LUGRE_DAMPING_SLIP)^2), so it damps presliding only. A
#   constant sigma1 adds sigma1 dz/dt while the bristles reload and overshoots mu_s N by about 16%.
# LUGRE_VISCOUS_COEFF: sigma2 [N s/m]. 0: the Stribeck curve already holds the velocity dependence.
# LUGRE_CREEP_SLIP: |v| in dz/dt is taken as sqrt(v^2 + creep^2), which removes the kink at zero slip
#   that otherwise makes the solver take tiny steps. It keeps the static limit (the bristles still
#   cannot carry more than mu_s N) and adds a creep slip under load: 0.13 mm/s at 80% of mu_s N.
#   (A v tanh(v / eps) smoothing does NOT keep the limit: it let a wheel carry 1.07 mu_s N unslipped.)
# LUGRE_RTOL: solver tolerance with "lugre" (PLANT_RTOL otherwise). At 1e-4 the plant runs about 2.2x
#   the time of the switch model, within 0.3 mm and 0.2 deg of a 1e-7 solution over 1.6 s.
LUGRE_PRESLIDING_DEFLECTION = 2e-6
LUGRE_DAMPING_RATIO = 1.0
LUGRE_DAMPING_SLIP = 5e-3
LUGRE_VISCOUS_COEFF = 0.0
LUGRE_CREEP_SLIP = 1e-4
LUGRE_RTOL = 1e-4
''')


def friction_functions(s):
    return rep(s, '''

def mu_a(v_slip_a):''', '''

def lugre_contact(slip, bristle_load, static_limit, damped_mass):
    """LuGre drive force [N] and bristle-load rate [1/s] for each wheel (DRIVE_SLIDING_LAW = "lugre").

    bristle_load is the bristle deflection as a fraction of LUGRE_PRESLIDING_DEFLECTION (delta): the
    static limit static_limit = mu_s N is carried at 1, and steady sliding at mu_d(|v|) / mu_s.
        d(load)/dt = (v - (mu_s / mu_d(|v|)) sqrt(v^2 + LUGRE_CREEP_SLIP^2) load) / delta
        F = -(static_limit load + sigma1(v) delta d(load)/dt + LUGRE_VISCOUS_COEFF v)
        sigma1(v) = 2 LUGRE_DAMPING_RATIO sqrt(sigma0 m) exp(-(v / LUGRE_DAMPING_SLIP)^2),  sigma0 = static_limit / delta
    m (damped_mass) is the inertia the bristles act on, the rim J_w / R_w^2. In steady sliding the
    force is -sign(v) mu_d(|v|) N, the Stribeck curve of the other laws; at zero slip the bristles
    hold their load (less a creep of order LUGRE_CREEP_SLIP), so the force does not drop to zero.
    """
    slip = np.asarray(slip, dtype=float)
    delta = LUGRE_PRESLIDING_DEFLECTION
    load_rate = (slip - (rc.MU_D_STAT / mu_d(slip)) * np.sqrt(slip**2 + LUGRE_CREEP_SLIP**2) * bristle_load) / delta
    sigma1 = (2.0 * LUGRE_DAMPING_RATIO * np.sqrt(static_limit / delta * damped_mass)
              * np.exp(-(slip / LUGRE_DAMPING_SLIP) ** 2))
    force = -(static_limit * bristle_load + sigma1 * delta * load_rate + LUGRE_VISCOUS_COEFF * slip)
    return force, load_rate


def mu_a(v_slip_a):''')


def slip_plant(s):
    s = rep(s, '''        result = solve_ivp(
            rates,
            (0.0, self.dt),
            np.asarray(state, dtype=float),
            method=PLANT_INTEGRATOR,
            rtol=self.integrator_rtol,
            atol=self.integrator_atol,''', '''        rtol, atol = self.integrator_rtol, self.integrator_atol
        if DRIVE_SLIDING_LAW == "lugre" and rtol == PLANT_RTOL:
            # The stiff bristles make tight tolerances costly; LUGRE_RTOL is accurate enough for them.
            rtol, atol = LUGRE_RTOL, LUGRE_RTOL * PLANT_ATOL / PLANT_RTOL
        result = solve_ivp(
            rates,
            (0.0, self.dt),
            np.asarray(state, dtype=float),
            method=PLANT_INTEGRATOR,
            rtol=rtol,
            atol=atol,''')
    s = rep(s, '''    # First of the four LuGre bristle deflections z_i [m] (zero with the other friction laws).
    bristle_start = 14''', '''    # First of the four LuGre bristle loads: deflection z_i / LUGRE_PRESLIDING_DEFLECTION (zero with the
    # other friction laws).
    bristle_start = 14''')
    s = rep(s, '''        # Grip/slide blend weights (rolling) and LuGre bristle deflections (relaxed) at the start,''',
               '''        # Grip/slide blend weights (rolling) and LuGre bristle loads (relaxed) at the start,''')
    s = rep(s, '''    def bristle_deflections(self, state_world):
        """LuGre bristle deflections z_i [m] (zero with the other friction laws)."""
        b = self.bristle_start
        return np.asarray(state_world[b:b + N_WHEELS], dtype=float)

    def bristle_rates(self):
        """dz_i/dt from the last contact solve (zero with the other friction laws)."""
        return self._force_selection_diagnostics["bristle_rate"]

    def lugre_coefficients(self, normal_loads):
        """LuGre bristle stiffness sigma0 [N/m] and damping sigma1 [N s/m] for each wheel.

        sigma0 puts the static limit mu_s N at LUGRE_PRESLIDING_DEFLECTION of deflection, so it is
        proportional to the load; sigma1 is LUGRE_DAMPING_RATIO of critical damping of sigma0
        against the rim inertia J_w / R_w^2. Neither is measured (friction_model_plan.md, 4.2).
        """
        sigma0 = rc.MU_D_STAT * self.friction_scale * np.asarray(normal_loads, dtype=float) / LUGRE_PRESLIDING_DEFLECTION
        sigma1 = 2.0 * LUGRE_DAMPING_RATIO * np.sqrt(sigma0 * rc.WHEEL_INERTIA / rc.WHEEL_RADIUS**2)
        return sigma0, sigma1
''', '''    def bristle_loads(self, state_world):
        """LuGre bristle loads z_i / LUGRE_PRESLIDING_DEFLECTION (zero with the other friction laws)."""
        b = self.bristle_start
        return np.asarray(state_world[b:b + N_WHEELS], dtype=float)

    def bristle_deflections(self, state_world):
        """LuGre bristle deflections z_i [m]."""
        return LUGRE_PRESLIDING_DEFLECTION * self.bristle_loads(state_world)

    def bristle_rates(self):
        """Bristle-load rates from the last contact solve (zero with the other friction laws)."""
        return self._force_selection_diagnostics["bristle_rate"]
''')
    s = rep(s, '''            # LuGre (Canudas de Wit et al. 1995): each wheel's drive force follows from its bristle
            # deflection z_i and its slip, so there is no grip/slide decision and the robot-level
            # equations are solved once:
            #   dz/dt = v - sigma0 |v| z / g(v),   F = -(sigma0 z + sigma1 dz/dt + sigma2 v),
            #   g(v) = mu_d(|v|) N, the Stribeck curve of the other laws.
            # sigma0 / g(v) = mu_s / (mu_d(|v|) LUGRE_PRESLIDING_DEFLECTION) does not depend on the
            # load, so a lifted wheel (N = 0) needs no special case.
            bristle = self.bristle_deflections(state_world)
            sigma0, sigma1 = self.lugre_coefficients(normal_loads)
            bristle_rate = slip_d - (rc.MU_D_STAT / (mu_d(slip_d) * LUGRE_PRESLIDING_DEFLECTION)) * np.abs(slip_d) * bristle
            force_used = -(sigma0 * bristle + sigma1 * bristle_rate + LUGRE_VISCOUS_COEFF * slip_d)''',
               '''            # LuGre (Canudas de Wit et al. 1995; lugre_contact): each wheel's drive force follows
            # from its bristle load and its slip, so there is no grip/slide decision and the
            # robot-level equations are solved once. A lifted wheel (N = 0) carries no force.
            bristle_load = self.bristle_loads(state_world)
            force_used, bristle_rate = lugre_contact(
                slip_d, bristle_load, traction_limits, rc.WHEEL_INERTIA / rc.WHEEL_RADIUS**2
            )''')
    s = rep(s, '''            sliding_force = self.friction_scale * np.asarray(normal_loads, dtype=float) * mu_d(slip_d)
            rolling = ~((np.abs(sigma0 * bristle) >= 0.95 * sliding_force)
                        & (np.abs(slip_d) >= rc.DRIVE_BREAKAWAY_BAND)) & self.enable_friction''',
               '''            rolling = ~((np.abs(bristle_load) >= 0.95 * mu_d(slip_d) / rc.MU_D_STAT)
                        & (np.abs(slip_d) >= rc.DRIVE_BREAKAWAY_BAND)) & self.enable_friction''')
    s = rep(s, '''        z = self.bristle_start
        snapshot["bristle_deflection"] = snapshot["state_after"][z:z + N_WHEELS].copy()''',
               '''        snapshot["bristle_deflection"] = self.bristle_deflections(snapshot["state_after"])''')
    return s


def voltage_plant(s):
    s = rep(s, '''    bristle_start = 18  # then the LuGre bristle deflections''', '''    bristle_start = 18  # then the LuGre bristle loads''')
    s = rep(s, '''4 blend weights, 4 bristle deflections.''', '''4 blend weights, 4 bristle loads.''')
    return s


def state_test(s):
    return rep(s, '''4 blend weights, 4 bristle deflections"''', '''4 blend weights, 4 bristle loads"''')


HARNESS_HELPERS = r'''

# Friction-model comparison helpers (sections 4.15 and 4.16).
import time
from contextlib import contextmanager

COMPARISON_DT = 1e-3
REAR = [2, 3]


@contextmanager
def friction_law(law, presliding=None, damping_ratio=None, blend_time=None):
    """Run a block with DRIVE_SLIDING_LAW = law and, optionally, other LuGre parameters or blend time."""
    global DRIVE_SLIDING_LAW, LUGRE_PRESLIDING_DEFLECTION, LUGRE_DAMPING_RATIO, DRIVE_MODE_BLEND_TIME
    saved = (DRIVE_SLIDING_LAW, LUGRE_PRESLIDING_DEFLECTION, LUGRE_DAMPING_RATIO, DRIVE_MODE_BLEND_TIME)
    DRIVE_SLIDING_LAW = law
    if presliding is not None:
        LUGRE_PRESLIDING_DEFLECTION = presliding
    if damping_ratio is not None:
        LUGRE_DAMPING_RATIO = damping_ratio
    if blend_time is not None:
        DRIVE_MODE_BLEND_TIME = blend_time
    try:
        yield
    finally:
        DRIVE_SLIDING_LAW, LUGRE_PRESLIDING_DEFLECTION, LUGRE_DAMPING_RATIO, DRIVE_MODE_BLEND_TIME = saved


def law_run(law, duration_s, command_fn=None, current_fn=None, presliding=None, damping_ratio=None, blend_time=None):
    """Open-loop run with one friction law: wheel-speed targets (command_fn) or motor currents (current_fn)."""
    with friction_law(law, presliding, damping_ratio, blend_time):
        with cad_cog():  # real CoG: the rear wheels can break loose
            sim = OpenLoopPlant(dt=COMPARISON_DT, load_model="static")
        plant = sim.plant
        voltage_plant = hasattr(plant, "current_to_voltage")
        rows = {key: [] for key in ("force", "needed", "limit", "slip", "mode", "twist", "pose", "bristle", "nfev")}
        started = time.perf_counter()
        for t in np.arange(0.0, duration_s, COMPARISON_DT):
            rows["bristle"].append(plant.bristle_deflections(sim.state))
            if current_fn is None:
                out = sim.step(inverse_kinematics(command_fn(t)))
                values = (out["ground_forces"], out["force_needed"], out["friction_limits"], out["slip_speeds"], out["rolling"])
            else:
                request = current_fn(t)
                plant.state = sim.state
                plant.update_contact_disturbances()
                applied = sim.state[10:14] if voltage_plant else plant.saturate_current(sim.state, request)
                plant._contact_wrenches(sim.state, applied, np.zeros(3))
                diag = plant._force_selection_diagnostics
                values = (diag["force_used"], diag["force_to_roll"], diag["traction_limits"], diag["wheel_slip_d"], diag["rolling_mask"])
                command = plant.current_to_voltage(sim.state, request) if voltage_plant else request
                sim.state = plant.discrete_map(sim.state, command, np.zeros(3))
            for key, value in zip(("force", "needed", "limit", "slip", "mode"), values):
                rows[key].append(np.array(value, copy=True))
            rows["twist"].append(np.r_[world_to_body(sim.state[3:5], sim.state[2]), sim.state[5]])
            rows["pose"].append(sim.state[:3].copy())
            rows["nfev"].append(plant.last_nfev)
        log = {key: np.array(value) for key, value in rows.items()}
        log["wall_s"] = time.perf_counter() - started
    log["time"] = np.arange(len(log["force"])) * COMPARISON_DT
    return log


COMPARISON_FORWARD_PATTERN = np.sign(inverse_kinematics(body_command(forward=1.0)))
COMPARISON_KEY_T = [0.0, 1.0, 1.1, 1.3, 1.4, 2.0, 2.1, 2.3, 2.4, 3.5]
COMPARISON_KEY_FORWARD = [0.0, 1.5, 1.5, 1.5, 1.5, 1.5, 3.0, 3.0, 1.5, 1.5]
COMPARISON_KEY_LEFT = [0.0, 0.0, -1.5, -1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def comparison_ramp_current(t):
    """4.12 slow current ramp: 0 -> current limit over 1.5 s, hold 0.3 s, back to 0 over 1.5 s."""
    return np.interp(t, [0.0, 1.5, 1.8, 3.3], [0.0, rc.IQ_LIMIT, rc.IQ_LIMIT, 0.0]) * COMPARISON_FORWARD_PATTERN


def comparison_reversals(t):
    """4.12 fast reversals: forward +-3 m/s, flipping every 0.15 s for 1.2 s, then stop."""
    if t < 1.2:
        return body_command(forward=3.0 if int(t / 0.15) % 2 == 0 else -3.0)
    return body_command()


def comparison_at_speed(t):
    """4.14: cruise at 1.5 m/s, sideways burst, cruise, push to 3 m/s, cruise (0.1 s ramps)."""
    return body_command(forward=float(np.interp(t, COMPARISON_KEY_T, COMPARISON_KEY_FORWARD)),
                        left=float(np.interp(t, COMPARISON_KEY_T, COMPARISON_KEY_LEFT)))


def held_then_stop(command, hold_s=1.0):
    """4.13 manoeuvre: hold the command for hold_s, then stop."""
    return lambda t: command if t < hold_s else body_command()


COMPARISON_SCENARIOS = {
    "slow current ramp (4.12)": dict(duration_s=4.0, current_fn=comparison_ramp_current),
    "fast reversals (4.12)": dict(duration_s=2.0, command_fn=comparison_reversals),
    "forward 2.5 m/s, then stop (4.13)": dict(duration_s=2.5, command_fn=held_then_stop(body_command(forward=2.5))),
    "sideways 2 m/s, then stop (4.13)": dict(duration_s=2.5, command_fn=held_then_stop(body_command(left=-2.0))),
    "diagonal 1.8 + 1.8 m/s, then stop (4.13)": dict(duration_s=2.5, command_fn=held_then_stop(body_command(forward=1.8, left=-1.8))),
    "transitions at speed (4.14)": dict(duration_s=3.5, command_fn=comparison_at_speed),
}


def curve_error(slip, force, limit):
    """|F - F_curve| / (mu_s N) per sample, with F_curve = -sign(v) mu_d(|v|) N the Stribeck curve."""
    return np.abs(force + np.sign(slip) * mu_d(slip) * limit / rc.MU_D_STAT) / limit


def clean_beyond(slip, error, tolerance):
    """Largest |slip| at which the curve error exceeds tolerance: beyond it the force is within tolerance."""
    bad = np.abs(slip[error > tolerance])
    return bad.max() if bad.size else 0.0


def steady_sliding(log, min_slip=0.02, settle_s=0.1):
    """Samples where a wheel has slid one way faster than min_slip for settle_s (transients over)."""
    fast = np.abs(log["slip"]) >= min_slip
    direction = np.sign(log["slip"])
    window = int(round(settle_s / COMPARISON_DT))
    steady = np.zeros_like(fast)
    for k in range(window, len(fast)):
        steady[k] = np.all(fast[k - window:k + 1], axis=0) & np.all(direction[k - window:k + 1] == direction[k], axis=0)
    return steady


def rear_slide_time(log):
    return (~log["mode"][:, REAR]).sum(axis=0) * COMPARISON_DT
'''


def harness(s):
    return s.rstrip('\n') + HARNESS_HELPERS.rstrip('\n')


def section_415_md(s):
    s = re.sub(r"- \$\\sigma_0 = \\mu_s N / \\delta\$, with presliding deflection .*\n",
               lambda m: "- $\\sigma_0 = \\mu_s N / \\delta$, with presliding deflection $\\delta$ = `LUGRE_PRESLIDING_DEFLECTION` (2 µm; section 4.16 and the config cell give the reasons);\n", s)
    s = re.sub(r"- \$\\sigma_1\$ = `LUGRE_DAMPING_RATIO` .*\n",
               lambda m: "- $\\sigma_1(v)$ = `LUGRE_DAMPING_RATIO` × critical damping against the rim inertia $J_w / R_w^2$, × $e^{-(v / 5\\,\\mathrm{mm/s})^2}$ (presliding only);\n", s)
    assert "(2 µm;" in s and "presliding only" in s
    return s


SECTION_415 = r'''# 4.15 LuGre vs the switch model (with and without the blend): the same open-loop scenarios.
# The run helpers (law_run, COMPARISON_SCENARIOS, ...) are in the open-loop harness cell.
LAW_VARIANTS = {
    "switch": dict(law="switch"),
    "switch, no blend": dict(law="switch", blend_time=0.0),
    "lugre": dict(law="lugre"),
}
LAW_COLORS = {"switch": "C0", "switch, no blend": "C2", "lugre": "C3"}
LUGRE_SCENARIOS = {name: COMPARISON_SCENARIOS[name] for name in (
    "slow current ramp (4.12)", "fast reversals (4.12)", "sideways 2 m/s, then stop (4.13)", "transitions at speed (4.14)")}

lugre_runs = {name: {label: law_run(**LAW_VARIANTS[label], **scenario) for label in LAW_VARIANTS}
              for name, scenario in LUGRE_SCENARIOS.items()}
for log in (log for runs in lugre_runs.values() for log in runs.values()):
    assert np.all(np.isfinite(log["force"])) and np.all(np.isfinite(log["pose"])), "a run diverged"

print("LuGre vs switch, with and without the blend (rear wheels; force as a fraction of mu_s N; reported, not asserted):")
for name, runs in lugre_runs.items():
    print(f"  {name}:")
    for law, log in runs.items():
        steady = steady_sliding(log)
        error = curve_error(log["slip"], log["force"], log["limit"])[steady]
        steady_text = (f"steady-sliding error vs the Stribeck curve median {100 * np.median(error):.2f}%, "
                       f"95th percentile {100 * np.percentile(error, 95):.2f}% ({steady.sum()} samples)") if steady.any() else "no steady sliding"
        print(f"    {law:16s}: peak force {np.max(np.abs(log['force'][:, REAR]) / log['limit'][:, REAR]):.3f}; "
              f"rear sliding time {np.round(rear_slide_time(log), 3)} s; peak |yaw rate| {np.abs(log['twist'][:, 2]).max():.2f} rad/s; "
              f"end pose x {log['pose'][-1, 0]:+.3f} m, y {log['pose'][-1, 1]:+.3f} m, heading {np.degrees(log['pose'][-1, 2]):+.1f} deg; "
              f"{steady_text}; peak |z| {1e6 * np.abs(log['bristle']).max():.3f} um; "
              f"solver evaluations per step median {np.median(log['nfev']):.0f}, max {log['nfev'].max()}; run {log['wall_s']:.0f} s")
    lugre_end = runs["lugre"]["pose"][-1]
    for label in ("switch", "switch, no blend"):
        end = runs[label]["pose"][-1]
        print(f"    lugre vs {label} at the end: position {1e3 * np.linalg.norm(lugre_end[:2] - end[:2]):.0f} mm, "
              f"heading {np.degrees(lugre_end[2] - end[2]):+.1f} deg")

# Sensitivity of the at-speed scenario to the unmeasured bristle parameters.
SENSITIVITY = [(1e-6, 1.0), (2e-6, 1.0), (5e-6, 1.0), (20e-6, 1.0), (2e-6, 0.5), (2e-6, 2.0)]
at_speed_scenario = LUGRE_SCENARIOS["transitions at speed (4.14)"]
references = {label: lugre_runs["transitions at speed (4.14)"][label] for label in ("switch", "switch, no blend")}
sensitivity_runs = {}
print("Sensitivity, transitions at speed:")
for label, log in references.items():
    print(f"  {label}: rear sliding time {np.round(rear_slide_time(log), 3)} s, peak |yaw rate| {np.abs(log['twist'][:, 2]).max():.2f} rad/s")
for presliding, damping_ratio in SENSITIVITY:
    if (presliding, damping_ratio) == (LUGRE_PRESLIDING_DEFLECTION, LUGRE_DAMPING_RATIO):
        log = lugre_runs["transitions at speed (4.14)"]["lugre"]
    else:
        log = law_run("lugre", presliding=presliding, damping_ratio=damping_ratio, **at_speed_scenario)
    sensitivity_runs[(presliding, damping_ratio)] = log
    end = log["pose"][-1]
    ends = "; ".join(f"vs {label}: position {1e3 * np.linalg.norm(end[:2] - ref['pose'][-1, :2]):.0f} mm, "
                     f"heading {np.degrees(end[2] - ref['pose'][-1, 2]):+.1f} deg" for label, ref in references.items())
    print(f"  lugre, presliding {1e6 * presliding:g} um, damping ratio {damping_ratio:.1f}: peak force {np.max(np.abs(log['force'][:, REAR]) / log['limit'][:, REAR]):.3f} of mu_s N, "
          f"rear sliding time {np.round(rear_slide_time(log), 3)} s, peak |yaw rate| {np.abs(log['twist'][:, 2]).max():.2f} rad/s; end {ends}; "
          f"solver evaluations per step median {np.median(log['nfev']):.0f}, max {log['nfev'].max()}; run {log['wall_s']:.0f} s")

# Figure 1: rear-left wheel and robot motion per scenario, all three models.
WHEEL = 2
fig, axes = plt.subplots(len(lugre_runs), 3, figsize=(19, 3.3 * len(lugre_runs)))
for row, (name, runs) in zip(axes, lugre_runs.items()):
    limit = runs["switch"]["limit"][:, WHEEL]
    time_s = runs["switch"]["time"]
    for sign in (1, -1):
        row[0].plot(time_s, sign * limit, "k--", linewidth=0.7)
        row[0].plot(time_s, sign * limit * rc.MU_D_DYN / rc.MU_D_STAT, "k-.", linewidth=0.6, alpha=0.6)
    for law, log in runs.items():
        color = LAW_COLORS[law]
        row[0].plot(log["time"], log["force"][:, WHEEL], color, label=f"applied, {law}")
        row[0].plot(log["time"], log["needed"][:, WHEEL], color, linestyle=":", linewidth=1.8, label=f"needed to roll, {law}")
        row[1].plot(log["time"], 1e3 * log["slip"][:, WHEEL], color, label=law)
        row[2].plot(log["time"], np.linalg.norm(log["twist"][:, :2], axis=1), color, label=f"speed (m/s), {law}")
        row[2].plot(log["time"], log["twist"][:, 2], color, linestyle="--", label=f"yaw rate (rad/s), {law}")
    row[0].set_ylim(-3 * limit.max(), 3 * limit.max())
    row[0].set(ylabel="Force (N)", title=f"{name}: {rc.WHEEL_NAMES[WHEEL]} force (dashed mu_s N, dash-dot mu_k N)")
    row[1].set(ylabel="Slip (mm/s)", title=f"{name}: {rc.WHEEL_NAMES[WHEEL]} slip")
    row[2].set(ylabel="m/s, rad/s", title=f"{name}: robot motion")
    for ax in row:
        ax.grid(True, alpha=0.3); ax.legend(fontsize=6, loc="upper right")
for ax in axes[-1]:
    ax.set_xlabel("Time (s)")
plt.tight_layout(); plt.show()

# Figure 2: friction coefficient F/N against slip speed, every wheel and sample, against the Stribeck curve.
fig, axes = plt.subplots(2, len(LAW_VARIANTS), figsize=(21, 9))
speeds = np.linspace(-3.0, 3.0, 2001)
for column, law in enumerate(LAW_COLORS):
    for row, span in enumerate((3.0, 0.03)):
        ax = axes[row, column]
        for index, (name, runs) in enumerate(lugre_runs.items()):
            log = runs[law]
            ax.plot(log["slip"].ravel(), (log["force"] / (log["limit"] / rc.MU_D_STAT)).ravel(), ".", markersize=1.5,
                    color=f"C{index + 1}", alpha=0.5, label=name)
        ax.plot(speeds, -np.sign(speeds) * mu_d(speeds), "k", linewidth=1.2, label="Stribeck curve -sign(v) mu_d(|v|)")
        for sign in (1, -1):
            ax.axhline(sign * rc.MU_D_STAT, color="k", linestyle="--", linewidth=0.7)
            ax.axhline(sign * rc.MU_D_DYN, color="k", linestyle="-.", linewidth=0.6, alpha=0.6)
        ax.set_xlim(-span, span); ax.set_ylim(-1.2 * rc.MU_D_STAT, 1.2 * rc.MU_D_STAT)
        ax.set(xlabel="Drive slip v (m/s)", ylabel="F / N",
               title=f"{law}: friction coefficient vs slip ({'full range' if row == 0 else 'near zero slip'})")
        ax.grid(True, alpha=0.3)
        if row == 0:
            ax.legend(fontsize=7, markerscale=6, loc="upper right")
plt.tight_layout(); plt.show()

# Figure 3: sensitivity of the at-speed scenario.
fig, axes = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
for (label, ref), style in zip(references.items(), ("-", ":")):
    axes[0].plot(ref["time"], ref["twist"][:, 2], "k", linestyle=style, linewidth=2, label=label)
    axes[1].plot(ref["time"], 1e3 * ref["slip"][:, WHEEL], "k", linestyle=style, linewidth=2, label=label)
for index, ((presliding, damping_ratio), log) in enumerate(sensitivity_runs.items()):
    label = f"lugre, presliding {1e6 * presliding:g} um, damping ratio {damping_ratio:.1f}"
    axes[0].plot(log["time"], log["twist"][:, 2], color=f"C{index + 1}", label=label)
    axes[1].plot(log["time"], 1e3 * log["slip"][:, WHEEL], color=f"C{index + 1}", label=label)
axes[0].set(ylabel="Yaw rate (rad/s)", title="Transitions at speed (4.14): sensitivity to the bristle parameters")
axes[1].set(ylabel=f"{rc.WHEEL_NAMES[WHEEL]} slip (mm/s)", xlabel="Time (s)")
for ax in axes:
    ax.grid(True, alpha=0.3); ax.legend(fontsize=7, loc="upper right")
plt.tight_layout(); plt.show()'''


SECTION_416_MD = r'''## 4.16 Switch model (no blend) and LuGre against the friction curve

A test of the two models that should both follow the static/kinetic curve
$F = -\mathrm{sign}(v)\,\mu_d(|v|)\,N$ once the slip has left zero: the switch model without the blend, and LuGre with the configured constants. Slip below `STRICT_IGNORE_SLIP` (100 mm/s) is excluded from the curve comparison. There the two differ by design: LuGre holds its bristle load through zero slip and changes sign a few tens of mm/s after a reversal, while the switch model has its grip solve and transition band.

1. **Wheel bench (LuGre only).** One wheel with the chassis held, so the rim alone takes the motor force and the slip accelerates fastest; the motor's voltage limit is included. The cases are a slow ramp to the current limit then a step reversal, a fast ramp with a fast reversal, and a 2 Hz sine of the full current. The force comes from `lugre_contact`, the function the plant uses. The switch model's sliding force is the curve by construction, so the bench tests LuGre.
2. **Robot level (both models).** The six open-loop scenarios of 4.12-4.14 (real CoG, `"static"` loads, 1 ms). Every wheel and sample with $|v| \ge$ 100 mm/s is compared with the curve. The two models' robot motion, force smoothness (largest change between 1 ms samples) and sliding are then compared.

Checks:
- The force does not drop toward zero when the slip reverses: on the bench LuGre holds at least 0.85 $\mu_s N$ at every reversal.
- LuGre's breakaway peak on the bench is between 0.95 and 1.005 of $\mu_s N$.
- Beyond 100 mm/s of slip, every sample is within `STRICT_TOLERANCE` (2% of $\mu_s N$) of the curve, for both models (and LuGre on the bench).
- Wheels still slip with LuGre wherever they slip with the switch model.'''


SECTION_416 = r'''# 4.16 Switch model (no blend) and LuGre against the Stribeck curve -sign(v) mu_d(|v|) N.
STRICT_IGNORE_SLIP = 0.1   # [m/s] near-zero slip, where the two models differ by design
STRICT_TOLERANCE = 0.02    # fraction of mu_s N
STRICT_MODELS = {"switch, no blend": dict(law="switch", blend_time=0.0), "lugre": dict(law="lugre")}
STRICT_COLORS = {"switch, no blend": "C2", "lugre": "C3"}

# Part 1: one wheel, chassis held, LuGre force from lugre_contact (the plant's function).
with cad_cog():
    bench_limit = rc.MU_D_STAT * WheelSlipRobotSimulator(load_model="static").static_loads[2]  # back-left
rim_mass = rc.WHEEL_INERTIA / rc.WHEEL_RADIUS**2


def bench_motor_force(current, slip):
    """Motor force at the rim [N] for a current command, within the current and voltage limits (chassis held)."""
    back_emf = rc.MOTOR_KT * slip / rc.WHEEL_RADIUS
    high = min(rc.IQ_LIMIT, (rc.MAX_MOTOR_VOLTAGE - back_emf) / rc.MOTOR_RESISTANCE)
    low = max(-rc.IQ_LIMIT, (-rc.MAX_MOTOR_VOLTAGE - back_emf) / rc.MOTOR_RESISTANCE)
    return rc.MOTOR_KT * min(max(current, low), high) / rc.WHEEL_RADIUS


BENCH_CASES = {
    "slow ramp, step reversal": (3.0, lambda t: np.interp(t, [0.0, 1.5], [0.0, rc.IQ_LIMIT]) if t < 2.0 else -rc.IQ_LIMIT),
    "fast ramp, fast reversal": (1.0, lambda t: np.interp(t, [0.0, 0.01, 0.5, 0.51], [0.0, rc.IQ_LIMIT, rc.IQ_LIMIT, -rc.IQ_LIMIT])),
    "2 Hz sine": (2.0, lambda t: rc.IQ_LIMIT * np.sin(4.0 * np.pi * t)),
}
bench_runs = {}
print(f"Wheel bench (LuGre, back-left load, mu_s N = {bench_limit:.2f} N, presliding {1e6 * LUGRE_PRESLIDING_DEFLECTION:g} um):")
for name, (duration, current) in BENCH_CASES.items():
    def rates(t, y, current=current):
        force, load_rate = lugre_contact(np.array([y[0]]), np.array([y[1]]), np.array([bench_limit]), rim_mass)
        return [(bench_motor_force(current(t), y[0]) + force[0]) / rim_mass, load_rate[0]]
    result = solve_ivp(rates, (0.0, duration), [0.0, 0.0], method="Radau", rtol=PLANT_RTOL, atol=PLANT_ATOL,
                       max_step=1e-3, dense_output=True)
    assert result.success, f"{name}: {result.message}"
    t = np.arange(0.0, duration, 2e-5)
    slip, load = result.sol(t)
    force = lugre_contact(slip, load, np.full(slip.size, bench_limit), rim_mass)[0]
    error = curve_error(slip, force, np.full(slip.size, bench_limit))
    away = np.abs(slip) >= STRICT_IGNORE_SLIP
    peak = np.max(np.abs(force)) / bench_limit
    reversals = np.flatnonzero((np.diff(np.sign(slip)) != 0) & (t[:-1] > 0.005))
    reversals = reversals[np.abs(slip[reversals]) + np.abs(slip[reversals + 1]) > 1e-3]  # real reversals, not creep
    held = np.abs(force[reversals]) / bench_limit
    flips = np.flatnonzero((np.diff(np.sign(force)) != 0) & (t[:-1] > 0.005))
    bench_runs[name] = (t, slip, force, error)
    print(f"  {name:26s}: breakaway peak {peak:.3f} of mu_s N; force held as the slip reverses "
          f"{(f'{held.min():.3f}-{held.max():.3f}' if held.size else 'no reversal')} of mu_s N, sign change "
          f"{(f'{1e3 * np.abs(slip[flips]).min():.1f}-{1e3 * np.abs(slip[flips]).max():.1f}' if flips.size else '-')} mm/s after; "
          f"beyond {1e3 * STRICT_IGNORE_SLIP:.0f} mm/s the largest curve error is {100 * error[away].max():.2f}% of mu_s N; "
          f"within 1% beyond {1e3 * clean_beyond(slip, error, 0.01):.1f} mm/s; peak slip {np.abs(slip).max():.2f} m/s")
    assert 0.95 <= peak <= 1.005, f"{name}: LuGre breakaway peak {peak:.3f} of mu_s N"
    assert held.size == 0 or held.min() >= 0.85, f"{name}: LuGre force fell to {held.min():.3f} mu_s N at a slip reversal"
    assert error[away].max() <= STRICT_TOLERANCE, f"{name}: LuGre off the curve by {100 * error[away].max():.2f}% of mu_s N"

# Part 2: robot level, both models, every wheel and sample.
strict_runs = {name: {label: law_run(**STRICT_MODELS[label], **scenario) for label in STRICT_MODELS}
               for name, scenario in COMPARISON_SCENARIOS.items()}
print(f"Robot level (every wheel and sample with |v| >= {1e3 * STRICT_IGNORE_SLIP:.0f} mm/s; error as a fraction of mu_s N):")
worst = {label: 0.0 for label in STRICT_MODELS}
for name, runs in strict_runs.items():
    print(f"  {name}:")
    for label, log in runs.items():
        assert np.all(np.isfinite(log["force"])) and np.all(np.isfinite(log["pose"])), f"{name}, {label}: run diverged"
        error = curve_error(log["slip"], log["force"], log["limit"])
        away = np.abs(log["slip"]) >= STRICT_IGNORE_SLIP
        largest = error[away].max() if away.any() else 0.0
        worst[label] = max(worst[label], largest)
        print(f"    {label:16s}: {away.sum():5d} samples, largest error {100 * largest:.3f}%, 99th percentile "
              f"{100 * (np.percentile(error[away], 99) if away.any() else 0.0):.3f}%; within 1% beyond "
              f"{1e3 * clean_beyond(log['slip'], error, 0.01):.1f} mm/s; peak force {np.max(np.abs(log['force']) / log['limit']):.3f} of mu_s N; "
              f"largest 1 ms force change {np.abs(np.diff(log['force'], axis=0) / log['limit'][1:]).max():.2f} of mu_s N; "
              f"rear sliding time {np.round(rear_slide_time(log), 3)} s; "
              f"solver evaluations per step median {np.median(log['nfev']):.0f}, max {log['nfev'].max()}; run {log['wall_s']:.0f} s")
    switch_log, lugre_log = runs["switch, no blend"], runs["lugre"]
    slides_switch, slides_lugre = rear_slide_time(switch_log), rear_slide_time(lugre_log)
    assert np.all((slides_switch == 0) | (slides_lugre > 0)), f"{name}: a wheel slides with the switch model but not with LuGre"
    print(f"    lugre vs switch, no blend: end position {1e3 * np.linalg.norm(lugre_log['pose'][-1, :2] - switch_log['pose'][-1, :2]):.0f} mm, "
          f"heading {np.degrees(lugre_log['pose'][-1, 2] - switch_log['pose'][-1, 2]):+.2f} deg; largest yaw-rate difference "
          f"{np.abs(lugre_log['twist'][:, 2] - switch_log['twist'][:, 2]).max():.3f} rad/s; largest rear slip difference "
          f"{1e3 * np.abs(lugre_log['slip'][:, REAR] - switch_log['slip'][:, REAR]).max():.0f} mm/s")
for label, value in worst.items():
    assert value <= STRICT_TOLERANCE, f"{label}: off the curve by {100 * value:.2f}% of mu_s N beyond {1e3 * STRICT_IGNORE_SLIP:.0f} mm/s"
print(f"PASS: LuGre holds the force through slip reversals and breaks away at mu_s N; wheels slide with both models; "
      f"beyond {1e3 * STRICT_IGNORE_SLIP:.0f} mm/s of slip both stay within {100 * STRICT_TOLERANCE:.0f}% of mu_s N of the curve "
      f"(largest: " + ", ".join(f"{label} {100 * value:.3f}%" for label, value in worst.items()) + ").")

# Figure 1: wheel bench, F/N against slip.
fig, axes = plt.subplots(1, 2, figsize=(15, 5))
speeds = np.linspace(-20.0, 20.0, 40001)
for ax, span in zip(axes, (None, 0.1)):
    for index, (name, (t, slip, force, error)) in enumerate(bench_runs.items()):
        ax.plot(slip, force / (bench_limit / rc.MU_D_STAT), color=f"C{index}", linewidth=0.8, label=name)
    ax.plot(speeds, -np.sign(speeds) * mu_d(speeds), "k--", linewidth=1.0, label="Stribeck curve")
    ax.axvspan(-STRICT_IGNORE_SLIP, STRICT_IGNORE_SLIP, color="0.85", label="excluded near zero")
    if span is not None:
        ax.set_xlim(-span, span)
    ax.set(xlabel="Slip v (m/s)", ylabel="F / N", title=f"Wheel bench, LuGre ({'near zero' if span else 'full range'})")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=7)
plt.tight_layout(); plt.show()

# Figure 2: robot level, F/N against slip and the curve error against |slip|, both models.
fig, axes = plt.subplots(2, len(STRICT_MODELS), figsize=(16, 10))
speeds = np.linspace(-3.0, 3.0, 6001)
for column, label in enumerate(STRICT_MODELS):
    ax = axes[0, column]
    for index, (name, runs) in enumerate(strict_runs.items()):
        log = runs[label]
        ax.plot(log["slip"].ravel(), (log["force"] / (log["limit"] / rc.MU_D_STAT)).ravel(), ".", markersize=1.5,
                color=f"C{index}", alpha=0.5, label=name)
    ax.plot(speeds, -np.sign(speeds) * mu_d(speeds), "k", linewidth=1.0, label="Stribeck curve")
    ax.axvspan(-STRICT_IGNORE_SLIP, STRICT_IGNORE_SLIP, color="0.85")
    ax.set(xlabel="Slip v (m/s)", ylabel="F / N", title=f"{label}: friction coefficient vs slip")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=6, markerscale=6)
    ax = axes[1, column]
    for index, (name, runs) in enumerate(strict_runs.items()):
        log = runs[label]
        moving = np.abs(log["slip"]) > 1e-4
        ax.loglog(np.abs(log["slip"][moving]), 100 * np.maximum(curve_error(log["slip"], log["force"], log["limit"])[moving], 1e-6),
                  ".", markersize=1.5, color=f"C{index}", alpha=0.5)
    ax.axhline(100 * STRICT_TOLERANCE, color="k", linestyle="--", linewidth=0.8, label=f"{100 * STRICT_TOLERANCE:.0f}% of mu_s N")
    ax.axvline(STRICT_IGNORE_SLIP, color="k", linestyle=":", linewidth=0.8, label=f"{1e3 * STRICT_IGNORE_SLIP:.0f} mm/s")
    ax.set_ylim(1e-4, 300)
    ax.set(xlabel="|Slip| (m/s)", ylabel="|F - curve| (% of mu_s N)", title=f"{label}: distance from the curve")
    ax.grid(True, which="both", alpha=0.3); ax.legend(fontsize=7)
plt.tight_layout(); plt.show()

# Figure 3: robot motion, both models, every scenario.
fig, axes = plt.subplots(len(strict_runs), 2, figsize=(16, 2.8 * len(strict_runs)))
for row, (name, runs) in zip(axes, strict_runs.items()):
    for label, log in runs.items():
        color, style = STRICT_COLORS[label], "-" if label == "lugre" else "--"
        for wheel in REAR:
            row[0].plot(log["time"], 1e3 * log["slip"][:, wheel], color=color, linestyle=style, linewidth=1.0,
                        label=f"{rc.WHEEL_NAMES[wheel]}, {label}")
        row[1].plot(log["time"], log["twist"][:, 2], color=color, linestyle=style, label=f"yaw rate, {label}")
    row[0].set(ylabel="Slip (mm/s)", title=f"{name}: rear-wheel slip")
    row[1].set(ylabel="rad/s", title=f"{name}: yaw rate")
    for ax in row:
        ax.grid(True, alpha=0.3); ax.legend(fontsize=6, loc="upper right")
for ax in axes[-1]:
    ax.set_xlabel("Time (s)")
plt.tight_layout(); plt.show()'''


EDITS = [('549569af', config), ('f26c71d2', friction_functions), ('9676641c', slip_plant), ('vplant02', voltage_plant),
         ('c4e5f6a7', state_test), ('plantv02', harness), ('lugre01', section_415_md)]

if __name__ == "__main__":
    for path in sys.argv[1:]:
        nb = json.load(open(path, encoding='utf-8'))
        cells = {c.get('id'): c for c in nb['cells']}
        for cid, fn in EDITS:
            if cid not in cells:
                continue
            c = cells[cid]
            c['source'] = lines(fn(''.join(c['source'])))
            if c['cell_type'] == 'code':
                c['outputs'] = []
                c['execution_count'] = None
        cells['lugre02']['source'] = lines(wrap(SECTION_415))
        cells['lugre02']['outputs'] = []
        cells['lugre02']['execution_count'] = None
        ids = [c.get('id') for c in nb['cells']]
        assert 'coul02' not in ids, "section 4.16 is already in this notebook"
        position = ids.index('lugre02') + 1
        nb['cells'][position:position] = [
            {"cell_type": "markdown", "id": "coul01", "metadata": {}, "source": lines(SECTION_416_MD)},
            {"cell_type": "code", "id": "coul02", "metadata": {}, "execution_count": None, "outputs": [],
             "source": lines(wrap(SECTION_416))},
        ]
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            json.dump(nb, f, ensure_ascii=False, indent=1)
            f.write('\n')
        print('patched', path)
