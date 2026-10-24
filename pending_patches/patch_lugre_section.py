"""Section 4.15: LuGre vs the switch model on the same open-loop scenarios (both laws on every plot).

Usage: python patch_lugre_section.py NOTEBOOK...   (after patch_lugre)
"""
import json
import sys


def lines(text):
    parts = text.rstrip('\n').split('\n')
    return [p + '\n' for p in parts[:-1]] + [parts[-1]]


SECTION_MD = r'''## 4.15 LuGre vs the switch model

The same open-loop scenarios run with three drive friction models:

- **switch** (the default): a gripping wheel's force comes from the robot-level solve, capped at $\mu_s N$; a sliding wheel follows the Stribeck curve $\mu_d(|v|) N$, with the transition band and the grip/slide blend (`DRIVE_MODE_BLEND_TIME`).
- **switch, no blend**: the same with the blend off. This is the plain static/kinetic (Coulomb-Stribeck) model: the force drops to the curve at breakaway.
- **lugre** (Canudas de Wit et al. 1995): each wheel has a bristle deflection $z$, with
  $\dot z = v - \sigma_0 |v| z / g(v)$, $F = -(\sigma_0 z + \sigma_1 \dot z + \sigma_2 v)$ and $g(v) = \mu_d(|v|) N$.
  There is no grip/slide decision. In steady sliding $F = -\mathrm{sign}(v)\, g(v)$, the same curve as the switch model.

The bristle parameters are not measured:
- $\sigma_0 = \mu_s N / \delta$, with presliding deflection $\delta$ = `LUGRE_PRESLIDING_DEFLECTION` (0.5 mm);
- $\sigma_1$ = `LUGRE_DAMPING_RATIO` × critical damping against the rim inertia $J_w / R_w^2$;
- $\sigma_2 = 0$.

The scenarios are taken from 4.12, 4.13 and 4.14: the slow current ramp, the fast reversals, sideways 2 m/s then stop, and the transitions at speed. All use the real CoG, `"static"` loads and a 1 ms step.

The figures:
1. **Rear-left wheel per scenario, all three models.** Applied force (solid), the force needed if every wheel gripped (dotted), $\pm\mu_s N$ (dashed) and $\pm\mu_k N$ (dash-dot); its slip; the robot speed and yaw rate.
2. **Friction coefficient $F/N$ against slip speed.** Every wheel and sample of every scenario, for each model, against the Stribeck curve $\mu_d(|v|)$ and $\mu_s$. This is the comparison with the static/kinetic (Coulomb) model, full range and near zero slip.
3. **Sensitivity of the at-speed scenario** to $\delta$ and the damping ratio. As $\delta \to 0$, LuGre should approach the switch model without the blend.

The comparisons are reported, not asserted (friction_model_plan.md, section 4.5, gives the targets).'''

SECTION_CODE = r'''# 4.15 LuGre vs the switch model (with and without the blend): the same open-loop scenarios.
import time
from contextlib import contextmanager

LUGRE_DT = 1e-3
LAW_VARIANTS = {
    "switch": dict(law="switch"),
    "switch, no blend": dict(law="switch", blend_time=0.0),
    "lugre": dict(law="lugre"),
}
LAW_COLORS = {"switch": "C0", "switch, no blend": "C2", "lugre": "C3"}
REAR = [2, 3]


@contextmanager
def friction_law(law, presliding=None, damping_ratio=None, blend_time=None):
    """Run a block with DRIVE_SLIDING_LAW = law and, optionally, other LuGre parameters or blend time."""
    global DRIVE_SLIDING_LAW, LUGRE_PRESLIDING_DEFLECTION, LUGRE_DAMPING_RATIO, DRIVE_MODE_BLEND_TIME
    saved = (DRIVE_SLIDING_LAW, LUGRE_PRESLIDING_DEFLECTION, LUGRE_DAMPING_RATIO, DRIVE_MODE_BLEND_TIME)
    DRIVE_SLIDING_LAW = law
    if blend_time is not None:
        DRIVE_MODE_BLEND_TIME = blend_time
    if presliding is not None:
        LUGRE_PRESLIDING_DEFLECTION = presliding
    if damping_ratio is not None:
        LUGRE_DAMPING_RATIO = damping_ratio
    try:
        yield
    finally:
        DRIVE_SLIDING_LAW, LUGRE_PRESLIDING_DEFLECTION, LUGRE_DAMPING_RATIO, DRIVE_MODE_BLEND_TIME = saved


def law_run(law, duration_s, command_fn=None, current_fn=None, presliding=None, damping_ratio=None, blend_time=None):
    """Open-loop run with one friction law: wheel-speed targets (command_fn) or motor currents (current_fn)."""
    with friction_law(law, presliding, damping_ratio, blend_time):
        with cad_cog():  # real CoG: the rear wheels can break loose
            sim = OpenLoopPlant(dt=LUGRE_DT, load_model="static")
        plant = sim.plant
        voltage_plant = hasattr(plant, "current_to_voltage")
        rows = {key: [] for key in ("force", "needed", "limit", "slip", "mode", "twist", "pose", "bristle", "nfev")}
        started = time.perf_counter()
        for t in np.arange(0.0, duration_s, LUGRE_DT):
            rows["bristle"].append(plant.bristle_deflections(sim.state).copy())
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
    log["time"] = np.arange(len(log["force"])) * LUGRE_DT
    return log


LUGRE_FORWARD_PATTERN = np.sign(inverse_kinematics(body_command(forward=1.0)))
LUGRE_KEY_T = [0.0, 1.0, 1.1, 1.3, 1.4, 2.0, 2.1, 2.3, 2.4, 3.5]
LUGRE_KEY_FORWARD = [0.0, 1.5, 1.5, 1.5, 1.5, 1.5, 3.0, 3.0, 1.5, 1.5]
LUGRE_KEY_LEFT = [0.0, 0.0, -1.5, -1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def lugre_ramp_current(t):
    """4.12 slow current ramp: 0 -> current limit over 1.5 s, hold 0.3 s, back to 0 over 1.5 s."""
    return np.interp(t, [0.0, 1.5, 1.8, 3.3], [0.0, rc.IQ_LIMIT, rc.IQ_LIMIT, 0.0]) * LUGRE_FORWARD_PATTERN


def lugre_reversals(t):
    """4.12 fast reversals: forward +-3 m/s, flipping every 0.15 s for 1.2 s, then stop."""
    if t < 1.2:
        return body_command(forward=3.0 if int(t / 0.15) % 2 == 0 else -3.0)
    return body_command()


def lugre_sideways(t):
    """4.13 sideways (+x) 2 m/s for 1 s, then stop."""
    return body_command(left=-2.0) if t < 1.0 else body_command()


def lugre_at_speed(t):
    """4.14: cruise at 1.5 m/s, sideways burst, cruise, push to 3 m/s, cruise (0.1 s ramps)."""
    return body_command(forward=float(np.interp(t, LUGRE_KEY_T, LUGRE_KEY_FORWARD)),
                        left=float(np.interp(t, LUGRE_KEY_T, LUGRE_KEY_LEFT)))


LUGRE_SCENARIOS = {
    "slow current ramp (4.12)": dict(duration_s=4.0, current_fn=lugre_ramp_current),
    "fast reversals (4.12)": dict(duration_s=2.0, command_fn=lugre_reversals),
    "sideways 2 m/s, then stop (4.13)": dict(duration_s=2.5, command_fn=lugre_sideways),
    "transitions at speed (4.14)": dict(duration_s=3.5, command_fn=lugre_at_speed),
}


def steady_sliding(log, min_slip=0.02, settle_s=0.1):
    """Samples where a wheel has slid one way faster than min_slip for settle_s (transients over)."""
    fast = np.abs(log["slip"]) >= min_slip
    direction = np.sign(log["slip"])
    window = int(round(settle_s / LUGRE_DT))
    steady = np.zeros_like(fast)
    for k in range(window, len(fast)):
        steady[k] = np.all(fast[k - window:k + 1], axis=0) & np.all(direction[k - window:k + 1] == direction[k], axis=0)
    return steady


def curve_force(log):
    """Stribeck (static/kinetic) force for each sample's slip: -sign(v) mu_d(|v|) N."""
    return -np.sign(log["slip"]) * mu_d(log["slip"]) * log["limit"] / rc.MU_D_STAT


def rear_slide_time(log):
    return (~log["mode"][:, REAR]).sum(axis=0) * LUGRE_DT


lugre_runs = {name: {label: law_run(**LAW_VARIANTS[label], **scenario) for label in LAW_VARIANTS}
              for name, scenario in LUGRE_SCENARIOS.items()}
for log in (log for runs in lugre_runs.values() for log in runs.values()):
    assert np.all(np.isfinite(log["force"])) and np.all(np.isfinite(log["pose"])), "a run diverged"

print("LuGre vs switch, with and without the blend (rear wheels; force as a fraction of mu_s N; reported, not asserted):")
for name, runs in lugre_runs.items():
    print(f"  {name}:")
    for law, log in runs.items():
        steady = steady_sliding(log)
        error = np.abs(log["force"] - curve_force(log))[steady] / log["limit"][steady]
        steady_text = (f"steady-sliding error vs the Stribeck curve median {100 * np.median(error):.2f}%, "
                       f"95th percentile {100 * np.percentile(error, 95):.2f}% ({steady.sum()} samples)") if steady.any() else "no steady sliding"
        print(f"    {law:16s}: peak force {np.max(np.abs(log['force'][:, REAR]) / log['limit'][:, REAR]):.3f}; "
              f"rear sliding time {np.round(rear_slide_time(log), 3)} s; peak |yaw rate| {np.abs(log['twist'][:, 2]).max():.2f} rad/s; "
              f"end pose x {log['pose'][-1, 0]:+.3f} m, y {log['pose'][-1, 1]:+.3f} m, heading {np.degrees(log['pose'][-1, 2]):+.1f} deg; "
              f"{steady_text}; peak |z| {1e3 * np.abs(log['bristle']).max():.2f} mm; "
              f"solver evaluations per step median {np.median(log['nfev']):.0f}, max {log['nfev'].max()}; run {log['wall_s']:.0f} s")
    lugre_end = runs["lugre"]["pose"][-1]
    for label in ("switch", "switch, no blend"):
        end = runs[label]["pose"][-1]
        print(f"    lugre vs {label} at the end: position {1e3 * np.linalg.norm(lugre_end[:2] - end[:2]):.0f} mm, "
              f"heading {np.degrees(lugre_end[2] - end[2]):+.1f} deg")

# Sensitivity of the at-speed scenario to the unmeasured bristle parameters.
SENSITIVITY = [(0.05e-3, 1.0), (0.1e-3, 1.0), (0.5e-3, 1.0), (2.0e-3, 1.0), (0.5e-3, 0.5), (0.5e-3, 2.0)]
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
    print(f"  lugre, presliding {1e3 * presliding:4.2f} mm, damping ratio {damping_ratio:.1f}: peak force {np.max(np.abs(log['force'][:, REAR]) / log['limit'][:, REAR]):.3f} of mu_s N, "
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
    label = f"lugre, presliding {1e3 * presliding:.2f} mm, damping ratio {damping_ratio:.1f}"
    axes[0].plot(log["time"], log["twist"][:, 2], color=f"C{index + 1}", label=label)
    axes[1].plot(log["time"], 1e3 * log["slip"][:, WHEEL], color=f"C{index + 1}", label=label)
axes[0].set(ylabel="Yaw rate (rad/s)", title="Transitions at speed (4.14): sensitivity to the bristle parameters")
axes[1].set(ylabel=f"{rc.WHEEL_NAMES[WHEEL]} slip (mm/s)", xlabel="Time (s)")
for ax in axes:
    ax.grid(True, alpha=0.3); ax.legend(fontsize=7, loc="upper right")
plt.tight_layout(); plt.show()'''


def wrap(source):
    compile(source, '<cell>', 'exec')
    body = '\n'.join(('    ' + line) if line.strip() else '' for line in source.rstrip('\n').split('\n'))
    wrapped = f'if RUN_PLANT_CHECKS:\n{body}\nelse:\n    print("Plant check skipped (RUN_PLANT_CHECKS = False).")'
    compile(wrapped, '<wrapped>', 'exec')
    return wrapped


if __name__ == "__main__":
    for path in sys.argv[1:]:
        nb = json.load(open(path, encoding='utf-8'))
        ids = [c.get('id') for c in nb['cells']]
        assert 'lugre02' not in ids, "section 4.15 is already in this notebook"
        position = ids.index('speed02') + 1
        nb['cells'][position:position] = [
            {"cell_type": "markdown", "id": "lugre01", "metadata": {}, "source": lines(SECTION_MD)},
            {"cell_type": "code", "id": "lugre02", "metadata": {}, "execution_count": None, "outputs": [],
             "source": lines(wrap(SECTION_CODE))},
        ]
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            json.dump(nb, f, ensure_ascii=False, indent=1)
            f.write('\n')
        print('patched', path)
