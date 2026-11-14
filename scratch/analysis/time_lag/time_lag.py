# The time-lag grading experiment (11-13): the reference waits whenever it is LEAD_LIMIT ahead of the robot's point on
# the path (TimeLagExecution, a WaypointExecution), graded by off-path error and time lost. The cells added after the
# copied setup cells of ControllerAnalysis (they need its settings, controllers and runner).


# ==============================================================================================================
# Markdown
# ==============================================================================================================
## Time-lag execution: the class, the grades and the plots


# ==============================================================================================================
# Code
# ==============================================================================================================
# =======================================================================
# TIME-LAG EXECUTION (Henry): the reference waits when it is LEAD_LIMIT ahead of the robot
# =======================================================================
LEAD_LIMIT = 0.4          # s of path: the most the reference may run ahead of the robot's point on the path
MAX_TIME_FACTOR = 2.0     # the run ends at this many times the path's scheduled time if the robot has not finished


class TimeLagExecution(WaypointExecution):
    """
    Henry's time-lag run, to pass to run_simulation(..., executor=...). The reference plays the path at its own pace,
    but waits whenever it is LEAD_LIMIT or more of path time ahead of the robot's point on the path (the nearest path
    sample, searched from a little behind the robot's last point up to the reference, so it never jumps to another
    loop of the path). At each waypoint (a stop) it also waits until the robot has settled there (WaypointExecution).
    Logs the robot's point on the path each step (robot_steps).
    """
    def __init__(self, lead_limit: float = LEAD_LIMIT, time_factor: float = MAX_TIME_FACTOR):
        super().__init__(time_allowance=0.0)
        self.lead_limit = lead_limit
        self.time_factor = time_factor

    def start(self, path: dict) -> int:
        super().start(path)                   # the waypoints the reference stops at
        self.path_xy = path['pos'][:, :2]
        self.robot_step = 0
        self.robot_steps = []
        return int(round(self.time_factor * len(path['pos'])))

    def next_step(self, r: int, state) -> int:
        # The robot's point on the path: the nearest path sample, from a little behind its last point up to the reference
        window = np.arange(max(0, self.robot_step - 10), r + 1)
        distance = np.hypot(*(self.path_xy[window] - state.pos_global[:2]).T)
        self.robot_step = int(window[np.argmin(distance)])
        self.robot_steps.append(self.robot_step)

        # How far ahead of the robot the reference is, in path time; wait if it is at the limit
        lead = (r - self.robot_step) * DT_CTRL
        if lead >= self.lead_limit:
            return r

        # Otherwise the waypoint rule: wait at a stop until settled there, else carry on
        return super().next_step(r, state)


def figure_eight_stop_path(size, lap_time, laps=2, ramp=0.6, swing=0.3, dt=DT_CTRL):
    """
    A figure eight that starts and ends at rest at the origin (Henry's eight, x = a sin(phi), y = a/2 sin(2 phi),
    heading swing sin(phi)), so its end is a waypoint the robot must settle at. The lap rate ramps up over `ramp`
    seconds (half cosine), holds 2 pi / lap_time, and ramps down the same way; total time laps x lap_time + ramp.
    """
    rate = 2.0 * np.pi / lap_time
    cruise = laps * lap_time - ramp
    total = laps * lap_time + ramp
    t = dt * np.arange(int(round(total / dt)) + 1)

    # Phase and its rate in the three parts: ramp up, cruise, ramp down
    tau_down = t - ramp - cruise
    phase_up = rate * (t / 2.0 - ramp / (2.0 * np.pi) * np.sin(np.pi * t / ramp))
    phase_cruise = rate * (ramp / 2.0 + (t - ramp))
    phase_down = rate * (ramp / 2.0 + cruise + tau_down / 2.0 + ramp / (2.0 * np.pi) * np.sin(np.pi * tau_down / ramp))
    rate_up = rate * (1.0 - np.cos(np.pi * t / ramp)) / 2.0
    rate_down = rate * (1.0 + np.cos(np.pi * tau_down / ramp)) / 2.0
    phase = np.where(t < ramp, phase_up, np.where(t < ramp + cruise, phase_cruise, phase_down))
    phase_dot = np.where(t < ramp, rate_up, np.where(t < ramp + cruise, rate, rate_down))

    pos = np.column_stack((size * np.sin(phase), 0.5 * size * np.sin(2.0 * phase), swing * np.sin(phase)))
    vel = np.column_stack((size * np.cos(phase) * phase_dot, size * np.cos(2.0 * phase) * phase_dot, swing * np.cos(phase) * phase_dot))
    return {'t': t, 'pos': pos, 'vel': vel, 'waypoints': [pos[0].copy(), pos[-1].copy()]}


def run_time_lag(controller_name: str, path: dict) -> dict:
    """One time-lag run, plus the two grades: accuracy (off the path, heading) and time (finish, time lost)."""
    executor = TimeLagExecution()
    res = run_simulation(controller_name, path, executor=executor)
    robot_steps = np.array(executor.robot_steps)
    res['robot_step'] = robot_steps

    # Accuracy: perpendicular distance from the path (time ignored), heading error against the path's heading at the robot's point
    res['perp_err'], res['lag_err'] = path_errors(res['pos'][:, :2], res['pos_ref'][:, :2])
    res['heading_err_path'] = np.abs((path['pos'][robot_steps, 2] - res['pos'][:, 2] + np.pi) % (2.0 * np.pi) - np.pi)

    # Time: how far behind schedule the robot is (the clock minus the path time at its point), the finish and the time lost
    res['time_behind'] = res['t'] + DT_CTRL - DT_CTRL * robot_steps
    res['scheduled_end'] = DT_CTRL * (len(path['pos']) - 1)
    res['finished'] = res['settled'][-1]
    if res['finished'] is None:
        res['time_lost'] = res['t'][-1] + DT_CTRL - res['scheduled_end']     # did not finish: the time it had
    else:
        res['time_lost'] = res['finished'] - res['scheduled_end']
    return res


def run_and_grade(title, paths, controllers):
    """Run every controller on every path with the time rule; print one table; return the results."""
    results = {}
    print(f"{title} (lead limit {LEAD_LIMIT} s, run ends at {MAX_TIME_FACTOR}x the scheduled time)")
    print(f"{'Path':30s} {'Controller':16s} {'Off path RMS (cm)':>17s} {'max (cm)':>8s} {'Heading RMS (deg)':>17s} "
          f"{'Scheduled (s)':>13s} {'Finished (s)':>12s} {'Time lost (s)':>13s} {'Peak I (A)':>10s} {'Energy (J)':>10s} {'ms/step':>7s}   Settled at, each waypoint (s)")
    for path_name, path in paths.items():
        for controller in controllers:
            res = run_time_lag(controller, path)
            results[(path_name, controller)] = res
            if res['finished'] is None:
                finished_text = 'DNF'
            else:
                finished_text = f"{res['finished']:.2f}"
            settled_text = " ".join("X" if t is None else f"{t:.2f}" for t in res['settled'])
            peak_current = np.sum(np.abs(res['I_1ms']), axis=1).max()
            energy = np.sum(np.abs(res['V_1ms'] * res['I_1ms'])) * DT_PLANT
            print(f"{path_name:30s} {controller:16s} {100 * np.sqrt(np.mean(res['perp_err']**2)):17.2f} {100 * res['perp_err'].max():8.1f} "
                  f"{np.degrees(np.sqrt(np.mean(res['heading_err_path']**2))):17.1f} {res['scheduled_end']:13.2f} {finished_text:>12s} "
                  f"{res['time_lost']:13.2f} {peak_current:10.1f} {energy:10.1f} {1000 * res['elapsed'] / len(res['t']):7.1f}   {settled_text}", flush=True)
    return results


def plot_off_path_and_time(results, paths, controllers, columns=2):
    """For each path, over time: how far off the path each controller was (top) and how far behind schedule (bottom; the
    clock minus the path time at its point). A dot marks when it finished (settled at the last waypoint)."""
    rows = int(np.ceil(len(paths) / columns))
    fig, axs = plt.subplots(2 * rows, columns, figsize=(7 * columns, 6.5 * rows), squeeze=False)
    for n, path_name in enumerate(paths):
        ax_off = axs[2 * (n // columns), n % columns]
        ax_time = axs[2 * (n // columns) + 1, n % columns]
        for controller in controllers:
            res = results[(path_name, controller)]
            color = CONTROLLERS[controller][1]
            ax_off.plot(res['t'], 100 * res['perp_err'], color=color, lw=1.4, label=controller)
            ax_time.plot(res['t'], res['time_behind'], color=color, lw=1.4, label=controller)
            if res['finished'] is not None:
                ax_time.plot(res['finished'], res['time_lost'], 'o', color=color, ms=7, mec='k', zorder=5)
        ax_off.axvline(results[(path_name, controllers[0])]['scheduled_end'], color='k', ls='--', lw=1.0)
        ax_time.axvline(results[(path_name, controllers[0])]['scheduled_end'], color='k', ls='--', lw=1.0, label='scheduled end')
        ax_off.set_title(path_name)
        ax_off.set_ylabel('Off the path (cm)')
        ax_time.set_ylabel('Behind schedule (s)')
        ax_time.set_xlabel('Time (s)  (dot: finished)')
        ax_off.legend(fontsize='small')
        ax_time.legend(fontsize='small')
        for ax in (ax_off, ax_time):
            ax.grid(True, linestyle=':', alpha=0.6)
    for n in range(len(paths), rows * columns):
        for r in range(2):
            axs[2 * (n // columns) + r, n % columns].axis('off')
    plt.tight_layout()
    plt.show()


def plot_grade(results, pairs, controllers, columns=3):
    """
    The grade, one panel per shape: time lost (x) against off-path RMS (y), down-left is best. Each controller is an
    arrow from its run at the normal pace (open circle) to its run with the time cut (filled). A short arrow: it stays
    accurate when the time is cut. X: did not finish (time lost is then the time it had).
    """
    rows = int(np.ceil(len(pairs) / columns))
    fig, axs = plt.subplots(rows, columns, figsize=(6 * columns, 5 * rows), squeeze=False)
    for ax, (shape, (normal, shortened)) in zip(axs.ravel(), pairs.items()):
        for controller in controllers:
            color = CONTROLLERS[controller][1]
            points = []
            for path_name, fill in ((normal, 'none'), (shortened, color)):
                res = results[(path_name, controller)]
                x = res['time_lost']
                y = 100 * np.sqrt(np.mean(res['perp_err']**2))
                if res['finished'] is None:
                    ax.plot(x, y, 'X', color=color, mec='k', ms=11, zorder=6)
                else:
                    ax.plot(x, y, 'o', mfc=fill, mec=color, mew=2, ms=9, zorder=6)
                points.append((x, y))
            ax.annotate('', xy=points[1], xytext=points[0], arrowprops=dict(arrowstyle='->', color=color, lw=1.6))
            ax.plot([], [], 'o', mfc=color, mec=color, label=controller)
        ax.set_title(f'{shape}: {normal} -> {shortened}', fontsize=10)
        ax.set_xlabel('Time lost (s)  (X: did not finish)')
        ax.set_ylabel('Off path, RMS (cm)')
        ax.grid(True, linestyle=':', alpha=0.6)
        ax.legend(fontsize='small', title='open: normal, filled: cut')
    for ax in axs.ravel()[len(pairs):]:
        ax.axis('off')
    plt.tight_layout()
    plt.show()


# ==============================================================================================================
# Markdown
# ==============================================================================================================
## Test cases

Each shape at its normal pace and with the time cut (no holds; the waypoints are the stops):

| Shape | Normal | Cut |
|---|---|---|
| Dana's waypoints (three stops) | 5 s segments | 3.5 s segments |
| Dana's move to (3, 3, π/4) | 8 s | 4 s |
| Turn to (3, 3, π) | 8 s | 5 s |
| Figure eight 0.4 m, two laps, from and to rest | 3.4 s per lap | 2.5 s per lap |
| Figure eight 0.3 m, two laps, from and to rest | 3.0 s per lap | 2.2 s per lap |


# ==============================================================================================================
# Code
# ==============================================================================================================
TIME_LAG_CONTROLLERS = ['PID tuned', 'Simple MPC', 'LQR', 'DOB MPC', 'Coulomb MPC', 'Linearized MPC']
WAYPOINTS = [np.array([0.0, 0.0, 0.0]), np.array([2.2, 1.2, 0.50]), np.array([1.0, 3.5, -0.60]), np.array([3.2, 2.8, 0.785])]

straight = PathGeneratorWithHold(dt=DT_CTRL)
time_lag_paths = {
    "Dana's waypoints 5 s": straight.generate_waypoints_sequence(WAYPOINTS, segment_duration=5.0, segment_hold=0.0),
    "Dana's waypoints 3.5 s": straight.generate_waypoints_sequence(WAYPOINTS, segment_duration=3.5, segment_hold=0.0),
    "Move 8 s": straight.generate_waypoints_sequence([np.zeros(3), np.array([3.0, 3.0, np.pi / 4])], segment_duration=8.0, segment_hold=0.0),
    "Move 4 s": straight.generate_waypoints_sequence([np.zeros(3), np.array([3.0, 3.0, np.pi / 4])], segment_duration=4.0, segment_hold=0.0),
    "Turn 8 s": straight.generate_waypoints_sequence([np.zeros(3), np.array([3.0, 3.0, np.pi])], segment_duration=8.0, segment_hold=0.0),
    "Turn 5 s": straight.generate_waypoints_sequence([np.zeros(3), np.array([3.0, 3.0, np.pi])], segment_duration=5.0, segment_hold=0.0),
    "Eight 0.4 m, 3.4 s": figure_eight_stop_path(0.4, 3.4),
    "Eight 0.4 m, 2.5 s": figure_eight_stop_path(0.4, 2.5),
    "Eight 0.3 m, 3.0 s": figure_eight_stop_path(0.3, 3.0),
    "Eight 0.3 m, 2.2 s": figure_eight_stop_path(0.3, 2.2),
}
time_lag_pairs = {
    "Dana's waypoints": ("Dana's waypoints 5 s", "Dana's waypoints 3.5 s"),
    "Move to (3, 3, pi/4)": ("Move 8 s", "Move 4 s"),
    "Turn to (3, 3, pi)": ("Turn 8 s", "Turn 5 s"),
    "Eight 0.4 m": ("Eight 0.4 m, 3.4 s", "Eight 0.4 m, 2.5 s"),
    "Eight 0.3 m": ("Eight 0.3 m, 3.0 s", "Eight 0.3 m, 2.2 s"),
}
time_lag_results = run_and_grade("Time-lag execution (Henry)", time_lag_paths, TIME_LAG_CONTROLLERS)


# ==============================================================================================================
# Markdown
# ==============================================================================================================
## The grade, the paths, and off path / behind schedule over time


# ==============================================================================================================
# Code
# ==============================================================================================================
plot_grade(time_lag_results, time_lag_pairs, TIME_LAG_CONTROLLERS)
plot_paths(time_lag_results, time_lag_paths, TIME_LAG_CONTROLLERS, columns=2)
plot_off_path_and_time(time_lag_results, time_lag_paths, TIME_LAG_CONTROLLERS, columns=2)
