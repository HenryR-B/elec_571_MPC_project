# Status and open issues (11-13, end of session)

For Henry to review. The change-by-change log is in `MERGE_2026-11-13.md`; this file is where things stand, what I think you want, and the issues I see.

## What is installed in `merged/` now

- The 5 merged notebooks (PlantModels, PathGeneration, ControllerModels, ControllerAnalysis, ModelChecks).
- **ControllerAnalysis has two analyses:**
  1. **Dana's fixed time** (sections 1 to 4, the default): `run_simulation(controller, path)`. Fixed timeline, her 0.5 s hold, a waypoint counts only if settled within its hold. "Who is better in the same time." Her unit-circle sweep ("how far can it reach while still arriving") is unchanged, on her fixed timeline.
  2. **Henry's waypoint-executed run** (section 5): a separate class, `WaypointExecution`, passed as `run_simulation(..., executor=WaypointExecution())`. "Who gets there first."
     - waypoints: the reference waits at each waypoint (no hold) until the robot has settled (3 cm, 1 cm/s, 10°), then starts the next segment
     - figure eights: 7 markers per lap; the reference pauses at each marker until the robot is within 3 cm, then carries on along the eight
     - until a waypoint or marker is reached, the controller sees the reference only up to it
     - the run bottoms out at the scheduled end + `WAYPOINT_TIME_ALLOWANCE` (1 s)
- Dana's waypoint path: right-angle corners (each segment leaves at exactly 90° to the previous arrival), every segment curved like an arc.
- Plots: path plots with an X where a controller was when the path moved on from a waypoint it did not attain; two-panel timing chart (time to settle; where each got to, in cm); close-ups of missed waypoints; marker plots for the eights (`plot_marker_misses` for fixed time, `plot_marker_times` for section 5).
- Turning-test table fixed (8, 5, 4 s; numbers from the ModelChecks path check).

## What I think you want

1. **Two ways to compare controllers, side by side, never one replacing the other:**
   - fixed time (Dana's original): same timeline for everyone, who does best in it;
   - waypoint-executed (yours): the controller tracks the trajectory at its own pace, the waypoints wait for it, and the run bottoms out at a maximum time (the scheduled time of the last waypoint plus an allowance). A slow but accurate controller (the PID) shows as "tracking, but did not get to the last waypoints in time", not as "failed everywhere".
2. **On the figure eights in your version:** the simulation itself follows the eight, driven by the controller (the reference does not run away from a slow controller), with waypoints along the eight that the controller must reach. Not just error measured against a clock, and not a stop-and-go eight.
3. Results that read clearly: who got where and when, not markers that all look the same.

## Open issues

1. **Between markers on the eights, the installed version still runs on the clock.** The reference pauses only at the 7 markers per lap; between them it moves at the eight's lap pace. A slow controller falls behind between markers and cuts across to catch up (seen in the quick check: PID tuned went straight from M5 to the centre on the 0.4 m, 2.5 s eight). If you want the reference never to run away from the controller, this is not it yet.
2. **I tried a "follow distance" rule (not installed):** the reference moves on only while the robot is within 5 cm of it, everywhere on the path. LQR followed well (all 15 markers, done at 6.3 s). **PID tuned nearly stalled** (about 0.7 m in 16 s on Dana's waypoints; 1 of 15 markers on the eight).
   - Why: your tuned PID chases a point and moves at about gain × distance to it (gain 3/s), so a 5 cm leash caps it at about 0.15 m/s, and it slows further as it closes in. The MPCs and LQR get the trajectory's velocity ahead, so they are not capped.
   - So any leash measured as a distance limits the PID by the leash, not by its own ability. Options:
     - a larger leash (20 to 30 cm): PID up to about 0.6 to 0.9 m/s, but on the 0.3 to 0.4 m eights it can cut across a loop;
     - a leash measured along the path (arc length ahead of the robot's nearest point on the path): no corner cutting, same PID speed cap;
     - give the PID the reference velocity (feedforward): changes the PID, which I have not done.
   - Source of that version: `scratchpad/section456.follow_distance_version.py` (not installed; `merged/` = the marker-pause version above).
3. **No full run has completed on the latest installed build.** I stopped several runs as the design changed. The quick checks (PID tuned and LQR only) all ran; the full ControllerAnalysis with all six controllers has not been checked on this build. Run ModelChecks, then ControllerAnalysis, once, before trusting the tables.
4. **Two runs overlapped for a while** (an earlier full run was not stopped while I ran quick checks). Only matters for the CPU/temperature rule; no results were mixed.
5. **The X on the path plots:** you found the X-only timing chart hard to read; that chart is now the two-panel bar version. The X is still on the path plots (where the robot was when the path moved on from a missed waypoint). Say if you want it gone there too.
6. **`HOLD_DURATION`:** 0.5 s again for Dana's fixed-time tests; section 5 uses no hold. Dana's unit-circle sweep sets its own (1 s) at the end of the notebook, which overwrites the global for any cell run after it.

## Decisions for you

- Eights in your version: keep the marker pauses (installed), or make the reference never run away from the controller (issue 2), and if so, which leash.
- Whether to run the full ControllerAnalysis next (about 10+ minutes, one run at a time).
