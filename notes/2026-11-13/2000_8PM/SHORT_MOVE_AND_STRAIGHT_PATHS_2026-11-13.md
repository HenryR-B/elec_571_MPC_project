# Short move with a long hold, and Dana's waypoint paths straight again (11-13, ~8:20 PM)

## Henry's test: (-1, -1, 45°) in 0.1 s, then a 5 s hold
Dana's generator, the six test controllers, true pose. Script: scratchpad `k/short_move.py`.
- The reference asks for the impossible: 1.41 m in 0.1 s is a peak of 26.5 m/s and 14.7 rad/s. After that the target is simply a 1.4 m step.
- Every controller ends on the target (within 0.6 cm, 45° ±1.5°).
- Heading swing on the way: MPCs 31–35°; PID tuned 64°; **LQR 135°**, and 19 cm off the straight line (MPCs 4–9 cm).
- PID tuned never settles: in the hold its voltage chatters at 2–6 V (current spikes), so it stays above 1 cm/s. The same happens on Dana 8 s ("X"). Not investigated.
- The linearized MPC had 2 solver failures in the jump (8.7 cm off the line, still settles first, 3.28 s).

**Why the LQR swings (tested):** an LQR has no limits of its own. Its large command is clipped wheel by wheel, at 12 V and 5 V per step. Clipping each wheel separately changes the mix of driving and turning, so the robot turns while it drives. As a diagnostic only, the same limits were applied by scaling all four wheels together:

| | Off the line (cm) | Heading swing | Energy |
|---|---|---|---|
| LQR as it is (per-wheel clipping) | 19.0 | 134.6° | 22.9 J |
| LQR, limits scaled together | 6.6 | 44.9° (just the turn to 45°) | 13.9 J |

The MPCs keep every limit inside their optimization, so their command never gets distorted. That is why they are fine. The PID also clips each wheel; its 64° is likely the same cause (not tested).

## Dana's waypoint paths are straight again (Henry: Dana wants straight lines to visualize the data)
- Only the paths she made (her waypoint set):
  - ControllerAnalysis section 1 (her original `PathGenerator_Multi` line)
  - section 5's "Dana's waypoints (no hold)"
  - ModelChecks' path check
  - the TimeLag notebook's "Dana's waypoints"
- `PathGeneratorCurvedWithHold` stays defined in PathGeneration, unused.
- Checked: section 1 and ModelChecks run. Her path needs 70% of top wheel speed (was 91% with the curves).
- Section 1, RMS (cm): PID 13.97, Simple MPC 1.72, LQR 0.43, DOB 1.05, Coulomb 1.72, Linearized 0.84.
