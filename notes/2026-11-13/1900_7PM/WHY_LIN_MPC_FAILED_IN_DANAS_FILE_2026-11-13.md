# Why Henry's linearized MPC did not work in Dana's analysis (11-13, tested)

Henry asked: was it `compute_optimal_u0`, or the local/global velocity? Tested on a scratch copy of Dana's repo at e974412 (her repo untouched), with her own setup cells, her runner and her weights. Script: scratchpad `dana/why_lin_failed.py`.

## What her record shows
- Her copies of Henry's classes first appear in e974412 (1030d10's analysis does not use them).
- e974412's saved output: "lin henry MPC" ran on her waypoint path, at 1.10 cm RMS, and reached all 3 waypoints. Her simple MPC: 0.85 cm.
- Her unit-circle cell (orientations −180° to 160°) was interrupted, and it calls the runner without the path arguments (TypeError for every controller).
- So nothing in the record shows the linearized MPC crashing. If she saw an error, we need its message. From a clean kernel on this PC, her notebook builds and runs it.

## compute_optimal_u0
Not the cause. Her runner calls `controller.compute_optimal_u0(obs, pos_h, vel_h)`, the same method Henry's runner calls. Her copy is Henry's eb1e35e version, and it builds and solves.

## Local vs global velocity: the cause, on any path that turns
Henry's model's velocity state is global, and the docstring of her copy says the reference velocity must be GLOBAL. Her setup gives it two local velocities:
1. her runner passes `V_LOCAL_REF` (`path_data['pos_local']`, the reference velocity along the robot's axes) as `vel_h`;
2. her `PoseVelocityEstimator` returns the velocity along the robot's axes, which Henry's class uses as x0's velocity.

At heading 0 the two frames are the same. The error grows with the heading.

| Path (her generator, 0.5 s hold, her weights) | As she ran it | Reference global only | x0 global only | Both global | Her Simple MPC |
|---|---|---|---|---|---|
| Her waypoints (headings −34° to 45°) | 1.1 cm RMS, arrives | 1.3 | 0.9 | 0.9 | 0.9 |
| To (3, 3, 90°) in 5 s | 60 cm RMS, 177 cm off at the end, never arrives | | | 1.6, arrives | 1.5 |
| To (3, 3, 180°) in 8 s | 152 cm RMS, 244 cm off at the end, never arrives | 20 cm, never arrives | 12 cm, arrives | 0.9, arrives | 0.9 |
| To (3, 3, 180°) in 5 s | 138 cm RMS, 288 cm off, never arrives | | | 3.3, arrives | 3.1 |

- On her own waypoint path the headings stay small, so it looked about right (1.1 cm). On a turn it runs away (1.5 m RMS, 2–3 m off), which looks like "it doesn't work".
- Both inputs must be global: with only one fixed, it is still 12–20 cm off on the 180° turn.
- With both global, it matches her simple MPC on her paths (as it should: her paths grip, and her weights and horizon 5 are the same for both).
- Her unit-circle sweep turns to every heading, so the linearized MPC would fail there for this reason.
- `merged/` already gives it the global reference and the global estimate (11-13).
