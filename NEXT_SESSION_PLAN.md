# Next session plan: open-loop model (Dana's architecture + Henry's constants)

Written 2026-10-26. Local note, not committed. Start here next time.

## Where things are

| What | Where |
|---|---|
| Dana's original model (unchanged) | `dana-models/RobotSimulationOpenLoopModel.ipynb` |
| Working copy: Dana's model, Henry's constants, math-quadrant axes, 4 switchable fixes, added sections A–D | `RobotSimulationOpenLoopModel_HenryConstants.ipynb` |
| Differences between the two models, with a proposal per row | `open_loop_model_comparison.md` |
| Branch (pushed; last commit `89797de`) | `open-loop-model-robot-constants` |
| Henry's full models (LuGre, load transfer, voltage plant) | `robot_model_voltage.ipynb`, `robot_model_current.ipynb` on `main` |
| Load-transfer / pitch-roll research | `chassis_load_transfer_plan.md` (branch `chassis-load-transfer`) |

How to run: open the working copy and run all; it takes about 2 minutes. **One notebook run at a time**, because two parallel runs pushed the CPU to 95 °C.

## Step 1: collect Dana's answers (before any code change)

Go through `open_loop_model_comparison.md` with Dana and record a decision per row. Already settled:

- [x] **Symbols:** Dana agreed to Henry's convention: θ̇ for wheel speeds, φ for heading, ω for yaw rate.
- [x] **Body axes:** Dana agreed to Henry's axes: x right, y forward.

Still to decide:

- [x] Symbol/axes conversion: done by Claude in the working copy (2026-10-26).
- [ ] **Tell Dana:** her chassis update integrated the body-frame velocity as if the axes did not turn. The copy now integrates the world velocity (`fix_rotating_frame`), and her H is correct in math-quadrant axes, so `fix_geometry` is gone. Her load split takes pitch from a_x, which is sideways in these axes.
- [ ] **Drive friction default:** `'dana'` (chatters 59× per rear wheel while braking), `'slip_velocity'` (fast, 1 toggle), or `'lugre'` (1 toggle, about 28× slower, needs `lugre_dt = 2e-5`).
- [ ] **Roller friction** (`roller_friction`) on by default? It cuts the peak forward acceleration from 4.98 to 4.38 m/s².
- [ ] **Wheel losses** `'smoothed'` by default? It removes the coast-down chatter.
- [ ] **Wheel inertia in the sticking force:** on or off (about 1% effect)?
- [ ] **CoG offset:** on (CAD, 10.2 mm forward) or off (centred)?
- [ ] **Disturbances:** Dana's white noise, Henry's model, or both selectable? Henry's load vibration reaches its ±50% cap at about 2.6 m/s and spins the open-loop robot out (−55° in 1.5 s), so agree on its amplitude (`LOAD_VIBRATION_*` in `robot_constants.py`).
- [ ] **Plant step:** keep 1 ms explicit for Dana's controller work?
- [ ] Do the four fixes stay on by default? (Recommended: yes.)
- [ ] Dana's Test 2 (equal ±2 V strafe) spins the 30/45 robot at −15 rad/s with wheel-4 chatter; this was there before the conversion. Change the test voltages to the strafe's wheel-speed ratios?

## Step 2: apply the agreed changes (working copy only)

Do them in this order, one notebook run after each, and check the listed result before moving on.

1. **Symbol and axes conversion: DONE 2026-10-26.** All numbers matched after relabelling; the only changes were O(dt) heading differences on spinning runs, from the world-frame integration.
   - Rename ω → `theta_dot` (wheel speeds), heading θ → `phi`, yaw rate → `omega`.
   - Switch the body axes to x right, y forward.
   - Update Dana's cells, the plots, and the markdown equations.
   - Check: every verification check in `hc000003` and sections A–D gives the same numbers as before, apart from the axis relabelling. In Henry's axes, forward is +y.
2. **New defaults** from Step 1: set the agreed option defaults in `EXTENDED_DEFAULTS` (or move them into the base class if Dana wants them as standard).
   - Check: the equality cell still passes for whatever counts as "Dana default".
3. **Clean-ups noticed this session:**
   - [ ] `plot_diagnostics` warns "No artists with labels" on empty legends; only call `legend()` when there are handles.
   - [ ] Dana's original plot cells have a redundant `'r-'` format plus a `ls` keyword (UserWarning); drop one.
   - [ ] Energy audit: 2.2% unaccounted. Check how much comes from the wheel-speed snap at re-stick (compare `'dana'` and `'slip_velocity'` friction) and say so in the text.
4. **Merge decision:** once Dana is happy, merge `open-loop-model-robot-constants` into `main`, and decide whether the working copy replaces `dana-models/RobotSimulationOpenLoopModel.ipynb` as the shared model.

## Step 3: then move on to controllers (Dana's plan)

Only after the plant is agreed:

1. **Path generator class:** time-indexed pose/velocity references, such as forward-stop, strafe, figure-eight, and the angled line.
2. **Two-level PID:**
   - Outer loop: pose error → body velocity → inverse kinematics.
   - Inner loops: four wheel-speed PIDs → voltage.
   - Anti-windup reads the plant's applied voltage and current (`plant.last["V_applied"]`).
3. **MPC** on the same plant and paths.

## Open items carried over (not blocking)

- **Current-model PID blow-up:** Henry's `robot_model_current.ipynb` blows up at 1.06 s with disturbances on (yaw rate 3·10⁴ rad/s). Not investigated; parked because the voltage model is the one in use.
- **Current file as FOC:** convert `robot_model_current.ipynb` to current-commanded voltage (FOC current loop inside the driver). Agreed in principle, not started.
- **Henry's LuGre work on the other desktop:** push it and reconcile with `main` (`main` already has LuGre commits `a83b871`, `87d6e17`, `ac01cf5`).
- **Dynamic heave/pitch/roll (Level 2)** in `chassis_load_transfer_plan.md`: needs agreed contact stiffness and damping. Level 1 (quasi-static load transfer) is already in both Henry's model and the working copy.
