# Next session plan: open-loop model (Dana's architecture + Henry's constants)

Written 2026-10-26. Local note, not committed. Start here next time.

## First, added 2026-10-27: find out what Dana changed

Dana has changed her own copy of the model since our review and has "got parameters for a Pololu motor". Our working copy was meanwhile reverted to her architecture: voltage input only, current input and Simovr driver removed (notes kept in `archive/bldc_current_loop/bldc_current_loop_notes.md`), LuGre, backward Euler for the stiff states, world-frame velocity, design toggle. We need the two models to be similar again.

- [x] **Get Dana's updated notebook (2026-10-28):** `dana-models/robot-simulation-open-loop-v3.ipynb`. Against her original: LuGre (Henry's parameters) with an exponential bristle update, the Pololu 37D 30:1 motor, `V_max` = 12 V and `I_max` = 20 A clips in the model, Henry's axes, our mass/Jzz/h/Rb/r, and new test voltages (Blocks 1–2: 1 V for 0.5 s; Block 3: 10 V; multi-stage still ±4 V).
- [x] **Run both models with her constants** (`tests/compare_dana_v3.py`, noise off): paths agree within 0.5–3.4 mm on the non-spinning tests; 74 mm on the side-to-side and 197 mm on the 10 V strafe, which both spin. Run time about the same (0.6 s vs 0.9 s for all 12 tests).
- [x] **Merged with Dana's v3 (2026-10-28), as agreed with Henry** — change 8 in `dana-models/open_loop_model_changes.md`:
  - from hers: the Pololu 37D 30:1 motor and J_w, the 12 V supply clip (`V_actual`), her tests and test voltages, her text (spec layout, flowchart, architecture, configuration, test descriptions, ASCII frame diagram);
  - kept ours: the backward-Euler wheel–bristle step (hers chatters), the world-frame chassis (hers has the curved-path bug), the load-torque sign, J_zz about the CoG (0.007444), constants from `rc`, the layout figure;
  - CoG: `COG_Y_MM` (0, centred) and `COG_Z_MM` (61.1 mm) as independent parameters; the built/proper toggle is gone;
  - not taken: her 20 A current clip (no motor driver chosen; the Pololu draws up to ~11 A when reversing).
- [ ] **Tell Dana** what was kept from ours and why: her wheel chatter (55–92 % of steps, 62–97 mm/s slip on gripping wheels), the local-velocity chassis bug, her T_load sign, and J_zz.
- [x] **Motor:** Dana's Pololu 37D 30:1 (in the notebook; the gear ratio is inside her K_t, K_b and J_w). Our BLDC values stay in `robot_constants.py` for December.
- [ ] **Controller rate:** Dana pointed out the plant runs at 1 kHz but the controller will run at about 40–50 Hz. So the simulation layer holds each voltage command for 20–25 plant steps (zero-order hold), and the MPC's own prediction model is discretised at 20–25 ms. The plant keeps its 1 ms step for the fast dynamics: the motor current (L/R = 0.55 ms with our motor) and the LuGre bristles.
- [x] **Backward Euler (2026-10-27):** Dana's 1 ms discrete step, with backward Euler instead of Forward Euler for the current, wheel speed and LuGre bristles, which are too stiff for Forward Euler at 1 ms (Dana asked for a discrete LuGre for runtime). Within 0.6–2.6 mm of the same model at 0.1 ms on Dana's tests; the full notebook runs in 12 s. See change 5 in `dana-models/open_loop_model_changes.md`. Tell Dana, and compare with her discrete LuGre when she sends her version.
- [x] **Voltage limit:** the plant clips |V| ≤ 12 V (Dana's). The controller should still respect the motor's traction limit.

The sections below are from 2026-10-26, before these changes, and are partly out of date.

## Where things are

| What | Where |
|---|---|
| Dana's original model (unchanged) | `dana-models/RobotSimulationOpenLoopModel.ipynb` |
| Working copy: Dana's model, Henry's constants, math-quadrant axes, 4 switchable fixes, added sections A–D | `RobotSimulationOpenLoopModel_HenryConstants.ipynb` |
| Differences between the two models, with a proposal per row | `archive/continuous_time/open_loop_model_comparison.md` |
| Branch (pushed; last commit `89797de`) | `open-loop-model-robot-constants` |
| Henry's full models (LuGre, load transfer, voltage plant) | `archive/continuous_time/robot_model_voltage.ipynb`, `robot_model_current.ipynb` |
| Load-transfer / pitch-roll research | `chassis_load_transfer_plan.md` (branch `chassis-load-transfer`) |

How to run: open the working copy and run all; it takes about 2 minutes. **One notebook run at a time**, because two parallel runs pushed the CPU to 95 °C.

