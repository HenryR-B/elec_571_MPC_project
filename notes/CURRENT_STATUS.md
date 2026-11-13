# Current status (read this first, every session)

Last updated: 2026-11-12 (end of day). Keep this file short; details go in the dated folders in `notes/`.

## Where the work is
- Active files: `dana-mpc-copy-with-estimator/` (`ControllerModels_Estimator.ipynb`, `ControllerAnalysis_Linearized.ipynb`, `ControllerAnalysis_Estimator.ipynb`, `PlantModels.ipynb`). Dana's originals: `dana-models/`.
- Change log against Dana's files: `notes/2026-11-11/CHANGES_FROM_DANA_2026-11-11.md` (add each change there).
- **Latest session: `notes/2026-11-12/SESSION_NOTES_2026-11-12.md` (read it first).** Older: `notes/2026-11-10/`, `notes/2026-11-09/`.
- Everything from 11-12 is uncommitted (Henry commits).

## Where we stopped (2026-11-12)
- The 1 ms observers and the linearized MPC use an exact copy of the plant's model (same parameters), and the observers re-linearize every 1 ms. Their good results are an upper bound (perfect model), and the observers are not realistic for the robot. Henry wants the observer like Matt's robot filter (Rustware `common_robot_kalman`, kinematic, no model).
- The main benchmark (`'mpc'`, `'mpc_lin'`, `'pid_tuned'`) still reads a perfect, on-time pose; the camera delay and noise only reach the observer options.

## Next (ask Henry before each; copy reference designs faithfully)
1. Fair test: plant parameters off 10-20% from the controllers'/observers' nominal ones.
2. Faithful port of Matt's 9-state kinematic KF as a new class (open: fill-in of z / a_past / I / w; optical flow).
3. One observer for all controllers, with the camera delay and noise applied to all.
4. Later: LQR + clipping vs MPC on saturated paths; total-current (~15 A) and grip constraints; `a_past` vs direct load solve; held loads every 1 ms (tested 11-12, `held_loads_test.py`); Dana's lumped disturbance observer.

## Environment
- Solver DLLs work again (Smart App Control still On). Python 3.14 at `C:\Users\Henry\AppData\Local\Programs\Python\Python314`; the notebook kernel name "myenv" does not exist on this PC, select that interpreter.
