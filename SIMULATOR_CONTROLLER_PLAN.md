# Simulator and controller plan

**Status:** plan only, written 2026-11-05. No simulator or controller code exists yet.

**Goal:** compare PID and MPC on the frozen plant, and find out whether a per-wheel friction limit in the MPC reduces wheel slip without losing tracking.

**Due:** December 9, 2026: a paper and a notebook. PID against MPC is the main comparison. Any course topic can be used, so DeePC is not required.

## Who does what

| Part | Knows | Does |
|---|---|---|
| **Simulation** | The true state: pose, chassis velocity, wheel speeds, currents, slip | Applies the voltages and noise, steps the plant, and hands over the measurements |
| **Controller** | Only the measurements: `[X, Y, theta, omega_1..4]`, plus the reference | Chooses the four voltages. Anything else, such as chassis velocity, it has to estimate |

The controller doesn't set the pose or the wheel speeds. It reads them, chooses voltages, and the simulation works out what happens.

## The new file

Everything goes in one new notebook, `RobotSimulatorController.ipynb`, separate from the plant notebook.

- **Plant:** the model stays frozen in `RobotSimulationOpenLoopModel_HenryStyle.ipynb`. The new notebook loads its `constants`, `model` and `timestep` cells by cell id and calls `step`. Nothing is copied, so there is one plant.
- **Style:** the same as `archive/williams/` and the plant notebook: plain functions, short variable names, inputs described in each docstring, one job per section.
- **Not changed:** the plant notebook's physics, including friction values and the roller switch.

## What the tests found

These came from throwaway scripts, not project code. They are one prototype and one noise seed, tracking velocity only, so they guide the plan and are not results.

| Question | Result | What it means for the plan |
|---|---|---|
| Is the plant fast enough? | About 10,000 plant steps per second, 10 times real time | Large experiment sweeps are practical |
| Is a QP fast enough? | OSQP with friction limits: at most 1.4 ms up to 30 prediction steps, against a 20 ms budget | Solve time is not a limit. No CVXPY is needed; `osqp` is already installed |
| How much voltage can a wheel take? | The motor gives 3.91 N per volt above back-EMF. At μ 0.9 and rest, that is 1.26 V on a rear wheel and 1.78 V on a front wheel before it slips | Voltage steps break grip very easily, so this limit matters more than ±12 V |
| Top speed at 12 V | 1.41–1.99 m/s depending on heading | Reference paths must stay below this |
| Does a simple prediction model work? | 200 ms ahead, speed error 0.022 m/s at ±2 V, 0.13 at ±4 V, 0.47 at ±10 V | Good while gripping, poor once slipping. Slip should be prevented, not predicted |
| Do wheel speeds give chassis velocity? | Gripping: 0.003 m/s error. Slipping: up to 1.7 m/s error. Pose difference over 20 ms stays within 0.05 m/s (no sensor noise modelled) | Wheel speeds are always read correctly, but during slip they stop matching the chassis, so the controller's estimate also needs pose |
| Does wheel mismatch detect slip? | Only 0.05–0.1 m/s while the speed error is over 1 m/s | Poor detector, because all wheels can spin together. Compare encoder speed with pose speed instead |
| What PI gains are stable at 50 Hz? | `kp` above about 0.35–0.4 V/(rad/s) limit-cycles at ±12 V, because a slipping wheel responds in 6 ms and the loop runs every 20 ms | PI needs real tuning and feedforward. Lower gains never reached 1 m/s in 1 s |
| Does the friction limit help? | MPC with ±12 V only slipped 22–39 % of the time. With a limit of 0.7, it slipped 0 % and still reached 90 % speed in 0.22–0.36 s | Promising, but not yet a fair comparison |
| How much margin does the limit need? | With a limit of 0.9 on a plant at 0.9, heading 0 still slipped 21 % of the time | Plan for a margin, and test it |

## Sections of the notebook

**1. Plant loader**
- `load_plant(mu_d=None, cog=None)`: runs the plant cells and overrides a constant for sensitivity runs.

**2. Reference paths**
- `reference(t)` returns `[X, Y, theta, x_dot, y_dot, theta_dot]`, global frame.
- Based on Dana's `sample_paths` in `dana-models/robot22102026D.ipynb`: forward and stop, sine, angled and semicircle. Add a hard-stop path and a path that accelerates near heading 215°, the weakest direction.
- `validate_reference`: rejects a path above the top speed for its heading.

**3. Prediction model** (used only by the MPC)
- No slip, with the motor treated as quasi-static:
  - `(M + J/r² HᵀH) a = c Hᵀ (V - Kb/r H v)`, with `c = Kt/(R r)`
  - Slowest time constant 62 ms, fastest 15 ms.
- `discretize(dt)`: exact zero-order hold at the controller rate.
- Rotations between frames follow the reference heading over the horizon, so each QP stays linear.

**4. Measurements and estimator**
- Simulation side: `measured_outputs(state)` gives `[X, Y, theta, omega_1..4]` only, as you and Dana agreed. Optional sensor noise and a one-sample delay are off by default, and their values would be assumptions until measured.
- Controller side: `estimate_velocity(...)` uses wheel speeds while they agree with pose, and pose difference when they don't.

**5. Simulator**
- `run_experiment(controller, reference, plant, seed)`: the only loop in the file.
- Each pass: measure, ask the controller, hold the command for `controller.hold_steps` plant steps, draw noise, step the plant, record.
- Records: time, reference, pose, true velocity, wheel speeds, **currents**, voltages, slip, load torque and solve time.

**6. Controllers**

All have the same interface, `controller(t, measurement, reference)`, returning four voltages clipped to ±12 V.

- `make_pid(...)`: pose error → chassis velocity → wheel speed targets → wheel PI with back-EMF feedforward (`V = Kb * omega_target + PI`) → voltage. An acceleration limit on the targets is optional.
- `make_mpc(mu_limit=None, ...)`: state is pose and chassis velocity, input is voltage. The cost covers tracking error and voltage changes. With `mu_limit=None`, only ±12 V applies.
- Friction limit in the MPC: `|F_i| ≤ μ_limit (N0_i + G_i a)`.
  - `F_i` and `a` are linear in the state and voltage, so it stays a QP.
  - The loads use the equal-stiffness split, which differs slightly from the plant's formula. That is deliberate: the controller shouldn't have a perfect copy of the plant.
- If a solve fails: hold the last command and count it.

**7. Metrics and plots**
- Tracking: RMS and largest pose error, final error, time to settle.
- Slip: percent of time above 0.05 m/s, and the peak.
- Effort: peak and RMS current per wheel, `I²R` energy, and how much the voltage changes between commands.
- Solver: mean and largest solve time, and failures.
- Currents are reported as MPC relative to PID, or as torque, because amps won't carry over to the BLDC.

**8. Experiments**
- One cell per experiment below, so each shows what it runs.

## Build order

Each step has a check, and the next step waits until it passes.

| Step | Build | It passes when |
|---|---|---|
| 1 | Plant loader and simulator, with an open-loop controller | It reproduces the plant notebook's Block 2 run exactly |
| 2 | Reference paths | Each path is continuous and stays under the top speed |
| 3 | Prediction model | Errors match the table above at ±2 V |
| 4 | PID, tuned with one written procedure on one path, then gains frozen | It tracks a gentle path with no limit cycle and little slip, including with the plant's noise on |
| 5 | MPC with ±12 V only | With a gentle path it matches or beats PID, without solve failures |
| 6 | Estimator | Velocity error stays small during a forced slip |
| 7 | MPC with friction limit | Hard paths slip less than step 5, with tracking reported either way |
| 8 | Full comparison | Every run in the grid below finishes and the tables are generated |

**Suggested schedule**

| By | Done |
|---|---|
| Nov 12 | Steps 1–3 |
| Nov 19 | Steps 4–5: a tuned PID and a working MPC |
| Nov 26 | Steps 6–7: estimator and friction limit |
| Dec 2 | Step 8: all runs and tables finished |
| Dec 9 | Paper and cleaned-up notebook handed in |

If time runs short, keep steps 1–5 and 8. They already give the PID against MPC comparison. Step 7 is the part that adds the friction-limit result.

## Experiment grid

All controllers run on the same paths, seeds and voltage limit.

| Setting | Values |
|---|---|
| Controllers | PID, PID with acceleration limit, MPC ±12 V, MPC with friction limit |
| Controller rate | 50 Hz (20 plant steps), one repeat at 40 Hz (25 plant steps), and one PID run at 1 kHz as a best case, since real motor loops run fast |
| Paths | Dana's four at gentle and fast speeds, hard stop, acceleration near 215° |
| Plant μ_d | 0.7, 0.9, 1.1, with the MPC's limit unchanged |
| Friction limit | 0.6, 0.7, 0.8 of the MPC's assumed value, to find the margin |
| Plant CoG | Centred, plus 10.2 mm forward to test a model mismatch |
| Noise | Off, then on with 5 seeds |

**Main result to show:** tracking against slip for each controller. It answers whether the friction limit gives up tracking to stop slip, and how sensitive that is to a wrong μ.

## Limits to state in the report

- **Rollers:** roller friction is off in the frozen plant, so roller drag is not tested.
- **Slip onset:** the plant at μ_d 0.9 slips at about 3.6 m/s² at its weakest heading, above the 2–2.5 m/s² you've observed. The μ_d sweep partly covers this.
- **Motor:** the plant is a DC model. Comparisons between controllers should carry over better than absolute numbers.
- **Sensors:** noise and delay are assumptions until measured.

## Not in this plan

Each of these is an addition only after step 8 is done, and only if there's time before December 9:
- Offset-free MPC.
- DeePC or another course topic as a comparison.
- Choosing the robot's heading to avoid weak directions.

## Settled

- **Controller rate:** build at 50 Hz, as a single setting that's easy to change, and repeat the comparison once at 40 Hz. The rate doesn't need deciding up front.

## Still needed

- **Dana:** she should agree to this plan and the rename of the plant notebook's terms, since you share the plant.
