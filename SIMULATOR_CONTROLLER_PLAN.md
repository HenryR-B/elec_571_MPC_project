# Simulator and controller plan

**Status:** plan only, updated 2026-11-06. No simulator or controller code exists yet.

**Due:** December 9, 2026: a paper and a notebook. PID against MPC is the main comparison. Any course topic can be used, so DeePC is not required.

## The question the paper answers

**With the same per-wheel current limit, what does MPC add over PID?**

Both controllers get the same plant, reference paths, measurements, controller rate, voltage limit and current limit. Either answer is a valid result, including "very little on this path".

## The key idea: limit current, not friction

The MPC cannot have a constraint on friction or slip. Current is what it limits instead.

| Candidate constraint | Usable? | Why |
|---|---|---|
| Friction curve or slip law (LuGre, Stribeck) | No | Nonlinear and non-convex |
| Slip itself | No | The MPC's prediction model assumes no slip, so slip is always zero there |
| Wheel speed alone | Doesn't help | It limits top speed, not grip |
| Voltage alone (±12 V) | Too loose | A wheel at rest slips at about 1.3–1.8 V |
| **Current** | **Yes** | Linear in voltage (input) and wheel speed (state), and it sets the motor force |

**The limit for each wheel:**

```text
|V_i - Kb * omega_i| / R  <=  I_max,i
```

- **What it means:** motor current is `(V - Kb*omega)/R`, so this keeps each wheel's force inside its grip.
- **Where friction comes in:** only offline, to choose the limit: `I_max,i = mu_limit * N_i * r / Kt`. Friction is never in the MPC's prediction model.
- **Wheel speed in the limit:** measured for the first prediction step, then `H*v/r` from the prediction model.

**Two versions:**

| Version | `N_i` comes from | Used by |
|---|---|---|
| Constant limit | Static wheel load `N0_i` | PID and MPC |
| Load-transfer limit | `N0_i + G_i * a`, which is linear because `a` is linear in state and input | MPC only |

**Example values** (CoG centred):

| `mu_limit` | Front wheels | Rear wheels |
|---|---|---|
| 0.7 | 0.64 A (1.39 V above back-EMF) | 0.45 A (0.98 V) |
| 0.9 | 0.82 A (1.78 V) | 0.58 A (1.26 V) |

**What the limit does not do:** it doesn't guarantee zero slip. `mu_limit` is an assumption about the floor, so it needs a margin, and the experiments test how much.

## Who does what

| Part | Knows | Does |
|---|---|---|
| **Simulation** | The true state: pose, chassis velocity, wheel speeds, currents, slip | Applies the voltages and noise, steps the plant, and hands over the measurements |
| **Controller** | Only the measurements: `[X, Y, theta, omega_1..4]`, plus the reference | Chooses the four voltages. Anything else, such as chassis velocity, it has to estimate |

The controller doesn't set the pose or the wheel speeds. It reads them, chooses voltages, and the simulation works out what happens. The simulation records true current for the results, but the controllers don't read it. They work out current from voltage and wheel speed.

## The new file

Everything goes in one new notebook, `RobotSimulatorController.ipynb`, separate from the plant notebook.

- **Plant:** the model stays frozen in `RobotSimulationOpenLoopModel_HenryStyle.ipynb`. The new notebook loads its `constants`, `model` and `timestep` cells by cell id and calls `step`. Nothing is copied, so there is one plant.
- **Style:** the same as `archive/williams/` and the plant notebook: plain functions, short variable names, inputs described in each docstring, one job per section.
- **Not changed:** the plant notebook's physics, including friction values and the roller switch.

## What the tests found

These came from throwaway scripts, not project code. They use one prototype and one noise seed, and they track velocity only, not a path. They guide the plan and are not results.

| Question | Result | What it means for the plan |
|---|---|---|
| Is the plant fast enough? | About 10,000 plant steps per second, 10 times real time | Large experiment sweeps are practical |
| Is a QP fast enough? | OSQP with the limits: at most 1.4 ms up to 30 prediction steps, against a 20 ms budget | Solve time is not a limit. No CVXPY is needed; `osqp` is already installed |
| How much voltage can a wheel take? | The motor gives 3.91 N per volt above back-EMF, so a wheel at rest slips at 1.26–1.78 V at μ 0.9 | The current limit matters far more than ±12 V |
| Top speed at 12 V | 1.41–1.99 m/s depending on heading | Reference paths must stay below this |
| Does a simple prediction model work? | 200 ms ahead, speed error 0.022 m/s at ±2 V, 0.13 at ±4 V, 0.47 at ±10 V | Good while gripping, poor once slipping. Slip should be prevented, not predicted |
| Do wheel speeds give chassis velocity? | Gripping: 0.003 m/s error. Slipping: up to 1.7 m/s error. Pose difference over 20 ms stays within 0.05 m/s (no sensor noise modelled) | Wheel speeds are read correctly, but during slip they stop matching the chassis, so the controller's estimate also needs pose |
| Does wheel mismatch detect slip? | Only 0.05–0.1 m/s while the speed error is over 1 m/s | Poor detector, because all wheels can spin together. Compare wheel speed with pose speed instead |
| What PI gains are stable at 50 Hz? | Without feedforward, `kp` above about 0.35–0.4 V/(rad/s) swings between ±12 V | PI needs back-EMF feedforward and real tuning |
| How much margin does the limit need? | On a plant at μ 0.9, a limit of 0.7 did not slip. A limit of 0.9 slipped 21 % of the time on one heading | Test the margin |

**The limit on PI and MPC** (1 m/s step then stop, three headings, `mu_limit` 0.7, PI with feedforward):

| Controller | Reaches 90 % speed in | Time slipping |
|---|---|---|
| PI, no limit | 0.24–0.46 s | 21–40 % |
| **PI, constant limit** | 0.24–0.38 s | **0 %** |
| MPC, ±12 V only | 0.22–0.46 s | 22–39 % |
| MPC, constant limit | 0.22–0.32 s | 0–17 % |
| MPC, load-transfer limit | 0.22–0.36 s | 0 % |

- **The limit stops slip on either controller,** so stopping slip is not by itself an advantage of MPC.
- **MPC's edge is small on this test.** A velocity step can't show looking ahead, which is MPC's main strength.
- **The prototypes differ slightly:** the MPC's limit also counted wheel inertia, and the PI's didn't. The real comparison uses the current limit written above for both.

## Where MPC might do better

These are what the experiments test. None is shown yet.

- **Looking ahead:** braking early for a stop or corner while staying inside the limit.
- **Coordinating wheels:** when one wheel reaches its limit, MPC can adjust the others to hold direction. PID clips each wheel on its own.
- **Load transfer:** a limit that changes with acceleration fits naturally in MPC and is awkward in PID.

## Sections of the notebook

**1. Plant loader**
- `load_plant(mu_d=None, cog=None)`: runs the plant cells and overrides a constant for sensitivity runs.

**2. Reference paths**
- `reference(t)` returns `[X, Y, theta, x_dot, y_dot, theta_dot]`, global frame.
- Based on Dana's `sample_paths` in `dana-models/robot22102026D.ipynb`: forward and stop, sine, angled and semicircle.
- New paths: a hard stop, a sharp corner, and acceleration near heading 215°, the weakest direction.
- `validate_reference`: rejects a path above the top speed for its heading.

**3. Prediction model** (used only by the MPC)
- No slip, with the motor treated as quasi-static:
  - `(M + J/r² HᵀH) a = c Hᵀ (V - Kb/r H v)`, with `c = Kt/(R r)`
  - Slowest time constant 62 ms, fastest 15 ms.
- `discretize(dt)`: exact zero-order hold at the controller rate.
- Rotations between frames follow the reference heading over the horizon, so each QP stays linear.

**4. Measurements and estimator**
- Simulation side: `measured_outputs(state)` gives `[X, Y, theta, omega_1..4]` only, as Henry and Dana agreed. Optional sensor noise and a one-sample delay are off by default, and their values would be assumptions until measured.
- Controller side: `estimate_velocity(...)` uses wheel speeds while they agree with pose, and pose difference when they don't.

**5. Simulator**
- `run_experiment(controller, reference, plant, seed)`: the only loop in the file.
- Each pass: measure, ask the controller, hold the command for `controller.hold_steps` plant steps, draw noise, step the plant, record.
- Records: time, reference, pose, true velocity, wheel speeds, **currents**, voltages, slip, load torque and solve time.

**6. Controllers**

All have the same interface, `controller(t, measurement, reference)`, returning four voltages clipped to ±12 V.

- `current_limit(mu_limit)`: gives `I_max,i` for each wheel from the static loads. PID and MPC both use it.
- `make_pid(i_max=None, ...)`:
  - Pose error → chassis velocity → wheel speed targets → wheel PI with back-EMF feedforward (`V = Kb * omega_target + PI`) → voltage.
  - With `i_max`, the voltage is clipped to `Kb*omega ± R*i_max`, and the integrator stops while clipped.
- `make_mpc(i_max=None, load_transfer=False, ...)`:
  - State is pose and chassis velocity, and input is voltage. The cost covers tracking error and voltage changes.
  - With `i_max=None`, only ±12 V applies.
  - With `i_max`, the current limit above is a linear constraint, so it stays a QP.
  - With `load_transfer=True`, the limit follows `N0_i + G_i * a`. These loads use the equal-stiffness split, which differs slightly from the plant's formula. That is deliberate: the controller shouldn't have a perfect copy of the plant.
  - If a solve fails, it holds the last command and counts the failure.

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
| 3 | Prediction model | Errors match the test table above at ±2 V |
| 4 | PID with and without the constant limit, tuned with one written procedure on one path, then gains frozen | No limit cycle, and the limited version has little slip on a hard path, with the plant's noise on |
| 5 | MPC with ±12 V only | On a gentle path it matches or beats PID, without solve failures |
| 6 | Estimator | Velocity error stays small during a forced slip |
| 7 | MPC with the constant limit, the same as PID's | True current stays close to the limit, and slip is similar to or lower than step 5 on hard paths |
| 8 | MPC with the load-transfer limit | It runs without solve failures on every path |
| 9 | Full comparison | Every run in the grid below finishes and the tables are generated |

**Suggested schedule**

| By | Done |
|---|---|
| Nov 12 | Steps 1–3 |
| Nov 19 | Steps 4–5: tuned PID and a working MPC |
| Nov 26 | Steps 6–8: estimator and both MPC limits |
| Dec 2 | Step 9: all runs and tables finished |
| Dec 9 | Paper and cleaned-up notebook handed in |

**If time runs short, cut in this order:**
1. The load-transfer limit (step 8).
2. The 40 Hz and 1 kHz runs.
3. The estimator (step 6), using wheel speeds only and saying so in the paper.

Steps 1–5, 7 and 9 are the minimum, because they answer the paper's question.

## Experiment grid

All controllers run on the same paths, seeds and voltage limit.

| Setting | Values |
|---|---|
| Controllers | PID no limit, PID constant limit, MPC ±12 V, MPC constant limit, MPC load-transfer limit |
| Controller rate | 50 Hz (20 plant steps), one repeat at 40 Hz (25 plant steps), and one PID run at 1 kHz as a best case, since real motor loops run fast |
| Paths | Dana's four at gentle and fast speeds, hard stop, sharp corner, acceleration near 215° |
| `mu_limit` used to set the current limit | 0.6, 0.7, 0.8, 0.9 |
| Plant μ_d (true floor) | 0.7, 0.9, 1.1, with the controllers' limit unchanged |
| Plant CoG | Centred, plus 10.2 mm forward to test a model mismatch |
| Noise | Off, then on with 5 seeds |

**Main result:** path tracking against slip for each controller on hard stops and corners, under equal current limits. It shows what MPC adds over PID, and how sensitive each is to a wrong μ.

## Limits to state in the paper

- **The limit is not a slip guarantee:** it depends on an assumed μ.
- **Rollers:** roller friction is off in the frozen plant, so roller drag is not tested.
- **Slip onset:** the plant at μ_d 0.9 slips at about 3.6 m/s² at its weakest heading, above the 2–2.5 m/s² Henry has observed. The μ_d sweep partly covers this.
- **Motor:** the plant is a DC model. Comparisons between controllers should carry over better than absolute numbers.
- **Sensors:** noise and delay are assumptions until measured.

## Not in this plan

Each of these is an addition only after step 9 is done, and only if there's time before December 9:
- Offset-free MPC.
- DeePC or another course topic as a comparison.
- Choosing the robot's heading to avoid weak directions.

## Settled

- **Controller rate:** build at 50 Hz, as a single setting that's easy to change, and repeat the comparison once at 40 Hz. The rate doesn't need deciding up front.

## Still needed

- **Prof:** confirm that the load-transfer limit is acceptable as a state and input constraint.
- **Dana:** she should agree to this plan and the rename of the plant notebook's terms, since Henry and Dana share the plant.
