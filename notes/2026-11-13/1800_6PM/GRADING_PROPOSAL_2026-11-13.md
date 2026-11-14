# How to grade the controllers: research and a proposal (11-13, not implemented)

Henry's problem: on the 2.5 s eight the LQR swings out of the loop and looks terrible, but its numbers (RMS) don't show it.
The time-lag reference was confusing and hard to compare. He wants the speed sweep, shown on one plot.

## What the publications do

1. **Same command path and speed for every controller; vary the speed as a test condition.**
   - ISO 9283 (industrial robot performance tests) has path accuracy, cornering deviation and path velocity criteria. To compare robots, the test paths, velocities, loads and so on must be the same ([ISO 9283:1998](https://iteh.es/catalog/standards/iso/b00c9665-53ff-4265-933d-77a544eb4e26/iso-9283-1998); [1990 preview](https://cdn.standards.iteh.ai/samples/16937/a90ecd3949ee4de7b15f9624bc09cc42/ISO-9283-1990.pdf)).
   - RoboDK's guide to running the ISO 9283 tests says to repeat them at different speeds and compare in one report ([RoboDK path accuracy](https://eu.robodk.com/doc/en/Robot-Validation-ISO9283-Path-Accuracy.html)). I could not check the standard's own velocity levels (it is paywalled).
2. **The closest controller comparison: Sun et al., NMPC vs flatness-based control on a quadrotor** (IEEE T-RO 2022, [arXiv 2109.01365](https://arxiv.org/html/2109.01365v6)).
   - The same elliptical shapes at a grid of top speeds and accelerations (144 trajectories), on a time-indexed reference, the same for both controllers.
   - Results are grouped by how demanding the trajectory is (its maximum acceleration), not by trajectory. That is how they put every shape on one plot.
   - Trajectories beyond the motors' limits are marked "infeasible" and reported separately.
   - Failures (crashes) are counted separately as a crash rate, not mixed into the error.
   - Metric: RMS of position and heading error.
3. **Split the error into "off the path" and "behind on the path".** This comes from CNC and contouring control:
   - **contour error:** the part of the error at right angles to the path (geometry)
   - **lag error:** the part along it (timing)
   - A controller can be good on one and bad on the other, so both are reported ([MPCC overview](https://predictive-control.com/en/design-patterns/path-following-contouring-mpcc/); [precision machining, ETH](https://ar5iv.labs.arxiv.org/html/1908.10609); [UAV MPCC](https://arxiv.org/pdf/2411.00668)).
   - Lam, Manzie and Good's contouring MPC: accuracy and traversal time are a trade-off set by the cost weights ([summary](https://bengravell.github.io/knowledge-base/papers/2010_lam_model_predictive_contouring_control/); [X-Y table](https://findanexpert.unimelb.edu.au/scholarlywork/336235-application-of-model-predictive-contouring-control-to-an-x-y-table)).
   - So a fair test shows both and does not fold them into one number.
4. **Shape match that respects order: the Fréchet distance** (Alt and Godau).
   - It is the shortest "leash" that lets a point on the robot's path and a point on the reference path both go from start to end without going backwards. So it is the worst mismatch in shape.
   - Unlike a nearest-point distance, it can't be fooled when the robot cuts across to another part of the path (e.g. through the middle of the eight, which is close to the other loop).
   - Used to compare robot paths with a reference in [Holladay and Srinivasa](https://personalrobotics.cs.washington.edu/publications//holladay2019frechetgraph.pdf) and [CoNVOI](https://arxiv.org/pdf/2403.15637). An overview recommends pairing it with a timing metric ([emergentmind](https://www.emergentmind.com/topics/trajectory-comparisons)).
5. **Accuracy limit as a tube around the path.** Tube following (Debrouwere et al., 2014) and CNC minimum-time machining set a tolerance band and ask how fast the motion can be inside it ([IFAC 2014](https://skoge.folk.ntnu.no/prost/proceedings/ifac2014/media/files/1672.pdf); [CNC, 2018](https://arodes.hes-so.ch/record/2926/files/Herzog_2018_CNC_machines.pdf?version=1)).
   - In those papers the band is set by the task (part tolerance, a corridor), not chosen by the tester.

None of these lets the test's reference slow down for a controller. In the contouring papers the slowing down is inside the controller (MPCC). That was the confusing part of my time-lag reference: it put a controller's job into the test.

## Why the LQR on the 2.5 s eight didn't show

- RMS averages over the whole run. The LQR is accurate most of the eight and swings out once, so the average stays low (4.0 cm RMS, 16.1 cm max).
- The worst part is what makes it look horrible. The max contour error or the Fréchet distance shows it.

## Proposal

1. **Fixed-time references, as everywhere else.** Every controller gets the same path at the same time (Dana's way, ISO and Sun). No waiting, no reference that reacts to the controller.
2. **The sweep:** each shape (Dana's waypoints, the move, the turn, the two eights) at about 5 total times, from easy to past what the motors can do.
3. **One plot, two panels sharing one x-axis:**
   - **x: how demanding the path is**, the peak wheel speed it needs as % of the motor's top speed at 12 V. It already exists: `peak_wheel_speed_share` in ModelChecks' path check. Like Sun's grouping by maximum acceleration, it puts every shape on one axis. Above 100% the motors cannot do it at all (Sun's "infeasible").
   - **top panel: shape:** the Fréchet distance between the robot's path and the reference path (cm). It shows the LQR's swing-out and the PID's cut-ins, and ignores lag.
   - **bottom panel: timing:** how far behind on the path it was at the scheduled end (cm along the path, or s).
   - one colour per controller, one marker per shape
   - a failure (did not settle at the end within the hold, Dana's rule) is an X, as Sun marks crashes separately
4. **What it answers (Henry's point):**
   - a controller that stays accurate as the time is cut stays low in the top panel further to the right
   - lagging but accurate: low on top, high on the bottom
   - fast but sloppy (the LQR on the fast eight): high on top
5. **Optional single number per controller:** the most demanding path (x) at which it still stayed inside an accuracy band and settled. The band must come from the task, not from me. The only existing number is the 3 cm arrival radius. Henry to decide.

## Open for Henry

- Fréchet (worst shape mismatch) or max contour error for the top panel? Fréchet also catches cutting across a loop, which the max contour error can miss.
- Lag in cm or in s for the bottom panel.
- The accuracy band for the single number (item 5), if wanted.
- Cost: about 5 shapes × 5 times × 6 controllers = 150 runs, roughly 12-15 min, one at a time.
