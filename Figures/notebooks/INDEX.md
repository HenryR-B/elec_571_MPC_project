# Notebook figures

Every plot from every committed notebook version, generated from the git history.

- Folders: `<notebook>/<date>_<commit>_<what that commit did>/`. Each commit is one run of the notebook.
- Files: `<cell number>_<notebook section>__<plot title>.png`.
- A plot that did not change between runs is saved once, under the earliest run; "same image in" lists the later runs that produced it.
- `0_latest_<date>_<commit>/` in each notebook folder holds every plot of that notebook's most recent committed run, complete, including plots unchanged from earlier runs.

## current_model

### 2026-10-23_1321 `6131159`: Split the model into voltage-driven and current-driven notebooks

| cell | section | plot | same image in |
|---|---|---|---|
| 19 | 4.2 Simple plant response verification: forward step and stop | [Plant verification: forward step, hold, and stop + World-frame trajectory (cross-coupling visible)](current_model/2026-10-23_1321_6131159_Split-the-model-into-voltage-d/19_4.2-Simple-plant-response-ve__Plant-verification-forward-step-hold-and-stop.png) | e422998 (2026-10-23_1347), da208d1 (2026-10-23_1403) |
| 21 | 4.3 Lateral step response: left → right → stop | [Lateral step response: left -> right -> stop + World-frame trajectory (cross-coupling visible)](current_model/2026-10-23_1321_6131159_Split-the-model-into-voltage-d/21_4.3-Lateral-step-response-le__Lateral-step-response-left-right-stop-+-World.png) | e422998 (2026-10-23_1347), da208d1 (2026-10-23_1403) |
| 23 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](current_model/2026-10-23_1321_6131159_Split-the-model-into-voltage-d/23_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal.png) | e422998 (2026-10-23_1347), da208d1 (2026-10-23_1403) |
| 23 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](current_model/2026-10-23_1321_6131159_Split-the-model-into-voltage-d/23_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal_b.png) | e422998 (2026-10-23_1347), da208d1 (2026-10-23_1403) |
| 23 | 4.4 Additional independent plant verification checks | [Friction ablation: chassis response](current_model/2026-10-23_1321_6131159_Split-the-model-into-voltage-d/23_4.4-Additional-independent-p__Friction-ablation-chassis-response.png) | e422998 (2026-10-23_1347), da208d1 (2026-10-23_1403) |
| 23 | 4.4 Additional independent plant verification checks | [Timestep convergence: displacement + Timestep sensitivity: spurious yaw diagnostic](current_model/2026-10-23_1321_6131159_Split-the-model-into-voltage-d/23_4.4-Additional-independent-p__Timestep-convergence-displacement-+-Timestep.png) | e422998 (2026-10-23_1347), da208d1 (2026-10-23_1403) |
| 27 | Wheel-angle sweep: lateral yaw coupling | [Lateral command: yaw coupling vs wheel angles + World-frame path](current_model/2026-10-23_1321_6131159_Split-the-model-into-voltage-d/27_Wheel-angle-sweep-lateral-ya__Lateral-command-yaw-coupling-vs-wheel-angles.png) | e422998 (2026-10-23_1347), da208d1 (2026-10-23_1403) |
| 29 | 4.5 Where the yaw comes from, and why it does not stop: model diagnosis | [Yaw release: current and voltage motor models](current_model/2026-10-23_1321_6131159_Split-the-model-into-voltage-d/29_4.5-Where-the-yaw-comes-from__Yaw-release-current-and-voltage-motor-models.png) | e422998 (2026-10-23_1347), da208d1 (2026-10-23_1403) |
| 36 | 7. Run the simulation and inspect diagnostics | [Position trajectory + Position tracking error](current_model/2026-10-23_1321_6131159_Split-the-model-into-voltage-d/36_7.-Run-the-simulation-and-in__Position-trajectory-+-Position-tracking-error.png) |  |
| 36 | 7. Run the simulation and inspect diagnostics | [Position trajectory, coloured by speed + Orientation](current_model/2026-10-23_1321_6131159_Split-the-model-into-voltage-d/36_7.-Run-the-simulation-and-in__Position-trajectory-coloured-by-speed-+-Orien.png) |  |

### 2026-10-23_1347 `e422998`: Plant enforces the motor limits; rolling-resistance switch in the notebooks

| cell | section | plot | same image in |
|---|---|---|---|
| 19 | Motor saturation (Dana's requirement) | [Torque-speed envelope + Free wheel at maximum command](current_model/2026-10-23_1347_e422998_Plant-enforces-the-motor-limit/19_Motor-saturation-Dana-s-requ__Torque-speed-envelope-+-Free-wheel-at-maximum.png) | da208d1 (2026-10-23_1403) |
| 38 | 7. Run the simulation and inspect diagnostics | [Position trajectory + Position tracking error](current_model/2026-10-23_1347_e422998_Plant-enforces-the-motor-limit/38_7.-Run-the-simulation-and-in__Position-trajectory-+-Position-tracking-error.png) |  |
| 38 | 7. Run the simulation and inspect diagnostics | [Position trajectory, coloured by speed + Orientation](current_model/2026-10-23_1347_e422998_Plant-enforces-the-motor-limit/38_7.-Run-the-simulation-and-in__Position-trajectory-coloured-by-speed-+-Orien.png) |  |

### 2026-10-23_1744 `b39e5a7`: Wheel loads from the CoG, full mass matrix, non-tangential wheels, nominal-model PID

| cell | section | plot | same image in |
|---|---|---|---|
| 19 | Motor saturation (Dana's requirement) | [Torque-speed envelope + Free wheel at maximum command](current_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/19_Motor-saturation-Dana-s-requ__Torque-speed-envelope-+-Free-wheel-at-maximum.png) | 99e772e (2026-10-23_1801) |
| 21 | 4.2 Simple plant response verification: forward step and stop | [Plant verification: forward step, hold, and stop + World-frame trajectory (cross-coupling visible)](current_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/21_4.2-Simple-plant-response-ve__Plant-verification-forward-step-hold-and-stop.png) | 99e772e (2026-10-23_1801) |
| 23 | 4.3 Lateral step response: left → right → stop | [Lateral step response: left -> right -> stop + World-frame trajectory (cross-coupling visible)](current_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/23_4.3-Lateral-step-response-le__Lateral-step-response-left-right-stop-+-World.png) | 99e772e (2026-10-23_1801) |
| 25 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](current_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/25_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal.png) | 99e772e (2026-10-23_1801) |
| 25 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](current_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/25_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal_b.png) | 99e772e (2026-10-23_1801) |
| 25 | 4.4 Additional independent plant verification checks | [Friction ablation: chassis response](current_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/25_4.4-Additional-independent-p__Friction-ablation-chassis-response.png) | 99e772e (2026-10-23_1801) |
| 25 | 4.4 Additional independent plant verification checks | [Timestep convergence: displacement + Timestep sensitivity: spurious yaw diagnostic](current_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/25_4.4-Additional-independent-p__Timestep-convergence-displacement-+-Timestep.png) | 99e772e (2026-10-23_1801), 4443f7e (2026-10-23_1816), 25bf646 (2026-10-23_1832) |
| 29 | Wheel-angle sweep: lateral yaw coupling | [Lateral command: yaw coupling vs wheel angles + World-frame path](current_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/29_Wheel-angle-sweep-lateral-ya__Lateral-command-yaw-coupling-vs-wheel-angles.png) | 99e772e (2026-10-23_1801) |
| 31 | Open-loop drift with the friction disturbance (Dana's question) | [Open-loop forward path + Heading drift](current_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/31_Open-loop-drift-with-the-fri__Open-loop-forward-path-+-Heading-drift.png) | 99e772e (2026-10-23_1801) |
| 33 | 4.5 Where the yaw comes from, and why it does not stop: model diagnosis | [Yaw release: current and voltage motor models](current_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/33_4.5-Where-the-yaw-comes-from__Yaw-release-current-and-voltage-motor-models.png) | 99e772e (2026-10-23_1801), 4443f7e (2026-10-23_1816), 25bf646 (2026-10-23_1832) |
| 35 | 4.6 Wheel loads from the centre of gravity | [Wheel loads at rest + Hard forward step: wheel loads](current_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/35_4.6-Wheel-loads-from-the-cen__Wheel-loads-at-rest-+-Hard-forward-step-wheel.png) | 99e772e (2026-10-23_1801) |
| 37 | 4.7 Non-tangential wheels | [Command](current_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/37_4.7-Non-tangential-wheels__Command.png) | 99e772e (2026-10-23_1801) |
| 44 | 7. Run the simulation and inspect diagnostics | [Position trajectory + Position tracking error](current_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/44_7.-Run-the-simulation-and-in__Position-trajectory-+-Position-tracking-error.png) | 99e772e (2026-10-23_1801) |
| 44 | 7. Run the simulation and inspect diagnostics | [Position trajectory, coloured by speed + Orientation](current_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/44_7.-Run-the-simulation-and-in__Position-trajectory-coloured-by-speed-+-Orien.png) | 99e772e (2026-10-23_1801) |

### 2026-10-23_1816 `4443f7e`: Remove the wheel-angle states (theta) from the plant

| cell | section | plot | same image in |
|---|---|---|---|
| 19 | Motor saturation (Dana's requirement) | [Torque-speed envelope + Free wheel at maximum command](current_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/19_Motor-saturation-Dana-s-requ__Torque-speed-envelope-+-Free-wheel-at-maximum.png) | 25bf646 (2026-10-23_1832) |
| 21 | 4.2 Simple plant response verification: forward step and stop | [Plant verification: forward step, hold, and stop + World-frame trajectory (cross-coupling visible)](current_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/21_4.2-Simple-plant-response-ve__Plant-verification-forward-step-hold-and-stop.png) | 25bf646 (2026-10-23_1832) |
| 23 | 4.3 Lateral step response: left → right → stop | [Lateral step response: left -> right -> stop + World-frame trajectory (cross-coupling visible)](current_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/23_4.3-Lateral-step-response-le__Lateral-step-response-left-right-stop-+-World.png) | 25bf646 (2026-10-23_1832) |
| 25 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](current_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/25_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal.png) | 25bf646 (2026-10-23_1832) |
| 25 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](current_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/25_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal_b.png) | 25bf646 (2026-10-23_1832) |
| 25 | 4.4 Additional independent plant verification checks | [Friction ablation: chassis response](current_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/25_4.4-Additional-independent-p__Friction-ablation-chassis-response.png) | 25bf646 (2026-10-23_1832) |
| 29 | Wheel-angle sweep: lateral yaw coupling | [Lateral command: yaw coupling vs wheel angles + World-frame path](current_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/29_Wheel-angle-sweep-lateral-ya__Lateral-command-yaw-coupling-vs-wheel-angles.png) | 25bf646 (2026-10-23_1832) |
| 31 | Open-loop drift with the friction disturbance (Dana's question) | [Open-loop forward path + Heading drift](current_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/31_Open-loop-drift-with-the-fri__Open-loop-forward-path-+-Heading-drift.png) |  |
| 35 | 4.6 Wheel loads from the centre of gravity | [Wheel loads at rest + Hard forward step: wheel loads](current_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/35_4.6-Wheel-loads-from-the-cen__Wheel-loads-at-rest-+-Hard-forward-step-wheel.png) | 25bf646 (2026-10-23_1832), 48b4978 (2026-10-23_1900) |
| 37 | 4.7 Non-tangential wheels | [Command](current_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/37_4.7-Non-tangential-wheels__Command.png) | 25bf646 (2026-10-23_1832), 48b4978 (2026-10-23_1900) |
| 44 | 7. Run the simulation and inspect diagnostics | [Position trajectory + Position tracking error](current_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/44_7.-Run-the-simulation-and-in__Position-trajectory-+-Position-tracking-error.png) |  |
| 44 | 7. Run the simulation and inspect diagnostics | [Position trajectory, coloured by speed + Orientation](current_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/44_7.-Run-the-simulation-and-in__Position-trajectory-coloured-by-speed-+-Orien.png) |  |

### 2026-10-23_1832 `25bf646`: Load vibration: the geometry/speed disturbance now acts on the wheel normal loads

| cell | section | plot | same image in |
|---|---|---|---|
| 31 | Open-loop drift with the friction disturbance (Dana's question) | [Open-loop forward path + Heading drift](current_model/2026-10-23_1832_25bf646_Load-vibration-the-geometry-sp/31_Open-loop-drift-with-the-fri__Open-loop-forward-path-+-Heading-drift.png) |  |
| 39 | 4.8 Wheel-load vibration | [Wheel loads with vibration,  m/s forward + Vibration multiplier vs speed](current_model/2026-10-23_1832_25bf646_Load-vibration-the-geometry-sp/39_4.8-Wheel-load-vibration__Wheel-loads-with-vibration-m-s-forward-+-Vibr.png) | 48b4978 (2026-10-23_1900) |
| 46 | 7. Run the simulation and inspect diagnostics | [Position trajectory + Position tracking error](current_model/2026-10-23_1832_25bf646_Load-vibration-the-geometry-sp/46_7.-Run-the-simulation-and-in__Position-trajectory-+-Position-tracking-error.png) |  |
| 46 | 7. Run the simulation and inspect diagnostics | [Position trajectory, coloured by speed + Orientation](current_model/2026-10-23_1832_25bf646_Load-vibration-the-geometry-sp/46_7.-Run-the-simulation-and-in__Position-trajectory-coloured-by-speed-+-Orien.png) |  |

### 2026-10-23_1900 `48b4978`: Dynamic wheel loads (heave, pitch, roll) on main's model

| cell | section | plot | same image in |
|---|---|---|---|
| 19 | Motor saturation (Dana's requirement) | [Torque-speed envelope + Free wheel at maximum command](current_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/19_Motor-saturation-Dana-s-requ__Torque-speed-envelope-+-Free-wheel-at-maximum.png) |  |
| 21 | 4.2 Simple plant response verification: forward step and stop | [Plant verification: forward step, hold, and stop + World-frame trajectory (cross-coupling visible)](current_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/21_4.2-Simple-plant-response-ve__Plant-verification-forward-step-hold-and-stop.png) |  |
| 23 | 4.3 Lateral step response: left → right → stop | [Lateral step response: left -> right -> stop + World-frame trajectory (cross-coupling visible)](current_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/23_4.3-Lateral-step-response-le__Lateral-step-response-left-right-stop-+-World.png) |  |
| 25 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](current_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/25_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal.png) |  |
| 25 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](current_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/25_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal_b.png) |  |
| 25 | 4.4 Additional independent plant verification checks | [Friction ablation: chassis response](current_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/25_4.4-Additional-independent-p__Friction-ablation-chassis-response.png) |  |
| 25 | 4.4 Additional independent plant verification checks | [Timestep convergence: displacement + Timestep sensitivity: spurious yaw diagnostic](current_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/25_4.4-Additional-independent-p__Timestep-convergence-displacement-+-Timestep.png) |  |
| 29 | Wheel-angle sweep: lateral yaw coupling | [Lateral command: yaw coupling vs wheel angles + World-frame path](current_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/29_Wheel-angle-sweep-lateral-ya__Lateral-command-yaw-coupling-vs-wheel-angles.png) |  |
| 31 | Open-loop drift with the friction disturbance (Dana's question) | [Open-loop forward path + Heading drift](current_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/31_Open-loop-drift-with-the-fri__Open-loop-forward-path-+-Heading-drift.png) |  |
| 33 | 4.5 Where the yaw comes from, and why it does not stop: model diagnosis | [Yaw release: current and voltage motor models](current_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/33_4.5-Where-the-yaw-comes-from__Yaw-release-current-and-voltage-motor-models.png) |  |
| 41 | 4.9 Dynamic wheel loads: heave, pitch, and roll | [Hard forward step: wheel loads + Hard forward step: chassis attitude](current_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/41_4.9-Dynamic-wheel-loads-heav__Hard-forward-step-wheel-loads-+-Hard-forward.png) |  |
| 48 | 7. Run the simulation and inspect diagnostics | [Position trajectory + Position tracking error](current_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/48_7.-Run-the-simulation-and-in__Position-trajectory-+-Position-tracking-error.png) |  |
| 48 | 7. Run the simulation and inspect diagnostics | [Position trajectory, coloured by speed + Orientation](current_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/48_7.-Run-the-simulation-and-in__Position-trajectory-coloured-by-speed-+-Orien.png) |  |

## dana_robot2

### 2026-10-22_0916 `4e95589`: adding comparison

| cell | section | plot | same image in |
|---|---|---|---|
| 1 |  | [Omniwheel Simulation + Robot Global Trajectory (X-Y)](dana_robot2/2026-10-22_0916_4e95589_adding-comparison/01_untitled__Omniwheel-Simulation-+-Robot-Global-Trajector.png) |  |
| 1 |  | [Omniwheel Simulation + Robot Global Trajectory (X-Y)](dana_robot2/2026-10-22_0916_4e95589_adding-comparison/01_untitled__Omniwheel-Simulation-+-Robot-Global-Trajector_b.png) |  |
| 1 |  | [Omniwheel Simulation + Robot Global Trajectory (X-Y)](dana_robot2/2026-10-22_0916_4e95589_adding-comparison/01_untitled__Omniwheel-Simulation-+-Robot-Global-Trajector_b_b.png) |  |
| 3 |  | [4-Omniwheel Closed-Loop Figure-8 Path Tracking Analysis + Global Trajectory (X vs Y)](dana_robot2/2026-10-22_0916_4e95589_adding-comparison/03_untitled__4-Omniwheel-Closed-Loop-Figure-8-Path-Trackin.png) |  |
| 4 |  | [Figure-8 Tracking Performance Across Different Speeds (Periods) + Trajectory Comparison (X vs Y)](dana_robot2/2026-10-22_0916_4e95589_adding-comparison/04_untitled__Figure-8-Tracking-Performance-Across-Differen.png) |  |

## dana_robot22102026D

### 2026-10-22_2335 `8471810`: Create robot22102026D.ipynb

| cell | section | plot | same image in |
|---|---|---|---|
| 16 | 7. Simple plant response verification: forward step and stop | [Plant verification: forward step, hold, and stop + Expected: approximately straight forward travel](dana_robot22102026D/2026-10-22_2335_8471810_Create-robot22102026D.ipynb/16_7.-Simple-plant-response-ver__Plant-verification-forward-step-hold-and-stop.png) |  |
| 18 | 7.1 Lateral step response: left → right → stop | [Lateral step response: left -> right -> stop + World-frame trajectory (cross-coupling visible)](dana_robot22102026D/2026-10-22_2335_8471810_Create-robot22102026D.ipynb/18_7.1-Lateral-step-response-le__Lateral-step-response-left-right-stop-+-World.png) |  |
| 20 | 7.1 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](dana_robot22102026D/2026-10-22_2335_8471810_Create-robot22102026D.ipynb/20_7.1-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal.png) |  |
| 20 | 7.1 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](dana_robot22102026D/2026-10-22_2335_8471810_Create-robot22102026D.ipynb/20_7.1-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal_b.png) |  |
| 20 | 7.1 Additional independent plant verification checks | [Friction ablation: chassis response](dana_robot22102026D/2026-10-22_2335_8471810_Create-robot22102026D.ipynb/20_7.1-Additional-independent-p__Friction-ablation-chassis-response.png) |  |
| 20 | 7.1 Additional independent plant verification checks | [Timestep convergence: displacement + Timestep sensitivity: spurious yaw diagnostic](dana_robot22102026D/2026-10-22_2335_8471810_Create-robot22102026D.ipynb/20_7.1-Additional-independent-p__Timestep-convergence-displacement-+-Timestep.png) |  |
| 25 | 8.1 Reusable sample-path generator for controller benchmarking | [Sample paths for controller benchmarking](dana_robot22102026D/2026-10-22_2335_8471810_Create-robot22102026D.ipynb/25_8.1-Reusable-sample-path-gen__Sample-paths-for-controller-benchmarking.png) |  |
| 29 | 8.2 PID benchmark across sample paths: nominal/disturbed × transverse friction OFF/ON | [Three-axis PID on sample paths: disturbance × transverse-friction ablation + trajectory](dana_robot22102026D/2026-10-22_2335_8471810_Create-robot22102026D.ipynb/29_8.2-PID-benchmark-across-sam__Three-axis-PID-on-sample-paths-disturbance-tr.png) |  |
| 31 | 10. Baseline tracking and physical telemetry | [Dynamic Four-Omniwheel Three-Axis PID Baseline + Global trajectory](dana_robot22102026D/2026-10-22_2335_8471810_Create-robot22102026D.ipynb/31_10.-Baseline-tracking-and-ph__Dynamic-Four-Omniwheel-Three-Axis-PID-Baselin.png) |  |
| 33 | 10.1 Static-to-kinetic friction transition comparison and slip diagnostics | [Three-axis PID: static-to-kinetic friction transition ON vs OFF + Trajectory tracking](dana_robot22102026D/2026-10-22_2335_8471810_Create-robot22102026D.ipynb/33_10.1-Static-to-kinetic-frict__Three-axis-PID-static-to-kinetic-friction-tra.png) |  |
| 33 | 10.1 Static-to-kinetic friction transition comparison and slip diagnostics | [RMS contact traction force (friction enabled in both runs)](dana_robot22102026D/2026-10-22_2335_8471810_Create-robot22102026D.ipynb/33_10.1-Static-to-kinetic-frict__RMS-contact-traction-force-friction-enabled-i.png) |  |
| 42 | 10.4 Optional transverse scrub friction: assumptions and paired sample-path test | [Optional transverse scrub friction on : OFF vs ON + Trajectory](dana_robot22102026D/2026-10-22_2335_8471810_Create-robot22102026D.ipynb/42_10.4-Optional-transverse-scr__Optional-transverse-scrub-friction-on-OFF-vs.png) |  |

## dana_robot2_pid_explained

### 2026-10-22_0916 `4e95589`: adding comparison

| cell | section | plot | same image in |
|---|---|---|---|
| 19 | 9. Baseline tracking and physical telemetry | [Dynamic Four-Omniwheel Three-Axis PID Baseline + Global trajectory](dana_robot2_pid_explained/2026-10-22_0916_4e95589_adding-comparison/19_9.-Baseline-tracking-and-phy__Dynamic-Four-Omniwheel-Three-Axis-PID-Baselin.png) |  |
| 21 | 9.1 Static-to-kinetic friction transition comparison and slip diagnostics | [Three-axis PID: static-to-kinetic friction transition ON vs OFF + Trajectory tracking](dana_robot2_pid_explained/2026-10-22_0916_4e95589_adding-comparison/21_9.1-Static-to-kinetic-fricti__Three-axis-PID-static-to-kinetic-friction-tra.png) |  |
| 21 | 9.1 Static-to-kinetic friction transition comparison and slip diagnostics | [RMS contact traction force (friction enabled in both runs)](dana_robot2_pid_explained/2026-10-22_0916_4e95589_adding-comparison/21_9.1-Static-to-kinetic-fricti__RMS-contact-traction-force-friction-enabled-i.png) |  |

## old_rolling_only_mpc

### 2026-10-22_2016 `a8a4485`: update to model. works.

| cell | section | plot | same image in |
|---|---|---|---|
| 20 | 7. Run the simulation and inspect diagnostics | [Roller-gap diagnostics: pitch=° + phase](old_rolling_only_mpc/2026-10-22_2016_a8a4485_update-to-model.-works/20_7.-Run-the-simulation-and-in__Roller-gap-diagnostics-pitch-+-phase.png) | 9334189 (2026-10-22_2027) |
| 20 | 7. Run the simulation and inspect diagnostics | [Position trajectory + Position tracking error](old_rolling_only_mpc/2026-10-22_2016_a8a4485_update-to-model.-works/20_7.-Run-the-simulation-and-in__Position-trajectory-+-Position-tracking-error.png) | 9334189 (2026-10-22_2027) |

### 2026-10-22_2247 `8060211`: saved it, not sure if i changed anything

| cell | section | plot | same image in |
|---|---|---|---|
| 21 | 7. Run the simulation and inspect diagnostics | [Position trajectory + Position tracking error](old_rolling_only_mpc/2026-10-22_2247_8060211_saved-it-not-sure-if-i-changed/21_7.-Run-the-simulation-and-in__Position-trajectory-+-Position-tracking-error.png) | 3fdec8d (2026-10-23_1202), 5cb101e (2026-10-23_1213), 5a5c6e0 (2026-10-23_1726), f47b86f (2026-10-23_1746) |
| 21 | 7. Run the simulation and inspect diagnostics | [Position trajectory + Position tracking error](old_rolling_only_mpc/2026-10-22_2247_8060211_saved-it-not-sure-if-i-changed/21_7.-Run-the-simulation-and-in__Position-trajectory-+-Position-tracking-error_b.png) | 3fdec8d (2026-10-23_1202), 5cb101e (2026-10-23_1213), 5a5c6e0 (2026-10-23_1726), f47b86f (2026-10-23_1746) |

### 2026-10-23_1202 `3fdec8d`: Dana's open-loop plant suite, yaw diagnosis, and voltage-driven motor

| cell | section | plot | same image in |
|---|---|---|---|
| 20 | Plant sanity checks | [Servo windup at the current limit + PI integrator state](old_rolling_only_mpc/2026-10-23_1202_3fdec8d_Dana-s-open-loop-plant-suite-y/20_Plant-sanity-checks__Servo-windup-at-the-current-limit-+-PI-integr.png) |  |
| 22 | 4.2 Simple plant response verification: forward step and stop | [Plant verification: forward step, hold, and stop + Expected: approximately straight forward travel (world +y)](old_rolling_only_mpc/2026-10-23_1202_3fdec8d_Dana-s-open-loop-plant-suite-y/22_4.2-Simple-plant-response-ve__Plant-verification-forward-step-hold-and-stop.png) |  |
| 24 | 4.3 Lateral step response: left → right → stop | [Lateral step response: left -> right -> stop + World-frame trajectory (cross-coupling visible)](old_rolling_only_mpc/2026-10-23_1202_3fdec8d_Dana-s-open-loop-plant-suite-y/24_4.3-Lateral-step-response-le__Lateral-step-response-left-right-stop-+-World.png) |  |
| 26 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](old_rolling_only_mpc/2026-10-23_1202_3fdec8d_Dana-s-open-loop-plant-suite-y/26_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal.png) |  |
| 26 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](old_rolling_only_mpc/2026-10-23_1202_3fdec8d_Dana-s-open-loop-plant-suite-y/26_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal_b.png) |  |
| 26 | 4.4 Additional independent plant verification checks | [Friction ablation: chassis response](old_rolling_only_mpc/2026-10-23_1202_3fdec8d_Dana-s-open-loop-plant-suite-y/26_4.4-Additional-independent-p__Friction-ablation-chassis-response.png) |  |
| 26 | 4.4 Additional independent plant verification checks | [Timestep convergence: displacement + Timestep sensitivity: spurious yaw diagnostic](old_rolling_only_mpc/2026-10-23_1202_3fdec8d_Dana-s-open-loop-plant-suite-y/26_4.4-Additional-independent-p__Timestep-convergence-displacement-+-Timestep.png) |  |
| 28 | 4.5 Where the yaw comes from, and why it does not stop: model diagnosis | [Voltage-mode yaw release](old_rolling_only_mpc/2026-10-23_1202_3fdec8d_Dana-s-open-loop-plant-suite-y/28_4.5-Where-the-yaw-comes-from__Voltage-mode-yaw-release.png) |  |
| 28 | 4.5 Where the yaw comes from, and why it does not stop: model diagnosis | [Voltage-mode yaw release](old_rolling_only_mpc/2026-10-23_1202_3fdec8d_Dana-s-open-loop-plant-suite-y/28_4.5-Where-the-yaw-comes-from__Voltage-mode-yaw-release_b.png) |  |

### 2026-10-23_1213 `5cb101e`: Assumed wheel losses, Dana-plant comparison, and wheel-direction check

| cell | section | plot | same image in |
|---|---|---|---|
| 20 | Plant sanity checks | [Servo windup at the current limit + PI integrator state](old_rolling_only_mpc/2026-10-23_1213_5cb101e_Assumed-wheel-losses-Dana-plan/20_Plant-sanity-checks__Servo-windup-at-the-current-limit-+-PI-integr.png) | 81120ab (2026-10-23_1237) |
| 22 | 4.2 Simple plant response verification: forward step and stop | [Plant verification: forward step, hold, and stop + Expected: approximately straight forward travel (world +y)](old_rolling_only_mpc/2026-10-23_1213_5cb101e_Assumed-wheel-losses-Dana-plan/22_4.2-Simple-plant-response-ve__Plant-verification-forward-step-hold-and-stop.png) | 81120ab (2026-10-23_1237) |
| 24 | 4.3 Lateral step response: left → right → stop | [Lateral step response: left -> right -> stop + World-frame trajectory (cross-coupling visible)](old_rolling_only_mpc/2026-10-23_1213_5cb101e_Assumed-wheel-losses-Dana-plan/24_4.3-Lateral-step-response-le__Lateral-step-response-left-right-stop-+-World.png) | 81120ab (2026-10-23_1237) |
| 26 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](old_rolling_only_mpc/2026-10-23_1213_5cb101e_Assumed-wheel-losses-Dana-plan/26_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal.png) | 81120ab (2026-10-23_1237) |
| 26 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](old_rolling_only_mpc/2026-10-23_1213_5cb101e_Assumed-wheel-losses-Dana-plan/26_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal_b.png) | 81120ab (2026-10-23_1237) |
| 26 | 4.4 Additional independent plant verification checks | [Friction ablation: chassis response](old_rolling_only_mpc/2026-10-23_1213_5cb101e_Assumed-wheel-losses-Dana-plan/26_4.4-Additional-independent-p__Friction-ablation-chassis-response.png) | 81120ab (2026-10-23_1237) |
| 26 | 4.4 Additional independent plant verification checks | [Timestep convergence: displacement + Timestep sensitivity: spurious yaw diagnostic](old_rolling_only_mpc/2026-10-23_1213_5cb101e_Assumed-wheel-losses-Dana-plan/26_4.4-Additional-independent-p__Timestep-convergence-displacement-+-Timestep.png) | 81120ab (2026-10-23_1237) |
| 30 | 4.5 Where the yaw comes from, and why it does not stop: model diagnosis | [Yaw release: current and voltage motor models](old_rolling_only_mpc/2026-10-23_1213_5cb101e_Assumed-wheel-losses-Dana-plan/30_4.5-Where-the-yaw-comes-from__Yaw-release-current-and-voltage-motor-models.png) | 81120ab (2026-10-23_1237) |

### 2026-10-23_1237 `81120ab`: Run the PID on the voltage-driven plant, with all outputs

| cell | section | plot | same image in |
|---|---|---|---|
| 37 | 7. Run the simulation and inspect diagnostics | [Position trajectory + Position tracking error](old_rolling_only_mpc/2026-10-23_1237_81120ab_Run-the-PID-on-the-voltage-dri/37_7.-Run-the-simulation-and-in__Position-trajectory-+-Position-tracking-error.png) |  |

### 2026-10-23_1305 `a024155`: DT = 10 ms with Radau, driver current-limit fix, new plots and angle sweep

| cell | section | plot | same image in |
|---|---|---|---|
| 20 | Plant sanity checks | [Servo windup at the current limit + PI integrator state](old_rolling_only_mpc/2026-10-23_1305_a024155_DT-10-ms-with-Radau-driver-cur/20_Plant-sanity-checks__Servo-windup-at-the-current-limit-+-PI-integr.png) |  |
| 22 | 4.2 Simple plant response verification: forward step and stop | [Plant verification: forward step, hold, and stop + World-frame trajectory (cross-coupling visible)](old_rolling_only_mpc/2026-10-23_1305_a024155_DT-10-ms-with-Radau-driver-cur/22_4.2-Simple-plant-response-ve__Plant-verification-forward-step-hold-and-stop.png) |  |
| 24 | 4.3 Lateral step response: left → right → stop | [Lateral step response: left -> right -> stop + World-frame trajectory (cross-coupling visible)](old_rolling_only_mpc/2026-10-23_1305_a024155_DT-10-ms-with-Radau-driver-cur/24_4.3-Lateral-step-response-le__Lateral-step-response-left-right-stop-+-World.png) |  |
| 26 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](old_rolling_only_mpc/2026-10-23_1305_a024155_DT-10-ms-with-Radau-driver-cur/26_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal.png) |  |
| 26 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](old_rolling_only_mpc/2026-10-23_1305_a024155_DT-10-ms-with-Radau-driver-cur/26_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal_b.png) |  |
| 26 | 4.4 Additional independent plant verification checks | [Friction ablation: chassis response](old_rolling_only_mpc/2026-10-23_1305_a024155_DT-10-ms-with-Radau-driver-cur/26_4.4-Additional-independent-p__Friction-ablation-chassis-response.png) |  |
| 26 | 4.4 Additional independent plant verification checks | [Timestep convergence: displacement + Timestep sensitivity: spurious yaw diagnostic](old_rolling_only_mpc/2026-10-23_1305_a024155_DT-10-ms-with-Radau-driver-cur/26_4.4-Additional-independent-p__Timestep-convergence-displacement-+-Timestep.png) |  |
| 30 | Wheel-angle sweep: lateral yaw coupling | [Lateral command: yaw coupling vs wheel angles + World-frame path](old_rolling_only_mpc/2026-10-23_1305_a024155_DT-10-ms-with-Radau-driver-cur/30_Wheel-angle-sweep-lateral-ya__Lateral-command-yaw-coupling-vs-wheel-angles.png) |  |
| 32 | 4.5 Where the yaw comes from, and why it does not stop: model diagnosis | [Yaw release: current and voltage motor models](old_rolling_only_mpc/2026-10-23_1305_a024155_DT-10-ms-with-Radau-driver-cur/32_4.5-Where-the-yaw-comes-from__Yaw-release-current-and-voltage-motor-models.png) |  |
| 39 | 7. Run the simulation and inspect diagnostics | [Position trajectory + Position tracking error](old_rolling_only_mpc/2026-10-23_1305_a024155_DT-10-ms-with-Radau-driver-cur/39_7.-Run-the-simulation-and-in__Position-trajectory-+-Position-tracking-error.png) |  |
| 39 | 7. Run the simulation and inspect diagnostics | [Position trajectory, coloured by speed + Orientation](old_rolling_only_mpc/2026-10-23_1305_a024155_DT-10-ms-with-Radau-driver-cur/39_7.-Run-the-simulation-and-in__Position-trajectory-coloured-by-speed-+-Orien.png) |  |

### 2026-10-23_1314 `7df6d4f`: Literature-based rolling resistance estimate with an on/off switch

| cell | section | plot | same image in |
|---|---|---|---|
| 20 | Plant sanity checks | [Servo windup at the current limit + PI integrator state](old_rolling_only_mpc/2026-10-23_1314_7df6d4f_Literature-based-rolling-resis/20_Plant-sanity-checks__Servo-windup-at-the-current-limit-+-PI-integr.png) | 6131159 (2026-10-23_1321) |
| 22 | 4.2 Simple plant response verification: forward step and stop | [Plant verification: forward step, hold, and stop + World-frame trajectory (cross-coupling visible)](old_rolling_only_mpc/2026-10-23_1314_7df6d4f_Literature-based-rolling-resis/22_4.2-Simple-plant-response-ve__Plant-verification-forward-step-hold-and-stop.png) | 6131159 (2026-10-23_1321), e422998 (2026-10-23_1347), da208d1 (2026-10-23_1403) |
| 24 | 4.3 Lateral step response: left → right → stop | [Lateral step response: left -> right -> stop + World-frame trajectory (cross-coupling visible)](old_rolling_only_mpc/2026-10-23_1314_7df6d4f_Literature-based-rolling-resis/24_4.3-Lateral-step-response-le__Lateral-step-response-left-right-stop-+-World.png) | 6131159 (2026-10-23_1321), e422998 (2026-10-23_1347), da208d1 (2026-10-23_1403) |
| 26 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](old_rolling_only_mpc/2026-10-23_1314_7df6d4f_Literature-based-rolling-resis/26_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal.png) | 6131159 (2026-10-23_1321), e422998 (2026-10-23_1347), da208d1 (2026-10-23_1403) |
| 26 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](old_rolling_only_mpc/2026-10-23_1314_7df6d4f_Literature-based-rolling-resis/26_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal_b.png) | 6131159 (2026-10-23_1321), e422998 (2026-10-23_1347), da208d1 (2026-10-23_1403) |
| 26 | 4.4 Additional independent plant verification checks | [Friction ablation: chassis response](old_rolling_only_mpc/2026-10-23_1314_7df6d4f_Literature-based-rolling-resis/26_4.4-Additional-independent-p__Friction-ablation-chassis-response.png) | 6131159 (2026-10-23_1321), e422998 (2026-10-23_1347), da208d1 (2026-10-23_1403) |
| 26 | 4.4 Additional independent plant verification checks | [Timestep convergence: displacement + Timestep sensitivity: spurious yaw diagnostic](old_rolling_only_mpc/2026-10-23_1314_7df6d4f_Literature-based-rolling-resis/26_4.4-Additional-independent-p__Timestep-convergence-displacement-+-Timestep.png) | 6131159 (2026-10-23_1321), e422998 (2026-10-23_1347), da208d1 (2026-10-23_1403) |
| 30 | Wheel-angle sweep: lateral yaw coupling | [Lateral command: yaw coupling vs wheel angles + World-frame path](old_rolling_only_mpc/2026-10-23_1314_7df6d4f_Literature-based-rolling-resis/30_Wheel-angle-sweep-lateral-ya__Lateral-command-yaw-coupling-vs-wheel-angles.png) | 6131159 (2026-10-23_1321), e422998 (2026-10-23_1347), da208d1 (2026-10-23_1403) |
| 32 | 4.5 Where the yaw comes from, and why it does not stop: model diagnosis | [Yaw release: current and voltage motor models](old_rolling_only_mpc/2026-10-23_1314_7df6d4f_Literature-based-rolling-resis/32_4.5-Where-the-yaw-comes-from__Yaw-release-current-and-voltage-motor-models.png) | 6131159 (2026-10-23_1321), e422998 (2026-10-23_1347), da208d1 (2026-10-23_1403) |
| 39 | 7. Run the simulation and inspect diagnostics | [Position trajectory + Position tracking error](old_rolling_only_mpc/2026-10-23_1314_7df6d4f_Literature-based-rolling-resis/39_7.-Run-the-simulation-and-in__Position-trajectory-+-Position-tracking-error.png) | 6131159 (2026-10-23_1321) |
| 39 | 7. Run the simulation and inspect diagnostics | [Position trajectory, coloured by speed + Orientation](old_rolling_only_mpc/2026-10-23_1314_7df6d4f_Literature-based-rolling-resis/39_7.-Run-the-simulation-and-in__Position-trajectory-coloured-by-speed-+-Orien.png) | 6131159 (2026-10-23_1321) |

## voltage_model

### 2026-10-23_1347 `e422998`: Plant enforces the motor limits; rolling-resistance switch in the notebooks

| cell | section | plot | same image in |
|---|---|---|---|
| 21 | Motor saturation (Dana's requirement) | [Torque-speed envelope + Free wheel at maximum command](voltage_model/2026-10-23_1347_e422998_Plant-enforces-the-motor-limit/21_Motor-saturation-Dana-s-requ__Torque-speed-envelope-+-Free-wheel-at-maximum.png) | da208d1 (2026-10-23_1403) |
| 22 | Motor saturation (Dana's requirement) | [Servo windup at the current limit + PI integrator state](voltage_model/2026-10-23_1347_e422998_Plant-enforces-the-motor-limit/22_Motor-saturation-Dana-s-requ__Servo-windup-at-the-current-limit-+-PI-integr.png) | da208d1 (2026-10-23_1403) |
| 41 | 7. Run the simulation and inspect diagnostics | [Position trajectory + Position tracking error](voltage_model/2026-10-23_1347_e422998_Plant-enforces-the-motor-limit/41_7.-Run-the-simulation-and-in__Position-trajectory-+-Position-tracking-error.png) |  |
| 41 | 7. Run the simulation and inspect diagnostics | [Position trajectory, coloured by speed + Orientation](voltage_model/2026-10-23_1347_e422998_Plant-enforces-the-motor-limit/41_7.-Run-the-simulation-and-in__Position-trajectory-coloured-by-speed-+-Orien.png) |  |

### 2026-10-23_1744 `b39e5a7`: Wheel loads from the CoG, full mass matrix, non-tangential wheels, nominal-model PID

| cell | section | plot | same image in |
|---|---|---|---|
| 21 | Motor saturation (Dana's requirement) | [Torque-speed envelope + Free wheel at maximum command](voltage_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/21_Motor-saturation-Dana-s-requ__Torque-speed-envelope-+-Free-wheel-at-maximum.png) | 99e772e (2026-10-23_1801) |
| 22 | Motor saturation (Dana's requirement) | [Servo windup at the current limit + PI integrator state](voltage_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/22_Motor-saturation-Dana-s-requ__Servo-windup-at-the-current-limit-+-PI-integr.png) | 99e772e (2026-10-23_1801) |
| 24 | 4.2 Simple plant response verification: forward step and stop | [Plant verification: forward step, hold, and stop + World-frame trajectory (cross-coupling visible)](voltage_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/24_4.2-Simple-plant-response-ve__Plant-verification-forward-step-hold-and-stop.png) | 99e772e (2026-10-23_1801) |
| 26 | 4.3 Lateral step response: left → right → stop | [Lateral step response: left -> right -> stop + World-frame trajectory (cross-coupling visible)](voltage_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/26_4.3-Lateral-step-response-le__Lateral-step-response-left-right-stop-+-World.png) | 99e772e (2026-10-23_1801) |
| 28 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](voltage_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/28_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal.png) | 99e772e (2026-10-23_1801), 4443f7e (2026-10-23_1816), 25bf646 (2026-10-23_1832) |
| 28 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](voltage_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/28_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal_b.png) | 99e772e (2026-10-23_1801) |
| 28 | 4.4 Additional independent plant verification checks | [Friction ablation: chassis response](voltage_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/28_4.4-Additional-independent-p__Friction-ablation-chassis-response.png) | 99e772e (2026-10-23_1801), 4443f7e (2026-10-23_1816), 25bf646 (2026-10-23_1832) |
| 28 | 4.4 Additional independent plant verification checks | [Timestep convergence: displacement + Timestep sensitivity: spurious yaw diagnostic](voltage_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/28_4.4-Additional-independent-p__Timestep-convergence-displacement-+-Timestep.png) | 99e772e (2026-10-23_1801) |
| 32 | Wheel-angle sweep: lateral yaw coupling | [Lateral command: yaw coupling vs wheel angles + World-frame path](voltage_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/32_Wheel-angle-sweep-lateral-ya__Lateral-command-yaw-coupling-vs-wheel-angles.png) | 99e772e (2026-10-23_1801), 4443f7e (2026-10-23_1816), 25bf646 (2026-10-23_1832) |
| 34 | Open-loop drift with the friction disturbance (Dana's question) | [Open-loop forward path + Heading drift](voltage_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/34_Open-loop-drift-with-the-fri__Open-loop-forward-path-+-Heading-drift.png) | 99e772e (2026-10-23_1801), 4443f7e (2026-10-23_1816) |
| 36 | 4.5 Where the yaw comes from, and why it does not stop: model diagnosis | [Yaw release: current and voltage motor models](voltage_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/36_4.5-Where-the-yaw-comes-from__Yaw-release-current-and-voltage-motor-models.png) | 99e772e (2026-10-23_1801), 4443f7e (2026-10-23_1816), 25bf646 (2026-10-23_1832) |
| 38 | 4.6 Wheel loads from the centre of gravity | [Wheel loads at rest + Hard forward step: wheel loads](voltage_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/38_4.6-Wheel-loads-from-the-cen__Wheel-loads-at-rest-+-Hard-forward-step-wheel.png) | 99e772e (2026-10-23_1801) |
| 40 | 4.7 Non-tangential wheels | [Command](voltage_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/40_4.7-Non-tangential-wheels__Command.png) | 99e772e (2026-10-23_1801), 4443f7e (2026-10-23_1816), 25bf646 (2026-10-23_1832), 48b4978 (2026-10-23_1900) |
| 47 | 7. Run the simulation and inspect diagnostics | [Position trajectory + Position tracking error](voltage_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/47_7.-Run-the-simulation-and-in__Position-trajectory-+-Position-tracking-error.png) | 99e772e (2026-10-23_1801) |
| 47 | 7. Run the simulation and inspect diagnostics | [Position trajectory, coloured by speed + Orientation](voltage_model/2026-10-23_1744_b39e5a7_Wheel-loads-from-the-CoG-full/47_7.-Run-the-simulation-and-in__Position-trajectory-coloured-by-speed-+-Orien.png) | 99e772e (2026-10-23_1801) |

### 2026-10-23_1816 `4443f7e`: Remove the wheel-angle states (theta) from the plant

| cell | section | plot | same image in |
|---|---|---|---|
| 21 | Motor saturation (Dana's requirement) | [Torque-speed envelope + Free wheel at maximum command](voltage_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/21_Motor-saturation-Dana-s-requ__Torque-speed-envelope-+-Free-wheel-at-maximum.png) | 25bf646 (2026-10-23_1832) |
| 22 | Motor saturation (Dana's requirement) | [Servo windup at the current limit + PI integrator state](voltage_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/22_Motor-saturation-Dana-s-requ__Servo-windup-at-the-current-limit-+-PI-integr.png) | 25bf646 (2026-10-23_1832) |
| 24 | 4.2 Simple plant response verification: forward step and stop | [Plant verification: forward step, hold, and stop + World-frame trajectory (cross-coupling visible)](voltage_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/24_4.2-Simple-plant-response-ve__Plant-verification-forward-step-hold-and-stop.png) | 25bf646 (2026-10-23_1832) |
| 26 | 4.3 Lateral step response: left → right → stop | [Lateral step response: left -> right -> stop + World-frame trajectory (cross-coupling visible)](voltage_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/26_4.3-Lateral-step-response-le__Lateral-step-response-left-right-stop-+-World.png) | 25bf646 (2026-10-23_1832) |
| 28 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](voltage_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/28_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal.png) | 25bf646 (2026-10-23_1832) |
| 28 | 4.4 Additional independent plant verification checks | [Timestep convergence: displacement + Timestep sensitivity: spurious yaw diagnostic](voltage_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/28_4.4-Additional-independent-p__Timestep-convergence-displacement-+-Timestep.png) | 25bf646 (2026-10-23_1832) |
| 38 | 4.6 Wheel loads from the centre of gravity | [Wheel loads at rest + Hard forward step: wheel loads](voltage_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/38_4.6-Wheel-loads-from-the-cen__Wheel-loads-at-rest-+-Hard-forward-step-wheel.png) | 25bf646 (2026-10-23_1832), 48b4978 (2026-10-23_1900) |
| 47 | 7. Run the simulation and inspect diagnostics | [Position trajectory + Position tracking error](voltage_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/47_7.-Run-the-simulation-and-in__Position-trajectory-+-Position-tracking-error.png) |  |
| 47 | 7. Run the simulation and inspect diagnostics | [Position trajectory, coloured by speed + Orientation](voltage_model/2026-10-23_1816_4443f7e_Remove-the-wheel-angle-states/47_7.-Run-the-simulation-and-in__Position-trajectory-coloured-by-speed-+-Orien.png) |  |

### 2026-10-23_1832 `25bf646`: Load vibration: the geometry/speed disturbance now acts on the wheel normal loads

| cell | section | plot | same image in |
|---|---|---|---|
| 34 | Open-loop drift with the friction disturbance (Dana's question) | [Open-loop forward path + Heading drift](voltage_model/2026-10-23_1832_25bf646_Load-vibration-the-geometry-sp/34_Open-loop-drift-with-the-fri__Open-loop-forward-path-+-Heading-drift.png) |  |
| 42 | 4.8 Wheel-load vibration | [Wheel loads with vibration,  m/s forward + Vibration multiplier vs speed](voltage_model/2026-10-23_1832_25bf646_Load-vibration-the-geometry-sp/42_4.8-Wheel-load-vibration__Wheel-loads-with-vibration-m-s-forward-+-Vibr.png) | 48b4978 (2026-10-23_1900) |
| 49 | 7. Run the simulation and inspect diagnostics | [Position trajectory + Position tracking error](voltage_model/2026-10-23_1832_25bf646_Load-vibration-the-geometry-sp/49_7.-Run-the-simulation-and-in__Position-trajectory-+-Position-tracking-error.png) |  |
| 49 | 7. Run the simulation and inspect diagnostics | [Position trajectory, coloured by speed + Orientation](voltage_model/2026-10-23_1832_25bf646_Load-vibration-the-geometry-sp/49_7.-Run-the-simulation-and-in__Position-trajectory-coloured-by-speed-+-Orien.png) |  |

### 2026-10-23_1900 `48b4978`: Dynamic wheel loads (heave, pitch, roll) on main's model

| cell | section | plot | same image in |
|---|---|---|---|
| 21 | Motor saturation (Dana's requirement) | [Torque-speed envelope + Free wheel at maximum command](voltage_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/21_Motor-saturation-Dana-s-requ__Torque-speed-envelope-+-Free-wheel-at-maximum.png) |  |
| 22 | Motor saturation (Dana's requirement) | [Servo windup at the current limit + PI integrator state](voltage_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/22_Motor-saturation-Dana-s-requ__Servo-windup-at-the-current-limit-+-PI-integr.png) |  |
| 24 | 4.2 Simple plant response verification: forward step and stop | [Plant verification: forward step, hold, and stop + World-frame trajectory (cross-coupling visible)](voltage_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/24_4.2-Simple-plant-response-ve__Plant-verification-forward-step-hold-and-stop.png) |  |
| 26 | 4.3 Lateral step response: left → right → stop | [Lateral step response: left -> right -> stop + World-frame trajectory (cross-coupling visible)](voltage_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/26_4.3-Lateral-step-response-le__Lateral-step-response-left-right-stop-+-World.png) |  |
| 28 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](voltage_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/28_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal.png) |  |
| 28 | 4.4 Additional independent plant verification checks | [Right -> left -> stop: heading + Yaw-rate reversal](voltage_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/28_4.4-Additional-independent-p__Right-left-stop-heading-+-Yaw-rate-reversal_b.png) |  |
| 28 | 4.4 Additional independent plant verification checks | [Friction ablation: chassis response](voltage_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/28_4.4-Additional-independent-p__Friction-ablation-chassis-response.png) |  |
| 28 | 4.4 Additional independent plant verification checks | [Timestep convergence: displacement + Timestep sensitivity: spurious yaw diagnostic](voltage_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/28_4.4-Additional-independent-p__Timestep-convergence-displacement-+-Timestep.png) |  |
| 32 | Wheel-angle sweep: lateral yaw coupling | [Lateral command: yaw coupling vs wheel angles + World-frame path](voltage_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/32_Wheel-angle-sweep-lateral-ya__Lateral-command-yaw-coupling-vs-wheel-angles.png) |  |
| 34 | Open-loop drift with the friction disturbance (Dana's question) | [Open-loop forward path + Heading drift](voltage_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/34_Open-loop-drift-with-the-fri__Open-loop-forward-path-+-Heading-drift.png) |  |
| 36 | 4.5 Where the yaw comes from, and why it does not stop: model diagnosis | [Yaw release: current and voltage motor models](voltage_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/36_4.5-Where-the-yaw-comes-from__Yaw-release-current-and-voltage-motor-models.png) |  |
| 44 | 4.9 Dynamic wheel loads: heave, pitch, and roll | [Hard forward step: wheel loads + Hard forward step: chassis attitude](voltage_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/44_4.9-Dynamic-wheel-loads-heav__Hard-forward-step-wheel-loads-+-Hard-forward.png) |  |
| 51 | 7. Run the simulation and inspect diagnostics | [Position trajectory + Position tracking error](voltage_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/51_7.-Run-the-simulation-and-in__Position-trajectory-+-Position-tracking-error.png) |  |
| 51 | 7. Run the simulation and inspect diagnostics | [Position trajectory, coloured by speed + Orientation](voltage_model/2026-10-23_1900_48b4978_Dynamic-wheel-loads-heave-pitc/51_7.-Run-the-simulation-and-in__Position-trajectory-coloured-by-speed-+-Orien.png) |  |

