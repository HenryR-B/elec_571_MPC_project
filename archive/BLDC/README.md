# BLDC

The robot's own motor (5010 360 KV BLDC, as a DC-equivalent motor) in the same project structure as `dana-mpc-copy-with-estimator/` (which uses Dana's Pololu 37D 30:1).

| Folder | What it is |
|---|---|
| `voltage_scaled/` | The same five notebooks and classes with the BLDC motor, its driver and battery current limits, and the controller voltages rescaled to mean the same currents. The controllers still command voltage, so it compares directly with the Pololu copy. See its `README.md`. |
| `current_loop_plan/` | The plan for the proper version, where the controllers command current and the driver's current loop sets the voltage, as on the robot. Builds on `../bldc_current_loop/`. |

The COG is kept centred (`cog_y` = 0) in both, to compare with the Pololu copy; the CAD value (10.2 mm forward) comes later.
