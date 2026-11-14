# Non-tangential wheels (Henry's, from the continuous-time models)

Where it was: `archive/continuous_time/robot_model_voltage.ipynb` (and `robot_model_current.ipynb`), with the constants in `robot_constants.py`:
- `WHEEL_ALPHA_DEG`: each wheel's axle direction (drive = axle + 90 deg)
- `WHEEL_POSITION_DEG`: where each contact point sits on the circle of radius `WHEEL_ROBOT_DIST`
- equal arrays = tangential (the real robot): drive yaw lever = L, roller lever = 0

Files:
- `robot_constants_excerpt.md`: the constants and their comment
- `wheel_geometry.py`: `_wheel_geometry`, `wheel_directions_body`, `wheel_positions_body`, `wheel_center_velocity_body` (cell f26c71d2)
- `plant_use.py`: where the plant builds its levers from the positions (cell 72c90d84)
- `non_tangential_test.py`: section 4.7 test (front wheels at 30 deg turned to point like 45 deg wheels)
- section 3 markdown (cell fa77d3ae) and 4.7 markdown (toe01) explain it

Not in merged/: Dana's plant builds `positions = Rb * a_hat - [0, cog_y]`, i.e. the position comes from the axle angle (tangential assumed). The controllers hard-code `Rb` as the yaw lever.
