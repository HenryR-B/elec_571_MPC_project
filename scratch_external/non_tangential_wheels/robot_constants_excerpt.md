# robot_constants.py, lines 81-95
```python
WHEEL_ALPHA_DEG = [
    0.000 + 30.0,
    180.0 - 30.0,
    180.0 + 45.0,
    360.0 - 45.0
]
# WHEEL_ALPHA_DEG sets each wheel's axle direction (CCW from +x); its drive direction is
# the axle turned +90 deg. WHEEL_POSITION_DEG sets where each wheel's contact point sits
# on the circle of radius WHEEL_ROBOT_DIST. Tangential wheels (the real robot): the two
# arrays are identical, the axle points at the centre, and every yaw lever r_i x d_i = L.
# Different values make a wheel non-tangential: e.g. a wheel at position 30 deg with
# alpha 45 deg drives along the 45-deg wheel's direction from the 30-deg spot.
WHEEL_POSITION_DEG = list(WHEEL_ALPHA_DEG)

# Wheel
```
