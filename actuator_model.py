"""
actuator_model.py
=================

Four-wheel velocity actuator model.

This file wraps the individual actuator function from actuator.py and
applies it to all four wheels.

State:

    theta_dot[4]       actual wheel angular velocities [rad/s]

Input:

    theta_dot_cmd[4]   commanded wheel angular velocities [rad/s]

Output:

    theta_dot[4]       updated wheel angular velocities [rad/s]

The actuator model internally integrates the wheel dynamics using RK4.
"""

from __future__ import annotations

import numpy as np

from actuator import actuator_derivatives


# ---------------------------------------------------------------------------
# Four-wheel actuator model
# ---------------------------------------------------------------------------
def derivatives(theta_dot_cmd, theta_dot_stage):
    theta_ddot = np.zeros(4)

    for i in range(4):
        theta_ddot[i] = actuator_derivatives(
            theta_dot_cmd[i],
            theta_dot_stage[i]
        )

    return theta_ddot

def actuator_model(theta_dot_cmd, theta_dot, dt):
    """Advance all four actuators by one timestep using RK4."""

    k1 = derivatives(theta_dot_cmd, theta_dot)
    k2 = derivatives(theta_dot_cmd, theta_dot + 0.5 * dt * k1)
    k3 = derivatives(theta_dot_cmd, theta_dot + 0.5 * dt * k2)
    k4 = derivatives(theta_dot_cmd, theta_dot + dt * k3)

    theta_dot_new = theta_dot + (dt / 6.0) * (
        k1 + 2.0 * k2 + 2.0 * k3 + k4
    )

    theta_ddot = derivatives(theta_dot_cmd, theta_dot_new)

    return theta_dot_new, theta_ddot