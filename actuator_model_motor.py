"""
actuator_model.py
=================

Four-wheel velocity actuator model.

This file wraps the individual actuator functions from actuator.py and
applies them to all four wheels.

State:

    theta_dot[4]       actual wheel angular velocities [rad/s]
    integral_state[4]  PI controller integral states

Inputs:

    theta_dot_cmd[4]   commanded wheel angular velocities [rad/s]
    load_torque[4]     wheel load torques [N*m]

Output:

    theta_dot[4]       updated wheel angular velocities [rad/s]

The actuator model internally integrates the wheel dynamics and PI
integral states using RK4.
"""

from __future__ import annotations

import numpy as np

from actuator import actuator_derivatives


integral_state = np.zeros(4)


def reset():
    global integral_state
    integral_state = np.zeros(4)


# ---------------------------------------------------------------------------
# Four-wheel actuator model
# ---------------------------------------------------------------------------

def actuator_model(theta_dot_cmd, theta_dot, load_torque, dt):
    """Advance all four actuators by one timestep using RK4."""

    global integral_state

    def derivatives(theta_dot_stage, integral_state_stage):
        theta_ddot = np.zeros(4)
        integral_state_dot = np.zeros(4)

        for i in range(4):
            integral_state_dot[i], theta_ddot[i] = actuator_derivatives(theta_dot_cmd[i], theta_dot_stage[i], integral_state_stage[i], load_torque[i])

        return theta_ddot, integral_state_dot

    k1_theta, k1_integral = derivatives(theta_dot, integral_state)

    k2_theta, k2_integral = derivatives(theta_dot + 0.5 * dt * k1_theta, integral_state + 0.5 * dt * k1_integral)

    k3_theta, k3_integral = derivatives(theta_dot + 0.5 * dt * k2_theta, integral_state + 0.5 * dt * k2_integral)

    k4_theta, k4_integral = derivatives(theta_dot + dt * k3_theta, integral_state + dt * k3_integral)

    theta_dot_new = theta_dot + (dt / 6.0) * (k1_theta + 2.0 * k2_theta + 2.0 * k3_theta + k4_theta)

    integral_state = integral_state + (dt / 6.0) * (k1_integral + 2.0 * k2_integral + 2.0 * k3_integral + k4_integral)

    return theta_dot_new