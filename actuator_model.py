
"""
actuator_model.py
=================

Four-wheel velocity actuator model.

State:
    theta_dot[4]       actual wheel angular velocities [rad/s]
    integral_state[4]  velocity PI integral states

Inputs:
    theta_dot_cmd[4]   commanded wheel angular velocities [rad/s]
    load_torque[4]     wheel load torques [N m]
    dt                 timestep [s]

Outputs:
    theta_dot_new[4]   updated wheel angular velocities [rad/s]
    theta_ddot[4]      wheel angular accelerations [rad/s^2]
    iq[4]              commanded motor currents [A]

The PI controller updates discretely once per timestep.
Current and load torque are held constant during the timestep.
"""

from __future__ import annotations

import numpy as np

from actuator import actuator_derivatives


integral_state = np.zeros(4)


def reset():
    global integral_state
    integral_state = np.zeros(4)


def derivatives(theta_dot_cmd, theta_dot_stage, integral_stage, load_torque):
    integral_dot = np.zeros(4)
    theta_ddot = np.zeros(4)
    iq = np.zeros(4)

    for i in range(4):
        integral_dot[i], theta_ddot[i], iq[i] = actuator_derivatives(theta_dot_cmd[i], theta_dot_stage[i], integral_stage[i], load_torque[i])

    return integral_dot, theta_ddot, iq


def actuator_model(theta_dot_cmd, theta_dot, load_torque, dt):
    global integral_state

    integral_dot, theta_ddot, iq = derivatives(theta_dot_cmd, theta_dot, integral_state, load_torque)

    theta_dot_new = theta_dot + theta_ddot * dt
    integral_state += integral_dot * dt

    return theta_dot_new, theta_ddot, iq

def compute_trial_iq(theta_dot_cmd, theta_dot, load_torque):
    """
    Calculates iq using the module's internal integral_state, 
    but does NOT modify or advance it.
    """
    # Use actuator_derivatives or your existing PI logic internally
    theta_ddot, integral_dot, iq = actuator_derivatives(theta_dot_cmd, theta_dot, integral_state, load_torque)
    return iq
