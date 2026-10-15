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

Wheel dynamics are integrated using RK4. Current responds
instantaneously to the controller command.
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

    k1_i, k1_w, _ = derivatives(theta_dot_cmd, theta_dot, integral_state, load_torque)
    k2_i, k2_w, _ = derivatives(theta_dot_cmd, theta_dot + 0.5 * dt * k1_w, integral_state + 0.5 * dt * k1_i, load_torque)
    k3_i, k3_w, _ = derivatives(theta_dot_cmd, theta_dot + 0.5 * dt * k2_w, integral_state + 0.5 * dt * k2_i, load_torque)
    k4_i, k4_w, _ = derivatives(theta_dot_cmd, theta_dot + dt * k3_w, integral_state + dt * k3_i, load_torque)

    theta_dot_new = theta_dot + (dt / 6.0) * (k1_w + 2.0 * k2_w + 2.0 * k3_w + k4_w)
    integral_state += (dt / 6.0) * (k1_i + 2.0 * k2_i + 2.0 * k3_i + k4_i)

    _, theta_ddot, iq = derivatives(theta_dot_cmd, theta_dot_new, integral_state, load_torque)

    return theta_dot_new, theta_ddot, iq