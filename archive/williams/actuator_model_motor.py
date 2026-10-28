"""
actuator_model_motor.py
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

from actuator_motor import actuator_derivatives
from robot_constants import IQ_LIMIT


integral_state = np.zeros(4)
iq_state = np.zeros(4)


def reset():
    global integral_state, iq_state

    integral_state = np.zeros(4)
    iq_state = np.zeros(4)


def derivatives(theta_dot_cmd, theta_dot, iq, integral_error, load_torque):
    integral_error_dot = np.zeros(4)
    theta_ddot = np.zeros(4)
    iq_dot = np.zeros(4)

    for i in range(4):
        integral_error_dot[i], theta_ddot[i], _, iq_dot[i] = actuator_derivatives(theta_dot_cmd[i], theta_dot[i], iq[i], integral_error[i], load_torque[i])

    return integral_error_dot, theta_ddot, iq_dot

def limit_current(iq):
    return np.clip(iq, -IQ_LIMIT, IQ_LIMIT)

def actuator_model(theta_dot_cmd, theta_dot, load_torque, dt):
    global integral_state, iq_state

    k1_integral, k1_theta, k1_iq = derivatives(theta_dot_cmd, theta_dot, iq_state, integral_state, load_torque)

    theta_dot_2 = theta_dot + 0.5 * dt * k1_theta
    iq_2 = limit_current(iq_state + 0.5 * dt * k1_iq)
    integral_2 = integral_state + 0.5 * dt * k1_integral
    k2_integral, k2_theta, k2_iq = derivatives(theta_dot_cmd, theta_dot_2, iq_2, integral_2, load_torque)

    theta_dot_3 = theta_dot + 0.5 * dt * k2_theta
    iq_3 = limit_current(iq_state + 0.5 * dt * k2_iq)
    integral_3 = integral_state + 0.5 * dt * k2_integral
    k3_integral, k3_theta, k3_iq = derivatives(theta_dot_cmd, theta_dot_3, iq_3, integral_3, load_torque)

    theta_dot_4 = theta_dot + dt * k3_theta
    iq_4 = limit_current(iq_state + dt * k3_iq)
    integral_4 = integral_state + dt * k3_integral
    k4_integral, k4_theta, k4_iq = derivatives(theta_dot_cmd, theta_dot_4, iq_4, integral_4, load_torque)

    integral_state += (dt / 6.0) * (k1_integral + 2 * k2_integral + 2 * k3_integral + k4_integral)
    iq_state = limit_current(iq_state + (dt / 6.0) * (k1_iq + 2 * k2_iq + 2 * k3_iq + k4_iq))

    theta_dot_new = theta_dot + (dt / 6.0) * (k1_theta + 2 * k2_theta + 2 * k3_theta + k4_theta)

    _, theta_ddot, iq_dot = derivatives(theta_dot_cmd, theta_dot_new, iq_state, integral_state, load_torque)

    return theta_dot_new, theta_ddot, iq_state, iq_dot
