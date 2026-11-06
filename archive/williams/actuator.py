
"""
actuator.py
=================

Velocity actuator model for a wheel motor.

The velocity PI controller generates a current command, which is
limited by IQ_LIMIT. Motor torque is calculated directly from current.
Electrical motor dynamics are not modeled.
"""

from __future__ import annotations

import numpy as np

from robot_constants import VEL_P_GAIN, VEL_I_GAIN
from robot_constants import IQ_LIMIT, MOTOR_KT, WHEEL_INERTIA


def current_command(velocity_error, integral_state):
    iq = VEL_P_GAIN * velocity_error + VEL_I_GAIN * integral_state

    return np.clip(iq, -IQ_LIMIT, IQ_LIMIT)


def actuator_derivatives(theta_dot_cmd, theta_dot, integral_state, load_torque):

    velocity_error = theta_dot_cmd - theta_dot

    iq = current_command(velocity_error, integral_state)

    integral_state_dot = velocity_error

    motor_torque = MOTOR_KT * iq
    
    theta_ddot = (motor_torque + load_torque) / WHEEL_INERTIA # load torque, from d axis force, is already negative
    
    return integral_state_dot, theta_ddot, iq