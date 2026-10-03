"""
actuator.py
===========

Velocity actuator model for the wheel motors.

The actuator is modeled as the velocity PI controller used by the
Tinymovr drivetrain, followed by the motor torque constant and wheel
inertia.
"""

from __future__ import annotations
import numpy as np


# ---------------------------------------------------------------------------
# Tinymovr velocity-controller values from The Bots' Rustware
# ---------------------------------------------------------------------------

VEL_P_GAIN = 0.0003
VEL_I_GAIN = 0.025
IQ_LIMIT = 20.0  # [A]

WHEEL_INERTIA = 16.576 * 1e-6       # kg*m^2
WHEEL_MASS = 0.066621               # kg (including rotor, all rotating mass)
MOTOR_KV = 360
MOTOR_KT = 60 / (2 * np.pi * MOTOR_KV)


def current_command(velocity_error, integral_error):
    """Return commanded motor current from the velocity PI controller."""

    iq = VEL_P_GAIN * velocity_error + VEL_I_GAIN * integral_error

    return max(-IQ_LIMIT, min(IQ_LIMIT, iq))


def actuator_derivatives(theta_dot_cmd, theta_dot, integral_error, load_torque):
    """Return integral error rate and wheel angular acceleration."""

    velocity_error = theta_dot_cmd - theta_dot

    integral_state_dot = velocity_error

    iq = current_command(velocity_error, integral_error)

    motor_torque = MOTOR_KT * iq

    theta_ddot = (motor_torque - load_torque) / WHEEL_INERTIA

    return integral_state_dot, theta_ddot