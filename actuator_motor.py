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
from robot_constants import VEL_I_GAIN, VEL_P_GAIN
from robot_constants import VOLTAGE_LIMIT, IQ_LIMIT
from robot_constants import MOTOR_INDUCTANCE, MOTOR_RESISTANCE, MOTOR_KT
from robot_constants import WHEEL_INERTIA


def voltage_command(velocity_error, integral_error):
    voltage = VEL_P_GAIN * velocity_error + VEL_I_GAIN * integral_error

    return np.clip(voltage, -VOLTAGE_LIMIT, VOLTAGE_LIMIT)


def actuator_derivatives(theta_dot_cmd, theta_dot, iq, integral_error, load_torque):
    velocity_error = theta_dot_cmd - theta_dot

    motor_voltage = voltage_command(velocity_error, integral_error)

    iq_dot = (motor_voltage - MOTOR_RESISTANCE * iq - MOTOR_KT * theta_dot) / MOTOR_INDUCTANCE

    if abs(iq) >= IQ_LIMIT and iq * iq_dot > 0:
        motor_voltage = MOTOR_RESISTANCE * iq + MOTOR_KT * theta_dot
        iq_dot = 0.0

    integral_error_dot = velocity_error

    V_unsaturated = VEL_P_GAIN * velocity_error + VEL_I_GAIN * integral_error

    V_saturated = abs(V_unsaturated) >= VOLTAGE_LIMIT
    iq_saturated = abs(iq) >= IQ_LIMIT and iq * velocity_error > 0

    if (V_saturated and velocity_error * V_unsaturated > 0) or iq_saturated:
        integral_error_dot = 0.0

    motor_torque = MOTOR_KT * iq

    theta_ddot = (motor_torque - load_torque) / WHEEL_INERTIA

    return integral_error_dot, theta_ddot, iq, iq_dot


