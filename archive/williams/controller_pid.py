"""
controller_pid.py
=================

Robot velocity PID controller.

Inputs:

    velocity_target[3]   desired [vx, vy, omega]
    velocity[3]          actual [vx, vy, omega]
    integral_error[3]    accumulated velocity error

Output:

    velocity_cmd[3]      commanded [vx, vy, omega]
"""

from __future__ import annotations

import numpy as np


def controller_pid(velocity_target, velocity, integral_error, previous_velocity, dt, kp, ki, kd):
    proportional_error = velocity_target - velocity

    integral_error_dot = proportional_error

    derivative_error = -(velocity - previous_velocity) / dt

    velocity_cmd = velocity_target + kp * proportional_error + ki * integral_error + kd * derivative_error

    return velocity_cmd, integral_error_dot