"""
actuator.py
===========

Velocity actuator model for one wheel.

The complete wheel velocity-control system is modeled as a first-order
velocity response from commanded wheel speed to actual wheel speed.

This is an actuator-level approximation. It does not assume a particular
motor inertia, electrical model, gear efficiency, or motor torque model.
"""

from __future__ import annotations


# ---------------------------------------------------------------------------
# Actuator response
# ---------------------------------------------------------------------------

ACTUATOR_TIME_CONSTANT = 1  # [s] assumed actuator response time


def actuator_derivatives(theta_dot_cmd, theta_dot):
    """Return wheel angular acceleration."""

    theta_ddot = (theta_dot_cmd - theta_dot) / ACTUATOR_TIME_CONSTANT

    return theta_ddot