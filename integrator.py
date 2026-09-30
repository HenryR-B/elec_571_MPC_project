"""
integrator.py
=============

Numerical integration of the robot state using the classical fourth-order
Runge-Kutta method (RK4), since the dynamics equations are non-linear.

For the state equation:

    X_dot = f(X)

the four derivative evaluations are:

    k1 = f(X_n)
    k2 = f(X_n + (dt/2) * k1)
    k3 = f(X_n + (dt/2) * k2)
    k4 = f(X_n + dt * k3)

and the next state is:

    X_(n+1) = X_n + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)

The derivative function is evaluated using the current trial state at each
RK4 stage.
"""

from __future__ import annotations

import numpy as np


def rk4_step(state, dt, derivative):
    """Advance the state by one timestep using fourth-order Runge-Kutta.

    state: current state vector
    dt: timestep [s]
    derivative: function that returns state_dot for a given state
    """
    k1 = derivative(state)
    k2 = derivative(state + 0.5 * dt * k1)
    k3 = derivative(state + 0.5 * dt * k2)
    k4 = derivative(state + dt * k3)

    return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)