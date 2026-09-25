#!/usr/bin/env python3
"""
Python plant model based on Williams, Carter, Gallina & Rosati (2002).

Reference:
    R. L. Williams II, B. E. Carter, P. Gallina, and G. Rosati,
    "Dynamic Model with Slip for Wheeled Omni-Directional Robots,"
    IEEE Transactions on Robotics and Automation, 18(3), 285-293, 2002.

This module is the Python reference plant for the MPC project.

State convention
----------------
The continuous state is

    X = [x, y, phi, Vx, Vy, omega, theta_1, ..., theta_N]

where x, y, phi, Vx, Vy, and omega are expressed in the inertial/world frame.

The four wheel-angle states are ordered as:
    theta_1 = front-left
    theta_2 = front-right
    theta_3 = back-right
    theta_4 = back-left

Each theta[i] is the wheel rotation angle used to select the roller/gap
friction sector.

Input convention
----------------
The control input is ordered as

    u = [u_1, u_2, u_3, u_4]
      = [theta_dot_1, theta_dot_2, theta_dot_3, theta_dot_4]

with the same wheel ordering:
    1 = front-left
    2 = front-right
    3 = back-right
    4 = back-left

Positive theta_dot[i] follows the wheel axial direction a_hat[i]. With the
wheel-frame convention used here, the wheel peripheral speed is +rho*u[i] in
d_hat[i], matching Eq. (2)-(3) of Williams et al.

Williams' improved friction model
----------------------------------
For each wheel:

    v_contact[i] = V_G + omega * p[i]
    v_W[i]       = v_contact[i] * d_hat[i] + rho[i] * u[i]
    v_T[i]       = v_contact[i] * a_hat[i]

The friction coefficient is selected from the current wheel angle theta[i]:

    roller sector -> (mu_W_roller, mu_T_roller)
    gap sector    -> (mu_W_gap,    mu_T_gap)

and

    mu(v) = mu_max * (2/pi) * atan(k*v)

The force exerted by the surface on the robot is

    F[i] = -(mg/N) * [mu_W(v_W[i]) d_hat[i] + mu_T(v_T[i]) a_hat[i]]

The body dynamics are

    Vdot_G = (sum F[i]) / m
    omegadot = sum (p[i] *F[i])_z / I

and

    thetadot[i] = u[i].

This is deliberately a simple, explicit implementation of the published
model. There is NO per-roller inertia, catch-up ODE, or rollerOmega array.
The only roller-related state is the wheel angle theta[i] that chooses the
published roller/gap friction regime.

Important project-specific parameters still require measurement
---------------------------------------------------------------
The numerical friction coefficients in the paper belong to the authors'
8-roller wheel and their paper/carpet surfaces. They are useful reference
values, but they are not measurements of the current robot's wheels.
Likewise, roller_fraction and phase_offset are geometry/calibration
parameters for the current wheel and must eventually be measured.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable

import numpy as np


@dataclass(frozen=True)
class Wheel:
    """One wheel in the robot body frame."""

    number: int
    name: str
    alpha_deg: float
    r_dist: float
    rho: float
    n_rollers: int
    roller_fraction: float
    phase_offset: float = 0.0

    def __post_init__(self) -> None:
        if self.number < 1:
            raise ValueError("wheel number must be positive")
        if self.n_rollers < 1:
            raise ValueError("n_rollers must be positive")
        if not 0.0 < self.roller_fraction < 1.0:
            raise ValueError("roller_fraction must be in (0, 1)")
        if self.r_dist <= 0.0 or self.rho <= 0.0:
            raise ValueError("wheel dimensions must be positive")


@dataclass(frozen=True)
class ModelParams:
    mass: float
    inertia: float
    g: float = 9.81
    mu_W_roller: float = 0.25
    mu_T_roller: float = 0.15
    mu_W_gap: float = 0.56
    mu_T_gap: float = 0.56
    k: float = 1000.0

    def __post_init__(self) -> None:
        if self.mass <= 0.0 or self.inertia <= 0.0 or self.g <= 0.0:
            raise ValueError("mass, inertia, and g must be positive")
        if self.k <= 0.0:
            raise ValueError("k must be positive")


# Current robot geometry.
#
# The roller/gap geometry is derived from the measured wheel dimensions:
#   inner roller-to-roller diameter = 31.04270 mm
#   outer roller-to-roller diameter = 59.82900 mm
#   roller diameter = (59.82900 - 31.04270) / 2 = 14.39315 mm
#   roller radius = 7.196575 mm
#
# The 5.59634 mm measurement is the GAP at the roller CENTER, not the floor
# contact gap. Extrapolating from the measured minimum gap gives:
#   floor-contact gap = 5.59634 + (5.59634 - 3.06464) = 8.12804 mm
#
# At the 29.915 mm wheel contact radius, that gap subtends 15.615808 deg of
# each 22.5 deg roller pitch, leaving 6.884192 deg of actual roller contact.
# Therefore the roller fraction is 0.3059640765.
ROLLER_COUNT = 16
WHEEL_CONTACT_RADIUS = 29.915e-3
ROLLER_INNER_DIAMETER = 31.04270e-3
ROLLER_OUTER_DIAMETER = 59.82900e-3
ROLLER_DIAMETER = (ROLLER_OUTER_DIAMETER - ROLLER_INNER_DIAMETER) / 2.0
ROLLER_RADIUS = ROLLER_DIAMETER / 2.0
ROLLER_MIN_GAP = 3.06464e-3
ROLLER_CENTER_GAP = 5.59634e-3
ROLLER_FLOOR_GAP = ROLLER_CENTER_GAP + (ROLLER_CENTER_GAP - ROLLER_MIN_GAP)
ROLLER_PITCH_DEG = 360.0 / ROLLER_COUNT
ROLLER_GAP_ANGLE_DEG = 2.0 * np.degrees(
    np.arcsin(ROLLER_FLOOR_GAP / (2.0 * WHEEL_CONTACT_RADIUS))
)
ROLLER_CONTACT_ANGLE_DEG = ROLLER_PITCH_DEG - ROLLER_GAP_ANGLE_DEG
ROLLER_FRACTION = ROLLER_CONTACT_ANGLE_DEG / ROLLER_PITCH_DEG

# Wheel numbering:
#   1 = front-left
#   2 = front-right
#   3 = back-right
#   4 = back-left
ROBOT_WHEELS = (
    Wheel(1, "front-left", 60.0, 0.076, WHEEL_CONTACT_RADIUS, ROLLER_COUNT, ROLLER_FRACTION),
    Wheel(2, "front-right", -60.0, 0.076, WHEEL_CONTACT_RADIUS, ROLLER_COUNT, ROLLER_FRACTION),
    Wheel(3, "back-right", -135.0, 0.076, WHEEL_CONTACT_RADIUS, ROLLER_COUNT, ROLLER_FRACTION),
    Wheel(4, "back-left", 135.0, 0.076, WHEEL_CONTACT_RADIUS, ROLLER_COUNT, ROLLER_FRACTION),
)

# Williams' measured friction values for carpet (Table I). These are reference
# coefficients only, not current-robot measurements.
PAPER_CARPET_COEFFICIENTS = {
    "mu_W_roller": 0.25,
    "mu_T_roller": 0.15,
    "mu_W_gap": 0.56,
    "mu_T_gap": 0.56,
}

PAPER_PAPER_COEFFICIENTS = {
    "mu_W_roller": 0.26,
    "mu_T_roller": 0.09,
    "mu_W_gap": 0.47,
    "mu_T_gap": 0.47,
}


def rotation_matrix(phi: float) -> np.ndarray:
    """2-D rotation matrix from body coordinates to inertial coordinates."""
    c = np.cos(phi)
    s = np.sin(phi)
    return np.array([[c, -s], 
                     [s, c]], dtype=float)


def wheel_frames(wheels: Iterable[Wheel]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return wheel positions, drive directions, and axial directions in body frame."""
    wheels = tuple(wheels)
    alpha = np.radians([w.alpha_deg for w in wheels])
    dist = np.array([w.r_dist for w in wheels], dtype=float)

    # Project convention:
    #   d_hat = drive direction
    #   a_hat = axial direction
    a_hat = np.stack([np.cos(alpha), np.sin(alpha)], axis=1)
    d_hat = np.stack([-np.sin(alpha), np.cos(alpha)], axis=1)
    p = np.stack([dist * np.cos(alpha), dist * np.sin(alpha)], axis=1)
    return p, d_hat, a_hat
def smooth_friction_coefficient(v_slip: np.ndarray | float, mu_max: float, k: float) -> np.ndarray:
    """Williams Eq. (11), retaining the sign of the sliding velocity."""
    return mu_max * (2.0 / np.pi) * np.arctan(k * np.asarray(v_slip, dtype=float))


def roller_phase(theta: float, wheel: Wheel) -> float:
    """Return theta phase in one roller pitch, in [0, pitch)."""
    pitch = 2.0 * np.pi / wheel.n_rollers
    return np.mod(theta - wheel.phase_offset, pitch)


def is_roller_contact(theta: float, wheel: Wheel) -> bool:
    """Whether the current contact sector is occupied by a roller."""
    pitch = 2.0 * np.pi / wheel.n_rollers
    return roller_phase(theta, wheel) < wheel.roller_fraction * pitch


def wheel_speeds_from_body_command(
    command: np.ndarray | list[float] | tuple[float, ...],
    wheels: Iterable[Wheel] = ROBOT_WHEELS,
) -> np.ndarray:
    """Convert one body-frame joystick velocity command to wheel speeds.

    command is [vx_cmd, vy_cmd] or [vx_cmd, vy_cmd, omega_cmd].

    This is an inverse-kinematics initialization only. The returned wheel
    speeds are intended to be initialized and only held for as long as the simulator commands it. 
    The command is NOT reapplied as feedback at every timestep.

    The sign is chosen so that, if the robot actually reaches the commanded
    body twist, v_W = 0 for every wheel.
    """
    wheels = tuple(wheels)
    command = np.asarray(command, dtype=float)
    if command.shape == (2,):
        vx_cmd, vy_cmd = command
        omega_cmd = 0.0
    elif command.shape == (3,):
        vx_cmd, vy_cmd, omega_cmd = command
    else:
        raise ValueError("command must be [vx, vy] or [vx, vy, omega]")

    v_body = np.array([vx_cmd, vy_cmd], dtype=float)
    p_body, d_body, _ = wheel_frames(wheels)
    contact_body = v_body + np.stack(
        (-omega_cmd * p_body[:, 1], omega_cmd * p_body[:, 0]), axis=1
    )
    v_drive = np.einsum("ij,ij->i", contact_body, d_body)
    rho = np.array([wheel.rho for wheel in wheels], dtype=float)

    # Williams' longitudinal slip is:
    #     v_W = v_drive + rho * theta_dot.
    # Set v_W = 0 to obtain the ideal wheel-speed command.
    return -v_drive / rho

def contact_kinematics(
    state: np.ndarray,
    wheel_speeds: np.ndarray,
    wheels: Iterable[Wheel],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return inertial contact velocities and the two Williams slip speeds.

    Returns
    -------
    v_contact : (N, 2)
        Contact-point velocity excluding the wheel peripheral speed.
    v_W : (N,)
        Longitudinal sliding velocity from Williams Eq. (3).
    v_T : (N,)
        Transverse sliding velocity from Williams Eq. (7).
    theta_dot : (N,)
        Wheel angular velocities, i.e. the input.
    """
    wheels = tuple(wheels)
    wheel_speeds = np.asarray(wheel_speeds, dtype=float)
    if wheel_speeds.shape != (len(wheels),):
        raise ValueError("wheel_speeds must have shape (N_wheels,)")
    if state.shape[0] < 6 + len(wheels):
        raise ValueError("state is too short for the supplied wheel set")

    phi = float(state[2])
    V_world = np.asarray(state[3:5], dtype=float)
    omega = float(state[5])

    p_body, d_body, a_body = wheel_frames(wheels)
    R = rotation_matrix(phi)
    p_world = p_body @ R.T
    d_world = d_body @ R.T
    a_world = a_body @ R.T

    omega_cross_p = np.stack((-omega * p_world[:, 1], omega * p_world[:, 0]), axis=1)
    v_contact = V_world + omega_cross_p

    v_W = np.einsum("ij,ij->i", v_contact, d_world)
    rho = np.array([w.rho for w in wheels], dtype=float)
    v_W = v_W + rho * wheel_speeds
    v_T = np.einsum("ij,ij->i", v_contact, a_world)
    return v_contact, v_W, v_T, wheel_speeds


def wheel_forces(
    state: np.ndarray,
    wheel_speeds: np.ndarray,
    params: ModelParams,
    wheels: Iterable[Wheel] = ROBOT_WHEELS,
) -> dict[str, np.ndarray]:
    """Evaluate Williams' improved friction model for all wheels."""
    wheels = tuple(wheels)
    # State ordering is [x, y, phi, Vx, Vy, omega, theta_1, ..., theta_N]
    # and ROBOT_WHEELS is ordered 1=FL, 2=FR, 3=BR, 4=BL.
    theta = np.asarray(state[6 : 6 + len(wheels)], dtype=float)
    if theta.shape != (len(wheels),):
        raise ValueError("state does not contain the expected wheel-angle states")

    v_contact, v_W, v_T, theta_dot = contact_kinematics(state, wheel_speeds, wheels)

    p_body, d_body, a_body = wheel_frames(wheels)
    R = rotation_matrix(float(state[2]))
    p_world = p_body @ R.T
    d_world = d_body @ R.T
    a_world = a_body @ R.T

    normal_load = params.mass * params.g / len(wheels)
    mu_W = np.empty(len(wheels), dtype=float)
    mu_T = np.empty(len(wheels), dtype=float)
    roller = np.empty(len(wheels), dtype=bool)

    for i, wheel in enumerate(wheels):
        roller[i] = is_roller_contact(theta[i], wheel)
        if roller[i]:
            mu_W[i] = smooth_friction_coefficient(v_W[i], params.mu_W_roller, params.k)
            mu_T[i] = smooth_friction_coefficient(v_T[i], params.mu_T_roller, params.k)
        else:
            mu_W[i] = smooth_friction_coefficient(v_W[i], params.mu_W_gap, params.k)
            mu_T[i] = smooth_friction_coefficient(v_T[i], params.mu_T_gap, params.k)

    F = -normal_load * (mu_W[:, None] * d_world + mu_T[:, None] * a_world)

    tau_z = p_world[:, 0] * F[:, 1] - p_world[:, 1] * F[:, 0]

    return {
        "force": F,
        "torque_z": tau_z,
        "contact_velocity": v_contact,
        "v_W": v_W,
        "v_T": v_T,
        "theta_dot": theta_dot,
        "mu_W": mu_W,
        "mu_T": mu_T,
        "roller_contact": roller,
        "position_world": p_world,
        "d_world": d_world,
        "a_world": a_world,
    }


def body_wrench(force_data: dict[str, np.ndarray]) -> tuple[np.ndarray, float]:
    """Return total inertial force and yaw torque."""
    F = np.asarray(force_data["force"], dtype=float)
    tau = float(np.sum(force_data["torque_z"]))
    return F.sum(axis=0), tau


def continuous_dynamics(
    state: np.ndarray,
    wheel_speeds: np.ndarray,
    params: ModelParams,
    wheels: Iterable[Wheel] = ROBOT_WHEELS,
) -> np.ndarray:
    """Continuous-time plant dynamics suitable for an MPC integrator."""
    wheels = tuple(wheels)
    expected = 6 + len(wheels)
    if state.shape != (expected,):
        raise ValueError(f"state must have shape ({expected},)")

    force_data = wheel_forces(state, wheel_speeds, params, wheels)
    F_world, tau_z = body_wrench(force_data)

    x_dot = np.empty_like(state, dtype=float)
    x_dot[0] = state[3]
    x_dot[1] = state[4]
    x_dot[2] = state[5]
    x_dot[3:5] = F_world / params.mass
    x_dot[5] = tau_z / params.inertia
    x_dot[6:] = np.asarray(wheel_speeds, dtype=float)
    return x_dot


def rk4_step(
    state: np.ndarray,
    wheel_speeds: np.ndarray,
    dt: float,
    params: ModelParams,
    wheels: Iterable[Wheel] = ROBOT_WHEELS,
) -> np.ndarray:
    """One fixed-input fourth-order Runge-Kutta integration step."""
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    f = lambda x: continuous_dynamics(x, wheel_speeds, params, wheels)
    k1 = f(state)
    k2 = f(state + 0.5 * dt * k1)
    k3 = f(state + 0.5 * dt * k2)
    k4 = f(state + dt * k3)
    return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def simulate(
    x0: np.ndarray,
    wheel_speed_fn: Callable[[float], np.ndarray],
    duration: float,
    dt: float,
    params: ModelParams,
    wheels: Iterable[Wheel] = ROBOT_WHEELS,
) -> tuple[np.ndarray, np.ndarray]:
    """Integrate the plant with a user-supplied wheel-speed input function."""
    if duration <= 0.0 or dt <= 0.0:
        raise ValueError("duration and dt must be positive")

    wheels = tuple(wheels)
    n_steps = int(np.floor(duration / dt)) + 1
    t = np.arange(n_steps, dtype=float) * dt
    x = np.empty((n_steps, len(x0)), dtype=float)
    x[0] = np.asarray(x0, dtype=float)

    for k in range(n_steps - 1):
        u = np.asarray(wheel_speed_fn(t[k]), dtype=float)
        if u.shape != (len(wheels),):
            raise ValueError("wheel_speed_fn must return one speed per wheel")
        x[k + 1] = rk4_step(x[k], u, dt, params, wheels)

    return t, x

def simulate_command(
    x0: np.ndarray,
    command_fn: Callable[[float], np.ndarray],
    duration: float,
    dt: float,
    params: ModelParams,
    wheels: Iterable[Wheel] = ROBOT_WHEELS,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Simulate a time-varying body-frame joystick/velocity command.

    command_fn(t) returns [vx_cmd, vy_cmd] or
    [vx_cmd, vy_cmd, omega_cmd].

    At each simulation step, the current high-level command is converted once
    to wheel angular velocities. Those wheel speeds are then held for that
    integration step. A later joystick change therefore changes the wheel
    speeds on the next step.

    There is no feedback from the simulated robot velocity to the command.
    Any lateral or yaw motion produced by the Williams friction model is
    therefore part of the plant response.

    Returns
    -------
    t : (K,)
        Simulation times.
    x : (K, state_dim)
        Plant state history.
    wheel_speed_history : (K-1, N)
        Wheel angular velocities applied during each integration step.
    """
    if duration <= 0.0 or dt <= 0.0:
        raise ValueError("duration and dt must be positive")

    wheels = tuple(wheels)
    n_steps = int(np.floor(duration / dt)) + 1
    t = np.arange(n_steps, dtype=float) * dt
    x = np.empty((n_steps, len(x0)), dtype=float)
    x[0] = np.asarray(x0, dtype=float)
    wheel_speed_history = np.empty((n_steps - 1, len(wheels)), dtype=float)

    for k in range(n_steps - 1):
        command = np.asarray(command_fn(t[k]), dtype=float)
        wheel_speeds = wheel_speeds_from_body_command(command, wheels)
        wheel_speed_history[k] = wheel_speeds
        x[k + 1] = rk4_step(x[k], wheel_speeds, dt, params, wheels)

    return t, x, wheel_speed_history

def make_rest_state(wheels: Iterable[Wheel] = ROBOT_WHEELS) -> np.ndarray:
    """Convenience constructor for a zero state with one angle per wheel."""
    return np.zeros(6 + len(tuple(wheels)), dtype=float)
