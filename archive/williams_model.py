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

    theta_dot = [theta_dot_1, theta_dot_2, theta_dot_3, theta_dot_4]

with the same wheel ordering:
    1 = front-left
    2 = front-right
    3 = back-right
    4 = back-left

Positive theta_dot[i] follows the wheel axial direction a_hat[i]. With the
wheel-frame convention used here, the wheel peripheral speed is +WHEEL_RADIUS * theta_dot[i] in
d_hat[i], matching Eq. (2)-(3) of Williams et al.

Williams' improved friction model
----------------------------------
For each wheel:

    v_contact[i] = V_G + omega × p[i]
    v_W[i]       = v_contact[i] · d_hat[i] + WHEEL_RADIUS * theta_dot[i]
    v_T[i]       = v_contact[i] · a_hat[i]

The dot products above are projections onto the wheel drive and axial
directions, respectively. The cross product in v_contact gives the velocity
from the robot's yaw rate.

The friction coefficient is selected from the current wheel angle theta[i]:

    roller sector -> (mu_W_roller, mu_T_roller)
    gap sector    -> (mu_W_gap,    mu_T_gap)

and

    mu(v) = mu_max * (2/pi) * atan(k * v)

The force exerted by the surface on the robot is

    F[i] = -(m*g/N) * [
        mu_W(v_W[i]) * d_hat[i] + mu_T(v_T[i]) * a_hat[i]
    ]

The body dynamics are

    Vdot_G = (sum F[i]) / m
    omegadot = sum (p[i] × F[i])_z / I

and

    thetadot[i] = theta_dot[i].

This is deliberately a simple, explicit implementation of the published
model. There is NO per-roller inertia, catch-up ODE, or rollerOmega array.
The only roller-related state is the wheel angle theta[i] that chooses the
published roller/gap friction regime.

Important project-specific parameters still require measurement
---------------------------------------------------------------
The numerical friction coefficients in the paper belong to the authors'
8-roller wheel and their paper/carpet surfaces. They are useful reference
values, but they are not measurements of the current robot's wheels.
Likewise, ROLLER_FRACTION and ROLLER_PHASE_OFFSET are geometry/calibration
parameters for the current wheel and must eventually be measured.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable

import numpy as np


@dataclass(frozen=True)
class Wheel:
    """One fixed wheel location and orientation in the robot body frame."""

    number: int
    name: str
    alpha_deg: float


@dataclass(frozen=True)
class ModelParams:
    mass: float
    inertia: float
    mu_W_roller: float = 0.25
    mu_T_roller: float = 0.15
    mu_W_gap: float = 0.56
    mu_T_gap: float = 0.56
    k: float = 1000.0

    def __post_init__(self) -> None:
        if self.mass <= 0.0 or self.inertia <= 0.0:
            raise ValueError("mass and inertia must be positive")
        if self.k <= 0.0:
            raise ValueError("k must be positive")


# Fixed robot geometry.
#
# These are properties of the current robot and do not change during a
# simulation. Their uppercase names make that explicit.
WHEEL_COUNT = 4
WHEEL_NAMES = ("front-left", "front-right", "back-right", "back-left")
WHEEL_ANGLES_DEG = (60.0, -60.0, -135.0, 135.0)
WHEEL_CENTER_DISTANCE = 0.076
WHEEL_RADIUS = 29.915e-3

ROLLER_COUNT = 16
ROLLER_INNER_DIAMETER = 31.04270e-3
ROLLER_OUTER_DIAMETER = 59.82900e-3
ROLLER_DIAMETER = (ROLLER_OUTER_DIAMETER - ROLLER_INNER_DIAMETER) / 2.0
ROLLER_RADIUS = ROLLER_DIAMETER / 2.0
ROLLER_MIN_GAP = 3.06464e-3
ROLLER_CENTER_GAP = 5.59634e-3
ROLLER_FLOOR_GAP = ROLLER_CENTER_GAP + (ROLLER_CENTER_GAP - ROLLER_MIN_GAP)
ROLLER_PITCH_DEG = 360.0 / ROLLER_COUNT
ROLLER_GAP_ANGLE_DEG = 2.0 * np.degrees(
    np.arcsin(ROLLER_FLOOR_GAP / (2.0 * WHEEL_RADIUS))
)
ROLLER_CONTACT_ANGLE_DEG = ROLLER_PITCH_DEG - ROLLER_GAP_ANGLE_DEG
ROLLER_FRACTION = ROLLER_CONTACT_ANGLE_DEG / ROLLER_PITCH_DEG
ROLLER_PHASE_OFFSET = 0.0

GRAVITY = 9.81

ROBOT_WHEELS = tuple(
    Wheel(number, name, angle)
    for number, (name, angle) in enumerate(
        zip(WHEEL_NAMES, WHEEL_ANGLES_DEG), start=1
    )
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


def wheel_frames(
    wheels: Iterable[Wheel] = ROBOT_WHEELS,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return wheel positions, drive directions, and axial directions."""
    wheels = tuple(wheels)
    wheel_angles_rad = np.radians([wheel.alpha_deg for wheel in wheels])

    axial_directions = np.stack(
        [np.cos(wheel_angles_rad), np.sin(wheel_angles_rad)], axis=1
    )
    drive_directions = np.stack(
        [-np.sin(wheel_angles_rad), np.cos(wheel_angles_rad)], axis=1
    )
    wheel_positions = np.stack(
        [
            WHEEL_CENTER_DISTANCE * np.cos(wheel_angles_rad),
            WHEEL_CENTER_DISTANCE * np.sin(wheel_angles_rad),
        ],
        axis=1,
    )
    return wheel_positions, drive_directions, axial_directions


def smooth_friction_coefficient(v_slip: np.ndarray | float, mu_max: float, k: float) -> np.ndarray:
    """Williams Eq. (11), retaining the sign of the sliding velocity."""
    return mu_max * (2.0 / np.pi) * np.arctan(k * np.asarray(v_slip, dtype=float))


def roller_phase(theta: float) -> float:
    """Return theta phase in one roller pitch, in [0, pitch)."""
    pitch = 2.0 * np.pi / ROLLER_COUNT
    return np.mod(theta - ROLLER_PHASE_OFFSET, pitch)


def is_roller_contact(theta: float) -> bool:
    """Whether the current contact sector is occupied by a roller."""
    pitch = 2.0 * np.pi / ROLLER_COUNT
    return roller_phase(theta) < ROLLER_FRACTION * pitch


def theta_dot_from_body_command(
    command: np.ndarray | list[float] | tuple[float, ...],
    wheels: Iterable[Wheel] = ROBOT_WHEELS,
) -> np.ndarray:
    """Convert a body-frame velocity command to wheel angular velocities."""
    wheels = tuple(wheels)
    command = np.asarray(command, dtype=float)
    if command.shape == (2,):
        vx_cmd, vy_cmd = command
        omega_cmd = 0.0
    elif command.shape == (3,):
        vx_cmd, vy_cmd, omega_cmd = command
    else:
        raise ValueError("command must be [vx, vy] or [vx, vy, omega]")

    body_velocity = np.array([vx_cmd, vy_cmd], dtype=float)
    wheel_positions_body, drive_directions_body, _ = wheel_frames(wheels)
    contact_velocity_body = body_velocity + np.stack(
        (-omega_cmd * wheel_positions_body[:, 1], omega_cmd * wheel_positions_body[:, 0]),
        axis=1,
    )
    drive_velocity = np.einsum(
        "ij,ij->i", contact_velocity_body, drive_directions_body
    )

    # Williams' longitudinal slip is:
    #     v_W = v_drive + WHEEL_RADIUS * theta_dot.
    # Set v_W = 0 to obtain the ideal no-slip wheel command.
    return -drive_velocity / WHEEL_RADIUS


def contact_kinematics(
    state: np.ndarray,
    theta_dot: np.ndarray,
    wheels: Iterable[Wheel],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return contact velocity and the two Williams slip velocities."""
    wheels = tuple(wheels)
    theta_dot = np.asarray(theta_dot, dtype=float)
    if theta_dot.shape != (len(wheels),):
        raise ValueError("theta_dot must have shape (N_wheels,)")
    if state.shape[0] < 6 + len(wheels):
        raise ValueError("state is too short for the supplied wheel set")

    robot_yaw = float(state[2])
    robot_velocity_world = np.asarray(state[3:5], dtype=float)
    robot_yaw_rate = float(state[5])

    wheel_positions_body, drive_directions_body, axial_directions_body = wheel_frames(wheels)
    body_to_world = rotation_matrix(robot_yaw)
    wheel_positions_world = wheel_positions_body @ body_to_world.T
    drive_directions_world = drive_directions_body @ body_to_world.T
    axial_directions_world = axial_directions_body @ body_to_world.T

    rotational_velocity_world = np.stack(
        (
            -robot_yaw_rate * wheel_positions_world[:, 1],
            robot_yaw_rate * wheel_positions_world[:, 0],
        ),
        axis=1,
    )
    contact_velocity_world = robot_velocity_world + rotational_velocity_world

    longitudinal_slip = np.einsum(
        "ij,ij->i", contact_velocity_world, drive_directions_world
    )
    longitudinal_slip += WHEEL_RADIUS * theta_dot
    transverse_slip = np.einsum(
        "ij,ij->i", contact_velocity_world, axial_directions_world
    )

    return contact_velocity_world, longitudinal_slip, transverse_slip, theta_dot


def wheel_forces(
    state: np.ndarray,
    theta_dot: np.ndarray,
    params: ModelParams,
    wheels: Iterable[Wheel] = ROBOT_WHEELS,
) -> dict[str, np.ndarray]:
    """Evaluate Williams' improved friction model for all wheels."""
    wheels = tuple(wheels)
    wheel_angles = np.asarray(state[6 : 6 + len(wheels)], dtype=float)

    contact_velocity, longitudinal_slip, transverse_slip, _ = (
        contact_kinematics(state, theta_dot, wheels)
    )

    wheel_positions_body, drive_directions_body, axial_directions_body = wheel_frames(wheels)
    body_to_world = rotation_matrix(float(state[2]))
    wheel_positions_world = wheel_positions_body @ body_to_world.T
    drive_directions_world = drive_directions_body @ body_to_world.T
    axial_directions_world = axial_directions_body @ body_to_world.T

    normal_load = params.mass * GRAVITY / len(wheels)
    longitudinal_friction_coefficient = np.empty(len(wheels), dtype=float)
    transverse_friction_coefficient = np.empty(len(wheels), dtype=float)
    roller_contact = np.empty(len(wheels), dtype=bool)

    for i in range(len(wheels)):
        roller_contact[i] = is_roller_contact(wheel_angles[i])
        if roller_contact[i]:
            longitudinal_friction_coefficient[i] = smooth_friction_coefficient(
                longitudinal_slip[i], params.mu_W_roller, params.k
            )
            transverse_friction_coefficient[i] = smooth_friction_coefficient(
                transverse_slip[i], params.mu_T_roller, params.k
            )
        else:
            longitudinal_friction_coefficient[i] = smooth_friction_coefficient(
                longitudinal_slip[i], params.mu_W_gap, params.k
            )
            transverse_friction_coefficient[i] = smooth_friction_coefficient(
                transverse_slip[i], params.mu_T_gap, params.k
            )

    wheel_force = -normal_load * (
        longitudinal_friction_coefficient[:, None] * drive_directions_world
        + transverse_friction_coefficient[:, None] * axial_directions_world
    )
    wheel_yaw_torque = (
        wheel_positions_world[:, 0] * wheel_force[:, 1]
        - wheel_positions_world[:, 1] * wheel_force[:, 0]
    )

    return {
        "force": wheel_force,
        "torque_z": wheel_yaw_torque,
        "contact_velocity": contact_velocity,
        "v_W": longitudinal_slip,
        "v_T": transverse_slip,
        "theta_dot": theta_dot,
        "longitudinal_friction_coefficient": longitudinal_friction_coefficient,
        "transverse_friction_coefficient": transverse_friction_coefficient,
        "roller_contact": roller_contact,
        "wheel_position_world": wheel_positions_world,
        "d_world": drive_directions_world,
        "a_world": axial_directions_world,
    }


def body_wrench(force_data: dict[str, np.ndarray]) -> tuple[np.ndarray, float]:
    """Return total inertial force and yaw torque."""
    F = np.asarray(force_data["force"], dtype=float)
    tau = float(np.sum(force_data["torque_z"]))
    return F.sum(axis=0), tau


def continuous_dynamics(
    state: np.ndarray,
    theta_dot: np.ndarray,
    params: ModelParams,
    wheels: Iterable[Wheel] = ROBOT_WHEELS,
) -> np.ndarray:
    """Continuous-time plant dynamics suitable for an MPC integrator."""
    wheels = tuple(wheels)
    expected_state_size = 6 + len(wheels)
    if state.shape != (expected_state_size,):
        raise ValueError(f"state must have shape ({expected_state_size},)")

    force_data = wheel_forces(state, theta_dot, params, wheels)
    total_force_world, yaw_torque = body_wrench(force_data)

    state_dot = np.empty_like(state, dtype=float)
    state_dot[0] = state[3]
    state_dot[1] = state[4]
    state_dot[2] = state[5]
    state_dot[3:5] = total_force_world / params.mass
    state_dot[5] = yaw_torque / params.inertia
    state_dot[6:] = theta_dot
    return state_dot


def rk4_step(
    state: np.ndarray,
    theta_dot: np.ndarray,
    dt: float,
    params: ModelParams,
    wheels: Iterable[Wheel] = ROBOT_WHEELS,
) -> np.ndarray:
    """One fixed-input fourth-order Runge-Kutta integration step."""
    if dt <= 0.0:
        raise ValueError("dt must be positive")

    dynamics = lambda current_state: continuous_dynamics(
        current_state, theta_dot, params, wheels
    )
    k1 = dynamics(state)
    k2 = dynamics(state + 0.5 * dt * k1)
    k3 = dynamics(state + 0.5 * dt * k2)
    k4 = dynamics(state + dt * k3)
    return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def simulate(
    x0: np.ndarray,
    theta_dot_fn: Callable[[float], np.ndarray],
    duration: float,
    dt: float,
    params: ModelParams,
    wheels: Iterable[Wheel] = ROBOT_WHEELS,
) -> tuple[np.ndarray, np.ndarray]:
    """Integrate the plant with a wheel angular velocity input function."""
    if duration <= 0.0 or dt <= 0.0:
        raise ValueError("duration and dt must be positive")

    wheels = tuple(wheels)
    n_steps = int(np.floor(duration / dt)) + 1
    t = np.arange(n_steps, dtype=float) * dt
    x = np.empty((n_steps, len(x0)), dtype=float)
    x[0] = np.asarray(x0, dtype=float)

    for k in range(n_steps - 1):
        theta_dot = np.asarray(theta_dot_fn(t[k]), dtype=float)
        if theta_dot.shape != (len(wheels),):
            raise ValueError("theta_dot_fn must return one angular velocity per wheel")
        x[k + 1] = rk4_step(x[k], theta_dot, dt, params, wheels)

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
    to wheel angular velocities. Those wheel angular velocities are then held
    for that integration step.

    There is no feedback from the simulated robot velocity to the command.
    Any lateral or yaw motion produced by the Williams friction model is
    therefore part of the plant response.

    Returns
    -------
    t : (K,)
        Simulation times.
    x : (K, state_dim)
        Plant state history.
    theta_dot_history : (K-1, N)
        Wheel angular velocities applied during each integration step.
    """
    if duration <= 0.0 or dt <= 0.0:
        raise ValueError("duration and dt must be positive")

    wheels = tuple(wheels)
    n_steps = int(np.floor(duration / dt)) + 1
    t = np.arange(n_steps, dtype=float) * dt
    x = np.empty((n_steps, len(x0)), dtype=float)
    x[0] = np.asarray(x0, dtype=float)
    theta_dot_history = np.empty((n_steps - 1, len(wheels)), dtype=float)

    for k in range(n_steps - 1):
        command = np.asarray(command_fn(t[k]), dtype=float)
        theta_dot = theta_dot_from_body_command(command, wheels)
        theta_dot_history[k] = theta_dot
        x[k + 1] = rk4_step(x[k], theta_dot, dt, params, wheels)

    return t, x, theta_dot_history


def make_rest_state(wheels: Iterable[Wheel] = ROBOT_WHEELS) -> np.ndarray:
    """Convenience constructor for a zero state with one angle per wheel."""
    return np.zeros(6 + len(tuple(wheels)), dtype=float)
