"""
simulation.py
=============

Top-level robot simulation.

Flow:

    desired robot velocity
        ↓
    velocity PID controller
        ↓
    inverse kinematics
        ↓
    wheel velocity commands
        ↓
    actuator model
        ↓
    actual wheel velocities
        ↓
    Williams wheel model
        ↓
    wheel forces
        ↓
    robot dynamics
        ↓
    robot state integration
"""

from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt

from actuator_model import actuator_model#, reset as reset_actuator
from controller_pid import controller_pid
from integrator import rk4_step
from robot_dynamics import body2world_accelerations
from williams_model import (
    WHEEL_ALPHA_DEG,
    WHEEL_ROBOT_DIST,
    WHEEL_RADIUS,
    all_wheel_forces,
    wheel_frame,
)


# ---------------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------------

DT = 0.001
ACTUATOR_DT = 0.0001
SIMULATION_TIME = 1.0

KP = np.array([5.0, 5.0, 2.0])
KI = np.array([15, 15, 20])
KD = np.array([0.1, 0.1, 0.02])

target = [1, 0.0, 0]

MAX_ACCELERATION = 2.5  # m/s^2


# ---------------------------------------------------------------------------
# Inverse kinematics
# ---------------------------------------------------------------------------

def inverse_kinematics(velocity,  phi):
    """Convert robot velocity [vx, vy, omega] into wheel speeds."""

    rotation = np.array([[np.cos(phi), np.sin(phi)], [-np.sin(phi), np.cos(phi)]])
    V = np.dot(rotation, velocity[:2])
    omega = velocity[2]

    theta_dot_cmd = np.zeros(4)

    for i in range(4):
        d_hat, a_hat = wheel_frame(WHEEL_ALPHA_DEG[i])
        r_i = a_hat * WHEEL_ROBOT_DIST

        V_center = V + np.array([-omega * r_i[1], omega * r_i[0]])

        theta_dot_cmd[i] = -np.dot(V_center, d_hat) / WHEEL_RADIUS

    return theta_dot_cmd


# ---------------------------------------------------------------------------
# Wheel load torque
# ---------------------------------------------------------------------------

def wheel_load_torques(F):
    """Calculate wheel load torque from the Williams wheel forces."""

    load_torque = np.zeros(4)

    for i in range(4):
        d_hat, a_hat = wheel_frame(WHEEL_ALPHA_DEG[i])
        load_torque[i] = -WHEEL_RADIUS * np.dot(F[i], d_hat)

    return load_torque


# ---------------------------------------------------------------------------
# Robot state derivative
# ---------------------------------------------------------------------------

def robot_state_derivative(state, theta_dot):
    """Return the derivative of the robot physical state."""

    x, y, phi, vx, vy, omega = state

    rotation = np.array([[np.cos(phi), np.sin(phi)], [-np.sin(phi), np.cos(phi)]])
    velocity_body = np.dot(rotation, np.array([vx, vy]))
    velocity = np.array([velocity_body[0], velocity_body[1], omega])

    F, v_slip_d, v_slip_a = all_wheel_forces(velocity[:2], omega, theta_dot)

    x_ddot, y_ddot, phi_ddot = body2world_accelerations(
        F,
        np.array([
            [np.cos(np.radians(WHEEL_ALPHA_DEG[i])) * WHEEL_ROBOT_DIST,
             np.sin(np.radians(WHEEL_ALPHA_DEG[i])) * WHEEL_ROBOT_DIST]
            for i in range(4)
        ]),
        phi,
    )

    return np.array([
        vx,
        vy,
        omega,
        x_ddot,
        y_ddot,
        phi_ddot,
    ])

from williams_model import all_wheel_forces, mu_d, mu_a
# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def run_simulation(velocity_target):
    """Run the robot simulation."""

    #reset_actuator()

    state = np.zeros(6)
    theta_dot = np.zeros(4)
    integral_error = np.zeros(3)
    previous_velocity = np.zeros(3)

    time = np.arange(0.0, SIMULATION_TIME, DT)

    state_history = np.zeros((len(time), 6))
    theta_dot_history = np.zeros((len(time), 4))
    theta_dot_cmd_history = np.zeros((len(time), 4))
    linear_accel_history = np.zeros(len(time))
    theta_ddot_history = np.zeros((len(time), 4))

    phi = state[2]

    def robot_derivative(state):
            return robot_state_derivative(state, theta_dot)

    for k, t in enumerate(time):

        state_history[k] = state
        theta_dot_history[k] = theta_dot

        velocity = state[3:6]

        velocity_cmd, integral_error_dot = controller_pid(velocity_target, velocity, integral_error, previous_velocity, DT, KP, KI, KD)
        previous_velocity = velocity.copy()

        integral_error = integral_error + DT * integral_error_dot
        
        theta_dot_cmd = inverse_kinematics(velocity_cmd, phi)
        for _ in range(int(DT / ACTUATOR_DT)):
            F, v_slip_d, v_slip_a = all_wheel_forces(velocity[:2], velocity[2], theta_dot)

            load_torque = wheel_load_torques(F)

            theta_dot, theta_ddot= actuator_model(theta_dot_cmd, theta_dot, ACTUATOR_DT) #theta_dot = actuator_model(theta_dot_cmd, theta_dot, load_torque, ACTUATOR_DT)
            linear_accel = np.linalg.norm(robot_state_derivative(state, theta_dot)[3:5])

            tau = np.cross(
                np.array([
                    [np.cos(np.radians(WHEEL_ALPHA_DEG[i])) * WHEEL_ROBOT_DIST,
                    np.sin(np.radians(WHEEL_ALPHA_DEG[i])) * WHEEL_ROBOT_DIST]
                    for i in range(4)
                ]),
                F,
            ).sum()

            if k % 100 == 0 and _ == 0:
                print(f"F={F} slips_d={v_slip_d} slips_a={v_slip_a} tau={tau}")
                rotation = np.array([[np.cos(phi), np.sin(phi)], [-np.sin(phi), np.cos(phi)]])
                velocity_body = rotation @ velocity[:2]

                print(f"t={t:.1f} v={velocity} cmd={velocity_cmd} wheels={theta_dot_cmd} actual={theta_dot}")
                print(f"  F={F} slips_d={v_slip_d} slips_a={v_slip_a}")
                print(f"  mu_d={mu_d(v_slip_d)} mu_a={mu_a(v_slip_a)}")
                print(f"t={t:.1f} speed={np.linalg.norm(velocity[:2]):.3f} m/s "
                    f"linear_accel={linear_accel:.3f} m/s² theta_ddot={theta_ddot}")

        acceleration = robot_state_derivative(state, theta_dot)[3:5]
        linear_accel_history[k] = np.linalg.norm(acceleration)
        theta_ddot_history[k] = theta_ddot

        state = rk4_step(state, DT, robot_derivative)

        phi = state[2]

        theta_dot_cmd_history[k] = theta_dot_cmd

    return time, state_history, theta_dot_history, theta_dot_cmd_history, linear_accel_history, theta_ddot_history

# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_simulation(time, state_history, theta_dot_history, theta_dot_cmd_history, acceleration_history, theta_ddot_history):
    """Plot robot velocity and wheel-speed response."""

    fig, axes = plt.subplots(4, 1, figsize=(10, 12))

    # Robot velocity
    axes[0].plot(time, state_history[:, 3], label="vx")
    axes[0].plot(time, state_history[:, 4], label="vy")
    axes[0].plot(time, state_history[:, 5], label="omega")
    axes[0].set_ylabel("Velocity")
    axes[0].set_title("Robot velocity")
    axes[0].grid()
    axes[0].legend()

    # Wheel speeds
    for i in range(4):
        axes[1].plot(time, theta_dot_history[:, i], label=f"wheel {i + 1} actual")
        axes[1].plot(time, theta_dot_cmd_history[:, i], "--", label=f"wheel {i + 1} cmd")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Wheel angular velocity [rad/s]")
    axes[1].set_title("Wheel velocities")
    axes[1].grid()
    axes[1].legend()

    axes[2].plot(time, theta_ddot_history)
    axes[2].axhline(83.57, linestyle="--", label="Target ±83.57 rad/s²")
    axes[2].axhline(-83.57, linestyle="--")
    axes[2].set_ylabel("Angular acceleration [rad/s²]")
    axes[2].set_title("Actuator angular acceleration")
    axes[2].grid()
    axes[2].legend([f"wheel {i + 1}" for i in range(4)] + ["Target ±83.57 rad/s²"])

    axes[3].plot(time, acceleration_history, label="Linear acceleration")
    axes[3].set_xlabel("Time [s]")
    axes[3].set_ylabel("Acceleration [m/s²]")
    axes[3].set_title("Robot linear acceleration")
    axes[3].grid()
    axes[3].legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    velocity_target = np.array(target)

    time, state_history, theta_dot_history, theta_dot_cmd_history, linear_accel_history, theta_ddot_history = run_simulation(velocity_target)

    plot_simulation(time, state_history, theta_dot_history, theta_dot_cmd_history, linear_accel_history, theta_ddot_history)