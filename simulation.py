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

from actuator_model_motor import actuator_model, reset as reset_actuator
from controller_pid import controller_pid
from integrator import rk4_step
from robot_dynamics import body2world_accelerations
from williams_model import (
    WHEEL_ALPHA_DEG,
    WHEEL_ROBOT_DIST,
    WHEEL_RADIUS,
    ROBOT_MASS,
    N_WHEELS,
    GRAVITY,
    all_wheel_forces,
    wheel_frame,
    wheel_center_velocity,
    wheel_angular_velocity,
)

from actuator_motor import MOTOR_KT


# ---------------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------------

DT = 0.001
ACTUATOR_DT = 0.0001
SIMULATION_TIME = 1.0

KP = np.array([10, 10, 150])
KI = np.array([3, 2, 500])
KD = np.array([0.5, 0.5, 0.3])

target = [0.5, 0, 0]

MAX_ACCELERATION = 2.5  # m/s^2

def run_motor_test():
    reset_actuator()

    velocity = np.array([0.0, 5.0, 0.0])
    theta_dot = np.zeros(4)
    theta_dot_cmd = np.zeros(4)
    load_torque = np.zeros(4)

    theta_dot_cmd = inverse_kinematics(velocity, 0.0)

    print("Commanded wheel speeds (rad/s):")
    for i in range(4):
        print(f"  Wheel {i + 1}: {theta_dot_cmd[i]:.3f}")

    time = np.arange(0.0, 1, ACTUATOR_DT)

    theta_dot_history = np.zeros((len(time), 4))
    theta_ddot_history = np.zeros((len(time), 4))
    iq_history = np.zeros((len(time), 4))

    for k, t in enumerate(time):
        theta_dot, theta_ddot, iq = actuator_model(
            theta_dot_cmd, theta_dot, load_torque, ACTUATOR_DT
        )

        theta_dot_history[k] = theta_dot
        theta_ddot_history[k] = theta_ddot
        iq_history[k] = iq

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    for i in range(4):
        axes[0].plot(time, theta_dot_history[:, i], label=f"Wheel {i + 1} actual")
        axes[0].axhline(theta_dot_cmd[i], linestyle="--", label=f"Wheel {i + 1} command: {theta_dot_cmd[i]:.2f} rad/s")

        axes[1].plot(time, iq_history[:, i], label=f"Wheel {i + 1}")

        axes[2].plot(time, theta_ddot_history[:, i], label=f"Wheel {i + 1}")

    axes[0].set_ylabel("Angular velocity (rad/s)")
    axes[0].legend(loc="center left", bbox_to_anchor=(1.02, 0.5))
    axes[0].grid()

    axes[1].set_ylabel("Iq (A)")
    axes[1].legend(loc="center left", bbox_to_anchor=(1.02, 0.5))
    axes[1].grid()

    axes[2].set_ylabel("Angular acceleration (rad/s²)")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend(loc="center left", bbox_to_anchor=(1.02, 0.5))
    axes[2].grid()

    plt.tight_layout()
    plt.show()

def inverse_kinematics(velocity, phi):
    rotation = np.array([[np.cos(phi), np.sin(phi)], [-np.sin(phi), np.cos(phi)]])
    V = np.dot(rotation, velocity[:2])
    omega = velocity[2]

    theta_dot_cmd = np.zeros(4)

    for i in range(4):
        V_center, d_hat, a_hat = wheel_center_velocity(V, omega, i)
        theta_dot_cmd[i] = wheel_angular_velocity(V_center, d_hat)

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

    F, _, v_slip_d, v_slip_a = all_wheel_forces(velocity[:2], omega, theta_dot)

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

    reset_actuator()

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
    mse_history = np.zeros(len(time))
    wheel_force_d_history = np.zeros((len(time), 4))
    wheel_slip_d_history = np.zeros((len(time), 4))

    iq_history = np.zeros((len(time), 4))

    phi = state[2]

    def robot_derivative(state):
            return robot_state_derivative(state, theta_dot)

    for k, t in enumerate(time):

        state_history[k] = state
        theta_dot_history[k] = theta_dot
        

        velocity = state[3:6]
        error = velocity_target - velocity
        mse_history[k] = np.mean(error**2)  

        velocity_cmd, integral_error_dot = controller_pid(velocity_target, velocity, integral_error, previous_velocity, DT, KP, KI, KD)
        previous_velocity = velocity.copy()

        integral_error = integral_error + DT * integral_error_dot

        theta_dot_cmd = inverse_kinematics(velocity_cmd, phi)

        for _ in range(int(DT / ACTUATOR_DT)):
            F, F_d, v_slip_d, v_slip_a = all_wheel_forces(velocity[:2], velocity[2], theta_dot)

            load_torque = wheel_load_torques(F)

            theta_dot, theta_ddot, iq = actuator_model(theta_dot_cmd, theta_dot, load_torque, ACTUATOR_DT) #theta_dot = actuator_model(theta_dot_cmd, theta_dot, load_torque, ACTUATOR_DT)

            for i in range(4):
                wheel_force_d_history[k, i] = F_d[i] / (ROBOT_MASS * GRAVITY / N_WHEELS)
                wheel_slip_d_history[k, i] = v_slip_d[i]
                iq_history[k, i] = iq[i]

            linear_accel = np.linalg.norm(robot_state_derivative(state, theta_dot)[3:5])

            if k % 100 == 0 and _ == 0:
                print(f"F={F} slips_d={v_slip_d} slips_a={v_slip_a}")
    
                print(f"t={t:.1f} v={velocity} cmd={velocity_cmd} wheels={theta_dot_cmd} actual={theta_dot}")
                print(f"  F={F} slips_d={v_slip_d} slips_a={v_slip_a}")
                print(f"  mu_d={mu_d(v_slip_d)} mu_a={mu_a(v_slip_a)}")
                print(f"t={t:.1f} speed={np.linalg.norm(velocity[:2]):.3f} m/s "
                    f"linear_accel={linear_accel:.3f} m/s² theta_ddot={theta_ddot}")

        acceleration = robot_state_derivative(state, theta_dot)[3:5]
        linear_accel_history[k] = np.linalg.norm(acceleration)
        theta_ddot_history[k] = theta_ddot
        theta_dot_cmd_history[k] = theta_dot_cmd

        if k % 100 == 0 and _ == 0:
            print(f"t={time[k]:.1f} theta_ddot={theta_ddot_history[k]}")

        state = rk4_step(state, DT, robot_derivative)

        phi = state[2]

    return time, state_history, theta_dot_history, theta_dot_cmd_history, linear_accel_history, theta_ddot_history, mse_history, wheel_force_d_history, wheel_slip_d_history, iq_history

# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_simulation(time, state_history, theta_dot_history, theta_dot_cmd_history, acceleration_history, theta_ddot_history, mse_history, wheel_force_d_history, wheel_slip_d_history, iq_history):
    """Plot robot velocity and wheel-speed response."""

    fig, axes = plt.subplots(6, 1, figsize=(10, 18))

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

    for i in range(4):
        axes[2].plot(time, theta_ddot_history[:, i], label=f"Wheel {i + 1}")
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

    axes[4].plot(time, wheel_force_d_history[:, 0], label="Wheel 1")
    axes[4].plot(time, wheel_force_d_history[:, 1], label="Wheel 2")
    axes[4].plot(time, wheel_force_d_history[:, 2], label="Wheel 3")
    axes[4].plot(time, wheel_force_d_history[:, 3], label="Wheel 4")

    axes[4].plot(time, wheel_slip_d_history[:, 0], "--", label="Wheel 1 Slip")
    axes[4].plot(time, wheel_slip_d_history[:, 1], "--", label="Wheel 2 Slip")
    axes[4].plot(time, wheel_slip_d_history[:, 2], "--", label="Wheel 3 Slip")
    axes[4].plot(time, wheel_slip_d_history[:, 3], "--", label="Wheel 4 Slip")

    axes[4].set_xlabel("Time (s)")
    axes[4].set_ylabel("Force / Slip")
    axes[4].set_title("Wheel d-axis Force and Slip Velocity")
    axes[4].legend()
    axes[4].grid()

    # Motor current
    for i in range(4):
        axes[5].plot(time, iq_history[:, i], label=f"Wheel {i + 1}")


    axes[5].set_xlabel("Time [s]")
    axes[5].set_ylabel("IQ [A]")
    axes[5].set_title("Motor Current Command")
    axes[5].grid()
    axes[5].legend()

    overall_mse = np.mean(mse_history)
    print(f"Overall velocity MSE = {overall_mse:.6f}")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    velocity_target = np.array(target)

    #run_motor_test()
    time, state_history, theta_dot_history, theta_dot_cmd_history, linear_accel_history, theta_ddot_history, mse_history, wheel_force_d_history, wheel_slip_d_history, iq_history = run_simulation(velocity_target)

    plot_simulation(time, state_history, theta_dot_history, theta_dot_cmd_history, linear_accel_history, theta_ddot_history, mse_history, wheel_force_d_history, wheel_slip_d_history, iq_history)