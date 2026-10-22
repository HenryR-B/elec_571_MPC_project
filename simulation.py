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

from actuator_model import actuator_model, compute_trial_iq, reset as reset_actuator
from controller_pid import controller_pid
from integrator import rk4_step
from robot_dynamics import body2world_accelerations
from williams_model import (
    all_wheel_forces,
    inverse_kinematics,
    required_static_friction, mu_d, mu_a
)

from robot_constants import WHEEL_RADIUS, MOTOR_KT, F_N, MU_D_STAT

# ---------------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------------

DT = 0.0001
SIMULATION_TIME = 1.0

KP = np.array([10, 10, 10])
KI = np.array([5, 5, 10])
KD = np.array([0.1, 0.1, 0.2])

target = [0, 1, 0]

MAX_ACCELERATION = 2.5  # m/s^2


def run_motor_test():
    reset_actuator()

    velocity = np.array([0.0, 5.0, 0.0])
    theta_dot = np.zeros(4)
    load_torque = np.zeros(4)
    theta_dot_cmd = inverse_kinematics(velocity, 0.0)

    print("Commanded wheel speeds (rad/s):")
    for i in range(4):
        print(f"  Wheel {i + 1}: {theta_dot_cmd[i]:.3f}")

    time = np.arange(0.0, SIMULATION_TIME, DT)

    theta_dot_history = np.zeros((len(time), 4))
    theta_ddot_history = np.zeros((len(time), 4))
    iq_history = np.zeros((len(time), 4))

    for k, t in enumerate(time):
        theta_dot, theta_ddot, iq = actuator_model(
            theta_dot_cmd, theta_dot, load_torque, DT
        )

        theta_dot_history[k] = theta_dot
        theta_ddot_history[k] = theta_ddot
        iq_history[k] = iq

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    for i in range(4):
        axes[0].plot(time, theta_dot_history[:, i], label=f"Wheel {i + 1} actual")
        axes[0].axhline(theta_dot_cmd[i], linestyle="--",
                        label=f"Wheel {i + 1} command")
        axes[1].plot(time, iq_history[:, i], label=f"Wheel {i + 1}")
        axes[2].plot(time, theta_ddot_history[:, i], label=f"Wheel {i + 1}")

    axes[0].set_ylabel("Angular velocity (rad/s)")
    axes[1].set_ylabel("Iq (A)")
    axes[2].set_ylabel("Angular acceleration (rad/s²)")
    axes[2].set_xlabel("Time (s)")

    for ax in axes:
        ax.grid()
        ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5))

    plt.tight_layout()
    plt.show()



# ---------------------------------------------------------------------------
# Wheel load torque
# ---------------------------------------------------------------------------

def wheel_load_torques(Fd):
    """Calculate wheel load torque from the Williams wheel forces."""

    load_torque = np.zeros(4)

    for i in range(4):
        load_torque[i] = WHEEL_RADIUS * Fd[i] # actuator defines the negative sign

    return load_torque


# ---------------------------------------------------------------------------
# Robot state derivative
# ---------------------------------------------------------------------------

def robot_state_derivative(state, theta_dot):
    """Return the derivative of the robot physical state."""

    x, y, phi, vx, vy, omega = state

    rotation = np.array([[np.cos(phi), np.sin(phi)], [-np.sin(phi), np.cos(phi)]])

    velocity_world = np.array([vx, vy])

    velocity_body = np.dot(rotation, velocity_world)

    F, _, _, _, _ = all_wheel_forces(velocity_body, omega, theta_dot)
    x_ddot, y_ddot, phi_ddot = body2world_accelerations(F, phi)

    return np.array([
        vx,
        vy,
        omega,
        x_ddot,
        y_ddot,
        phi_ddot,
    ])

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
    load_torque = np.zeros(4)
    robot_accel = np.zeros(2)



    time = np.arange(0.0, SIMULATION_TIME, DT)
    history = initialize_history(time)

    def robot_derivative(state):
        return robot_state_derivative(state, theta_dot)

    for k, t in enumerate(time):
        velocity = state[3:6]
        x, y, phi, vx, vy, omega = state

        velocity_cmd, integral_error_dot = controller_pid(velocity_target, velocity, integral_error, previous_velocity, DT, KP, KI, KD)
        previous_velocity = velocity.copy()

        integral_error = integral_error + DT * integral_error_dot

        theta_dot_cmd = inverse_kinematics(velocity_cmd, phi)

        rotation = np.array([[np.cos(phi), np.sin(phi)], [-np.sin(phi), np.cos(phi)]])
        velocity_body = np.dot(rotation, np.array([vx, vy]))

        
        F, F_d, F_a, v_slip_d, v_slip_a = all_wheel_forces(velocity_body, omega, theta_dot)

        load_torque = wheel_load_torques(F_d)

        theta_dot, theta_ddot, iq = actuator_model(theta_dot_cmd, theta_dot, load_torque, DT)

        motor_torque = iq * MOTOR_KT

        state_derivative = robot_state_derivative(state, theta_dot) # vx, vy, omega, x_ddot, y_ddot, phi_ddot

        robot_accel = state_derivative[3:5]
        omega_dot = state_derivative[5]

        linear_accel = np.linalg.norm(robot_accel)

        record_history(k, history, state, theta_dot, theta_dot_cmd, linear_accel, theta_ddot, F_d, F_a, v_slip_d, v_slip_a, iq)

        state = rk4_step(state, DT, robot_derivative)

        if k % 500 == 0:
            fmt = lambda a: np.array2string(a, precision=3, separator=',', max_line_width=10000)
            print(f"t={t:.4f} v={fmt(velocity)} cmd={fmt(velocity_cmd)} w={fmt(theta_dot)} wc={fmt(theta_dot_cmd)} F={fmt(F)} Fd={fmt(F_d)} Fa={fmt(F_a)} sd={fmt(v_slip_d)} sa={fmt(v_slip_a)} mud={fmt(mu_d(v_slip_d))} mua={fmt(mu_a(v_slip_a))}")


    return time, history

# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_simulation(time, history):
    """Plot robot velocity and wheel-speed response."""
    state_history = history["state"]
    theta_dot_history = history["theta_dot"]
    theta_dot_cmd_history = history["theta_dot_cmd"]
    linear_accel_history = history["linear_accel"]
    theta_ddot_history = history["theta_ddot"]
    wheel_force_d_history = history["wheel_force_d"]
    wheel_slip_d_history = history["wheel_slip_d"]
    iq_history = history["iq"]
    wheel_force_a_history = history["wheel_force_a"]
    wheel_slip_a_history = history["wheel_slip_a"]

    fig, axes = plt.subplots(5, 1, figsize=(8, 9))

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

    axes[3].plot(time, linear_accel_history, label="Linear acceleration")
    axes[3].set_xlabel("Time [s]")
    axes[3].set_ylabel("Acceleration [m/s²]")
    axes[3].set_title("Robot linear acceleration")
    axes[3].grid()
    axes[3].legend()

    # Motor current
    for i in range(4):
        axes[4].plot(time, iq_history[:, i], label=f"Wheel {i + 1}")


    axes[4].set_xlabel("Time [s]")
    axes[4].set_ylabel("IQ [A]")
    axes[4].set_title("Motor Current Command")
    axes[4].grid()
    axes[4].legend()

    plt.tight_layout()



    fig, axes = plt.subplots(4, 1, figsize=(8, 8), sharex=True)

    # D-axis slip velocity
    for i in range(4):
        axes[0].plot(time, wheel_slip_d_history[:, i], label=f"Wheel {i + 1}")
    axes[0].axhline(0, color="black", linewidth=0.8)
    axes[0].set_ylabel("Slip velocity [m/s]")
    axes[0].set_title("D-axis slip velocity")
    axes[0].grid()
    axes[0].legend()

    # A-axis slip velocity
    for i in range(4):
        axes[1].plot(time, wheel_slip_a_history[:, i], label=f"Wheel {i + 1}")
    axes[1].axhline(0, color="black", linewidth=0.8)
    axes[1].set_ylabel("Slip velocity [m/s]")
    axes[1].set_title("A-axis slip velocity")
    axes[1].grid()
    axes[1].legend()

    # D-axis friction force
    for i in range(4):
        axes[2].plot(time, wheel_force_d_history[:, i], label=f"Wheel {i + 1} actual")[0]

    axes[2].axhline(0, color="black", linewidth=0.8)
    axes[2].set_ylabel("Force [N]")
    axes[2].set_title("D-axis friction force")
    axes[2].grid()
    axes[2].legend()

    # A-axis friction force
    for i in range(4):
        axes[3].plot(time, wheel_force_a_history[:, i], label=f"Wheel {i + 1}")
    axes[3].axhline(0, color="black", linewidth=0.8)
    axes[3].set_xlabel("Time [s]")
    axes[3].set_ylabel("Force [N]")
    axes[3].set_title("A-axis friction force")
    axes[3].grid()
    axes[3].legend()

    fig.tight_layout()
    
    plt.show()

def initialize_history(time):
    n = len(time)
    return {
        "state": np.zeros((n, 6)),
        "theta_dot": np.zeros((n, 4)),
        "theta_dot_cmd": np.zeros((n, 4)),
        "linear_accel": np.zeros(n),
        "theta_ddot": np.zeros((n, 4)),
        "wheel_force_d": np.zeros((n, 4)),
        "wheel_force_a": np.zeros((n, 4)),
        "wheel_slip_d": np.zeros((n, 4)),
        "wheel_slip_a": np.zeros((n, 4)),
        "iq": np.zeros((n, 4)),
    }

def record_history(k, history, state, theta_dot, theta_dot_cmd, linear_accel, theta_ddot, F_d, F_a, v_slip_d, v_slip_a, iq):
    history["state"][k] = state
    history["theta_dot"][k] = theta_dot
    history["theta_dot_cmd"][k] = theta_dot_cmd
    history["linear_accel"][k] = linear_accel
    history["theta_ddot"][k] = theta_ddot
    history["wheel_force_d"][k] = F_d
    history["wheel_force_a"][k] = F_a
    history["wheel_slip_d"][k] = v_slip_d
    history["wheel_slip_a"][k] = v_slip_a
    history["iq"][k] = iq

if __name__ == "__main__":
    velocity_target = np.array(target)

    #run_motor_test()
    time, history = run_simulation(velocity_target)

    plot_simulation(time, history)