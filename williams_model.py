"""
williams_model.py
==================

Wheel contact kinematics and friction force, following Henry's own derivation
(Project notes, ELEC 571) which follows Williams, Carter, Gallina & Rosati,
"Dynamic Model with Slip for Wheeled Omni-Directional Robots," IEEE T-RA
18(3):285-293, 2002 -- eq. (1)-(11).

Calculates:
    - wheel contact point velocity
    - d and a (drive and axial) wheel slip velocities
    - wheel force from slip velocities, via assumed/measured friction coefficients

NOTE ON THE FRICTION MODEL USED HERE:
Williams' paper also presents an "improved" model that switches between two
friction regimes depending on wheel angle, because their 8-roller wheel has
exposed rigid material between adjacent rollers. Per direct inspection of
our own wheel (16 rollers), the rollers are packed closely enough that the
wheel material does NOT contact the floor between rollers before the next one
engages -- so that two-regime switch does not apply here, and is not
implemented. Only the base friction model (eq. 1-11) is used.
"""

from __future__ import annotations

from robot_constants import MU_D_STAT, MU_D_DYN, MU_A_STAT, MU_A_DYN, ALPHA_D, ALPHA_A, K_SMOOTH_D, K_SMOOTH_A, F_N
from robot_constants import WHEEL_INERTIA, ROBOT_INERTIA, ROBOT_MASS, N_WHEELS, WHEEL_NAMES
from robot_constants import WHEEL_ALPHA_DEG, WHEEL_ROBOT_DIST, WHEEL_RADIUS

import numpy as np


# Wheel frame calculation based on the angle of the wheel from the first
# quadrant +x/+y in a counterclockwise direction.
#
# Inputs: alpha [0,360 deg]
# Outputs:
#   d_hat: drive direction
#   a_hat: axial direction
#
# d_hat: wheel drive direction
# a_hat: wheel axle / passive roller direction
def wheel_frame(alpha_deg):
    if alpha_deg < 0:
        alpha_deg = alpha_deg + 360

    alpha = np.radians(alpha_deg)

    a_hat_i = np.array([np.cos(alpha), np.sin(alpha)])

    delta = alpha + np.pi / 2

    d_hat_i = np.array([np.cos(delta), np.sin(delta)])

    return d_hat_i, a_hat_i


def cross_2d_z(omega, r):
    """omega x r for omega = omega*z_hat, r a 2D vector.

    From the determinant expansion:

        | i      j     k |
        | 0      0   omega|
        | r_x  r_y     0 |

    = (-omega*r_y, omega*r_x)
    """
    omega_vec = np.array([0.0, 0.0, omega])
    r_vec = np.array([r[0], r[1], 0.0])

    return np.cross(omega_vec, r_vec)[:2]


# ---------------------------------------------------------------------------
# Wheel kinematics
# ---------------------------------------------------------------------------

def wheel_center_velocity(V, omega, wheel_num):
    """Wheel-centre velocity in the robot frame.

    V_center = V + omega x r_i
    """
    d_hat, a_hat = wheel_frame(WHEEL_ALPHA_DEG[wheel_num])
    d_hat = np.array(d_hat)
    a_hat = np.array(a_hat)

    r_i = a_hat * WHEEL_ROBOT_DIST

    V_center = np.asarray(V) + cross_2d_z(omega, r_i)

    return V_center, d_hat, a_hat


def wheel_angular_velocity(V_center, d_hat):
    """Calculate the wheel angular velocity for zero d-axis slip.

    This is the inverse-kinematic wheel velocity.

    Williams' sign convention gives:

        theta_dot = -V_center . d_hat / R
    """
    theta_dot = -np.dot(V_center, d_hat) / WHEEL_RADIUS

    return theta_dot


def contact_point_velocity(V, omega, theta_dot, wheel_num):
    """Contact-point velocity in the robot frame.

    Williams' contact-point velocity is the wheel-centre velocity plus
    the peripheral velocity produced by wheel rotation:

        V_contact = V_center + theta_dot * R * d_hat

    theta_dot is the ACTUAL wheel angular velocity. It is not calculated
    here from the current robot velocity.
    """
    V_center, d_hat, a_hat = wheel_center_velocity(V, omega, wheel_num)

    V_contact = V_center + theta_dot * WHEEL_RADIUS * d_hat

    return V_contact, d_hat, a_hat


def slip_velocities(V, omega, theta_dot, wheel_num):
    """Calculate contact velocity components along d_hat and a_hat.

    v_slip,d = V_contact . d_hat
    v_slip,a = V_contact . a_hat

    theta_dot is the actual wheel angular velocity.

    If theta_dot is the inverse-kinematic no-slip value, then:

        v_slip,d = 0

    If the actual wheel velocity differs from that value, d-axis slip
    is nonzero.
    """
    V_contact, d_hat, a_hat = contact_point_velocity(V, omega, theta_dot, wheel_num)

    v_slip_d = np.dot(V_contact, d_hat)
    v_slip_a = np.dot(V_contact, a_hat)

    return v_slip_d, v_slip_a


# ---------------------------------------------------------------------------
# Friction model -- Williams 2002, eq. (11)
# ---------------------------------------------------------------------------


def mu_d(v_slip_d):
    """mu_d(v_slip_d) = mu_d_max * (2/pi) * atan(k * v_slip_d)."""
    return  (MU_D_DYN + (MU_D_STAT - MU_D_DYN)*np.exp(-ALPHA_D * np.abs(v_slip_d)))


def mu_a(v_slip_a):
    """mu_a(v_slip_a) = mu_a_max * (2/pi) * atan(k * v_slip_a)."""
    return  (MU_A_DYN + (MU_A_STAT - MU_A_DYN)*np.exp(-ALPHA_A * np.abs(v_slip_a)))


def wheel_force(V, omega, theta_dot, wheel_num, F_required):
    """Calculate wheel force from slip velocities."""
    d_hat, a_hat = wheel_frame(WHEEL_ALPHA_DEG[wheel_num])

    v_slip_d, v_slip_a = slip_velocities(V, omega, theta_dot, wheel_num)
    

    if np.abs(F_required) <= MU_D_STAT*F_N:
        F_d = F_required
    else:
        F_d = - F_N * mu_d(v_slip_d) * (2.0 / np.pi) * np.arctan(K_SMOOTH_D * v_slip_d)

    F_a = - F_N * mu_a(v_slip_a) * (2.0 / np.pi) * np.arctan(K_SMOOTH_A * v_slip_a) 

    F_i = F_d * d_hat +  F_a * a_hat

    return F_i, F_d, F_a, v_slip_d, v_slip_a


def all_wheel_forces(V, omega, theta_dot, F_required):
    """Calculate forces for all wheels.

    theta_dots contains the ACTUAL angular velocity of each wheel.
    """
    F = np.zeros((N_WHEELS, 2))
    v_slip_d = np.zeros(N_WHEELS)
    v_slip_a = np.zeros(N_WHEELS)
    F_d = np.zeros(N_WHEELS)
    F_a = np.zeros(N_WHEELS)

    for i in range(N_WHEELS):
        F[i], F_d[i], F_a[i], v_slip_d[i], v_slip_a[i] = wheel_force(V, omega, theta_dot[i], i, F_required[i])

    return F, F_d, F_a, v_slip_d, v_slip_a

def required_static_friction(V, omega, motor_torque):
    """Solve the all-wheels-sticking equations for required drive friction."""

    d_hat = np.zeros((N_WHEELS, 2))
    a_hat = np.zeros((N_WHEELS, 2))
    v_slip_a = np.zeros(N_WHEELS)
    F_a = np.zeros((N_WHEELS, 2))


    for i in range(N_WHEELS):
        d_hat[i], a_hat[i] = wheel_frame(WHEEL_ALPHA_DEG[i])
        #theta_dot can be zero because it has no effect on the axial slip velocity
        #F_required can be zero because we do not want F_d nor care for it to be accurate. 
        _, _, F_a[i], _, v_slip_a[i] = wheel_force(V, omega, 0, i, 0)

    F_a_total = np.sum(F_a, axis=0)

    A = np.zeros((N_WHEELS, N_WHEELS))
    b = np.zeros(N_WHEELS)

    for i in range(N_WHEELS):

        # Known acceleration contribution from axial friction.
        a_axial = np.dot(d_hat[i], F_a_total) / ROBOT_MASS # d_hat on a_axis is not necessarily zero for other wheels

        # Known contribution from the changing wheel-frame direction.
        a_slip = - omega * v_slip_a[i]

        # Known wheel angular acceleration contribution from motor torque.
        a_motor = WHEEL_RADIUS * motor_torque[i] / WHEEL_INERTIA

        # Right-hand side: move known terms to the other side.
        b[i] = -(a_axial + a_slip + a_motor)

        for j in range(N_WHEELS):

            # Translational acceleration caused by drive friction at wheel j.
            a_translation = np.dot(d_hat[i], d_hat[j]) / ROBOT_MASS

            # Rotational acceleration coupling between wheels i and j.
            a_rotation = WHEEL_ROBOT_DIST**2 / ROBOT_INERTIA

            # Wheel angular acceleration caused by its own drive friction.
            a_wheel = WHEEL_RADIUS**2 / WHEEL_INERTIA if i == j else 0.0

            # Combine the three contributions into A[i, j].
            A[i, j] = a_translation + a_rotation - a_wheel

    return np.linalg.solve(A, b)

# ---------------------------------------------------------------------------
# Self-tests
# ---------------------------------------------------------------------------

def _test_orthonormality():
    for wheel_num in range(N_WHEELS):
        d_hat, a_hat = wheel_frame(WHEEL_ALPHA_DEG[wheel_num])

        assert abs(np.linalg.norm(d_hat) - 1.0) < 1e-12, (f"{WHEEL_NAMES[wheel_num]}: d_hat is not a unit vector")
        assert abs(np.linalg.norm(a_hat) - 1.0) < 1e-12, (f"{WHEEL_NAMES[wheel_num]}: a_hat is not a unit vector")
        assert abs(np.dot(d_hat, a_hat)) < 1e-12, (f"{WHEEL_NAMES[wheel_num]}: d_hat and a_hat are not orthogonal")

    print("  PASS: d_hat and a_hat are orthonormal for all wheels")


def _test_zero_velocity_zero_force():
    """Zero robot and wheel velocity should produce zero slip and force."""
    V = np.array([0.0, 0.0])
    omega = 0.0
    theta_dots = np.zeros(N_WHEELS)

    F, v_slip_d, v_slip_a = all_wheel_forces(V, omega, theta_dots)

    assert np.allclose(v_slip_d, 0.0)
    assert np.allclose(v_slip_a, 0.0)
    assert np.allclose(F, 0.0, atol=1e-9), (f"nonzero force at zero velocity: {F}")

    print("  PASS: zero velocity produces zero slip and zero force")


def _test_pure_rolling_zero_drive_slip():
    """Matched wheel speed should produce zero drive-direction slip."""
    wheel_num = 0

    V = np.array([0.5, 0.2])
    omega = 0.3

    V_center, d_hat, _ = wheel_center_velocity(V, omega, wheel_num)

    theta_dot = wheel_anglular_velocity(V_center, d_hat)

    v_slip_d, v_slip_a = slip_velocities(V, omega, theta_dot, wheel_num)

    assert abs(v_slip_d) < 1e-9, (f"expected zero drive-direction slip, got {v_slip_d}")

    print("  PASS: matched wheel speed gives zero drive-direction slip "
                                        f"(v_slip_a={v_slip_a:.4f} m/s)")


def _test_nonzero_drive_slip():
    """A wheel speed different from the no-slip speed produces d-slip."""
    wheel_num = 0

    V = np.array([0.5, 0.2])
    omega = 0.3

    V_center, d_hat, _ = wheel_center_velocity(V, omega, wheel_num)

    theta_dot_no_slip = wheel_anglular_velocity(V_center, d_hat)

    # Deliberately perturb the wheel speed.
    theta_dot = theta_dot_no_slip + 10.0

    v_slip_d, _ = slip_velocities(V, omega, theta_dot, wheel_num)

    assert abs(v_slip_d) > 1e-6, ("different wheel speed should produce nonzero d-axis slip")

    print("  PASS: wheel-speed mismatch produces nonzero d-axis slip "
        f"(v_slip_d={v_slip_d:.4f} m/s)")


if __name__ == "__main__":
    print("williams_model.py self-tests")
    print("=" * 60)

    _test_orthonormality()
    _test_zero_velocity_zero_force()
    _test_pure_rolling_zero_drive_slip()
    _test_nonzero_drive_slip()

    print("=" * 60)
    print("All checks passed.")
    print()

    print(
        f"WHEEL_RADIUS = {WHEEL_RADIUS * 1000:.4f} mm, "
        f"ROLLER_RADIUS = {ROLLER_RADIUS * 1000:.4f} mm, "
        f"roller pitch = {np.degrees(ROLLER_PITCH):.2f} deg"
    )

    print(
        "MU_D_MAX, MU_A_MAX are PLACEHOLDERS -- replace with your own "
        "tilt-test measurements before trusting any force magnitude."
    )