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
wheel material does NOT contact the floor between rollers before the next
one engages -- so that two-regime switch does not apply here, and is not
implemented. Only the base friction model (eq. 1-11) is used.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# Geometry constants\
# ---------------------------------------------------------------------------

ROLLER_RADIUS = 0.014378 / 2       # roller radius [m] = 7.189 mm
N_ROLLERS = 16
ROLLER_PITCH = 2 * np.pi / N_ROLLERS   # 22.5 deg
ROBOT_MASS = 2.698       # [kg]
GRAVITY = 9.81            # [m/s^2]
N_WHEELS = 4

WHEEL_RADIUS = 0.0598292 / 2      # wheel radius [m] = 29.9146 mm
WHEEL_INERTIA = 16.576 * 1e-6  # kg*m^2
WHEEL_MASS = 0.066621 # kg (including rotor, all rotating mass)


@dataclass
class Wheel:
    """One wheel's fixed geometry, in the robot body frame."""
    name: str
    alpha_deg: float   # mounting position angle from body +x axis, CCW
    r_dist: float       # |r_i|, distance from robot centre to wheel centre [m]

    def frame(self):
        """Returns (r_i, d_hat, a_hat) -- position and unit vectors, body frame.

        d_hat: drive direction (the wheel pushes along this)
        a_hat: axial direction (the wheel's own axle direction; passive rollers
               let the wheel slide freely along this)
        Convention: axle mounted radially, so drive direction is tangential
        (delta = alpha + 90 deg).
        """
        alpha = np.radians(self.alpha_deg)
        delta = alpha + np.pi / 2
        r_i = self.r_dist * np.array([np.cos(alpha), np.sin(alpha)])
        d_hat = np.array([np.cos(delta), np.sin(delta)])
        a_hat = np.array([np.sin(delta), -np.cos(delta)])
        return r_i, d_hat, a_hat


# ---------------------------------------------------------------------------
# The actual robot -- 4 wheels, geometry per CAD (front 60 deg, rear 135 deg)
# ---------------------------------------------------------------------------

ROBOT_WHEELS = [
    Wheel("front-left",  60.0,   0.076),
    Wheel("front-right", -60.0,  0.076),
    Wheel("back-left",   135.0,  0.076),
    Wheel("back-right",  -135.0, 0.076),
]


def cross_2d_z(omega, r):
    """omega x r for omega = omega*z_hat (planar rotation), r a 2D vector.

    From the determinant expansion in the notes:
    | i      j     k |
    | 0      0   omega|  =  (-omega*r_y, omega*r_x)
    | r_x  r_y     0 |
    """
    omega_vec = np.array([0.0, 0.0, omega])
    r_vec = np.array([r[0], r[1], 0.0])
    return np.cross(omega_vec, r_vec)[:2] #takes the first two elements of the 3D vector from the cross product


def wheel_center_velocity(V, omega, wheel: Wheel):
    """Wheel-centre velocity in the robot frame.

    V_center = V + omega x r_i
    """
    r_i, d_hat, a_hat = wheel.frame()
    V_center = np.asarray(V) + cross_2d_z(omega, r_i)
    return V_center, r_i, d_hat, a_hat


def contact_point_velocity(V, omega, theta_dot, wheel: Wheel):
    """Contact-point velocity in the robot frame.

    V_contact = V_center + theta_dot * R_w * d_hat

    where:
        V_center = V + omega x r_i
        theta_dot * R_w * d_hat = velocity of the wheel surface
        at the contact point due to wheel spin.
    """
    V_center, r_i, d_hat, a_hat = wheel_center_velocity(V, omega, wheel)

    V_contact = V_center + theta_dot * R_w * d_hat

    return V_contact, r_i, d_hat, a_hat


def slip_velocities(V, omega, theta_dot, wheel: Wheel):
    """Calculate contact velocity components along d_hat and a_hat.

    v_slip,d = V_contact . d_hat
    v_slip,a = V_contact . a_hat

    The wheel-spin contribution is:
        (theta_dot * R_w * d_hat) . d_hat = theta_dot * R_w

    and:
        (theta_dot * R_w * d_hat) . a_hat = 0
    """
    V_contact, r_i, d_hat, a_hat = contact_point_velocity(V, omega, theta_dot, wheel)

    v_slip_d = np.dot(V_contact, d_hat)
    v_slip_a = np.dot(V_contact, a_hat)

    return v_slip_d, v_slip_a


# ---------------------------------------------------------------------------
# Friction model -- Williams 2002, eq. (11)
# ---------------------------------------------------------------------------

# ASSUMED -- run tilt-test measurement first, but assumed values are okay
MU_D_MAX = 0.30   # TBD: measure via tilt test (wheels aligned, common direction)
MU_A_MAX = 0.10   # TBD: measure via tilt test (wheels perpendicular)
K_SMOOTH = 1000.0  # Williams' own choice, for numerical stability


def mu_d(v_slip_d, mu_d_max=MU_D_MAX, k=K_SMOOTH):
    """mu_d(v_slip_d) = mu_d_max * (2/pi) * atan(k * v_slip_d)  -- eq. (11)."""
    return mu_d_max * (2.0 / np.pi) * np.arctan(k * v_slip_d)


def mu_a(v_slip_a, mu_a_max=MU_A_MAX, k=K_SMOOTH):
    """mu_a(v_slip_a) = mu_a_max * (2/pi) * atan(k * v_slip_a)  -- eq. (11)."""
    return mu_a_max * (2.0 / np.pi) * np.arctan(k * v_slip_a)


def wheel_force(V, omega, theta_dot, wheel: Wheel):
    """Step 5: Calculate wheel force from slip velocities."""
    _, _, d_hat, a_hat = wheel.frame()

    v_slip_d, v_slip_a = slip_velocities(V, omega, theta_dot, wheel)

    F_i = -(ROBOT_MASS * GRAVITY / N_WHEELS) * (mu_d(v_slip_d) * d_hat + mu_a(v_slip_a) * a_hat)

    return F_i, v_slip_d, v_slip_a


def all_wheel_forces(V, omega, theta_dot, wheels):
    """Calculate forces and slip velocities for all wheels."""
    F = np.zeros((N_WHEELS, 2))
    v_slip_d = np.zeros(N_WHEELS)
    v_slip_a = np.zeros(N_WHEELS)

    for i, wheel in enumerate(wheels):
        F[i], v_slip_d[i], v_slip_a[i] = wheel_force(V, omega, theta_dot[i], wheel)

    return F, v_slip_d, v_slip_a


# ---------------------------------------------------------------------------
# Self-tests -- per the checklist at the end of the project notes:
#   "Check wheel directions / Check orthonormality where expected (d and a) /
#    Correct wheel symmetry"
# ---------------------------------------------------------------------------

def _test_orthonormality():
    for wheel in ROBOT_WHEELS:
        _, d_hat, a_hat = wheel.frame()

        assert abs(np.linalg.norm(d_hat) - 1.0) < 1e-12, f"{wheel.name}: d_hat is not a unit vector"
        assert abs(np.linalg.norm(a_hat) - 1.0) < 1e-12, f"{wheel.name}: a_hat is not a unit vector"
        assert abs(np.dot(d_hat, a_hat)) < 1e-12, f"{wheel.name}: d_hat and a_hat are not orthogonal"

    print("  PASS: d_hat and a_hat are orthonormal for all wheels")


def _test_wheel_symmetry():
    """Check that left-right wheel pairs have mirrored positions."""
    wheel_pairs = [
        ("front-left", "front-right"),
        ("back-left", "back-right"),
    ]
    wheels_by_name = {wheel.name: wheel for wheel in ROBOT_WHEELS}

    for left_name, right_name in wheel_pairs:
        r_left, _, _ = wheels_by_name[left_name].frame()
        r_right, _, _ = wheels_by_name[right_name].frame()

        assert np.allclose(r_left * np.array([1.0, -1.0]), r_right, atol=1e-9), f"{left_name}/{right_name}: positions are not mirrored"

    print("  PASS: left-right wheel pairs have mirrored positions")


def _test_zero_velocity_zero_force():
    """Zero robot and wheel velocity should produce zero slip and force."""
    V = np.array([0.0, 0.0])
    omega = 0.0
    theta_dots = np.zeros(N_WHEELS)

    F, v_slip_d, v_slip_a = all_wheel_forces(V, omega, theta_dots, ROBOT_WHEELS)

    assert np.allclose(v_slip_d, 0.0)
    assert np.allclose(v_slip_a, 0.0)
    assert np.allclose(F, 0.0, atol=1e-9), f"nonzero force at zero velocity: {F}"

    print("  PASS: zero velocity produces zero slip and zero force")


def _test_pure_rolling_zero_drive_slip():
    """Matched wheel speed should produce zero drive-direction slip."""
    wheel = ROBOT_WHEELS[0]
    V = np.array([0.5, 0.2])
    omega = 0.3

    V_center, _, d_hat, _ = wheel_center_velocity(V, omega, wheel)
    v_drive = np.dot(V_center, d_hat)

    theta_dot = -v_drive / R_w
    v_slip_d, v_slip_a = slip_velocities(V, omega, theta_dot, wheel)

    assert abs(v_slip_d) < 1e-9, f"expected zero drive-direction slip, got {v_slip_d}"

    print(f"  PASS: matched wheel speed gives zero drive-direction slip (v_slip_a={v_slip_a:.4f} m/s)")


if __name__ == "__main__":
    print("williams_model.py self-tests")
    print("=" * 60)
    _test_orthonormality()
    _test_wheel_symmetry()
    _test_zero_velocity_zero_force()
    _test_pure_rolling_zero_drive_slip()
    print("=" * 60)
    print("All checks passed.")
    print()
    print(f"R_w = {R_w*1000:.4f} mm, R_r = {R_r*1000:.4f} mm, "
          f"roller pitch = {np.degrees(ROLLER_PITCH):.2f} deg")
    print(f"MU_D_MAX, MU_A_MAX are PLACEHOLDERS -- replace with your own "
          f"tilt-test measurements before trusting any force magnitude.")
