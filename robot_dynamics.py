"""
robot_dynamics.py
==================

Rigid-body dynamics: wheel forces -> body accelerations.

Calculates:
    - x, y, phi accelerations from the 4 wheel forces (body frame)

Dynamic equations (project notes, step 6):
    M * a_x = sum_i ( x_hat . F_i )
    M * a_y = sum_i ( y_hat . F_i )
    I_zz * phi_ddot = sum_i ( z_hat . (r_i x F_i) )

These are calculated in the WORLD frame here. The wheel forces from
williams_model.py are calculated in the BODY frame and must be rotated into
the WORLD frame before applying the force equations below.
"""

from __future__ import annotations

from robot_constants import WHEEL_ALPHA_DEG, WHEEL_ROBOT_DIST
from robot_constants import ROBOT_MASS, ROBOT_INERTIA

import numpy as np

def body2world_accelerations(F, phi):
    """F: (n_wheels, 2) wheel force vectors, body frame.

    Returns (x_ddot, y_ddot, phi_ddot) in the world frame.
    """
    rotation = np.array([
        [np.cos(phi), -np.sin(phi)],
        [np.sin(phi),  np.cos(phi)]
    ])

    r_positions = np.column_stack((
        np.cos(np.radians(WHEEL_ALPHA_DEG)) * WHEEL_ROBOT_DIST,
        np.sin(np.radians(WHEEL_ALPHA_DEG)) * WHEEL_ROBOT_DIST
    ))

    F_world = np.dot(F, rotation.T)
    r_world = np.dot(r_positions, rotation.T)

    
    # z_hat . (r_i x F_i) for 2D vectors: r_i,x * F_i,y - r_i,y * F_i,x
    torque_sum = np.cross(r_world, F_world).sum()

    Fx_sum = F_world[:, 0].sum()
    Fy_sum = F_world[:, 1].sum()

    x_ddot = Fx_sum / ROBOT_MASS
    y_ddot = Fy_sum / ROBOT_MASS
    phi_ddot = torque_sum / ROBOT_INERTIA
    return x_ddot, y_ddot, phi_ddot


# ---------------------------------------------------------------------------
# Self-tests
# ---------------------------------------------------------------------------

def _test_izz_positive_and_reasonable():
    """Check that the calculated robot inertia is positive and reasonable."""
    disk_estimate = 0.5 * ROBOT_MASS * 0.088**2

    assert ROBOT_INERTIA > 0.0, f"I_ZZ must be positive, got {ROBOT_INERTIA}"
    assert 0.1 * disk_estimate < ROBOT_INERTIA < 10.0 * disk_estimate, (
        f"I_ZZ = {ROBOT_INERTIA:.6f} kg*m^2 is outside the expected range "
        f"relative to the disk estimate of {disk_estimate:.6f} kg*m^2"
    )

    print(f"  PASS: I_ZZ = {ROBOT_INERTIA:.6f} kg*m^2")


def _test_symmetric_forces_cancel():
    """Check that symmetric forces produce no lateral force or torque."""
    r_positions = np.array([
        [0.038,  0.0658],
        [0.038, -0.0658],
    ])
    F = np.array([
        [1.0,  0.5],
        [1.0, -0.5],
    ])

    phi = 0.0
    x_ddot, y_ddot, phi_ddot = body2world_accelerations(F, r_positions, phi)

    assert abs(y_ddot) < 1e-9, f"expected zero y acceleration, got {y_ddot}"
    assert abs(phi_ddot) < 1e-9, f"expected zero angular acceleration, got {phi_ddot}"

    print("  PASS: symmetric forces produce zero lateral and angular acceleration")


def _test_pure_x_force():
    """Check that a pure x-direction force produces only x acceleration."""
    r_positions = np.zeros((2, 2))
    F = np.array([
        [1.0, 0.0],
        [1.0, 0.0],
    ])

    phi = 0.0
    x_ddot, y_ddot, phi_ddot = body2world_accelerations(F, r_positions, phi)

    expected_x_ddot = 2.0 / ROBOT_MASS

    assert np.isclose(x_ddot, expected_x_ddot), (
        f"expected x acceleration of {expected_x_ddot:.6f}, got {x_ddot:.6f}"
    )
    assert abs(y_ddot) < 1e-12, f"expected zero y acceleration, got {y_ddot}"
    assert abs(phi_ddot) < 1e-12, f"expected zero angular acceleration, got {phi_ddot}"

    print("  PASS: pure x force produces the expected x acceleration")


def _test_pure_y_force():
    """Check that a pure y-direction force produces only y acceleration."""
    r_positions = np.zeros((2, 2))
    F = np.array([
        [0.0, 1.0],
        [0.0, 1.0],
    ])

    phi = 0.0
    x_ddot, y_ddot, phi_ddot = body2world_accelerations(F, r_positions, phi)

    expected_y_ddot = 2.0 / ROBOT_MASS

    assert abs(x_ddot) < 1e-12, f"expected zero x acceleration, got {x_ddot}"
    assert np.isclose(y_ddot, expected_y_ddot), (
        f"expected y acceleration of {expected_y_ddot:.6f}, got {y_ddot:.6f}"
    )
    assert abs(phi_ddot) < 1e-12, f"expected zero angular acceleration, got {phi_ddot}"

    print("  PASS: pure y force produces the expected y acceleration")


def _test_pure_torque():
    """Check that a force pair produces the expected angular acceleration."""
    r_positions = np.array([
        [0.0,  0.05],
        [0.0, -0.05],
    ])
    F = np.array([
        [1.0, 0.0],
        [-1.0, 0.0],
    ])

    phi = 0.0
    x_ddot, y_ddot, phi_ddot = body2world_accelerations(F, r_positions, phi)

    expected_tau = 0.05 + 0.05
    expected_phi_ddot = expected_tau / ROBOT_INERTIA

    assert abs(x_ddot) < 1e-12, f"expected zero x acceleration, got {x_ddot}"
    assert abs(y_ddot) < 1e-12, f"expected zero y acceleration, got {y_ddot}"
    assert np.isclose(phi_ddot, expected_phi_ddot), (
        f"expected angular acceleration of {expected_phi_ddot:.6f}, "
        f"got {phi_ddot:.6f}"
    )

    print("  PASS: pure torque produces the expected angular acceleration")

def _test_body_to_world_rotation():
    """A body-frame force should rotate correctly into the world frame."""

    F = np.array([
        [1.0, 0.0],
        [0.0, 0.0],
        [0.0, 0.0],
        [0.0, 0.0]
    ])

    r_positions = np.zeros((4, 2))
    phi = np.pi / 2

    x_ddot, y_ddot, phi_ddot = body2world_accelerations(F, r_positions, phi)

    assert np.isclose(x_ddot, 0.0)
    assert np.isclose(y_ddot, 1.0 / ROBOT_MASS)
    assert np.isclose(phi_ddot, 0.0)

def _test_body_to_world_torque_rotation():
    """Torque should be unchanged when both r and F are rotated."""

    F = np.array([
        [0.0, 1.0],
        [0.0, 0.0],
        [0.0, 0.0],
        [0.0, 0.0]
    ])

    r_positions = np.array([
        [1.0, 0.0],
        [0.0, 0.0],
        [0.0, 0.0],
        [0.0, 0.0]
    ])

    phi = np.pi / 2

    x_ddot, y_ddot, phi_ddot = body2world_accelerations(F, r_positions, phi)

    expected_phi_ddot = 1.0 / ROBOT_INERTIA

    assert np.isclose(phi_ddot, expected_phi_ddot)
    
if __name__ == "__main__":
    print("robot_dynamics.py self-tests")
    print("=" * 60)

    _test_izz_positive_and_reasonable()
    _test_symmetric_forces_cancel()
    _test_pure_x_force()
    _test_pure_y_force()
    _test_pure_torque()
    _test_body_to_world_rotation()
    _test_body_to_world_torque_rotation()


    print("=" * 60)
    print("All checks passed.")
