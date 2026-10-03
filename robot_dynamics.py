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

import numpy as np

ROBOT_MASS = 2.698   # kg

# ---------------------------------------------------------------------------
# I_zz via parallel axis theorem, from CAD mass properties.
#
# CAD reports (about the centre of gravity):
#   I_zz(CoG) = 7443.681kg*mm^2
#   CoG = (x=0.597, y=10.206, z=61.146) mm, relative to the CAD origin.
#
# I_zz = I_zz(CoG) + M * (x_cog^2 + y_cog^2)     -- parallel axis theorem,
# shifting from the CoG to whatever point is used as the robot's rotation
# origin.
# ---------------------------------------------------------------------------

IZZ_COG_KG_MM2 = 7443.681          # kg*mm^2
COG_X_MM, COG_Y_MM = 0.597, 10.206 #mm

I_ZZ = (IZZ_COG_KG_MM2 + ROBOT_MASS * (COG_X_MM**2 + COG_Y_MM**2))*1e-6 # 7725.673 kg mm^2 = 0.007725673 kgm^2
# constant robot inertia cause the robot does not change shape

def body2world_accelerations(F, r_positions, phi):
    """F: (n_wheels, 2) wheel force vectors, body frame.
    r_positions: (n_wheels, 2) wheel position vectors r_i, body frame.

    Returns (x_ddot, y_ddot, phi_ddot) in the world frame.
    """
    rotation = np.array([
        [np.cos(phi), -np.sin(phi)],
        [np.sin(phi),  np.cos(phi)]
    ])

    F_world = np.dot(F, rotation.T)
    r_world = np.dot(r_positions, rotation.T)

    
    # z_hat . (r_i x F_i) for 2D vectors: r_i,x * F_i,y - r_i,y * F_i,x
    tau_sum = np.cross(r_world, F_world).sum()

    Fx_sum = F_world[:, 0].sum()
    Fy_sum = F_world[:, 1].sum()

    x_ddot = Fx_sum / ROBOT_MASS
    y_ddot = Fy_sum / ROBOT_MASS
    phi_ddot = tau_sum / I_ZZ
    return x_ddot, y_ddot, phi_ddot


# ---------------------------------------------------------------------------
# Self-tests
# ---------------------------------------------------------------------------

def _test_izz_positive_and_reasonable():
    """Check that the calculated robot inertia is positive and reasonable."""
    disk_estimate = 0.5 * ROBOT_MASS * 0.088**2

    assert I_ZZ > 0.0, f"I_ZZ must be positive, got {I_ZZ}"
    assert 0.1 * disk_estimate < I_ZZ < 10.0 * disk_estimate, (
        f"I_ZZ = {I_ZZ:.6f} kg*m^2 is outside the expected range "
        f"relative to the disk estimate of {disk_estimate:.6f} kg*m^2"
    )

    print(f"  PASS: I_ZZ = {I_ZZ:.6f} kg*m^2")


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
    expected_phi_ddot = expected_tau / I_ZZ

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

    expected_phi_ddot = 1.0 / I_ZZ

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
