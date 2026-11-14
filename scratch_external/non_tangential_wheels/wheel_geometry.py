# From archive/continuous_time/robot_model_voltage.ipynb, cell f26c71d2 (Henry, continuous-time model): wheel positions
# separate from wheel directions. Constants from robot_constants.py (WHEEL_ALPHA_DEG, WHEEL_POSITION_DEG, WHEEL_ROBOT_DIST).

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

# Wheel frame calculation based on the angle of the wheel from the first
# quadrant +x/+y in a counterclockwise direction.
#
# d_hat: wheel drive direction
# a_hat: wheel axle / passive roller direction
# The geometry depends only on the wheel layout in robot_constants, so it is computed once per
# layout and reused: the plant evaluates it tens of thousands of times per simulated second.
# The cache key is the layout itself, so a test that changes rc.WHEEL_ALPHA_DEG or
# rc.WHEEL_POSITION_DEG gets the new geometry. The arrays are read-only because they are shared.
_wheel_geometry_cache = {}


def _wheel_geometry():
    """(d_hat, a_hat, positions) for the current rc wheel layout."""
    key = (tuple(rc.WHEEL_ALPHA_DEG), tuple(rc.WHEEL_POSITION_DEG), float(rc.WHEEL_ROBOT_DIST))
    geometry = _wheel_geometry_cache.get(key)
    if geometry is None:
        alpha = np.radians(np.asarray(rc.WHEEL_ALPHA_DEG, dtype=float))
        delta = alpha + np.pi / 2
        beta = np.radians(np.asarray(rc.WHEEL_POSITION_DEG, dtype=float))
        geometry = (
            np.vstack((np.cos(delta), np.sin(delta))),
            np.vstack((np.cos(alpha), np.sin(alpha))),
            rc.WHEEL_ROBOT_DIST * np.vstack((np.cos(beta), np.sin(beta))),
        )
        for array in geometry:
            array.flags.writeable = False
        _wheel_geometry_cache[key] = geometry
    return geometry

def wheel_directions_body():
    """Return drive and axle unit vectors for all wheels in body coordinates."""
    d_hat, a_hat, _ = _wheel_geometry()
    return d_hat, a_hat

def wheel_positions_body():
    """Contact-point positions r_i from the robot centre P [m], body axes, shape (2, 4).

    Each wheel sits at WHEEL_POSITION_DEG on the circle of radius WHEEL_ROBOT_DIST. Its
    direction comes separately from WHEEL_ALPHA_DEG (wheel_directions_body). With equal
    arrays the wheels are tangential: r_i is parallel to a_i and every yaw lever is L.
    """
    return _wheel_geometry()[2]

def wheel_center_velocity_body(velocity_body, omega):
    """Return each wheel center's velocity in body coordinates [m/s]."""
    radius = wheel_positions_body()
    # omega x r_i for omega along z: (-omega r_y, omega r_x), as in cross_2d_z.
    return np.asarray(velocity_body, dtype=float)[:, None] + omega * np.array([-radius[1], radius[0]])

