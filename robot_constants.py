import numpy as np
# Robot
ROBOT_MASS = 2.698                  # [kg]
GRAVITY = 9.81                       # [m/s^2]
N_WHEELS = 4

WHEEL_ROBOT_DIST = (158.97 / 2) * 1e-3  # contact point circle radius

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

ROBOT_INERTIA = (IZZ_COG_KG_MM2 + ROBOT_MASS * (COG_X_MM**2 + COG_Y_MM**2))*1e-6 # 7725.673 kg mm^2 = 0.007725673 kgm^2
# constant robot inertia cause the robot does not change shape

# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

WHEEL_NAMES = [
    "front-right",
    "front-left",
    "back-left",
    "back-right"
]

WHEEL_ALPHA_DEG = [
    0.000 + 30.0,
    180.0 - 30.0,
    180.0 + 45.0,
    360.0 - 45.0
]

# Wheel
F_N = (ROBOT_MASS * GRAVITY / N_WHEELS)
WHEEL_RADIUS = 0.0598292 / 2        # wheel radius [m] = 29.9146 mm
WHEEL_INERTIA = 16.576 * 1e-6       # kg*m^2
WHEEL_MASS = 0.066621               # kg (including rotor, all rotating mass)

# Roller
ROLLER_RADIUS = 0.014378 / 2       # roller radius [m] = 7.189 mm
N_ROLLERS = 16
ROLLER_PITCH = 2 * np.pi / N_ROLLERS   # 22.5 deg
ROLLER_WIDTH = 3.545 * 1e-3 # roller width [m] = 3.545 mm
ROLLER_GAP = 8.151 * 1e-3 # roller to roller gap at edge point on rollers [m] = 8.151 mm
ROLLER_WIDTH_DTHETA = np.degrees(ROLLER_PITCH) * ROLLER_WIDTH / (ROLLER_WIDTH + ROLLER_GAP)  # roller contact width [deg]
ROLLER_GAP_DTHETA = np.degrees(ROLLER_PITCH) - ROLLER_WIDTH_DTHETA  # gap width [deg]


# Motor
VEL_P_GAIN = 0.1
VEL_I_GAIN = 1

IQ_LIMIT = 5.0  # [A]

MOTOR_KV = 360
MOTOR_KT = 60 / (2 * np.pi * MOTOR_KV)

MOTOR_RESISTANCE = 0.24 # Ohms
MOTOR_INDUCTANCE = 0.133e-3 # H
MAX_MOTOR_VOLTAGE = 16.8 # V

# Wheel Roller Friction
MU_D_STAT = 0.9   # TBD: measure via tilt test (wheels aligned, common direction)
MU_D_DYN = 0.422 # dynamic friction coefficient, assumed based on lower than the calculated slip at 2.5m/s => 0.422
ALPHA_D = 10 # relatively fast transition from rolling to slipping due to motor torque
K_SMOOTH_D = 200.0 

# F_N is the nominal normal load on one wheel. MU_D_STAT * F_N is the
# corresponding per-wheel drive-traction threshold for equal load sharing.
F_ROLL_LIMIT = MU_D_STAT * F_N


MU_A_STAT = 0.2   # TBD: measure via tilt test (wheels perpendicular)
MU_A_DYN = 0.322 # calculated as increase because bushings will be forced to move at a faster rate than they can without a driving force, increasing friction
ALPHA_A = 1 # relatively slow transition from rolling to slipping
K_SMOOTH_A = 200




















# Wheel losses (rolling resistance + bearing drag). Torque on each wheel:
#   tau_loss = -C_rr * N * R_w * tanh(theta_dot / w_eps) - b_w * theta_dot
# These act while the wheel ROLLS, so they are separate from MU_D_* / MU_A_*,
# which set the force limit and force once a contact SLIDES. They are the only
# losses that act on pure yaw: for pure spin the axial (roller) slip is zero
# because each axle points at the robot centre. With both at 0 the plant is
# lossless in yaw and coasts forever.
# ASSUMED (not measured):
#   C_rr = 0.02: rolling resistance on carpet is ~0.014-0.020 for wheelchair
#     wheels (RESNA 2018); small wheels sit at the high end.
#   b_w = 2e-4: same as WHEEL_VISCOUS_DRAG_NM_S_RAD in Dana's robot22102026D.
# For scale, Conceicao et al. (2008) identified the equivalent of
# b_w ~ 4.7e-4 and C_rr ~ 0.07 on a 2.34 kg 4-wheel robot with a 5:1 gearbox.
WHEEL_ROLLING_RESISTANCE_COEFF = 0.02  # C_rr [-]            ASSUMED
WHEEL_BEARING_VISCOUS_DRAG = 2.0e-4    # b_w [N m s/rad]     ASSUMED
WHEEL_LOSS_SMOOTHING_SPEED = 0.5       # w_eps [rad/s], numerical regularization of sign(theta_dot)
