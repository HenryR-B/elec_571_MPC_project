import numpy as np

# Robot
ROBOT_MASS = 2.698                  # [kg]
GRAVITY = 9.81                       # [m/s^2]
N_WHEELS = 4

WHEEL_ROBOT_DIST = (158.97 / 2) * 1e-3  # contact point circle radius

# ---------------------------------------------------------------------------
# Mass properties from CAD (Onshape, whole robot).
#
# CAD reports, about the centre of gravity (CoG):
#   I_xx = 6398.534, I_yy = 6120.905, I_zz = 7443.681 kg*mm^2
#   products: I_xy = -45.569, I_xz = -3.279, I_yz = -424.089 kg*mm^2 (not used)
#   CoG = (x=0.597, y=10.206, z=61.146) mm from the robot's reference point P:
#   the centre of the wheel-contact circle, on the floor. (The CAD assembly frame
#   reports the CoG at y = 34.729 mm, which puts P at y = 24.523 mm there: inferred
#   from the two values, not read from CAD.)
# Body axes: x right, y forward (front wheels at +y), z up. CAD x/y are ASSUMED
# to be the same axes.
#
# The planar state tracks P, not the CoG, so the plant uses the full planar mass
# matrix about P (off-diagonal -m*c_y, m*c_x). Its yaw entry is I_zz about P:
# I_zz = I_zz(CoG) + M * (x_cog^2 + y_cog^2)     -- parallel axis theorem.
# ---------------------------------------------------------------------------

IZZ_COG_KG_MM2 = 7443.681          # kg*mm^2
IXX_COG_KG_MM2 = 6398.534          # kg*mm^2, roll/pitch: about body x through the CoG
IYY_COG_KG_MM2 = 6120.905          # kg*mm^2, about body y through the CoG
COG_X_MM, COG_Y_MM = 0.597, 10.206 #mm, from P
COG_Z_MM = 61.146                  # mm, CoG height above the floor (h)



def set_center_of_gravity(x_mm=COG_X_MM, y_mm=COG_Y_MM, z_mm=COG_Z_MM):
    """Place the CoG at (x, y, z) mm from P (body axes) and update the values derived from it.

    The mass distribution moves rigidly, so the inertia about the CoG stays the CAD value
    and the yaw inertia about P follows from the parallel-axis theorem. With (0, 0, h) the
    mass is centred: ROBOT_INERTIA = I_zz(CoG) and the planar mass matrix is diagonal.
    Plants read these values when they are built, so call this before building one.
    """
    global COG_OFFSET, COG_HEIGHT, ROBOT_INERTIA
    COG_OFFSET = np.array([x_mm, y_mm], dtype=float) * 1e-3   # [m] CoG from P, body axes
    COG_HEIGHT = z_mm * 1e-3                                   # [m]
    ROBOT_INERTIA = (IZZ_COG_KG_MM2 + ROBOT_MASS * (x_mm**2 + y_mm**2)) * 1e-6  # [kg m^2] about P


set_center_of_gravity()  # CAD position: ROBOT_INERTIA = 7725.673 kg mm^2 = 0.007725673 kg m^2
# REAL SYSTEM VALUES: the CAD numbers above (CoG 0.597, 10.206, 61.146 mm from P;
# I_zz(CoG) = 7443.681 kg mm^2). The notebooks use them by default (COG_POSITION_MM in
# their configuration cell). Dana's simplification, (0, 0, h), centres the CoG in x and y,
# which gives ROBOT_INERTIA = I_zz(CoG) = 7443.681 kg mm^2 and a diagonal mass matrix.
# constant robot inertia cause the robot does not change shape
ROBOT_INERTIA_XX = IXX_COG_KG_MM2 * 1e-6              # [kg m^2] about the CoG
ROBOT_INERTIA_YY = IYY_COG_KG_MM2 * 1e-6              # [kg m^2] about the CoG

# Wheel-ground contact compliance (X-ring rollers on the field surface), used by the
# "dynamic" chassis load model. Both UNMEASURED; treat as sensitivity parameters.
#   Stiffness from an ASSUMED static deflection of 0.3 mm under the nominal wheel load
#   (chassis_load_transfer_plan.md, section 5): k = (m g / 4) / 0.3 mm ~ 2.2e4 N/m.
#   Damping from an ASSUMED heave damping ratio zeta: per contact c = zeta * sqrt(k m),
#   which gives zeta for the heave mode of all four contacts (4k, 4c, m).
CONTACT_STATIC_DEFLECTION = 0.3e-3       # [m]   ASSUMED
CONTACT_DAMPING_RATIO = 0.2              # [-]   ASSUMED
CONTACT_STIFFNESS = (ROBOT_MASS * GRAVITY / N_WHEELS) / CONTACT_STATIC_DEFLECTION   # [N/m]
CONTACT_DAMPING = CONTACT_DAMPING_RATIO * np.sqrt(CONTACT_STIFFNESS * ROBOT_MASS)    # [N s/m]

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
# WHEEL_ALPHA_DEG sets each wheel's axle direction (CCW from +x); its drive direction is
# the axle turned +90 deg. WHEEL_POSITION_DEG sets where each wheel's contact point sits
# on the circle of radius WHEEL_ROBOT_DIST. Tangential wheels (the real robot): the two
# arrays are identical, the axle points at the centre, and every yaw lever r_i x d_i = L.
# Different values make a wheel non-tangential: e.g. a wheel at position 30 deg with
# alpha 45 deg drives along the 45-deg wheel's direction from the 30-deg spot.
WHEEL_POSITION_DEG = list(WHEEL_ALPHA_DEG)

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
# Sliding drive friction, "continuous" law in the notebooks (DRIVE_SLIDING_LAW):
#   F_d = -N mu_d(|v|) tanh(v / DRIVE_SLIP_REGULARIZATION)
# A numerical smoothing of sign(v), not a measured value: the force reaches mu_d,s N within a
# few mm/s of slip, so a wheel that breaks away keeps the force it had.
DRIVE_SLIP_REGULARIZATION = 1.0e-3  # [m/s]
# Karnopp breakaway band: a wheel that breaks loose with |v_d| below this keeps mu_d(|v|) N in the
# direction of the force it needed (the slip law is zero at zero slip). 5 x the regularisation, so
# the slip law has reached 99.99% of mu_d N at the band edge and the two agree there.
DRIVE_BREAKAWAY_BAND = 5.0 * DRIVE_SLIP_REGULARIZATION  # [m/s]

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
# ESTIMATED C_rr = 0.025. The notebooks' INCLUDE_ESTIMATED_ROLLING_RESISTANCE switch
# (configuration cell) sets WHEEL_ROLLING_RESISTANCE_COEFF to this value or to 0.
#   Duda et al. 2025 (Sensors 25:5026) measured a 125 mm omniwheel with
#   95A polyurethane rollers on concrete/aluminium: drive-direction (0 deg)
#   force/load = 0.008-0.010 (bearing friction included); axle-direction (90 deg,
#   rollers rolling) = 0.044-0.047.
#   Route 1: scale 0.009 for our 60 mm wheel (~x1.4, C_rr ~ 1/sqrt(D)) and carpet
#     vs hard floor (~x2; carpet 0.014-0.020 for wheelchair wheels, RESNA 2018)
#     -> ~0.025.
#   Route 2: drive/axle ratio from Duda, 0.0088/0.044 ~ 0.2, times our measured
#     MU_A_STAT (0.1-0.2, a lumped roller rolling resistance) -> 0.02-0.04.
#   Upper bound: MU_A_STAT (it adds roller-bushing friction the main wheel lacks).
#   For scale, Conceicao et al. (2008) identified the equivalent of C_rr ~ 0.07
#   (and b_w ~ 4.7e-4) on a 2.34 kg 4-wheel robot with a 5:1 gearbox.
# b_w = 2e-4 ASSUMED: same as WHEEL_VISCOUS_DRAG_NM_S_RAD in Dana's robot22102026D.
WHEEL_ROLLING_RESISTANCE_ESTIMATE = 0.025  # C_rr [-]        ESTIMATED from literature
WHEEL_ROLLING_RESISTANCE_COEFF = WHEEL_ROLLING_RESISTANCE_ESTIMATE  # value used; the notebook switch can set 0
WHEEL_BEARING_VISCOUS_DRAG = 2.0e-4    # b_w [N m s/rad]     ASSUMED
WHEEL_LOSS_SMOOTHING_SPEED = 0.5       # w_eps [rad/s], numerical regularization of sign(theta_dot)


# ---------------------------------------------------------------------------
# Disturbance parameters (ESTIMATED surrogates, not measured). The on/off switches are in
# the notebooks' configuration cell.
#   Friction offset (surface): each wheel's friction coefficients are scaled by
#     f_i = clip(1 + b_i, FRICTION_SCALE_MIN, FRICTION_SCALE_MAX), b_i ~ N(0, FRICTION_OFFSET_STD),
#     fixed for the run.
#   Load vibration (wheel geometry and motion): each wheel's normal load is scaled by
#     g_i = 1 + a_i n_i (n_i smooth correlated noise in [-1, 1]), then the four are rescaled to
#     keep their sum:
#     a_i = min(MAX, [BASE + kappa (eps/2) (N_ROLLERS v_total / R_w)^2 / g]
#                    * (1 + ACCEL_GAIN |theta_ddot_i| / REF_ACCEL))
#     v_total = sqrt(|v|^2 + (L omega)^2) is the robot's total speed at the wheels. The speed
#     term is the vertical acceleration of an axle following the roller envelope, as a
#     fraction of g: the relative load change it causes. theta_ddot_i is low-pass filtered.
# ---------------------------------------------------------------------------
# Envelope out-of-roundness eps (peak-to-peak axle height change while rolling).
# Duda et al. 2021 (Sensors 21:6931) measured ~0.05 mm on a 125 mm double-row
# omniwheel at 0 deg (pure rolling), 16 passes per revolution; ~0.03 mm at 90 deg.
# A single-row gap estimate for our wheel, ROLLER_GAP^2 / (8 R_w) = 0.28 mm, is an upper
# bound; double rows fill each other's gaps. 0 = perfectly round wheel.
WHEEL_ENVELOPE_IRREGULARITY = 0.05e-3   # eps [m]                          ESTIMATED (Duda 2021)
LOAD_VIBRATION_COMPLIANCE = 0.3     # kappa: share of the rigid-axle acceleration that
                                        # reaches the contact (X-ring compliance)  ESTIMATED
LOAD_VIBRATION_BASE = 0.02          # spread at rest                   ESTIMATED
LOAD_VIBRATION_ACCEL_GAIN = 0.10    # amplification per REF_ACCEL     ESTIMATED
LOAD_VIBRATION_REF_ACCEL = 100.0    # [rad/s^2]
LOAD_VIBRATION_ACCEL_FILTER_TIME = 0.05  # [s] low-pass on |theta_ddot| (keeps the amplitude from jumping)
LOAD_VIBRATION_MAX = 0.5           # cap on a_i (< 1 keeps every load positive)
LOAD_VIBRATION_CORRELATION_TIME = 0.1   # [s] how fast the random part changes (was 0.02: near white noise per DT)
FRICTION_OFFSET_STD = 0.05              # fixed per-wheel spread (Dana's FRICTION_NOISE_STD)
FRICTION_SCALE_MIN = 0.2                # clip on f_i
FRICTION_SCALE_MAX = 1.8
