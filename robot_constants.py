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

# Motor
VEL_P_GAIN = 0.05
VEL_I_GAIN = 0.8  

VOLTAGE_LIMIT = 16.8
IQ_LIMIT = 20.0  # [A]

MOTOR_KV = 360
MOTOR_KT = 60 / (2 * np.pi * MOTOR_KV)

MOTOR_RESISTANCE = 0.24
MOTOR_INDUCTANCE = 0.133e-3

# Wheel Roller Friction
MU_D_STAT = 1.15   # TBD: measure via tilt test (wheels aligned, common direction)
MU_D_DYN = 0.422 # dynamic friction coefficient, assumed based on lower than the calculated slip at 2.5m/s => 0.422
ALPHA_D = 10 # relatively fast transition from rolling to slipping due to motor torque
K_SMOOTH_D = 200.0 


MU_A_STAT = 0.11   # TBD: measure via tilt test (wheels perpendicular)
MU_A_DYN = 0.324 # calculated as increase because bushings will be forced to move at a faster rate than they can without a driving force, increasing friction
ALPHA_A = 1 # relatively slow transition from rolling to slipping
K_SMOOTH_A = 200



















