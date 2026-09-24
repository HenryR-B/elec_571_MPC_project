# ELEC 571 MPC Project

Python plant model for the Small Size League omni-wheel robot.

The main physics implementation is williams_model.py. It is based on:

> R. L. Williams II, B. E. Carter, P. Gallina, and G. Rosati, "Dynamic Model with Slip for Wheeled Omni-Directional Robots," IEEE Transactions on Robotics and Automation, 18(3), 285-293, 2002.

Paper: https://people.ohio.edu/williams/html/PDF/IEEETRA02.pdf

This README describes the equations that are actually implemented in the current Python code. It separates published equations from project-specific assumptions and helper functions.

---

## 1. State and input

The continuous state used by williams_model.py is

$$
\mathbf{x}
=
\begin{bmatrix}
x & y & \phi & V_x & V_y & \omega &
\theta_1 & \cdots & \theta_N
\end{bmatrix}^{T}.
$$

Where:

- x,y: robot centre position in the inertial frame
- phi: robot yaw angle
- Vx,Vy: robot centre velocity in the inertial frame
- omega = dot(phi): robot yaw rate
- theta_i: wheel rotation angle used to determine roller/gap contact
- N: number of driven omni-wheels, four for the current robot

The plant input is

$$
u_i=\dot{\theta}_i
$$

for each wheel.

In this model, wheel angular velocity is therefore an input, not a motor torque or PWM command.

---

## 2. Wheel coordinate system

For wheel i:

- p_i is the vector from the robot centre to the wheel centre
- r_hat_i is the wheel axle direction
- s_hat_i is the wheel drive/peripheral direction

The current code assumes the axle is radial:

$$
\hat{\mathbf r}_i
=
\begin{bmatrix}
\cos\alpha_i\\
\sin\alpha_i
\end{bmatrix}
$$

and the drive direction is tangential:

$$
\hat{\mathbf s}_i
=
\begin{bmatrix}
-\sin\alpha_i\\
\cos\alpha_i
\end{bmatrix}.
$$

The wheel position is

$$
\mathbf p_{i,M}
=
r_i
\begin{bmatrix}
\cos\alpha_i\\
\sin\alpha_i
\end{bmatrix}.
$$

These are body-frame vectors.

The body-to-inertial rotation matrix used by the code is

$$
\mathbf R(\phi)
=
\begin{bmatrix}
\cos\phi & -\sin\phi\\
\sin\phi & \cos\phi
\end{bmatrix}.
$$

Therefore,

$$
\mathbf p_i=\mathbf R(\phi)\mathbf p_{i,M},
$$

$$
\hat{\mathbf r}_i=\mathbf R(\phi)\hat{\mathbf r}_{i,M},
$$

$$
\hat{\mathbf s}_i=\mathbf R(\phi)\hat{\mathbf s}_{i,M}.
$$

This matches the frame transformation used by Williams.

---

## 3. Wheel contact-point kinematics

Williams defines the instantaneous velocity of the point on wheel i that contacts the ground as

$$
\mathbf v_i
=
\mathbf V_G
+
\boldsymbol{\omega}\times\mathbf r_i
+
\mathbf v_{r,i}.
$$

The current Python implementation computes the first two terms as

$$
\mathbf v_{c,i}
=
\mathbf V_G
+
\boldsymbol{\omega}\times\mathbf p_i
$$

with

$$
\mathbf V_G
=
\begin{bmatrix}
V_x\\
V_y
\end{bmatrix},
\qquad
\boldsymbol{\omega}
=
\begin{bmatrix}
0\\
0\\
\omega
\end{bmatrix}.
$$

In 2-D,

$$
\boldsymbol{\omega}\times\mathbf p_i
=
\begin{bmatrix}
-\omega p_{i,y}\\
\omega p_{i,x}
\end{bmatrix}.
$$

The code calls this v_contact. It intentionally excludes the wheel peripheral velocity because that term is added when computing longitudinal slip.

This is equivalent to Williams' Eq. (1) before adding the peripheral term.

---

## 4. Wheel peripheral velocity

Williams defines the wheel angular-velocity vector as

$$
\boldsymbol{\dot{\theta}}_i
=
\dot{\theta}_i\hat{\mathbf r}_i
$$

and the wheel-centre-to-contact radius vector as rho_i.

The peripheral contact velocity is

$$
\mathbf v_{r,i}
=
\boldsymbol{\dot{\theta}}_i
\times
\boldsymbol{\rho}_i.
$$

Because r_hat_i and s_hat_i are perpendicular, the magnitude of the peripheral velocity is

$$
\rho_i\dot{\theta}_i.
$$

The current code uses the equivalent scalar projection directly in the longitudinal slip equation:

$$
+\rho_i\dot{\theta}_i.
$$

The sign is a convention determined by the chosen positive theta_i direction and s_hat_i. The current code uses that convention consistently in both inverse kinematics and the plant model.

---

## 5. Longitudinal slip

Williams' longitudinal sliding velocity is obtained by projecting the contact-point velocity onto s_hat_i.

The implementation is

$$
\boxed{
v_{W,i}
=
\mathbf v_{c,i}\cdot\hat{\mathbf s}_i
+
\rho_i u_i
}
$$

with

$$
u_i=\dot{\theta}_i.
$$

This is the equation implemented in contact_kinematics().

### Sign check

The code's inverse-kinematics helper chooses

$$
u_i
=
-\frac{
\mathbf v_{c,i}^{cmd}\cdot\hat{\mathbf s}_i
}{\rho_i}
$$

so that the commanded ideal motion satisfies

$$
v_{W,i}=0.
$$

That is internally consistent with the plant's sign convention.

---

## 6. Transverse slip

Williams defines transverse sliding velocity by projecting the contact-point velocity onto the wheel axle direction:

$$
\boxed{
v_{T,i}
=
\mathbf v_{c,i}\cdot\hat{\mathbf r}_i
}
$$

There is no wheel-speed term in this equation.

The current Python implementation matches this directly.

---

## 7. Smooth friction law

Williams replaces discontinuous Coulomb friction at zero slip with the smooth function

$$
\boxed{
\mu(v)
=
\mu_{\max}
\frac{2}{\pi}
\tan^{-1}(kv)
}
$$

The current smooth_friction_coefficient() function implements this equation directly.

The important detail is that the function is signed:

$$
\operatorname{sign}(\mu(v))
=
\operatorname{sign}(v).
$$

The force equation supplies the negative sign, so the friction force acts opposite the slip.

Williams states that k=1000 was selected empirically for steepness and numerical stability in their Simulink model.

The current Python default is also

$$
k=1000.
$$

This is a numerical smoothing function, not a claim about the microscopic friction law of the SSL carpet.

---

## 8. Roller/gap friction model

The improved Williams model recognizes that an omni-wheel does not always present the same surface to the floor.

For N_r rollers, the angular pitch is

$$
\boxed{
\Delta\theta_{\mathrm{pitch}}
=
\frac{2\pi}{N_r}
}
$$

For the current wheel:

$$
N_r=16
$$

and therefore

$$
\boxed{
\Delta\theta_{\mathrm{pitch}}=22.5^\circ.
}
$$

Each pitch consists of:

- a roller-contact sector Delta theta prime
- a rigid-material gap sector Delta theta double-prime

such that

$$
\boxed{
\Delta\theta'
+
\Delta\theta''
=
\frac{2\pi}{N_r}.
}
$$

The current code represents this using roller_fraction:

$$
f_r
=
\frac{\Delta\theta'}{\Delta\theta_{\mathrm{pitch}}}.
$$

For a given wheel angle, the code selects one of two coefficient pairs.

### Roller contact

$$
\mu_W=\mu'_W(v_W),
\qquad
\mu_T=\mu'_T(v_T).
$$

### Rigid-gap contact

$$
\mu_W=\mu''_W(v_W),
\qquad
\mu_T=\mu''_T(v_T).
$$

This is the essential improvement made by Williams.

The code does not give each passive roller its own angular-velocity state or inertia. Only the driven wheel angle theta_i is stored.

---

## 9. Current wheel geometry

The current wheel geometry in williams_model.py is derived from the measured dimensions:

$$
D_{\mathrm{inner}}=31.04270\ \mathrm{mm}
$$

$$
D_{\mathrm{outer}}=59.82900\ \mathrm{mm}.
$$

The roller diameter is calculated by

$$
D_{\mathrm{roller}}
=
\frac{
D_{\mathrm{outer}}-D_{\mathrm{inner}}
}{2}
$$

giving

$$
\boxed{
D_{\mathrm{roller}}=14.39315\ \mathrm{mm}
}
$$

and

$$
\boxed{
R_{\mathrm{roller}}=7.196575\ \mathrm{mm}.
}
$$

The wheel contact radius currently used by the code is

$$
\boxed{
\rho=29.915\ \mathrm{mm}.
}
$$

The measured minimum gap is

$$
g_{\min}=3.06464\ \mathrm{mm}
$$

and the measured gap at the roller centre is

$$
g_{\mathrm{center}}=5.59634\ \mathrm{mm}.
$$

The current code extrapolates the floor-contact gap as

$$
g_{\mathrm{floor}}
=
g_{\mathrm{center}}
+
(g_{\mathrm{center}}-g_{\min})
$$

which gives

$$
\boxed{
g_{\mathrm{floor}}=8.12804\ \mathrm{mm}.
}
$$

The code then interprets that value as a chord width at the wheel contact radius and obtains

$$
\Delta\theta_{\mathrm{gap}}
=
2\sin^{-1}
\left(
\frac{g_{\mathrm{floor}}}{2\rho}
\right).
$$

The resulting values currently in the code are approximately

$$
\Delta\theta_{\mathrm{gap}}=15.6158^\circ
$$

and

$$
\Delta\theta_{\mathrm{roller}}
=
22.5^\circ-15.6158^\circ
=
6.8842^\circ.
$$

Therefore

$$
\boxed{
f_r=0.305964.
}
$$

### Important status of this geometry

Everything above matches the current code, but the floor-gap extrapolation and chord interpretation are project-specific geometric assumptions.

They are not equations taken from Williams.

They should eventually be checked directly against the CAD/physical wheel.

---

## 10. Wheel friction force

Williams' friction force on wheel i is

$$
\boxed{
\mathbf F_i
=
-N_i
\left[
\mu_W(v_{W,i})\hat{\mathbf s}_i
+
\mu_T(v_{T,i})\hat{\mathbf r}_i
\right].
}
$$

The current implementation uses equal static loading:

$$
N_i=\frac{mg}{N}.
$$

For the four-wheel robot:

$$
\boxed{
N_i=\frac{mg}{4}.
}
$$

The Python code then evaluates

$$
F_i
=
-\frac{mg}{4}
\left[
\mu_W\hat{\mathbf s}_i
+
\mu_T\hat{\mathbf r}_i
\right].
$$

This is a direct generalization of Williams' three-wheel equation to N=4 wheels.

The equal-load assumption is a modelling assumption. It ignores dynamic load transfer and unequal static loading.

---

## 11. Robot translational dynamics

The total ground force is

$$
\mathbf F_{\mathrm{total}}
=
\sum_{i=1}^{N}\mathbf F_i.
$$

The translational dynamics are

$$
\boxed{
m\dot V_x
=
\sum_i F_{i,x}
}
$$

and

$$
\boxed{
m\dot V_y
=
\sum_i F_{i,y}.
}
$$

The current code implements these as

$$
\dot V_x
=
\frac{F_{\mathrm{total},x}}{m},
\qquad
\dot V_y
=
\frac{F_{\mathrm{total},y}}{m}.
$$

This matches Williams' Eq. (9), generalized from three wheels to four.

---

## 12. Robot yaw dynamics

The total yaw torque about the robot centre is

$$
\tau_z
=
\sum_i
\left(
\mathbf p_i\times\mathbf F_i
\right)_z.
$$

In scalar 2-D form,

$$
\boxed{
\tau_z
=
\sum_i
\left(
p_{i,x}F_{i,y}
-
p_{i,y}F_{i,x}
\right).
}
$$

The rotational dynamics are

$$
\boxed{
I_z\dot\omega
=
\tau_z
}
$$

or

$$
\boxed{
\dot\omega
=
\frac{\tau_z}{I_z}.
}
$$

The current wheel_forces() and body_wrench() functions implement this directly.

---

## 13. Kinematic state equations

The remaining state equations are

$$
\boxed{
\dot x=V_x,
\qquad
\dot y=V_y,
\qquad
\dot\phi=\omega.
}
$$

The wheel-angle states evolve according to

$$
\boxed{
\dot\theta_i=u_i.
}
$$

Together, these equations form the continuous-time plant.

---

## 14. Complete implemented plant

The current Python code therefore implements

$$
\boxed{
\begin{aligned}
\dot x &= V_x\\
\dot y &= V_y\\
\dot\phi &= \omega\\
m\dot V_x &= \sum_i F_{i,x}\\
m\dot V_y &= \sum_i F_{i,y}\\
I_z\dot\omega &=
\sum_i
\left(
p_{i,x}F_{i,y}
-
p_{i,y}F_{i,x}
\right)\\
\dot\theta_i &= u_i
\end{aligned}
}
$$

with

$$
\boxed{
v_{W,i}
=
\left(
\mathbf V_G+
\boldsymbol\omega\times\mathbf p_i
\right)
\cdot
\hat{\mathbf s}_i
+
\rho_i u_i
}
$$

$$
\boxed{
v_{T,i}
=
\left(
\mathbf V_G+
\boldsymbol\omega\times\mathbf p_i
\right)
\cdot
\hat{\mathbf r}_i
}
$$

and

$$
\boxed{
\mathbf F_i
=
-N_i
\left[
\mu_W(v_{W,i})\hat{\mathbf s}_i
+
\mu_T(v_{T,i})\hat{\mathbf r}_i
\right].
}
$$

The coefficient functions are

$$
\boxed{
\mu(v)
=
\mu_{\max}
\frac{2}{\pi}
\tan^{-1}(kv).
}
$$

The values of mu_max depend on whether the wheel angle is in the roller or rigid-gap sector.

---

## 15. Numerical integration

rk4_step() integrates the continuous-time plant using classical fourth-order Runge-Kutta:

$$
k_1=f(x_k,u_k)
$$

$$
k_2=f\left(x_k+\frac{\Delta t}{2}k_1,u_k\right)
$$

$$
k_3=f\left(x_k+\frac{\Delta t}{2}k_2,u_k\right)
$$

$$
k_4=f\left(x_k+\Delta t\,k_3,u_k\right)
$$

and

$$
\boxed{
x_{k+1}
=
x_k+
\frac{\Delta t}{6}
(k_1+2k_2+2k_3+k_4).
}
$$

RK4 is an integration method, not part of Williams' physical model.

The roller/gap regime changes when the wheel angle crosses the sector boundary, so timestep selection matters near these discontinuities.

---

## 16. Helper: body command to wheel speed

wheel_speeds_from_body_command() is a project helper, not an equation from Williams.

Given a desired body twist

$$
\begin{bmatrix}
V_x^{cmd}\\
V_y^{cmd}\\
\omega^{cmd}
\end{bmatrix},
$$

the helper calculates the wheel contact velocity that would occur if that twist were achieved and chooses wheel speeds such that

$$
v_{W,i}=0.
$$

For each wheel,

$$
v_{drive,i}^{cmd}
=
\left(
\mathbf V^{cmd}
+
\boldsymbol\omega^{cmd}\times\mathbf p_i
\right)
\cdot
\hat{\mathbf s}_i
$$

and therefore

$$
\boxed{
u_i^{cmd}
=
-\frac{v_{drive,i}^{cmd}}{\rho_i}.
}
$$

This is simply the inverse of the implemented slip equation under the ideal no-longitudinal-slip assumption.

It should not be confused with a motor controller or with the MPC plant input if the eventual MPC uses torque/current/PWM commands.

---

## 17. Parameters in the current code

ModelParams currently contains:

| Parameter | Meaning | Current default |
|---|---|---:|
| mass | robot mass | supplied by caller |
| inertia | planar yaw inertia | supplied by caller |
| g | gravitational acceleration | 9.81 m/s² |
| mu_W_roller | roller longitudinal friction | 0.25 |
| mu_T_roller | roller transverse friction | 0.15 |
| mu_W_gap | rigid-gap longitudinal friction | 0.56 |
| mu_T_gap | rigid-gap transverse friction | 0.56 |
| k | arctangent smoothing parameter | 1000 |

The four friction defaults are the values Williams measured for their carpet surface. They are not measurements of our SSL wheel/carpet combination and should be treated as provisional reference values until we replace them with measurements or justified project assumptions.

---

## 18. What is directly from Williams vs. what is project-specific?

### Directly based on Williams

- wheel/contact-point kinematics
- wheel peripheral velocity
- longitudinal slip
- transverse slip
- signed arctangent friction law
- friction force opposing slip
- equal-load assumption
- translational dynamics
- yaw dynamics
- wheel-angle-dependent roller/rigid friction
- separate roller and rigid-gap coefficient sets

### Project-specific

- four-wheel geometry
- wheel mounting angles
- wheel centre radius r_dist
- 16 rollers
- measured roller diameter
- measured gap dimensions
- floor-gap extrapolation
- roller-contact fraction
- phase offset
- robot mass and inertia
- eventual SSL friction coefficients
- use of RK4
- inverse-kinematics helper

---

## 19. Current correctness assessment

The current implementation is structurally consistent with the Williams model.

In particular:

- the +rho * wheel_speed term in v_W matches the sign convention used by this implementation and is consistent with Williams' wheel peripheral velocity construction;
- the transverse slip contains no wheel-speed term;
- friction uses the negative sign in the force equation, while the smooth mu(v) function preserves the sign of slip;
- the three-wheel normal-load term mg/3 from the paper has been generalized to mg/N for the current four-wheel robot;
- the wheel-angle-dependent friction regime is implemented as a periodic roller/gap sector;
- no passive-roller inertia state has been introduced.

The implementation is not yet fully physically validated for our robot. The main unresolved physical quantities are the SSL-specific friction coefficients, exact roller/gap contact geometry and phase, normal-load distribution, and eventual experimental validation.

The geometry calculation for ROLLER_FRACTION is also an explicit project assumption: the extrapolated floor gap is interpreted as a chord at the wheel contact radius. That should be verified against the physical wheel/CAD before treating the resulting fraction as exact.

---

## 20. Reference

Williams et al. 2002:

https://people.ohio.edu/williams/html/PDF/IEEETRA02.pdf

The paper explicitly states that its improved model accounts for the changing friction coefficients when the rigid material between omni-wheel rollers contacts the motion surface, and reports that this improved model agreed substantially better with experimentally measured slipping trajectories than the initial friction model.
