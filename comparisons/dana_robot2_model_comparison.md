# Comparison of Dana's `robot2.ipynb` Model and Our Rolling-Only MPC Model

## 1. Overview

Dana's `robot2.ipynb` attempts to model robot motion, wheel speeds, motor torque, and traction limits. Our `rolling_only_matrix_trajectory_mpc.ipynb` uses a matrix-based rigid-body model with wheel rotational inertia incorporated into an effective mass matrix.

The main difference is that Dana attempts to introduce traction saturation, while our current model assumes no-slip rolling. Neither model, as currently implemented, fully captures wheelspin through coupled wheel and ground-contact dynamics.

### Angular-velocity notation

Throughout this document:

- $\omega$ is the **robot's yaw rate**, in rad/s.
- $\dot{\omega}$ is the **robot's yaw angular acceleration**, in rad/s².
- $\dot{\theta}_i$ is the **angular velocity of wheel $i$**, in rad/s.
- $\ddot{\theta}_i$ is the **angular acceleration of wheel $i$**, in rad/s².

These quantities must not be confused. Robot yaw acceleration and wheel angular acceleration describe different physical motions.

## 2. Dana's Model

### 2.1 Motor torque and driving force

Dana calculates a requested wheel torque using a speed-error controller:

$$
\tau_{\mathrm{motor},i}
=
K_t(\dot{\theta}_{i,\mathrm{target}}-\dot{\theta}_i)
$$

She then applies a torque limit based on stall torque and maximum wheel speed.

The requested driving force is calculated as

$$
F_{\mathrm{drive},i}
=
\frac{\tau_{\mathrm{motor},i}}{R_w}
$$

This assumes that all motor torque is available to generate ground-driving force. It does not account for the torque required to accelerate the wheel's rotational inertia.

The wheel rotational equation is

$$
\tau_{\mathrm{motor},i}
-
R_wF_{\mathrm{contact},i}
=
J_w\ddot{\theta}_i
$$

Therefore,

$$
F_{\mathrm{contact},i}
=
\frac{\tau_{\mathrm{motor},i}
-J_w\ddot{\theta}_i}{R_w}
$$

The contact force cannot generally be calculated from motor torque and wheel radius alone when the wheel is accelerating.

There is also a dimensional concern with Dana's controller equation. If $K_t$ is a motor torque constant in N·m/A, multiplying it directly by a speed error does not produce torque. A controller gain with appropriate units, or a current-control calculation, is needed.

### 2.2 Traction saturation

Dana compares the requested driving force against the static-friction limit:

$$
F_{\mathrm{static,max},i}=\mu_sF_{z,i}
$$

If the requested force exceeds that limit, she substitutes a kinetic-friction force:

$$
F_{\mathrm{effective},i}
=
\mu_kF_{z,i}
\operatorname{sign}(F_{\mathrm{drive},i})
$$

This attempts to represent the transition from static to kinetic friction. However, it is not a complete wheel-slip model because wheel angular acceleration and contact force are not coupled through the wheel's rotational equation.

The model should not simply calculate force from motor torque, clamp it to a friction limit, and independently update wheel speed. Those quantities must be dynamically consistent.

### 2.3 Wheel-speed update

Dana updates wheel speed using a first-order filter toward the commanded speed:

$$
\dot{\theta}_{i,k+1}
=
\dot{\theta}_{i,k}
+
\min(1,20\Delta t)
\left(
\dot{\theta}_{i,\mathrm{target}}
-\dot{\theta}_{i,k}
\right)
$$

This update is independent of the motor torque balance and ground-contact force. Consequently, wheel speed does not emerge from the physical torque acting on the wheel, and wheelspin is not dynamically represented.

### 2.4 Robot rigid-body dynamics

Dana sums wheel forces and their moments to calculate robot acceleration. However, the robot's translational velocity is stored in body coordinates, and the acceleration update omits the rotating-frame terms.

For body-frame translational velocity $(u,v)$ and robot yaw rate $\omega$, the equations should include

$$
\dot{u}=\frac{F_x}{m}+\omega v
$$

$$
\dot{v}=\frac{F_y}{m}-\omega u
$$

where $F_x$ and $F_y$ are the net physical force components expressed in body coordinates.

The robot's yaw acceleration is calculated separately:

$$
\dot{\omega}=\frac{\tau_z}{I_{zz}}
$$

Dana uses explicit Euler integration, whereas our model uses RK4.

### 2.5 Inverse-kinematics geometry

Dana's inverse kinematics uses a rotational contribution of $R_{\mathrm{base}}\omega$ for every wheel:

$$
\dot{\theta}_i
=
\frac{\mathbf{d}_i^T\mathbf{v}
+R_{\mathrm{base}}\omega}{R_w}
$$

Here, $\mathbf{d}_i$ is the drive direction and $\mathbf{v}$ is the robot's translational velocity in the relevant coordinate frame.

However, the actual rotational contribution depends on each wheel's position and drive direction. The moment arm is

$$
h_i=r_{ix}d_{iy}-r_{iy}d_{ix}
$$

so the rotational contribution should be $h_i\omega$, subject to the chosen sign convention.

Dana's constant $+R_{\mathrm{base}}\omega$ term does not appear consistent with the wheel directions and corner positions defined in her model. Her inverse-kinematics equations and force/moment geometry therefore need to be reconciled.

### 2.6 Additional implementation issue

Dana's slip-penalty calculation uses `f_drive` after it has been overwritten inside the wheel loop. As a result, the calculation compares the last wheel's requested force against the effective forces for all wheels. The requested force should instead be retained separately for each wheel.

## 3. Our Rolling-Only Matrix Model

Our model uses the state vector

$$
\mathbf{x}
=
\begin{bmatrix}
x & y & \phi & v_x & v_y & \omega &
\dot{\theta}_1 & \dot{\theta}_2 &
\dot{\theta}_3 & \dot{\theta}_4
\end{bmatrix}^{T}
$$

Here, $\omega$ is the robot's yaw rate, while $\dot{\theta}_1,\ldots,\dot{\theta}_4$ are the four wheel angular velocities.

The wheel angular velocities are currently derived from the no-slip rolling constraint. They do not independently affect the contact forces. They are therefore redundant with the robot's motion under the current model, and adding them as states alone does not allow the model to simulate wheelspin.

### 3.1 Drive-direction and moment-arm matrix

For each wheel, let $\mathbf{d}_i$ be its drive direction and $\mathbf{r}_i$ its position relative to the robot centre. The moment arm is

$$
h_i=r_{ix}d_{iy}-r_{iy}d_{ix}
$$

The matrix mapping wheel forces to the body wrench is

$$
H=
\begin{bmatrix}
d_{1x}&d_{2x}&d_{3x}&d_{4x}\\
d_{1y}&d_{2y}&d_{3y}&d_{4y}\\
h_1&h_2&h_3&h_4
\end{bmatrix}
$$

Thus,

$$
\mathbf{w}_{\mathrm{drive}}=H\mathbf{F}
$$

This construction is algebraically consistent when wheel directions, positions, and signs are defined consistently.

In our current geometry, the drive directions are tangential to the radial wheel positions, so the moment arms have the same magnitude and sign under the chosen convention.

### 3.2 Effective mass matrix

Our model uses

$$
M_{\mathrm{body}}
=
\begin{bmatrix}
m&0&0\\
0&m&0\\
0&0&I_{zz}
\end{bmatrix}
$$

and

$$
M_{\mathrm{eff}}
=
M_{\mathrm{body}}
+
\frac{J_w}{R_w^2}HH^T
$$

The second term accounts for wheel rotational inertia under the no-slip constraint. It follows from eliminating the wheel angular accelerations using the rolling relationship.

The matrix is valid under the model's no-slip assumptions and consistent wheel geometry.

### 3.3 Motor-current input matrix

For motor current $I_{q,i}$ and torque constant $K_t$, the motor torque is

$$
\tau_{\mathrm{motor},i}=K_tI_{q,i}
$$

With the sign convention used in our model, the generalized acceleration contribution from motor current is represented by

$$
G=-\frac{K_t}{R_w}M_{\mathrm{eff}}^{-1}H
$$

The sign depends on how wheel rotation, contact force, and drive directions are defined. It must be consistent throughout the wheel torque balance and body dynamics.

Our model also includes body-frame rotating-coordinate terms and uses RK4 integration.

## 4. Physical Parameter Comparison

The following values are the parameters previously identified in the two notebooks. They should be treated as separate model configurations, not as values that can be mixed without reconciliation.

| Parameter | Dana's `robot2.ipynb` | Our model |
|---|---:|---:|
| Robot mass, $m$ | 2.7 kg | Approximately 2.698 kg |
| Robot yaw inertia, $I_{zz}$ | 0.0074 kg·m² | 0.007725673 kg·m² |
| Wheel radius, $R_w$ | 0.030 m | 0.0299146 m |
| Wheel-centre geometry | $R_{\mathrm{base}}=0.060$ m, with corner positions defined using $R_{\mathrm{base}}/\sqrt{2}$ | Approximately 0.0795 m wheel-centre radius |
| Wheel rotational inertia, $J_w$ | Must be verified from Dana's parameter definitions | Must use the wheel inertia defined for our model |
| Static friction coefficient, $\mu_s$ | 0.8 | Use the value defined in our friction model |
| Kinetic friction coefficient, $\mu_k$ | 0.5 | Use the value defined in our friction model |

The robot mass and yaw inertia are close but not identical. The wheel radii are also close, while the wheel-centre geometry differs substantially.

**Do not silently replace our CAD-derived parameters with Dana's values.** For a fair comparison, we should first confirm the intended wheel-centre positions and wheel-drive directions, then use one consistent parameter set for both models. The exact wheel rotational inertia and our current friction coefficients should be read from the corresponding source definitions rather than inferred.

## 5. Main Differences

| Feature | Dana's `robot2.ipynb` | Our rolling-only model |
|---|---|---|
| Motor input | Desired wheel speed and a torque approximation | Motor current |
| Wheel torque balance | Does not account for wheel angular acceleration | Wheel inertia incorporated under the no-slip constraint |
| Traction | Static/kinetic force-limit heuristic | No-slip rolling assumption |
| Wheel angular velocity | Updated independently using a first-order filter | Integrated from the no-slip constraint |
| Wheelspin | Not dynamically modelled | Not dynamically modelled |
| Robot yaw rate | $\omega$ | $\omega$ |
| Wheel angular velocity | $\dot{\theta}_i$ | $\dot{\theta}_i$ |
| Robot body-frame dynamics | Omits rotating-frame terms | Includes rotating-frame terms |
| Integration | Explicit Euler | RK4 |
| Geometry | Rotational inverse-kinematics term appears inconsistent with wheel geometry | Uses explicit wheel drive directions and moment arms |
| Motor model | Simplified speed-error torque calculation | Current-to-torque relationship |

## 6. Recommended Direction

I would keep our matrix-based model as the rolling-only baseline rather than copying Dana's equations directly. The effective mass matrix and motor-current mapping are consistent with a no-slip model, provided the geometry and sign conventions are correct.

To model wheelspin, we need to extend the dynamics rather than merely add wheel angular-velocity states:

1. Calculate motor torque from motor current and the torque constant.
2. Use the wheel rotational equation to couple motor torque, contact force, and wheel angular acceleration.
3. Calculate relative wheel-ground slip using the actual wheel angular velocity $\dot{\theta}_i$ and robot motion, including robot yaw rate $\omega$.
4. Determine contact force using the friction model, including the static-to-kinetic transition.
5. Integrate the robot dynamics and wheel dynamics together.

The key point is that the wheel angular velocities must become dynamically independent of the no-slip robot velocity when traction is lost.

Dana's model is useful as an attempt to include traction saturation, but its torque-to-force calculation, wheel-speed update, and geometry should not be copied without correction.
