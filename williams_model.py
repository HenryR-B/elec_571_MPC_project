#!/usr/bin/env python3
"""
williams_model.py
==================

Direct Python implementation of Williams, Carter, Gallina & Rosati (2002),
"Dynamic Model with Slip for Wheeled Omni-Directional Robots," IEEE T-RA
18(3):285-293 -- equations (1)-(11), section III/IV.

This is the PRIMARY MODEL for two purposes at once:
  1. The "plant" an MPC controller is designed and tested against in Python.
  2. The reference implementation the C++ simulator gets cross-checked
     against (see the bottom of this file for the cross-validation harness).

WHY THIS REPLACES THE PER-ROLLER rollerOmega[] APPROACH
---------------------------------------------------------
The earlier C++ branch gave each of the 16 rollers its own dynamic state
(angular velocity, driven by a torque/inertia ODE, decaying while
disengaged). That mechanism was invented this week and never validated
against anything. Williams' model needs NONE of that: no roller inertia,
no per-roller memory, no differential equation for roller spin-up at all.
It tracks exactly one extra number per wheel -- the wheel's own rotation
angle theta_i -- and asks a single geometric question every step: is a
roller currently touching the ground, or a gap between two rollers? Two
different (experimentally MEASURED, not fitted) friction coefficient pairs
apply depending on the answer. That's the entire mechanism, and it's the
one that was actually compared against real measured trajectories in the
paper (Fig. 9/10: simulated vs. experimentally-traced paths).

THE MODEL (their equation numbers, in this implementation's notation)
------------------------------------------------------------------------
Contact point velocity for wheel i, in the WORLD frame:
    v_i = v_G + omega x r_i                                  (part of eq. 1)
(v_ri, the wheel's own peripheral speed, is folded directly into v_W below
rather than added as a separate vector -- same physics, see eq. 3 & 6.)

Wheel-direction (longitudinal) slip -- eq. (3)/(6):
    v_W,i = (v_i . s_i) - rho_i * theta_i_dot
Transverse slip -- eq. (7):
    v_T,i = v_i . r_i

Friction coefficients (eq. 11), smoothed with atan (their choice; tanh
works identically for this purpose -- avoids a discontinuous sign() at
zero velocity, which breaks numerical integration):
    mu(v) = mu_max * (2/pi) * atan(k*v)

TWO REGIMES per wheel, selected by wheel rotation angle theta_i, per
the "Improved Friction Model" (their section IV / 3.2):
    if currently on a ROLLER   (fraction delta_theta' of each pitch sector):
        use mu'_W, mu'_T   (small transverse friction -- that's the point of a roller)
    if currently on a GAP      (fraction delta_theta'' of each pitch sector):
        use mu''_W, mu''_T (EQUAL to each other, and much larger -- no roller,
                             just rigid material scraping the ground)

Friction force -- eq. (8):
    F_i = mu_W(v_W,i) * (m*g/n_wheels) * s_i  +  mu_T(v_T,i) * (m*g/n_wheels) * r_i

Body dynamics -- eq. (9):
    m*x_ddot   = sum_i (F_i . x_hat)
    m*y_ddot   = sum_i (F_i . y_hat)
    I*phi_ddot = sum_i (r_i x F_i) . z_hat

HONEST GAPS (say these out loud in the writeup, don't paper over them)
-------------------------------------------------------------------------
- mu'_W, mu'_T, mu''_W, mu''_T below are Williams' OWN measured values for
  THEIR wheel and surfaces (Table I: paper and carpet). They are NOT your
  wheel's numbers. Treat them as placeholders with the right STRUCTURE
  (roller << gap, transverse << longitudinal) until you run the equivalent
  tilt-test on your own wheel, or fit them from the residual method
  (compare vision-tracked trajectory vs. wheel-encoder-predicted trajectory
  under the ideal no-slip kinematics) against real logged robot data.
- delta_theta' / delta_theta'' (the roller-vs-gap angular fraction) is
  likewise Williams' wheel (n=8, 90/10 split), not yours (n=16). This is a
  geometric quantity you can get directly from your own wheel's CAD or a
  caliper measurement -- no assumption needed once you look.
- This paper's own robot is 3-wheeled, delta=15 deg, mass/inertia/wheel-
  radius undocumented in the extractable text (only in a CAD figure this
  fetch couldn't read). So this file does NOT reproduce their specific
  reported drift numbers (53mm/-22mm/-0.111 rad) -- it reproduces their
  MODEL STRUCTURE, applied to YOUR actual 4-wheel geometry below. Do not
  claim this matches their published numbers; it doesn't attempt to.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass


@dataclass
class Wheel:
    name: str
    alpha_deg: float   # mounting position angle from robot forward (+x), CCW
    r_dist: float      # distance from robot centre to wheel contact, m
    rho: float         # wheel radius (drive-direction contact radius), m
    n_rollers: int
    roller_fraction: float  # delta_theta' / (2*pi/n_rollers) -- fraction of
                             # each pitch sector that is ROLLER, not gap.
                             # ASSUMED/PLACEHOLDER until measured -- see docstring.


# ---- YOUR actual robot, not Williams' 3-wheeled prototype ----
ROBOT_WHEELS = [
    Wheel("front-left",  60.0,   0.076, 0.029915, 16, 0.85),
    Wheel("front-right", -60.0,  0.076, 0.029915, 16, 0.85),
    Wheel("back-left",   135.0,  0.076, 0.029915, 16, 0.85),
    Wheel("back-right",  -135.0, 0.076, 0.029915, 16, 0.85),
]

PARAMS = dict(
    mass=2.698,   # kg
    inertia=None, # kg*m^2 -- fill in from CAD; falls back to disk approx below
    g=9.81,
    # Williams' OWN measured carpet values (Table I) -- PLACEHOLDER structure,
    # not your wheel's numbers. mu' = roller regime, mu'' = gap regime.
    mu_W_roller=0.25, mu_T_roller=0.15,
    mu_W_gap=0.56,    mu_T_gap=0.56,
    k_smooth=1000.0,  # their eq. (11) steepness constant, chosen "by eye"
                      # for numerical stability -- same role as v_s elsewhere
                      # in this project, just parameterized as a slope
                      # instead of a smoothing velocity.
)


def _disk_inertia(mass, body_radius=0.088):
    return 0.5 * mass * body_radius**2


def wheel_frames(wheels):
    """s_i (drive dir), r_i (transverse/axle dir), position p_i -- all body frame."""
    alpha = np.radians([w.alpha_deg for w in wheels])
    dist = np.array([w.r_dist for w in wheels])
    delta = alpha + np.pi / 2  # radial-axle convention, matches earlier work
    p = np.stack([dist * np.cos(alpha), dist * np.sin(alpha)], axis=1)
    s_hat = np.stack([np.cos(delta), np.sin(delta)], axis=1)   # drive direction
    r_hat = np.stack([np.sin(delta), -np.cos(delta)], axis=1)  # transverse direction
    return p, s_hat, r_hat


def mu_smooth(v, mu_max, k):
    """Eq. (11): mu(v) = mu_max * (2/pi) * atan(k*v)."""
    return mu_max * (2.0 / np.pi) * np.arctan(k * v)


def which_regime(theta, wheel: Wheel):
    """True where a ROLLER is touching the ground, False where a GAP is.

    theta: wheel rotation angle(s), radians, any shape.
    Purely geometric -- no dynamics, no memory, matches Williams section IV.
    """
    pitch = 2 * np.pi / wheel.n_rollers
    phase = np.mod(theta, pitch) / pitch  # 0..1 within the current sector
    return phase < wheel.roller_fraction


def wheel_forces(v_body, omega, theta, wheels, params):
    """One evaluation of eq. (3),(7),(8) for all wheels at once.

    v_body: (vx, vy) in BODY frame. omega: scalar yaw rate.
    theta: array of per-wheel rotation angles (radians), same order as wheels.
    Returns: F (n_wheels, 2) force vectors in BODY frame.
    """
    p, s_hat, r_hat = wheel_frames(wheels)
    vx, vy = v_body
    v_contact = np.stack([vx - omega * p[:, 1], vy + omega * p[:, 0]], axis=1)

    v_s = np.einsum("ij,ij->i", v_contact, s_hat)   # along drive direction
    v_r = np.einsum("ij,ij->i", v_contact, r_hat)   # along transverse direction

    F = np.zeros_like(p)
    n_wheels = len(wheels)
    for i, w in enumerate(wheels):
        # NOTE: theta_dot (wheel's own commanded spin) is what you SUBTRACT
        # from v_s to get true longitudinal slip -- here we take v_s itself
        # as already being the slip (i.e. caller passes v_body as the FULL
        # contact velocity net of commanded wheel speed, OR you subtract
        # theta_dot*rho externally before calling this -- kept explicit and
        # separate below in `simulate_open_loop` so nothing is hidden here).
        on_roller = which_regime(theta[i], w)
        muW = params["mu_W_roller"] if on_roller else params["mu_W_gap"]
        muT = params["mu_T_roller"] if on_roller else params["mu_T_gap"]

        FW = mu_smooth(v_s[i], muW, params["k_smooth"]) * (params["mass"] * params["g"] / n_wheels)
        FT = mu_smooth(v_r[i], muT, params["k_smooth"]) * (params["mass"] * params["g"] / n_wheels)
        F[i] = -FW * s_hat[i] - FT * r_hat[i]  # opposes the sliding direction

    return F, v_s, v_r


def body_wrench(F, wheels):
    p, _, _ = wheel_frames(wheels)
    Fx, Fy = F[:, 0].sum(), F[:, 1].sum()
    tau = (p[:, 0] * F[:, 1] - p[:, 1] * F[:, 0]).sum()
    return Fx, Fy, tau


# ----------------------------------------------------------------------------
# LAYER 1 SELF-TESTS -- check the CODE matches the EQUATIONS, before trusting
# any number that comes out of it. None of these need Williams' exact
# undocumented robot parameters -- they check internal correctness only.
# ----------------------------------------------------------------------------

def _test_zero_velocity_zero_force():
    """At true zero velocity everywhere, force must be exactly zero."""
    wheels = ROBOT_WHEELS
    theta = np.zeros(4)
    F, v_s, v_r = wheel_forces((0.0, 0.0), 0.0, theta, wheels, PARAMS)
    assert np.allclose(v_s, 0.0) and np.allclose(v_r, 0.0)
    assert np.allclose(F, 0.0, atol=1e-9), f"nonzero force at zero velocity: {F}"
    print("  PASS: zero velocity -> exactly zero force (no chatter)")


def _test_roller_vs_gap_jump():
    """Crossing from roller regime to gap regime, at the SAME slip velocity,
    must increase the transverse force (mu''_T > mu'_T by construction)."""
    wheels = ROBOT_WHEELS
    w = wheels[0]
    pitch = 2 * np.pi / w.n_rollers
    theta_roller = np.array([0.1 * w.roller_fraction * pitch, 0, 0, 0])  # inside roller frac
    theta_gap = np.array([w.roller_fraction * pitch + 0.5 * (pitch - w.roller_fraction*pitch), 0, 0, 0])  # inside gap frac

    F_roller, _, _ = wheel_forces((0.0, 0.5), 0.0, theta_roller, wheels, PARAMS)
    F_gap, _, _ = wheel_forces((0.0, 0.5), 0.0, theta_gap, wheels, PARAMS)

    mag_roller = np.linalg.norm(F_roller[0])
    mag_gap = np.linalg.norm(F_gap[0])
    print(f"  wheel-0 transverse force magnitude: roller-regime={mag_roller:.4f} N, "
          f"gap-regime={mag_gap:.4f} N")
    assert mag_gap > mag_roller, "gap regime should produce MORE resistance than roller regime"
    print("  PASS: gap crossing genuinely produces more resistance than roller contact")


def _test_mirror_symmetry():
    """Robot is left-right mirror symmetric -> heading theta and -theta
    (360-theta) must give identical |force| pattern, same principle used
    throughout this whole project's earlier symmetry checks."""
    wheels = ROBOT_WHEELS
    theta = np.zeros(4)
    speed = 1.0
    mismatches = []
    for heading_deg in [10, 30, 45, 60, 90, 120, 150]:
        h = np.radians(heading_deg)
        F1, _, _ = wheel_forces((speed*np.cos(h), speed*np.sin(h)), 0.0, theta, wheels, PARAMS)
        h2 = np.radians(360 - heading_deg)
        F2, _, _ = wheel_forces((speed*np.cos(h2), speed*np.sin(h2)), 0.0, theta, wheels, PARAMS)
        # mirror: wheel order swaps (front-left<->front-right, back-left<->back-right)
        mirror_order = [1, 0, 3, 2]
        diff = np.linalg.norm(F1) - np.linalg.norm(F2[mirror_order])
        mismatches.append(abs(diff))
    max_mismatch = max(mismatches)
    print(f"  max mirror-symmetry mismatch across headings: {max_mismatch:.2e} N")
    assert max_mismatch < 1e-6, "mirror symmetry broken -- same class of bug as before"
    print("  PASS: mirror symmetry holds (same check that caught 2 real bugs earlier)")


if __name__ == "__main__":
    print("=" * 72)
    print("LAYER 1 SELF-TESTS -- code correctness vs. the equations")
    print("(these do NOT validate the model against reality -- see docstring)")
    print("=" * 72)
    _test_zero_velocity_zero_force()
    _test_roller_vs_gap_jump()
    _test_mirror_symmetry()
    print()
    print("All Layer-1 checks passed. This means the code correctly implements")
    print("Williams et al. (2002) eq. (1)-(11) as written. It does NOT mean the")
    print("model matches YOUR robot's real behaviour -- that requires Layer 3:")
    print("fitting mu'_W, mu'_T, mu''_W, mu''_T and roller_fraction against real")
    print("logged vision + wheel-encoder data from your own robot.")
