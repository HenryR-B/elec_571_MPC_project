#!/usr/bin/env python3
"""
omni_roller_drag.py
===================

Geometric roller-drag model for an N-wheel omnidirectional robot.

The point of this script: the *shape* of the direction-dependent drag signature
is pure geometry (wheel mounting angles + drive directions). The friction
coefficient only sets the *scale*. So you can generate the whole picture before
measuring anything, and drop in a measured mu_r later without changing the code.

CONVENTIONS (set these straight once and never say "roller axis" again)
----------------------------------------------------------------------
For each wheel i:
    d_hat[i]  drive / traction direction, in the ground plane.
              The hub propels along this. Rollers do NOT spin for motion
              along d_hat.
    a_hat[i]  free-slide direction, perpendicular to d_hat, along the wheel
              axle. Rollers DO spin for motion along a_hat, so this direction
              gives ~no propulsion and only parasitic drag.

    a_hat = R(-90 deg) @ d_hat

For a conventional layout the axle points radially at the robot centre, so
d_hat is tangential: delta = alpha + 90 deg. Override delta_deg per wheel if
your CAD says otherwise.

Note: d_hat as coded is a VECTOR with an arbitrary sign (delta = alpha + 90,
not alpha - 90). Physically the wheel is a LINE, not a vector -- a motor can
spin either direction, so the wheel can push along +d_hat or -d_hat equally
well. Anywhere you use d_hat to ask "how much can this wheel push toward
heading X", use abs(dot(d_hat, heading)), never a signed dot product clipped
to positive -- clipping silently assumes the wheel can only push one
direction, and because the +90 sign convention is not mirror-consistent
across left/right wheel pairs, clipping breaks the robot's real left-right
symmetry (see PHYSICAL SYMMETRIES below).

Slip angle:
    beta[i] = atan2(u_a[i], u_d[i])
    beta = 0    -> wheel is doing pure traction, rollers still
    beta = +-90 -> wheel is purely freewheeling, contributing only drag

Roller drag force on the robot from wheel i (opposing the slide):
    F_r[i] = -( mu_r * N[i] * tanh(u_a[i]/v_s) + b * u_a[i] ) * a_hat[i]

The tanh is a smoothed Coulomb term. A hard sign() is discontinuous at zero
velocity, which wrecks ODE integration and QP conditioning downstream, so keep
it smooth if you plan to feed this into an optimiser.

PHYSICAL SYMMETRIES (read this before touching the scrub model)
-----------------------------------------------------------------------
A standard omni wheel's roller resistance depends only on the ORIENTATION of
sliding relative to the axle, not on direction or sign. Two symmetries follow:

    r(beta) = r(-beta)        sliding left vs right along the same line
                               gives the same resistance
    r(beta) = r(beta + 180)   towing forward vs backward along the same
                               drive line gives the same resistance

Combined, resistance is fully determined by folding beta into [0, 90]:

    gamma = |beta| mod 180
    gamma = min(gamma, 180 - gamma)

Any scrub/resistance model must be a function of `gamma`, never of raw beta
or of `beta mod 90` (a naive mod-90 does NOT respect r(beta)=r(beta+180) and
will silently break the left-right mirror symmetry your robot's geometry
actually has -- heading theta and heading 360-theta stop matching even
though the robot is physically a mirror image of itself). See
`calculate_scrub_score` below.

USAGE
-----
    python3 omni_roller_drag.py                  # save figures to ./figures
    python3 omni_roller_drag.py --show           # also open interactive windows
    python3 omni_roller_drag.py --mu-r 0.15      # override roller coefficient
    python3 omni_roller_drag.py --speed 2.0      # sweep at 2 m/s

Everything in ROBOT_WHEELS and PARAMS below is meant to be edited.
"""

from __future__ import annotations

import argparse
import os
from dataclasses import dataclass

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# ----------------------------------------------------------------------------
# CONFIGURATION  -- edit these
# ----------------------------------------------------------------------------


@dataclass
class Wheel:
    """One omni wheel.

    alpha_deg : mounting position angle, measured from robot forward (+x),
                CCW positive. This is WHERE the wheel sits on the body.
    L         : distance from robot centre to the wheel contact patch [m].
    delta_deg : drive direction angle from +x, CCW positive. This is WHICH WAY
                the wheel pushes. Leave None for the conventional radial-axle
                case, which gives delta = alpha + 90.
    """

    name: str
    alpha_deg: float
    L: float
    delta_deg: float | None = None


# VERIFY THESE AGAINST CAD before trusting any number that comes out.
# From CAD: front wheels 30 deg from side axis = 60 deg from forward.
#           rear wheels  45 deg from side axis = 45 deg from forward
#           -> sit at 180 - 45 = 135 deg from forward in CCW convention.
# 152mm is the diameter at which the wheels touch the ground, not the outer diameter
ROBOT_WHEELS = [
    Wheel("front-left",  alpha_deg=60.0,   L=0.076),
    Wheel("front-right", alpha_deg=-60.0,  L=0.076),
    Wheel("back-left",   alpha_deg=135.0,  L=0.076),
    Wheel("back-right",  alpha_deg=-135.0, L=0.076),
]

PARAMS = dict(
    mass=2.698,        # robot mass [kg]  (SSL max is in this ballpark; check yours)
    g=9.81,
    mu_r=0.10,       # transverse (roller) friction coefficient -- ASSUMED.
                     # This is the single scalar that sets the drag scale.
    mu_d=0.70,       # drive-direction traction coefficient -- ASSUMED.
    b_visc=0.0,      # viscous roller term [N per (m/s)]. 0 = pure Coulomb.
    v_s=0.03,        # tanh smoothing velocity [m/s]
    wheel_radius=0.0236,  # 47.2 mm diameter / 2
    n_rollers=16,
)

# ----------------------------------------------------------------------------
# KINEMATICS + FORCE MODEL
# ----------------------------------------------------------------------------


def wheel_frames(wheels: list[Wheel]):
    """Return contact positions p (n,2), drive dirs d_hat (n,2), slide dirs a_hat (n,2)."""
    alpha = np.radians([w.alpha_deg for w in wheels])
    L = np.array([w.L for w in wheels])
    delta = np.array(
        [
            np.radians(w.delta_deg) if w.delta_deg is not None else a + np.pi / 2
            for w, a in zip(wheels, alpha)
        ]
    )

    p = np.stack([L * np.cos(alpha), L * np.sin(alpha)], axis=1)
    d_hat = np.stack([np.cos(delta), np.sin(delta)], axis=1)
    # a_hat = d_hat rotated by -90 deg
    a_hat = np.stack([np.sin(delta), -np.cos(delta)], axis=1)
    return p, d_hat, a_hat


def contact_velocity(p, vx, vy, omega):
    """Ground-contact velocity of each wheel for a body twist. Returns (n,2)."""
    return np.stack(
        [vx - omega * p[:, 1], vy + omega * p[:, 0]],
        axis=1,
    )


def decompose(u, d_hat, a_hat):
    """Split contact velocity into drive / slide components and slip angle."""
    u_d = np.einsum("ij,ij->i", u, d_hat)
    u_a = np.einsum("ij,ij->i", u, a_hat)
    beta = np.arctan2(u_a, u_d)
    return u_d, u_a, beta


def roller_drag(u_a, a_hat, N, mu_r, b_visc, v_s):
    """Parasitic roller drag force on the robot from each wheel. Returns (n,2)."""
    mag = mu_r * N * np.tanh(u_a / v_s) + b_visc * u_a
    return -(mag[:, None] * a_hat)


def total_wrench(F, p):
    """Sum per-wheel forces into a body wrench (Fx, Fy, tau_z)."""
    Fx, Fy = F[:, 0].sum(), F[:, 1].sum()
    tau = (p[:, 0] * F[:, 1] - p[:, 1] * F[:, 0]).sum()
    return Fx, Fy, tau


def sweep_heading(wheels, params, speed=1.0, omega=0.0, n_theta=721):
    """Sweep travel direction 0..2pi. Returns a dict of arrays."""
    p, d_hat, a_hat = wheel_frames(wheels)
    n = len(wheels)
    N = np.full(n, params["mass"] * params["g"] / n)  # equal static load split

    theta = np.linspace(0.0, 2 * np.pi, n_theta)
    beta = np.zeros((n_theta, n))
    Fmag = np.zeros((n_theta, n))
    F_all = np.zeros((n_theta, n, 2))
    tot = np.zeros((n_theta, 3))

    for k, th in enumerate(theta):
        vx, vy = speed * np.cos(th), speed * np.sin(th)
        u = contact_velocity(p, vx, vy, omega)
        _, u_a, b = decompose(u, d_hat, a_hat)
        F = roller_drag(u_a, a_hat, N, params["mu_r"], params["b_visc"], params["v_s"])

        beta[k] = b
        F_all[k] = F
        Fmag[k] = np.linalg.norm(F, axis=1)
        tot[k] = total_wrench(F, p)

    # Decompose the total drag relative to the direction of travel.
    t_hat = np.stack([np.cos(theta), np.sin(theta)], axis=1)      # along travel
    n_hat = np.stack([-np.sin(theta), np.cos(theta)], axis=1)     # left of travel
    F_tot = tot[:, :2]
    resist = -np.einsum("ij,ij->i", F_tot, t_hat)   # +ve = opposes motion
    lateral = np.einsum("ij,ij->i", F_tot, n_hat)   # +ve = pushes left of travel

    return dict(
        theta=theta,
        beta=beta,
        Fmag=Fmag,
        F_all=F_all,
        Fx=tot[:, 0],
        Fy=tot[:, 1],
        tau=tot[:, 2],
        Fnorm=np.linalg.norm(F_tot, axis=1),
        resist=resist,
        lateral=lateral,
        p=p,
        d_hat=d_hat,
        a_hat=a_hat,
        N=N,
    )


def lateral_map(wheels, params, speed=1.0, omega_max=8.0, n_theta=181, n_omega=161):
    """Lateral drag force over a grid of (travel heading, yaw rate).

    This is the rotate-while-translate coupling: at omega = 0 the lateral term
    is one curve, and spinning the body reshapes it asymmetrically.
    """
    p, d_hat, a_hat = wheel_frames(wheels)
    n = len(wheels)
    N = np.full(n, params["mass"] * params["g"] / n)

    theta = np.linspace(0.0, 2 * np.pi, n_theta)
    omega = np.linspace(-omega_max, omega_max, n_omega)
    Z = np.zeros((n_omega, n_theta))

    for j, om in enumerate(omega):
        for i, th in enumerate(theta):
            vx, vy = speed * np.cos(th), speed * np.sin(th)
            u = contact_velocity(p, vx, vy, om)
            _, u_a, _ = decompose(u, d_hat, a_hat)
            F = roller_drag(u_a, a_hat, N, params["mu_r"], params["b_visc"], params["v_s"])
            Fx, Fy, _ = total_wrench(F, p)
            Z[j, i] = -np.sin(th) * Fx + np.cos(th) * Fy

    return theta, omega, Z


def calculate_scrub_score(res):
    """Coefficient-free scrub score per wheel, per heading. 0 = good, 1 = worst.

    Uses the physical symmetries documented at the top of this file:
        r(beta) = r(-beta)         fold sign out
        r(beta) = r(beta + 180)    fold half-period out
    which together reduce any beta to gamma in [0, 90] -- the same range the
    Duda et al. (2025, Sensors 25(16):5026) experiment tested directly.

    Within [0, 90], their measured omni-wheel rolling resistance peaks at
    gamma = 60 deg (not 45), for both tested loads, on both tested surfaces.
    This piecewise-linear fit matches that: rises 0->1 from gamma=0 to 60,
    falls 1->0 from gamma=60 to 90.

    IMPORTANT: do not fold with `% 90` directly on |beta| -- that does NOT
    respect r(beta) = r(beta+180) and will break the left-right mirror
    symmetry your robot's geometry actually has (heading theta and heading
    360-theta will stop matching even though the robot is a physical mirror
    image of itself). Always go through the two-step fold below.
    """
    beta_deg = np.degrees(res["beta"])
    gamma = np.abs(beta_deg) % 180.0           # fold out sign + half-period
    gamma = np.minimum(gamma, 180.0 - gamma)   # mirror the far half onto [0, 90]

    scrub = np.where(
        gamma <= 60.0,
        gamma / 60.0,
        (90.0 - gamma) / 30.0,
    )
    return scrub

def calculate_total_scrub_score(res):
    scrub_per_wheel = calculate_scrub_score(res)   # (n_theta, n_wheels), 0..1
    scrub_total = scrub_per_wheel.sum(axis=1)      # (n_theta,), 0..n_wheels
    return scrub_total, scrub_per_wheel

def calculate_goodness(wheels, res):
    theta   = res["theta"]
    th_deg  = np.degrees(theta)
    n       = len(wheels)

    p, d_hat, a_hat = wheel_frames(wheels)
    t_hat      = np.stack([np.cos(theta), np.sin(theta)], axis=1)
    drive_proj = np.einsum("tj,wj->tw", t_hat, d_hat)
    # abs(), not clip: the wheel can push along +d_hat or -d_hat depending
    # on which way the motor spins, so both directions count as available
    # propulsive contribution toward the commanded heading.
    drive_raw  = np.abs(drive_proj).sum(axis=1)
    drive_norm = drive_raw / drive_raw.max()

    scrub_per_wheel = calculate_scrub_score(res)
    scrub_total = scrub_per_wheel.sum(axis=1)
    scrub_norm  = scrub_total / n
    clean       = 1.0 - scrub_norm

    goodness = drive_norm * clean

    return goodness, clean, drive_norm


# ----------------------------------------------------------------------------
# ROBUST FIGURE SAVING
# ----------------------------------------------------------------------------
#
# A single locked file (e.g. still open in an image viewer, or an antivirus
# scan holding a handle) used to abort the entire script partway through,
# leaving some figures updated and others stale with no clear indication of
# which. safe_savefig() below contains any save failure to that one figure,
# always closes the figure to release memory (matplotlib otherwise
# accumulates open figures across a run, which caused the earlier
# "Errno 22 Invalid argument" crashes on Windows), and reports clearly what
# happened so a rerun is safe.


def safe_savefig(fig, path, dpi=150):
    """Save a figure, never raise on failure, always release the figure."""
    try:
        fig.savefig(path, dpi=dpi)
        print(f"  wrote {path}")
    except OSError as e:
        print(f"  [!] could not write {path}: {e}")
        print("      (close any program that has this file open -- an image")
        print("       viewer, an antivirus scan, a locked OneDrive sync -- and rerun)")
    finally:
        plt.close(fig)


def prepare_outdir(outdir):
    """Create outdir if needed, and clear old PNGs without ever crashing the run."""
    os.makedirs(outdir, exist_ok=True)
    for filename in os.listdir(outdir):
        filepath = os.path.join(outdir, filename)
        if os.path.isfile(filepath) and filename.lower().endswith(".png"):
            try:
                os.remove(filepath)
            except OSError as e:
                print(f"  [!] could not remove old {filepath}: {e}")
                print("      (it will be overwritten in place instead)")

# ----------------------------------------------------------------------------
# PLOTS
# ----------------------------------------------------------------------------


def plot_geometry(wheels, res, outdir):
    p, d_hat, a_hat = res["p"], res["d_hat"], res["a_hat"]
    fig, ax = plt.subplots(figsize=(6.2, 6.2))

    body_r = max(np.linalg.norm(p, axis=1)) * 1.06
    ax.add_patch(plt.Circle((0, 0), body_r, fill=False, ls="--", lw=1, color="0.6"))
    ax.annotate("", xy=(body_r * 0.55, 0), xytext=(0, 0),
                arrowprops=dict(arrowstyle="-|>", color="0.35", lw=1.6))
    ax.text(body_r * 0.58, 0.004, "forward (+x)", color="0.35", fontsize=9)

    s = body_r * 0.42
    for i, w in enumerate(wheels):
        ax.plot(*p[i], "o", ms=9, color="black", zorder=5)
        ax.annotate("", xy=p[i] + s * d_hat[i], xytext=p[i],
                    arrowprops=dict(arrowstyle="-|>", color="tab:blue", lw=2.2))
        ax.annotate("", xy=p[i] + s * 0.75 * a_hat[i], xytext=p[i],
                    arrowprops=dict(arrowstyle="-|>", color="tab:orange", lw=2.2))
        ax.text(p[i][0] * 1.20, p[i][1] * 1.20, w.name, fontsize=9,
                ha="center", va="center")

    ax.plot([], [], color="tab:blue", lw=2.2, label=r"$\hat{d}$  drive / traction")
    ax.plot([], [], color="tab:orange", lw=2.2, label=r"$\hat{a}$  free slide (rollers spin)")
    ax.legend(loc="lower right", fontsize=8.5, framealpha=0.9)

    ax.set_aspect("equal")
    lim = body_r * 1.65
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Wheel geometry and direction conventions (top view)")
    ax.grid(alpha=0.25)
    fig.tight_layout()
    safe_savefig(fig, os.path.join(outdir, "01_geometry.png"))


def plot_per_wheel_slip(wheels, res, outdir):
    """Slip angle |beta| vs travel direction, one polar panel per wheel."""
    theta, beta = res["theta"], res["beta"]
    n = len(wheels)
    ncol = min(n, 4)
    nrow = int(np.ceil(n / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(3.5 * ncol, 3.9 * nrow),
                             subplot_kw=dict(projection="polar"))
    axes = np.atleast_1d(axes).ravel()

    for i, w in enumerate(wheels):
        ax = axes[i]
        b = np.degrees(np.abs(beta[:, i]))
        ax.plot(theta, b, lw=2, color="tab:purple")
        ax.fill(theta, b, color="tab:purple", alpha=0.15)
        ax.set_rlim(0, 90)
        ax.set_rticks([0, 30, 60, 90])
        ax.set_title(f"{w.name}\n"
                     r"$|\beta|$: 0$^\circ$=pure traction, 90$^\circ$=freewheel",
                     fontsize=9.5, pad=14)
        ax.tick_params(labelsize=7.5)
        ax.grid(alpha=0.35)

    for ax in axes[n:]:
        ax.axis("off")

    fig.suptitle("Per-wheel slip angle vs direction of travel "
                 "(angle = travel heading in body frame)", fontsize=11.5)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    safe_savefig(fig, os.path.join(outdir, "02_slip_angle_per_wheel.png"))


def plot_per_wheel_drag(wheels, res, params, outdir):
    """Roller drag magnitude per wheel vs travel direction."""
    theta, Fmag = res["theta"], res["Fmag"]
    scale = params["mu_r"] * res["N"][0]  # normalisation: mu_r * N
    n = len(wheels)
    ncol = min(n, 4)
    nrow = int(np.ceil(n / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(3.5 * ncol, 3.9 * nrow),
                             subplot_kw=dict(projection="polar"))
    axes = np.atleast_1d(axes).ravel()

    rmax = max(Fmag.max(), 1e-9) * 1.12
    for i, w in enumerate(wheels):
        ax = axes[i]
        ax.plot(theta, Fmag[:, i], lw=2, color="tab:red")
        ax.fill(theta, Fmag[:, i], color="tab:red", alpha=0.15)
        ax.set_rlim(0, rmax)
        ax.set_title(f"{w.name}\nroller drag [N]  (max {Fmag[:, i].max():.2f} N)",
                     fontsize=9.5, pad=14)
        ax.tick_params(labelsize=7.5)
        ax.grid(alpha=0.35)

    for ax in axes[n:]:
        ax.axis("off")

    fig.suptitle(f"Per-wheel parasitic roller drag vs travel direction "
                 f"(scale factor $\\mu_r N$ = {scale:.2f} N)", fontsize=11.5)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    safe_savefig(fig, os.path.join(outdir, "03_drag_per_wheel.png"))


def plot_total(res, params, outdir):
    """Total drag signature: magnitude, resistive vs lateral split, yaw torque."""
    theta = res["theta"]
    th_deg = np.degrees(theta)

    fig = plt.figure(figsize=(13.5, 5.4))
    gs = fig.add_gridspec(1, 3, width_ratios=[1.05, 1.25, 1.05])

    ax0 = fig.add_subplot(gs[0], projection="polar")
    ax0.plot(theta, res["Fnorm"], lw=2.2, color="tab:red")
    ax0.fill(theta, res["Fnorm"], color="tab:red", alpha=0.15)
    ax0.set_title("Total roller drag magnitude [N]", fontsize=10.5, pad=16)
    ax0.tick_params(labelsize=8)
    ax0.grid(alpha=0.35)

    ax1 = fig.add_subplot(gs[1])
    ax1.plot(th_deg, res["resist"], lw=2, color="tab:blue",
             label="resistive (opposes travel)")
    ax1.plot(th_deg, res["lateral"], lw=2, color="tab:orange",
             label="lateral (pushes sideways)")
    ax1.axhline(0, color="0.5", lw=0.9)
    ax1.set_xlim(0, 360)
    ax1.set_xticks(np.arange(0, 361, 45))
    ax1.set_xlabel("travel heading [deg, body frame]")
    ax1.set_ylabel("force [N]")
    ax1.set_title("Drag split relative to direction of travel", fontsize=10.5)
    ax1.legend(fontsize=8.5)
    ax1.grid(alpha=0.3)

    ax2 = fig.add_subplot(gs[2])
    tau_mNm = res["tau"] * 1e3
    tau_peak = np.abs(tau_mNm).max()
    ax2.axhline(0, color="0.5", lw=0.9)
    if tau_peak < 1e-6:
        # Radial axles -> drag is collinear with the moment arm -> exactly zero.
        # Anything on screen here would be float rounding noise, so don't plot it.
        ax2.plot(th_deg, np.zeros_like(th_deg), lw=2, color="tab:green")
        ax2.set_ylim(-1, 1)
        ax2.text(180, 0.42, "numerically zero\n(all wheel axles radial)",
                 ha="center", va="center", fontsize=9.5, color="0.35",
                 bbox=dict(boxstyle="round,pad=0.45", fc="white", ec="0.75"))
    else:
        ax2.plot(th_deg, tau_mNm, lw=2, color="tab:green")
    ax2.set_xlim(0, 360)
    ax2.set_xticks(np.arange(0, 361, 90))
    ax2.set_xlabel("travel heading [deg]")
    ax2.set_ylabel("yaw torque [mN·m]")
    ax2.set_title("Direct yaw torque from roller drag", fontsize=10.5)
    ax2.grid(alpha=0.3)

    fig.suptitle("Total roller-drag wrench vs travel direction "
                 f"(pure translation, $\\mu_r$ = {params['mu_r']})", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    safe_savefig(fig, os.path.join(outdir, "04_total_drag.png"))


def plot_lateral_map(wheels, params, outdir, speed):
    theta, omega, Z = lateral_map(wheels, params, speed=speed)
    fig, ax = plt.subplots(figsize=(8.2, 5.0))
    lim = np.abs(Z).max()
    lim = lim if lim > 0 else 1.0
    im = ax.pcolormesh(np.degrees(theta), omega, Z, cmap="RdBu_r",
                       vmin=-lim, vmax=lim, shading="auto")
    ax.contour(np.degrees(theta), omega, Z, levels=[0.0], colors="k", linewidths=0.8)
    ax.axhline(0, color="0.25", lw=0.9, ls="--")
    ax.set_xlabel("travel heading [deg, body frame]")
    ax.set_ylabel("yaw rate $\\omega$ [rad/s]")
    ax.set_title(f"Lateral drag force [N] vs heading and yaw rate  (|v| = {speed} m/s)\n"
                 "nonzero = drag pushes the robot off the commanded line")
    fig.colorbar(im, ax=ax, label="lateral force [N]  (+ve = left of travel)")
    fig.tight_layout()
    safe_savefig(fig, os.path.join(outdir, "05_lateral_vs_yawrate.png"))


def plot_scrub_score(wheels, res, outdir):
    """Coefficient-free scrub score: how far each wheel is from a good state.

    0 = pure traction or pure freewheeling (both fine).
    1 = worst scrubbing, located at gamma=60 deg per Duda et al. (2025).
    Total scrub = sum across all wheels, range 0 (perfect) to N_wheels (worst).
    Entirely geometric — no friction coefficients anywhere.
    """
    theta = res["theta"]
    n = len(wheels)

    scrub_total, scrub_per_wheel = calculate_total_scrub_score(res)

    ncol = min(n + 1, 5)
    fig, axes = plt.subplots(1, ncol, figsize=(3.5 * ncol, 4.2),
                             subplot_kw=dict(projection="polar"))
    axes = np.atleast_1d(axes).ravel()

    colours = ["tab:blue", "tab:orange", "tab:green", "tab:purple"]
    for i, w in enumerate(wheels):
        ax = axes[i]
        s = scrub_per_wheel[:, i]
        ax.plot(theta, s, lw=2, color=colours[i % len(colours)])
        ax.fill(theta, s, color=colours[i % len(colours)], alpha=0.15)
        ax.set_rlim(0, 1.05)
        ax.set_rticks([0, 0.25, 0.5, 0.75, 1.0])
        ax.set_title(f"{w.name}\nscrub  0=good  1=worst",
                     fontsize=9.0, pad=14)
        ax.tick_params(labelsize=7.5)
        ax.grid(alpha=0.35)

    ax_tot = axes[n]
    norm = scrub_total / n
    ax_tot.plot(theta, norm, lw=2.5, color="crimson")
    ax_tot.fill(theta, norm, color="crimson", alpha=0.18)
    ax_tot.set_rlim(0, 1.05)
    ax_tot.set_rticks([0, 0.25, 0.5, 0.75, 1.0])
    ax_tot.set_title(f"TOTAL (all {n} wheels)\n0=all good  1=all worst",
                     fontsize=9.0, pad=14)
    ax_tot.tick_params(labelsize=7.5)
    ax_tot.grid(alpha=0.35)

    for ax in axes[n + 1:]:
        ax.axis("off")

    fig.suptitle(
        "Scrub score — coefficient-free, geometry only\n"
        "0 = pure traction or pure freewheeling (both fine).  "
        "1 = worst scrubbing (peak at 60° per Duda et al. 2025).",
        fontsize=11.0,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.89])
    safe_savefig(fig, os.path.join(outdir, "06_scrub_score.png"))

    fig2, ax2 = plt.subplots(figsize=(10, 4.2))
    th_deg = np.degrees(theta)
    ax2.plot(th_deg, scrub_total, lw=2.5, color="crimson", label="total scrub score")
    for i, w in enumerate(wheels):
        ax2.plot(th_deg, scrub_per_wheel[:, i], lw=1.2,
                 color=colours[i % len(colours)], alpha=0.6, label=w.name)

    threshold = n * 0.5
    ax2.fill_between(th_deg, scrub_total, threshold,
                     where=(scrub_total > threshold),
                     color="crimson", alpha=0.12, label=f"avoid (total > {threshold:.1f})")
    ax2.axhline(threshold, color="crimson", lw=1.0, ls="--", alpha=0.5)

    ax2.set_xlim(0, 360)
    ax2.set_xticks(np.arange(0, 361, 45))
    ax2.set_xlabel("travel heading [deg, body frame]")
    ax2.set_ylabel("scrub score  (0 = good,  N_wheels = worst)")
    ax2.set_title(
        "Total scrub score vs travel heading — coefficient-free, geometry only\n"
        "shaded = headings to avoid (all four wheels simultaneously scrubbing hard)"
    )
    ax2.legend(fontsize=8.5, loc="upper right")
    ax2.grid(alpha=0.28)
    fig2.tight_layout()
    safe_savefig(fig2, os.path.join(outdir, "06b_scrub_score_cartesian.png"))


def plot_goodness(wheels, res, outdir):
    """Combined heading goodness score — coefficient-free.

    Combines two signals, both pure geometry:

    1. drive_contrib = sum over wheels of |cos(angle between d_hat_i and
       travel heading)|, normalised 0..1.
       This is how much of each wheel's drive force goes toward the
       commanded heading -- taking abs() because a motor can drive the
       wheel in either rotational direction, so it can push along +d_hat
       or -d_hat, whichever the heading needs. (Clipping to positive
       instead of using abs() assumes the wheel can only push one way,
       which silently breaks the robot's left-right mirror symmetry --
       see the note in wheel_frames / the module docstring.)

    2. clean = 1 - normalised scrub  (1=no scrubbing, 0=worst)

    goodness = drive_contrib * clean

    Result: peaks where wheels are both well-aligned to push you AND not
    scrubbing. Dips at cardinals are moderate (drive force still decent
    going forward, just scrub is high). Diagonals score best.
    """
    goodness, clean, drive_norm = calculate_goodness(wheels, res)
    theta   = res["theta"]
    th_deg  = np.degrees(theta)

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    ax = axes[0]
    ax.plot(th_deg, clean,      lw=2,   color="tab:green",  label="cleanliness  (1 − scrub)")
    ax.plot(th_deg, drive_norm, lw=2,   color="tab:blue",   label="drive contribution  |cos(Δ)|, normalised")
    ax.plot(th_deg, goodness,   lw=2.8, color="crimson",    label="goodness = clean × drive")
    ax.fill_between(th_deg, goodness, alpha=0.15, color="crimson")
    ax.set_ylabel("score  (0–1)")
    ax.set_ylim(0, 1.05)
    ax.set_yticks([0, 0.25, 0.5, 0.75, 1.0])
    ax.legend(fontsize=9, loc="lower right")
    ax.set_title(
        "Heading goodness score — coefficient-free, geometry only\n"
        "High = wheels well-aligned to push toward heading AND clean roller behaviour\n"
        "Low  = wheels poorly aligned (little propulsive force) OR scrubbing",
        fontsize=10.5,
    )
    ax.grid(alpha=0.28)

    from scipy.signal import argrelmax
    peaks = argrelmax(goodness, order=15)[0]
    for pk in peaks:
        ax.annotate(f"{th_deg[pk]:.0f}°",
                    xy=(th_deg[pk], goodness[pk]),
                    xytext=(0, 10), textcoords="offset points",
                    ha="center", fontsize=8.5, color="crimson",
                    arrowprops=dict(arrowstyle="-", color="crimson", lw=0.8))

    ax2 = axes[1]
    ax2.fill_between(th_deg, goodness, alpha=0.20, color="crimson")
    ax2.plot(th_deg, goodness, lw=2.8, color="crimson")
    ax2.axhline(goodness.mean(), color="0.5", lw=1.0, ls="--",
                label=f"mean = {goodness.mean():.2f}")
    ax2.set_xlim(0, 360)
    ax2.set_xticks(np.arange(0, 361, 45))
    ax2.set_xlabel("travel heading [deg, body frame]")
    ax2.set_ylabel("goodness  (0–1)")
    ax2.set_ylim(0, 1.05)
    ax2.legend(fontsize=9)
    ax2.set_title("Goodness score — prefer headings where this is HIGH")
    ax2.grid(alpha=0.28)

    fig.tight_layout()
    safe_savefig(fig, os.path.join(outdir, "07_goodness.png"))

    fig2, ax3 = plt.subplots(figsize=(6, 6),
                              subplot_kw=dict(projection="polar"))
    ax3.plot(theta, goodness, lw=2.5, color="crimson")
    ax3.fill(theta, goodness, color="crimson", alpha=0.18)
    ax3.set_rlim(0, 1.05)
    ax3.set_rticks([0.25, 0.5, 0.75, 1.0])
    ax3.set_title(
        "Heading goodness (polar)\npeaks = best travel directions\n"
        "geometry only — no friction coefficients",
        fontsize=10.5, pad=20,
    )
    ax3.grid(alpha=0.35)
    fig2.tight_layout()
    safe_savefig(fig2, os.path.join(outdir, "07b_goodness_polar.png"))


# ----------------------------------------------------------------------------
# REPORTING
# ----------------------------------------------------------------------------


def print_summary(wheels, res, params, speed):
    theta = res["theta"]
    print("=" * 72)
    print("OMNI ROLLER-DRAG MODEL")
    print("=" * 72)
    print(f"  wheels           : {len(wheels)}")
    print(f"  sweep speed      : {speed} m/s (pure translation, omega = 0)")
    print(f"  mu_r (ASSUMED)   : {params['mu_r']}")
    print(f"  normal load/wheel: {res['N'][0]:.2f} N   (equal static split)")
    print(f"  drag scale mu_r*N: {params['mu_r'] * res['N'][0]:.3f} N")
    print()

    print("  Per-wheel drag magnitude over all headings:")
    for i, w in enumerate(wheels):
        f = res["Fmag"][:, i]
        print(f"    {w.name:<14s} min {f.min():6.3f} N   max {f.max():6.3f} N")
    print()

    i_lat = int(np.argmax(np.abs(res["lateral"])))
    i_res = int(np.argmax(res["resist"]))
    print("  Totals:")
    print(f"    peak resistive force : {res['resist'][i_res]:.3f} N "
          f"at heading {np.degrees(theta[i_res]):.1f} deg")
    print(f"    peak LATERAL force   : {res['lateral'][i_lat]:.3f} N "
          f"at heading {np.degrees(theta[i_lat]):.1f} deg")
    print(f"    peak |yaw torque|    : {np.abs(res['tau']).max() * 1e3:.4f} mN·m")
    print()
    print("  Lateral force is the term that pushes you off the commanded line.")
    print("  Direct yaw torque is ~0 whenever every wheel axle points radially:")
    print("  the drag is then collinear with the moment arm. Unwanted rotation")
    print("  in that case comes from the tangential forces needed to cancel the")
    print("  lateral drag, not from the drag torque itself.")
    print()

    R = params["wheel_radius"]
    nr = params["n_rollers"]
    dr = R * (1 - np.cos(np.pi / nr))
    f_pass = speed / R * nr / (2 * np.pi)
    print("  Roller-transition geometry (no friction data needed):")
    print(f"    roller pitch            : {360 / nr:.2f} deg")
    print(f"    effective radius ripple : {dr * 1e3:.3f} mm "
          f"({100 * dr / R:.2f} % of R)")
    print(f"    roller-pass frequency   : {f_pass:.1f} Hz at {speed} m/s")
    print("    -> periodic disturbance + systematic wheel-odometry error.")
    print("=" * 72)

def run_one_wheel_angle_config(params, args, wheels):
    res = sweep_heading(wheels, params, speed=args.speed, omega=args.omega)

    #print_summary(wheels, res, params, args.speed)

    goodness, clean, drive_norm = calculate_goodness(wheels, res)

    return goodness, clean, drive_norm

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speed", type=float, default=1.0, help="sweep speed [m/s]")
    ap.add_argument("--omega", type=float, default=0.0,
                    help="yaw rate for the main sweep [rad/s]")
    ap.add_argument("--mu-r", type=float, default=None,
                    help="override roller friction coefficient")
    ap.add_argument("--outdir", default="figures", help="output directory")
    ap.add_argument("--show", action="store_true", help="open interactive windows")
    args = ap.parse_args()

    params = dict(PARAMS)
    if args.mu_r is not None:
        params["mu_r"] = args.mu_r

    prepare_outdir(args.outdir)

    res = sweep_heading(ROBOT_WHEELS, params, speed=args.speed, omega=args.omega)

    print_summary(ROBOT_WHEELS, res, params, args.speed)

    plot_geometry(ROBOT_WHEELS, res, args.outdir)
    plot_per_wheel_slip(ROBOT_WHEELS, res, args.outdir)
    plot_per_wheel_drag(ROBOT_WHEELS, res, params, args.outdir)
    plot_total(res, params, args.outdir)
    plot_lateral_map(ROBOT_WHEELS, params, args.outdir, args.speed)
    plot_scrub_score(ROBOT_WHEELS, res, args.outdir)
    plot_goodness(ROBOT_WHEELS, res, args.outdir)

    print(f"\nFigures written to ./{args.outdir}/")

    if args.show:
        matplotlib.use("TkAgg", force=True)
        plt.show()


if __name__ == "__main__":
    main()
