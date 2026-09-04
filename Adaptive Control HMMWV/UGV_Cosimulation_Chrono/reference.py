"""
reference.py
Reference trajectory generator for the bicycle model MRAC simulation.

Translated from reference.m (Willebeek-LeMair, Widman, L'Afflitto, 2025)
"""

import numpy as np


# ── Trajectory selector ───────────────────────────────────────────────────────
# Set to 'circle' or 'straight'
TRAJECTORY = 'circle'

# Shared
ORIGIN = np.array([0.0, -1.7])   # trajectory start point [m]
T_HOLD = 3.0                      # seconds to hold at ORIGIN before moving

# Circle parameters
T_CIRCLE = 10.0   # period [s]
T_RAMP   = 1.0    # cosine blend-in duration [s] after T_HOLD

# Straight-line parameters
V_FINAL    = 5.0   # [m/s]  cruise speed along +Y
T_RAMP_VEL = 3.0   # [s]    constant-acceleration ramp duration
#   implied acceleration = V_FINAL / T_RAMP_VEL


def reference(t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """
    Return (r_user, r_user_dot, r_user_ddot, T_HOLD) at time t.
    Select trajectory with the module-level TRAJECTORY variable.
    """
    if t < T_HOLD:
        return ORIGIN.copy(), np.zeros(2), np.zeros(2), T_HOLD

    tau = t - T_HOLD

    if TRAJECTORY == 'straight':
        return _straight(tau)

    return _circle(tau)


# ── Circle ────────────────────────────────────────────────────────────────────

def _circle(tau: float):
    T = T_CIRCLE

    if tau <= T:
        r_user      = ORIGIN + 10.0 * np.array([1 - np.cos(-2*np.pi*tau/T),
                                        np.sin(2*np.pi*tau/T)])
        r_user_dot  = 10.0 * np.array([(-2*np.pi/T)*np.sin(-2*np.pi*tau/T),
                                        (2*np.pi/T)*np.cos(2*np.pi*tau/T)])
        r_user_ddot = 10.0 * np.array([(-2*np.pi/T)**2 * np.cos(-2*np.pi*tau/T),
                                        -(2*np.pi/T)**2 * np.sin(2*np.pi*tau/T)])
    else:
        r_user      = ORIGIN + 10.0 * np.array([-1 + np.cos(-2*np.pi*tau/T),
                                         np.sin(2*np.pi*tau/T)])
        r_user_dot  = 10.0 * np.array([-(-2*np.pi/T)*np.sin(-2*np.pi*tau/T),
                                         (2*np.pi/T)*np.cos(2*np.pi*tau/T)])
        r_user_ddot = 10.0 * np.array([-(-2*np.pi/T)**2 * np.cos(-2*np.pi*tau/T),
                                        -(2*np.pi/T)**2 * np.sin(2*np.pi*tau/T)])

    # Smooth blend-in: r_user, r_user_dot, r_user_ddot all start at 0.
    # r_user is blended from ORIGIN to the circle so position and velocity
    # stay consistent — r_cmd feedforward is never starved of a matching r_user.
    # (r_circle(tau=0) == ORIGIN, so the blend is seamless at both ends.)
    if tau < T_RAMP:
        sigma      = 0.5 * (1.0 - np.cos(np.pi * tau / T_RAMP))
        sigma_dot  = 0.5 * (np.pi / T_RAMP) * np.sin(np.pi * tau / T_RAMP)
        sigma_ddot = 0.5 * (np.pi / T_RAMP)**2 * np.cos(np.pi * tau / T_RAMP)
        delta_r     = r_user - ORIGIN
        r_user_ddot = sigma_ddot * delta_r + 2.0 * sigma_dot * r_user_dot + sigma * r_user_ddot
        r_user_dot  = sigma_dot * delta_r + sigma * r_user_dot
        r_user      = ORIGIN + sigma * delta_r

    return r_user, r_user_dot, r_user_ddot, T_HOLD


# ── Straight line ─────────────────────────────────────────────────────────────

def _straight(tau: float):
    """
    Straight line along +Y from ORIGIN minus rear axle offset.
    Constant acceleration from rest to V_FINAL over T_RAMP_VEL seconds,
    then constant velocity (zero acceleration).
     """
    a_y = V_FINAL / T_RAMP_VEL

    if tau < T_RAMP_VEL:
        v_y    = a_y * tau
        y_disp = 0.5 * a_y * tau**2
        acc_y  = a_y
    else:
        v_y    = V_FINAL
        y_disp = 0.5 * V_FINAL * T_RAMP_VEL + V_FINAL * (tau - T_RAMP_VEL)
        acc_y  = 0.0

    r_user      = ORIGIN + np.array([0.0, y_disp])
    r_user_dot  = np.array([0.0, v_y])
    r_user_ddot = np.array([0.0, acc_y])

    return r_user, r_user_dot, r_user_ddot, T_HOLD
