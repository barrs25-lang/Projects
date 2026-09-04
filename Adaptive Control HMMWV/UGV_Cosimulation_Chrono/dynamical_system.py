"""
dynamical_system.py
ODE right-hand side for the bicycle MRAC simulation.

Translated from dynamical_system.m (Willebeek-LeMair, Widman, L'Afflitto, 2025)

Two calling modes
─────────────────
  Standalone (main.py):
      xdot, outputs = dynamical_system(t, x, parameters)
      All plant quantities (v, v_dot, theta_dot) computed from bicycle model.

  Co-simulation (main_cosim.py):
      xdot, outputs = dynamical_system(t, x, parameters, chrono_state=cs)
      where cs = {
          'v'         : (2,) true rear-axle world velocity   [m/s]
          'v_dot'     : (2,) true rear-axle world acceleration [m/s²]
          'theta_dot' : float true yaw rate                   [rad/s]
      }
      These replace the bicycle-model reconstructions so the outer-loop
      error signal and rho_tran are computed from actual Chrono physics.
      Plant-slot derivatives (xdot[0:4]) are still zeroed by make_ctrl_ode
      in main_cosim.py so RK4 never integrates the plant states.

State vector layout
───────────────────
Indices 0-1   : r            — rear wheel position        (Eq. 3.70a)
Index   2     : v1           — rear wheel longitudinal speed (Eq. 3.70c)
Index   3     : theta        — heading angle               (Eq. 3.70b)
Indices 4-5   : r_cmd        — outer-loop position command  (Eq. 3.73)
Indices 6-7   : E_OL_PI      — outer-loop PI error integral (Eq. 3.73)
Indices 8-9   : v_ref        — outer-loop reference model   (Eq. 3.76)
Indices 10-11 : E_OL_RM_PI_1 — 1st outer-loop RM PI integral (Eq. 3.77)
Indices 12-13 : E_OL_RM_PI_2 — 2nd outer-loop RM PI integral (Eq. 3.77)
[14 ...]      : K_hat_OL     — outer-loop adaptive gains
[...]         : x_IL_LPF     — inner-loop LPF state         (Eq. 3.89-3.93)
[...]         : E_IL_RM_PI   — inner-loop RM PI integral     (Eq. 3.86)
[...]         : theta_ref    — inner-loop reference model    (Eq. 3.85)
[...]         : K_hat_IL     — inner-loop adaptive gains
[...]         : v_debug      — debugging velocity
[...]         : e_ref_OL     — Two_Layer outer reference error
[...]         : e_ref_IL     — Two_Layer inner reference error
"""

import numpy as np
from reference import reference
from adaptive_control_law import adaptive_control_law


def dynamical_system(t: float,
                     x: np.ndarray,
                     parameters: dict,
                     chrono_state: dict = None):
    """
    Compute xdot = f(t, x) for the bicycle MRAC system.

    Parameters
    ----------
    t             : current time
    x             : full state vector
    parameters    : dict of model and controller parameters
    chrono_state  : (optional) dict with true plant quantities from Chrono.
                    Keys: 'v' (2,), 'v_dot' (2,), 'theta_dot' (float).
                    When None the bicycle model equations are used (standalone).

    Returns
    -------
    xdot    : (N,) state derivative
    outputs : dict with logged signals
    """
    two_layer = parameters['Two_Layer_Active']

    # ── Unpack states ─────────────────────────────────────────────────────────
    r            = x[0:2]
    v1           = x[2]
    theta        = x[3]
    r_cmd        = x[4:6]
    E_OL_PI      = x[6:8]
    v_ref        = x[8:10]
    E_OL_RM_PI_1 = x[10:12]
    E_OL_RM_PI_2 = x[12:14]

    if two_layer:
        K_hat_OL   = x[14:32].reshape(9, 2)
        x_IL_LPF   = x[32:34]
        E_IL_RM_PI = x[34]
        theta_ref  = x[35]
        K_hat_IL   = x[36:41]
        v_debug    = x[41:43]
        e_ref_OL   = x[43:45]
        e_ref_IL   = x[45]
    else:
        K_hat_OL   = x[14:28].reshape(7, 2)
        x_IL_LPF   = x[28:30]
        E_IL_RM_PI = x[30]
        theta_ref  = x[31]
        K_hat_IL   = x[32:36]
        v_debug    = x[36:38]
        e_ref_OL   = None
        e_ref_IL   = None

    emax = parameters['emax']
    M    = parameters['M']

    # ── Velocity of rear wheel in global frame ────────────────────────────────
    # Co-sim: use Chrono's true world velocity of the rear axle.
    # Standalone: reconstruct from bicycle model kinematics (Eq. 3.70a).
    if chrono_state is not None:
        r       = np.asarray(chrono_state['r'], dtype=float)   # rear axle position from Chrono
        v1       = np.asarray(chrono_state['v1'],
        dtype=float)   # longitudinal speed from Chrono
        theta    = np.asarray(chrono_state['theta'], dtype=float)                # heading from Chrono
        v         = np.asarray(chrono_state['v'],         dtype=float)
        v_dot     = np.asarray(chrono_state['v_dot'],     dtype=float)
        theta_dot = float(chrono_state['theta_dot'])
    else:
        v         = np.array([np.cos(theta), np.sin(theta)]) * v1
        v_dot     = None   # filled later from bicycle model
        theta_dot = None   # filled later from bicycle model

    # ── Reference trajectory ──────────────────────────────────────────────────
    r_user, r_user_dot, r_user_ddot, _ = reference(t)

    # ── Outer-loop: position command (Eq. 3.73) ───────────────────────────────
    K_P_tran_kin = parameters['K_P_tran_kin']
    K_I_tran_kin = parameters['K_I_tran_kin']

    E_OL_PI_dot = r - r_user
    r_cmd_dot   = r_user_dot - K_P_tran_kin @ (r - r_user) - K_I_tran_kin @ E_OL_PI

    # ── Outer-loop: reference model (Eq. 3.76-3.78) ───────────────────────────
    A_ref_tran   = parameters['A_ref_tran']
    K_P_tran_dyn = parameters['K_P_tran_dyn']
    K_I_tran_dyn = parameters['K_I_tran_dyn']

    E_OL_RM_PI_1_dot = v_ref - r_cmd_dot
    E_OL_RM_PI_2_dot = E_OL_RM_PI_1_dot

    # r_cmd_ddot (Eq. 3.78):
    #   Co-sim: uses true v from Chrono so velocity error is accurate.
    #   Standalone: same formula, v is from bicycle model.
    r_cmd_ddot = (-K_P_tran_kin @ (v - r_user_dot)
                  - K_I_tran_kin @ (r - r_user)
                  + r_user_ddot)

    rho_tran = (r_cmd_ddot
                - A_ref_tran @ r_cmd_dot
                - K_P_tran_dyn @ E_OL_RM_PI_1
                - K_I_tran_dyn @ E_OL_RM_PI_2)

    v_ref_dot = A_ref_tran @ v_ref + rho_tran

    # ── Outer-loop adaptive law ───────────────────────────────────────────────
    # OL_error = v - v_ref: co-sim uses true Chrono v, standalone uses model v.
    OL_error = v - v_ref
    error    = OL_error

    Phi       = np.array([1.0, np.linalg.norm(v) * v[0],
                               np.linalg.norm(v) * v[1]])
    pi_vector = np.concatenate([v, rho_tran, Phi])
    K_hat     = K_hat_OL
    al_settings = parameters['outer_loop_adaptive_law_settings']

    constrained = parameters['Constrained_Active']

    if constrained:
        Q  = al_settings['Q'];  P = al_settings['P']
        H  = emax**2 - error.T @ M @ error
        V_ = error.T @ P @ error / H
        diff_h       = -1.0 if error.T @ Q @ error > V_ else 0.0
        eps_error_OL = None;  e_ref_OL_dot = None
    elif two_layer:
        A_transient  = al_settings['A_e']
        eps_error_OL = error - e_ref_OL
        e_ref_OL_dot = A_transient @ e_ref_OL
        diff_h = H = V_ = None
    else:
        diff_h = H = V_ = eps_error_OL = e_ref_OL_dot = None

    u_tilde_force, K_hat_OL_dot, _ = adaptive_control_law(
        pi_vector, error, K_hat, H, V_, M, diff_h, eps_error_OL, al_settings
    )

    # ── Traction force and heading command (Eq. 3.79-3.80) ───────────────────
    u_force   = float(np.linalg.norm(u_tilde_force))
    theta_cmd = float(np.arctan2(u_tilde_force[1], u_tilde_force[0]))

    # Yaw unwrapping (Eq. 3.94)
    theta_cmd = theta_cmd + 2*np.pi * np.round(
        (x_IL_LPF[0] - theta_cmd) / (2*np.pi)
    )

    # ── Inner-loop: second-order LPF (Eq. 3.89-3.93) ─────────────────────────
    w = parameters['natural_frequency_IL_LPF']
    z = parameters['damping_ratio_IL_LPF']

    A_IL_LPF     = np.array([[0.0,    1.0],
                              [-w**2, -2*z*w]])
    B_IL_LPF     = np.array([0.0, w**2])
    x_IL_LPF_dot = A_IL_LPF @ x_IL_LPF + B_IL_LPF * theta_cmd

    # ── Inner-loop: reference model (Eq. 3.85-3.86) ───────────────────────────
    A_ref_rot   = parameters['A_ref_rot']
    K_I_rot_kin = parameters['K_I_rot_kin']

    theta_lp     = x_IL_LPF[0]
    theta_lp_dot = x_IL_LPF_dot[0]

    E_IL_RM_PI_dot = theta_ref - theta_lp
    rho_rot        = theta_lp_dot - A_ref_rot * theta_lp - K_I_rot_kin * E_IL_RM_PI
    theta_ref_dot  = A_ref_rot * theta_ref + rho_rot

    # ── Inner-loop adaptive law ───────────────────────────────────────────────
    # IL_error = theta - theta_ref: theta comes from Chrono in co-sim mode,
    # so this error is against the true vehicle heading.
    IL_error  = theta - theta_ref
    error     = np.array([IL_error])

    Phi       = np.array([1.0, IL_error])
    pi_vector = np.array([theta, rho_rot, Phi[0], Phi[1]])
    K_hat     = K_hat_IL
    al_settings = parameters['inner_loop_adaptive_law_settings']

    if constrained:
        Q  = al_settings['Q'];  P = al_settings['P']
        H  = emax**2 - float(error.T @ M @ error)
        V_ = float(error.T @ P @ error) / H
        diff_h       = -1.0 if float(error.T @ Q @ error) > V_ else 0.0
        eps_error_IL = None;  e_ref_IL_dot = None
    elif two_layer:
        A_transient  = al_settings['A_e']
        eps_error_IL = error - np.array([e_ref_IL])
        e_ref_IL_dot = float(A_transient) * e_ref_IL
        diff_h = H = V_ = None
    else:
        diff_h = H = V_ = eps_error_IL = e_ref_IL_dot = None

    u_moment, K_hat_IL_dot, _ = adaptive_control_law(
        pi_vector, error, K_hat, H, V_, M, diff_h, eps_error_IL, al_settings
    )

    # ── Steering angle (Eq. 3.88) ─────────────────────────────────────────────
    delta = float(np.arctan2(float(u_moment), v1))

    # ── Plant dynamics (Eq. 3.70) ─────────────────────────────────────────────
    # Co-sim: Chrono owns all plant derivatives; these values are only written
    #         to xdot[0:4] which make_ctrl_ode zeros before RK4 sees them.
    #         We still compute them so standalone mode works identically.
    L      = parameters['L']
    m_p    = parameters['m']
    b_lin  = 1.0
    b_quad = 3.0

    if chrono_state is not None:
        # Use Chrono truth for plant derivatives (ignored by RK4 anyway,
        # but kept consistent for any diagnostic output that reads xdot[0:4])
        r_dot_out     = v                                    # true velocity
        theta_dot_out = theta_dot                            # true yaw rate
        v1_dot_out    = (u_force / m_p
                         - b_lin  * v1 / m_p
                         - b_quad * v1 * abs(v1) / m_p)     # model-based
    else:
        r_dot_out     = v
        theta_dot_out = v1 * np.tan(delta) / L
        v1_dot_out    = (u_force / m_p
                         - b_lin  * v1 / m_p
                         - b_quad * v1 * abs(v1) / m_p
                         + parameters['alpha_unmatched'] * np.sin(0.2 * t))

    v_debug_dot = u_tilde_force / m_p

    # ── Assemble xdot ─────────────────────────────────────────────────────────
    xdot_parts = [
        r_dot_out,                          # 0:2
        np.array([v1_dot_out]),             # 2
        np.array([theta_dot_out]),          # 3
        r_cmd_dot,                          # 4:6
        E_OL_PI_dot,                        # 6:8
        v_ref_dot,                          # 8:10
        E_OL_RM_PI_1_dot,                   # 10:12
        E_OL_RM_PI_2_dot,                   # 12:14
        K_hat_OL_dot.ravel(),               # 14:...
        x_IL_LPF_dot,
        np.array([E_IL_RM_PI_dot]),
        np.array([theta_ref_dot]),
        K_hat_IL_dot.ravel(),
        v_debug_dot,
    ]

    if two_layer:
        xdot_parts.append(e_ref_OL_dot)
        xdot_parts.append(np.array([e_ref_IL_dot]))

    xdot = np.concatenate([np.atleast_1d(p) for p in xdot_parts])

    # ── Outputs dict ──────────────────────────────────────────────────────────
    outputs = {
        'u_force':       u_force,
        'u_moment':      float(u_moment),
        'theta_cmd':     theta_cmd,
        'delta':         delta,
        'rho_tran':      rho_tran,
        'rho_rot':       rho_rot,
        'u_tilde_force': u_tilde_force,
        'OL_error':      OL_error,
        'IL_error':      IL_error,
        'v':             v,
        'r_user_dot':      r_user_dot,
    }

    return xdot, outputs
