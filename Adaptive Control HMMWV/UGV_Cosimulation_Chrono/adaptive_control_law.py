"""
adaptive_control_law.py
Adaptive control law for MRAC — supports all variants.

Translated from adaptive_control_law.m (Willebeek-LeMair, Widman, L'Afflitto, 2025)

Supported variants
------------------
'Classical'          — standard MRAC update law
'Deadzone'           — deadzone modification
'Modified-Deadzone'  — smooth (polynomial) deadzone
'sigma-Modification' — σ-modification
'e-Modification'     — e-modification (norm-weighted σ)
'Projection_Operator'— projection-based constraint
'Constrained'        — constrained MRAC (barrier function)
'Two_Layer'          — two-layer MRAC (transient + steady-state)
"""

import numpy as np
from math import comb
from projection import Projection


def adaptive_control_law(
    pi_vector: np.ndarray,
    error: np.ndarray,
    K_hat: np.ndarray,
    H,
    V,
    M: np.ndarray,
    diff_h,
    eps_error,
    adaptive_law_settings: dict,
) -> tuple[np.ndarray, np.ndarray, object]:
    """
    Compute the control input u and the adaptive gain update K_dot.

    Parameters
    ----------
    pi_vector  : regressor vector  (N,) or (N, 1)
    error      : tracking error  (n,) or (n, 1)
    K_hat      : current adaptive gain matrix  (N, n)  or vector (N,)
    H, V, M    : barrier/constraint quantities (used in 'Constrained' only)
    diff_h     : constraint indicator (used in 'Constrained' only)
    eps_error  : reference-model error (used in 'Two_Layer' only)
    adaptive_law_settings : dict with all hyperparameters

    Returns
    -------
    u     : control input  (n,)
    K_dot : gain derivative — shape matches K_hat
    mu    : scalar modification term (or None)
    """
    variant = adaptive_law_settings.get('variant', 'Classical')

    Gamma       = adaptive_law_settings['Gamma']
    P           = adaptive_law_settings['P']
    B           = adaptive_law_settings['B']
    P_transient = adaptive_law_settings.get('P_e', None)

    # Ensure column vectors
    pi_vector = np.atleast_1d(pi_vector).astype(float).reshape(-1, 1)
    error     = np.atleast_1d(error).astype(float).reshape(-1, 1)
    K_hat     = np.atleast_1d(K_hat).astype(float)
    if K_hat.ndim == 1:
        K_hat = K_hat.reshape(-1, 1)

    P = np.atleast_2d(P).astype(float)
    B = np.atleast_2d(B).astype(float)

    # Control output (all variants except Two_Layer compute u = K_hat^T pi)
    if variant != 'Two_Layer':
        u = (K_hat.T @ pi_vector).ravel()
    else:
        u = None   # filled below

    mu = None

    # ── Classical ─────────────────────────────────────────────────────────────
    if variant == 'Classical':
        K_dot = -(Gamma @ pi_vector @ (error.T @ P @ B))

    # ── Deadzone ──────────────────────────────────────────────────────────────
    elif variant == 'Deadzone':
        E0      = adaptive_law_settings['E0']
        sqrtP   = adaptive_law_settings['sqrtP']
        alpha   = np.linalg.norm(sqrtP @ error, 2)
        mu_step = float(alpha >= E0)
        K_dot   = mu_step * (-(Gamma @ pi_vector @ (error.T @ P @ B)))
        mu      = mu_step

    # ── Modified-Deadzone ─────────────────────────────────────────────────────
    elif variant == 'Modified-Deadzone':
        E0    = adaptive_law_settings['E0']
        E1    = adaptive_law_settings['E1']
        d     = adaptive_law_settings['d']
        sqrtP = adaptive_law_settings['sqrtP']

        alpha       = np.linalg.norm(sqrtP @ error, 2)
        alpha_tilde = (alpha - E0) / (E1 - E0)

        middle_term = 0.0
        for k in range(d + 1):
            middle_term += (alpha_tilde ** (d + 1)
                            * comb(d + k, k)
                            * comb(2*d + 1, d - k)
                            * (-alpha_tilde) ** k)

        middle_term = float(np.clip(middle_term, 0.0, 1.0))

        mu_d = (middle_term * (0.0 <= alpha_tilde <= 1.0)
                + float(alpha_tilde > 1.0))
        K_dot = mu_d * (-(Gamma @ pi_vector @ (error.T @ P @ B)))
        mu    = mu_d

    # ── σ-Modification ────────────────────────────────────────────────────────
    elif variant == 'sigma-Modification':
        Sigma = adaptive_law_settings['Sigma']
        K_dot = -(Gamma @ (pi_vector @ (error.T @ P @ B) + Sigma * K_hat))

    # ── e-Modification ────────────────────────────────────────────────────────
    elif variant == 'e-Modification':
        Sigma = adaptive_law_settings['Sigma']
        K_dot = -(Gamma @ (pi_vector @ (error.T @ P @ B)
                            + Sigma * np.linalg.norm(error.T @ P @ B, 2) * K_hat))

    # ── Projection Operator ───────────────────────────────────────────────────
    elif variant == 'Projection_Operator':
        epsilon = adaptive_law_settings['epsilon']
        s       = adaptive_law_settings['s']
        K_dot_unconstrained = -(Gamma @ pi_vector @ (error.T @ P @ B))
        K_dot = Projection(K_hat, K_dot_unconstrained, K_hat, epsilon, s)

    # ── Constrained ───────────────────────────────────────────────────────────
    elif variant == 'Constrained':
        K_dot = -(Gamma / H * pi_vector @ (error.T @ (P - diff_h * V * M) @ B))
        mu    = diff_h

    # ── Two_Layer ─────────────────────────────────────────────────────────────
    elif variant == 'Two_Layer':
        Gamma_g   = adaptive_law_settings['Gamma_g']
        eps_error = np.atleast_1d(eps_error).astype(float).reshape(-1, 1)

        P_e = np.atleast_2d(P_transient).astype(float)

        K_dot_normal = -(Gamma     @ pi_vector @ (eps_error.T @ P_e @ B))
        K_dot_g      = -(Gamma_g   @ error     @ (eps_error.T @ P_e @ B))

        K_dot = np.vstack([K_dot_normal, K_dot_g])

        pi_ext = np.vstack([pi_vector, error])
        u      = (K_hat.T @ pi_ext).ravel()

    else:
        raise ValueError(
            f"adaptive_control_law(): unrecognised variant '{variant}'."
        )

    return u, K_dot, mu
