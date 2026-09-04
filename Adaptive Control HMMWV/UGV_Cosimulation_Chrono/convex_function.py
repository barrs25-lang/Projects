"""
convex_function.py
Convex function h(x) and its gradient used by the projection operator.

Translated from convex_function.m (Willebeek-LeMair, Widman, L'Afflitto, 2025)
"""

import numpy as np


def convex_function(
    x: np.ndarray,
    x_e: np.ndarray,
    epsilon: float,
    s,          # scalar or 1-D array matching the size of x
) -> tuple[float, np.ndarray]:
    """
    Compute the convex function h and its gradient dh/dx.

    h(x) = [(1+ε)(x - x_e)ᵀ diag(1/s) (x - x_e) - 1] / ε

    Parameters
    ----------
    x       : current parameter vector  (n,)
    x_e     : nominal parameter vector  (n,)
    epsilon : positive scalar ε
    s       : shape parameter(s) — scalar or array (n,)

    Returns
    -------
    h       : scalar value of h at x
    dh_dx   : (1, n) row gradient of h at x
    """
    x   = np.atleast_1d(x).astype(float)
    x_e = np.atleast_1d(x_e).astype(float)
    s   = np.atleast_1d(s).astype(float)

    diff = x - x_e
    inv_s = 1.0 / s

    # h = [(1+ε) diffᵀ diag(1/s) diff - 1] / ε
    h = ((1.0 + epsilon) * (diff * inv_s) @ diff - 1.0) / epsilon

    # dh/dx = [2(1+ε) (x - x_e)ᵀ diag(1/s)] / ε  →  row vector (1, n)
    dh_dx = (2.0 * (1.0 + epsilon) * diff * inv_s / epsilon).reshape(1, -1)

    return float(h), dh_dx
