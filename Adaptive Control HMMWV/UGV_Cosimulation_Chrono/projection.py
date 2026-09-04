"""
projection.py
Projection operator for constraining adaptive gains.

Translated from proj.m and Projection.m (Willebeek-LeMair, Widman, L'Afflitto, 2025)
"""

import numpy as np
from convex_function import convex_function


def proj(
    x: np.ndarray,
    x_d: np.ndarray,
    x_e: np.ndarray,
    epsilon: float,
    s,
) -> np.ndarray:
    """
    Scalar-column projection operator.

    Modifies the update direction x_d so that x stays within the convex set
    defined by h(x) ≤ 0.

    Parameters
    ----------
    x       : current parameter column  (n,)
    x_d     : unconstrained update direction  (n,)
    x_e     : nominal parameter centre  (n,)
    epsilon : convex-function shape scalar ε > 0
    s       : shape parameter(s) — scalar or array (n,)

    Returns
    -------
    x_d_modified : (n,) possibly modified update direction
    """
    x   = np.atleast_1d(x).astype(float)
    x_d = np.atleast_1d(x_d).astype(float)

    h, dh_dx = convex_function(x, x_e, epsilon, s)

    # Projection correction (added 1e-8*eps to denominator to guard against /0)
    # Equivalent to MATLAB:  dh_dx*x_d > 0  →  (dh_dx @ x_d) > 0
    # dh_dx is (1, n), x_d is (n,)  →  inner product is a (1,1) array
    inner = float((dh_dx @ x_d).ravel()[0])
    denom = float((dh_dx @ dh_dx.T).ravel()[0]) + 1e-8 * np.finfo(float).eps
    if h > 0 and inner > 0:
        correction = h / denom * (dh_dx.T @ dh_dx) @ x_d
        x_d_modified = x_d - correction.ravel()
    else:
        x_d_modified = x_d.copy()

    return x_d_modified


def Projection(
    X: np.ndarray,
    X_d: np.ndarray,
    X_e: np.ndarray,
    epsilon: float,
    s,
) -> np.ndarray:
    """
    Matrix projection operator — applies proj() column-by-column.

    Parameters
    ----------
    X       : (n, q) current parameter matrix
    X_d     : (n, q) unconstrained update matrix
    X_e     : (n, q) nominal parameter matrix  (same epsilon/s applied to each col)
    epsilon : convex-function scalar ε
    s       : shape parameter(s)

    Returns
    -------
    X_d_modified : (n, q) modified update matrix
    """
    X   = np.atleast_2d(X).astype(float)
    X_d = np.atleast_2d(X_d).astype(float)
    X_e = np.atleast_2d(X_e).astype(float)

    p, q = X_d.shape
    X_d_modified = np.empty_like(X_d)

    for jj in range(q):
        X_d_modified[:, jj] = proj(X[:, jj], X_d[:, jj], X_e[:, jj], epsilon, s)

    return X_d_modified
