"""
vec.py
Vectorise a matrix (column-major, matching MATLAB's M(:) behaviour).

Translated from vec.m (Willebeek-LeMair, Widman, L'Afflitto, 2025)
"""

import numpy as np


def vec(M: np.ndarray) -> np.ndarray:
    """Return M stacked column-by-column into a 1-D vector (MATLAB M(:))."""
    return np.asarray(M).ravel(order='F')
