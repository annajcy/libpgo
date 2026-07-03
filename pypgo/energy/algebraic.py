"""Closed-form algebraic energies: linear and quadratic."""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo._utils import float_vector
from pypgo.sparse import as_coo
from pypgo.energy.base import PotentialEnergy


class LinearEnergy(PotentialEnergy):
    """Linear potential energy: b^T x.

    Parameters
    ----------
    b : ndarray (n,) float64
        Linear coefficient vector.  Copied into C++ owned storage.
    """

    def __init__(self, b: np.ndarray) -> None:
        b_arr = float_vector("b", b)
        super().__init__(_core._create_linear_energy(b_arr))

    def __repr__(self) -> str:
        return f"LinearEnergy({self.num_dofs} DOFs)"


class QuadraticEnergy(PotentialEnergy):
    """Quadratic potential energy: 1/2 x^T A x + b^T x.

    Parameters
    ----------
    A : ndarray (m, m) float64, PySparseMatrix, or COO 5-tuple
        Quadratic coefficient matrix.  Dense ndarrays are automatically
        converted to sparse COO (zeros are dropped).
    b : ndarray (m,) float64 or None
        Optional linear term.
    """

    def __init__(self, A, b=None) -> None:
        rows, cols, row_indices, col_indices, values = as_coo(A)

        if b is not None:
            b_arr = float_vector("b", b)
            handle = _core._create_quadratic_energy_from_coo_with_b(
                rows, cols, row_indices, col_indices, values, b_arr,
            )
        else:
            handle = _core._create_quadratic_energy_from_coo(
                rows, cols, row_indices, col_indices, values,
            )
        super().__init__(handle)

    def __repr__(self) -> str:
        return f"QuadraticEnergy({self.num_dofs} DOFs)"
