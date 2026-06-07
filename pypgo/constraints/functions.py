"""Concrete constraint functions.

This is the side of the constraints package expected to grow: each concrete
constraint type lives here.  When this file gets heavy, promote it to a
``functions/`` subpackage (one module per constraint type).
"""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo._utils import float_vector
from pypgo.sparse import as_sparse_matrix
from pypgo.constraints.base import ConstraintFunction


class Linear(ConstraintFunction):
    """Linear vector constraint ``C(x) = A @ x + offset``."""

    def __init__(self, A, offset=None):
        sparse = as_sparse_matrix(A)
        if offset is None:
            offset_arr = np.zeros(sparse.shape[0], dtype=np.float64)
        else:
            offset_arr = float_vector("offset", offset)
            if offset_arr.shape != (sparse.shape[0],):
                raise ValueError(
                    f"offset must have shape ({sparse.shape[0]},), got shape {offset_arr.shape}"
                )
        super().__init__(_core._create_linear_constraint(sparse._handle, offset_arr.copy()))

    def __repr__(self) -> str:
        return f"Linear({self.num_constraints} constraints, {self.num_dofs} DOFs)"


class ConstraintFunctionSet(ConstraintFunction):
    """Concatenate vector-valued constraints in input row order."""

    def __init__(self, constraints):
        handles = []
        for i, constraint in enumerate(constraints):
            if not isinstance(constraint, ConstraintFunction):
                raise TypeError(f"term {i}: expected a pypgo.constraints.ConstraintFunction")
            handles.append(constraint._handle)
        super().__init__(_core._create_constraint_function_set(handles))

    def __repr__(self) -> str:
        return f"ConstraintFunctionSet({self.num_constraints} constraints, {self.num_dofs} DOFs)"
