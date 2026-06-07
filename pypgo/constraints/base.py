"""Read-only handle base class for vector-valued hard constraint functions."""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo.sparse import SparseMatrix


class ConstraintFunction:
    """Read-only handle for vector-valued hard constraint functions."""

    def __init__(self, handle):
        if not isinstance(handle, _core.PyConstraintFunctions):
            raise TypeError(
                f"handle must be a _core.PyConstraintFunctions, got {type(handle).__name__}"
            )
        object.__setattr__(self, "_handle", handle)

    def __setattr__(self, name, value):
        raise AttributeError(f"ConstraintFunction is immutable; cannot set {name!r}")

    def __delattr__(self, name):
        raise AttributeError(f"ConstraintFunction is immutable; cannot delete {name!r}")

    @property
    def num_dofs(self) -> int:
        return self._handle.num_dofs

    @property
    def num_constraints(self) -> int:
        return self._handle.num_constraints

    @property
    def is_linear(self) -> bool:
        return self._handle.is_linear

    def value(self, x):
        return self._handle.value(np.asarray(x, dtype=np.float64, order="C"))

    def jacobian(self, x) -> SparseMatrix:
        return SparseMatrix(self._handle.jacobian(np.asarray(x, dtype=np.float64, order="C")))

    def hessian(self, x, multipliers) -> SparseMatrix:
        return SparseMatrix(
            self._handle.hessian(
                np.asarray(x, dtype=np.float64, order="C"),
                np.asarray(multipliers, dtype=np.float64, order="C"),
            )
        )

    def __repr__(self) -> str:
        return repr(self._handle)
