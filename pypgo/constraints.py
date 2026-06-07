"""Hard constraint function helpers for pypgo."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

import pypgo._core as _core
from pypgo._arrays import float_vector
from pypgo.sparse import SparseMatrix, as_sparse_matrix


def _bounds_vector(name, value, size):
    if np.isscalar(value):
        return np.full(size, float(value), dtype=np.float64)
    arr = float_vector(name, value)
    if arr.shape != (size,):
        raise ValueError(f"{name} must be scalar or shape ({size},), got shape {arr.shape}")
    return arr.copy()


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
            if not hasattr(constraint, "_handle"):
                raise TypeError(f"term {i}: expected a constraint function")
            handles.append(constraint._handle)
        super().__init__(_core._create_constraint_function_set(handles))

    def __repr__(self) -> str:
        return f"ConstraintFunctionSet({self.num_constraints} constraints, {self.num_dofs} DOFs)"


@dataclass(frozen=True, repr=False)
class Bounded:
    """Bounds for ``lower <= functions(x) <= upper``."""

    functions: ConstraintFunction
    lower: np.ndarray = field(kw_only=True)
    upper: np.ndarray = field(kw_only=True)

    def __post_init__(self):
        if not isinstance(self.functions, ConstraintFunction):
            raise TypeError("functions must be a pypgo.constraints.ConstraintFunction")
        object.__setattr__(
            self, "lower", _bounds_vector("lower", self.lower, self.functions.num_constraints))
        object.__setattr__(
            self, "upper", _bounds_vector("upper", self.upper, self.functions.num_constraints))

    def __repr__(self) -> str:
        return f"Bounded({self.functions.num_constraints} constraints)"


__all__ = [
    "Bounded",
    "ConstraintFunction",
    "ConstraintFunctionSet",
    "Linear",
]
