"""Hard constraint function helpers for pypgo."""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo.sparse import SparseMatrix


def _float_vector(name, arr):
    a = np.asarray(arr, dtype=np.float64, order="C")
    if a.ndim != 1:
        raise ValueError(f"{name} must be 1-D, got shape {a.shape}")
    return a


def _int_vector(name, arr):
    a = np.asarray(arr, dtype=np.int64, order="C")
    if a.ndim != 1:
        raise ValueError(f"{name} must be 1-D, got shape {a.shape}")
    return a.tolist()


def _dense_to_sparse(mat) -> SparseMatrix:
    a = np.asarray(mat, dtype=np.float64, order="C")
    if a.ndim != 2:
        raise ValueError(f"dense matrix must be 2-D, got shape {a.shape}")
    rows, cols = a.shape
    row_indices, col_indices = np.nonzero(a)
    values = a[row_indices, col_indices]
    return SparseMatrix(
        _core.create_sparse_matrix(
            int(rows),
            int(cols),
            row_indices.astype(np.int64).tolist(),
            col_indices.astype(np.int64).tolist(),
            np.asarray(values, dtype=np.float64).tolist(),
        )
    )


def _coo_to_sparse(data) -> SparseMatrix:
    if not isinstance(data, (list, tuple)) or len(data) != 5:
        raise TypeError(
            "A must be a SparseMatrix, 2-D ndarray, or "
            "(rows, cols, row_indices, col_indices, values) tuple"
        )
    rows, cols, row_indices, col_indices, values = data
    return SparseMatrix(
        _core.create_sparse_matrix(
            int(rows),
            int(cols),
            _int_vector("row_indices", row_indices),
            _int_vector("col_indices", col_indices),
            _float_vector("values", values).tolist(),
        )
    )


def _as_sparse_matrix(A) -> SparseMatrix:
    if isinstance(A, SparseMatrix):
        return A
    if isinstance(A, _core.PySparseMatrix):
        return SparseMatrix(A)
    if isinstance(A, np.ndarray) or (hasattr(A, "__array__") and not isinstance(A, (list, tuple))):
        return _dense_to_sparse(A)
    return _coo_to_sparse(A)


def _bounds_vector(name, value, size):
    if np.isscalar(value):
        return np.full(size, float(value), dtype=np.float64)
    arr = _float_vector(name, value)
    if arr.shape != (size,):
        raise ValueError(f"{name} must be scalar or shape ({size},), got shape {arr.shape}")
    return arr.copy()


class ConstraintFunction:
    """Read-only handle for vector-valued hard constraint functions."""

    def __init__(self, handle):
        if not isinstance(handle, _core.ConstraintFunctions):
            raise TypeError(
                f"handle must be a _core.ConstraintFunctions, got {type(handle).__name__}"
            )
        object.__setattr__(self, "_handle", handle)

    def __setattr__(self, name, value):
        raise AttributeError(f"ConstraintFunction is immutable; cannot set {name!r}")

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
        sparse = _as_sparse_matrix(A)
        if offset is None:
            offset_arr = np.zeros(sparse.shape[0], dtype=np.float64)
        else:
            offset_arr = _float_vector("offset", offset)
            if offset_arr.shape != (sparse.shape[0],):
                raise ValueError(
                    f"offset must have shape ({sparse.shape[0]},), got shape {offset_arr.shape}"
                )
        super().__init__(_core._create_linear_constraint(sparse._core_obj, offset_arr.copy()))

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


class Bounded:
    """Bounds for ``lower <= functions(x) <= upper``."""

    def __init__(self, functions, *, lower, upper):
        if not isinstance(functions, ConstraintFunction):
            raise TypeError("functions must be a pypgo.constraints.ConstraintFunction")
        object.__setattr__(self, "functions", functions)
        object.__setattr__(self, "lower", _bounds_vector("lower", lower, functions.num_constraints))
        object.__setattr__(self, "upper", _bounds_vector("upper", upper, functions.num_constraints))

    def __setattr__(self, name, value):
        raise AttributeError(f"Bounded is immutable; cannot set {name!r}")

    def __repr__(self) -> str:
        return f"Bounded({self.functions.num_constraints} constraints)"
