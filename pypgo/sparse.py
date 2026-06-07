"""Sparse matrix wrapper with NumPy COO export.

Also provides the matrix-coercion helpers :func:`as_sparse_matrix` (to a
:class:`SparseMatrix`) and :func:`as_coo` (to COO arrays for the C++
constructors), which accept a SparseMatrix, a dense 2-D array, a
``_core.PySparseMatrix``, or a COO 5-tuple.
"""

from __future__ import annotations

import numpy as np

import pypgo._core as _core


class SparseMatrix:
    """Owned sparse matrix exposed through COO arrays."""

    def __init__(self, core_obj):
        if not isinstance(core_obj, _core.PySparseMatrix):
            raise TypeError(f"core_obj must be PySparseMatrix, got {type(core_obj).__name__}")
        self._handle = core_obj

    @classmethod
    def from_coo(cls, shape, rows, cols, values) -> "SparseMatrix":
        """Create a sparse matrix from COO triplets.

        Parameters
        ----------
        shape : tuple[int, int]
            Matrix shape ``(num_rows, num_cols)``.
        rows, cols : array-like int
            COO row and column indices.
        values : array-like float
            COO values. Duplicate entries are summed by the C++ sparse builder.
        """
        if len(shape) != 2:
            raise ValueError("shape must be a pair (rows, cols)")
        num_rows, num_cols = int(shape[0]), int(shape[1])
        if num_rows < 0 or num_cols < 0:
            raise ValueError("shape entries must be non-negative")
        row_indices = np.asarray(rows, dtype=np.int64)
        col_indices = np.asarray(cols, dtype=np.int64)
        data = np.asarray(values, dtype=np.float64)
        if row_indices.ndim != 1 or col_indices.ndim != 1 or data.ndim != 1:
            raise ValueError("rows, cols, and values must be 1-D arrays")
        if not (row_indices.size == col_indices.size == data.size):
            raise ValueError("rows, cols, and values must have the same length")
        return cls(_core.create_sparse_matrix(num_rows, num_cols, row_indices, col_indices, data))

    @property
    def shape(self) -> tuple[int, int]:
        return (self._handle.rows(), self._handle.cols())

    @property
    def nnz(self) -> int:
        return self._handle.nnz()

    def to_coo(self):
        rows, cols, values = self._handle.to_coo()
        return (
            np.asarray(rows, dtype=np.int64),
            np.asarray(cols, dtype=np.int64),
            np.asarray(values, dtype=np.float64),
        )

    def to_dense(self) -> np.ndarray:
        """Return a dense (rows, cols) float64 ndarray copy."""
        flat = np.asarray(self._handle.to_dense(), dtype=np.float64)
        return flat.reshape(self._handle.rows(), self._handle.cols())

    def __repr__(self) -> str:
        return f"SparseMatrix(shape={self.shape}, nnz={self.nnz})"

    def __matmul__(self, other: np.ndarray) -> np.ndarray:
        other = np.asarray(other, dtype=np.float64)
        if other.ndim not in (1, 2):
            return NotImplemented
        if other.shape[0] != self.shape[1]:
            raise ValueError(
                f"shape mismatch for matmul: {self.shape} @ {other.shape}"
            )
        # Delegate the multiply to C++ (MKL-accelerated when available).
        if other.ndim == 1:
            return self._handle.matvec(np.ascontiguousarray(other))
        return self._handle.matmat(np.ascontiguousarray(other))


def as_sparse_matrix(A) -> "SparseMatrix":
    """Coerce a matrix-like argument to a :class:`SparseMatrix`.

    Accepts a ``SparseMatrix``, a ``_core.PySparseMatrix``, a dense 2-D
    array-like (zeros are dropped), or a
    ``(rows, cols, row_indices, col_indices, values)`` COO 5-tuple.
    """
    if isinstance(A, SparseMatrix):
        return A
    if isinstance(A, _core.PySparseMatrix):
        return SparseMatrix(A)
    if isinstance(A, np.ndarray) or (hasattr(A, "__array__") and not isinstance(A, (list, tuple))):
        a = np.asarray(A, dtype=np.float64, order="C")
        if a.ndim != 2:
            raise ValueError(f"dense matrix must be 2-D, got shape {a.shape}")
        row_indices, col_indices = np.nonzero(a)
        return SparseMatrix.from_coo(a.shape, row_indices, col_indices, a[row_indices, col_indices])
    if isinstance(A, (list, tuple)):
        if len(A) == 5:
            rows, cols, row_indices, col_indices, values = A
            return SparseMatrix.from_coo((int(rows), int(cols)), row_indices, col_indices, values)
        if len(A) == 4:
            raise TypeError(
                "Four-tuple input is ambiguous; pass "
                "(rows, cols, row_indices, col_indices, values)."
            )
    raise TypeError(
        "A must be a SparseMatrix, a 2-D ndarray, a PySparseMatrix, or a "
        "(rows, cols, row_indices, col_indices, values) tuple, "
        f"got {type(A).__name__}"
    )


def as_coo(A):
    """Coerce a matrix-like ``A`` to ``(rows, cols, row_list, col_list, values)``.

    Like :func:`as_sparse_matrix` followed by ``to_coo``, but also returns the
    matrix dimensions and gives the row/column indices as plain ``list`` objects
    — the shape expected by the C++ COO constructors. ``values`` is a float64
    ndarray. Accepts the same inputs as :func:`as_sparse_matrix`.
    """
    matrix = as_sparse_matrix(A)
    rows, cols = matrix.shape
    row_indices, col_indices, values = matrix.to_coo()
    return rows, cols, row_indices.tolist(), col_indices.tolist(), values
