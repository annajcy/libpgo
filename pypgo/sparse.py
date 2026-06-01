"""Sparse matrix wrapper with NumPy COO export."""

from __future__ import annotations

import numpy as np

import pypgo._core as _core


class SparseMatrix:
    """Owned sparse matrix exposed through COO arrays."""

    def __init__(self, core_obj):
        if not isinstance(core_obj, _core.PySparseMatrix):
            raise TypeError(f"core_obj must be PySparseMatrix, got {type(core_obj).__name__}")
        self._core_obj = core_obj

    @property
    def shape(self) -> tuple[int, int]:
        return (self._core_obj.rows(), self._core_obj.cols())

    @property
    def nnz(self) -> int:
        return self._core_obj.nnz()

    def to_coo(self):
        rows, cols, values = self._core_obj.to_coo()
        return (
            np.asarray(rows, dtype=np.int64),
            np.asarray(cols, dtype=np.int64),
            np.asarray(values, dtype=np.float64),
        )

    def to_dense(self) -> np.ndarray:
        """Return a dense (rows, cols) float64 ndarray copy."""
        flat = np.asarray(self._core_obj.to_dense(), dtype=np.float64)
        return flat.reshape(self._core_obj.rows(), self._core_obj.cols())

    def __matmul__(self, other: np.ndarray) -> np.ndarray:
        other = np.asarray(other, dtype=np.float64)
        if other.ndim not in (1, 2):
            return NotImplemented
        rows, cols, values = self.to_coo()
        if other.ndim == 1:
            result = np.zeros(self.shape[0], dtype=np.float64)
            np.add.at(result, rows, values * other[cols])
        else:
            result = np.zeros((self.shape[0], other.shape[1]), dtype=np.float64)
            np.add.at(result, rows, values[:, None] * other[cols])
        return result
