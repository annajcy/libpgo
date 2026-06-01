"""pypgo.energy — General-purpose energy types for pypgo.

All Python energy classes hold a PyPotentialEnergy handle internally;
evaluation always dispatches through C++ evaluation.h helpers so
Python users never see hessianInPlace / hessianAlloc / isHessianTopologyFixed.
"""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo.sparse import SparseMatrix


def _float_vector(name, arr):
    """Validate and return a contiguous float64 (n,) copy."""
    a = np.asarray(arr, dtype=np.float64, order="C")
    if a.ndim != 1:
        raise ValueError(f"{name} must be 1-D, got shape {a.shape}")
    return a


def _int_vector(name, arr):
    """Validate and return a list of ints from an int64 1-D array."""
    a = np.asarray(arr, dtype=np.int64, order="C")
    if a.ndim != 1:
        raise ValueError(f"{name} must be 1-D, got shape {a.shape}")
    return a.tolist()


def _dense_to_coo(mat):
    """Convert a 2-D float64 ndarray to (rows, cols, row_indices, col_indices, values)."""
    a = np.asarray(mat, dtype=np.float64, order="C")
    if a.ndim != 2:
        raise ValueError(f"dense matrix must be 2-D, got shape {a.shape}")
    rows, cols = a.shape
    ri, ci = np.nonzero(a)
    vals = a[ri, ci]
    return int(rows), int(cols), ri.tolist(), ci.tolist(), np.asarray(vals, dtype=np.float64)


def _dispatch_matrix_input(A):
    """Accept a PySparseMatrix, a dense 2-D ndarray, or a COO 5-tuple.

    Returns (rows, cols, row_indices, col_indices, values) where:
      rows, cols: int
      row_indices, col_indices: list[int]
      values: np.ndarray (n,) float64 C-contiguous
    """
    # 1. PySparseMatrix
    if isinstance(A, _core.PySparseMatrix):
        rows, cols = A.rows(), A.cols()
        ri, ci, vals = A.to_coo()
        return (
            rows, cols,
            list(ri), list(ci),
            np.asarray(vals, dtype=np.float64),
        )

    # 2. Dense 2-D ndarray / array-like → auto COO
    if isinstance(A, np.ndarray) or (hasattr(A, "__array__") and not isinstance(A, (list, tuple))):
        return _dense_to_coo(A)

    # 3. COO 5-tuple: (rows, cols, row_indices, col_indices, values)
    if isinstance(A, (list, tuple)) and len(A) == 5:
        rows, cols, row_indices, col_indices, values = A
        return (
            int(rows), int(cols),
            _int_vector("row_indices", row_indices),
            _int_vector("col_indices", col_indices),
            _float_vector("values", values),
        )

    # 4. COO 4-tuple is ambiguous — require explicit 5-tuple
    if isinstance(A, (list, tuple)) and len(A) == 4:
        raise TypeError(
            "Four-tuple input expects (rows, cols, row_indices, col_indices, values); "
            "a separate values array is required for the COO path."
        )

    raise TypeError(
        f"A must be a 2-D ndarray, a PySparseMatrix, or a "
        f"(rows, cols, row_indices, col_indices, values) tuple, "
        f"got {type(A).__name__}"
    )


class LinearEnergy:
    """Linear potential energy: b^T x.

    Parameters
    ----------
    b : ndarray (n,) float64
        Linear coefficient vector.  Copied into C++ owned storage.
    """

    def __init__(self, b: np.ndarray) -> None:
        b_arr = _float_vector("b", b)
        self._handle = _core._create_linear_energy(b_arr)

    @property
    def num_dofs(self) -> int:
        return self._handle.num_dofs

    @property
    def dofs(self) -> np.ndarray:
        return self._handle.dofs()

    @property
    def state_kind(self) -> str:
        return self._handle.state_kind

    def zero_state(self) -> np.ndarray:
        return self._handle.zero_state()

    def value(self, x: np.ndarray) -> float:
        return self._handle.value(np.asarray(x, dtype=np.float64))

    def gradient(self, x: np.ndarray) -> np.ndarray:
        return self._handle.gradient(np.asarray(x, dtype=np.float64))

    def hessian(self, x: np.ndarray):
        return SparseMatrix(self._handle.hessian(np.asarray(x, dtype=np.float64)))

    def max_step(self, x: np.ndarray, dx: np.ndarray):
        return self._handle.max_step(
            np.asarray(x, dtype=np.float64),
            np.asarray(dx, dtype=np.float64),
        )

    def __repr__(self) -> str:
        return f"LinearEnergy({self.num_dofs} DOFs)"


class QuadraticEnergy:
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
        rows, cols, row_indices, col_indices, values = _dispatch_matrix_input(A)

        if b is not None:
            b_arr = _float_vector("b", b)
            self._handle = _core._create_quadratic_energy_from_coo_with_b(
                rows, cols, row_indices, col_indices, values, b_arr,
            )
        else:
            self._handle = _core._create_quadratic_energy_from_coo(
                rows, cols, row_indices, col_indices, values,
            )

    @property
    def num_dofs(self) -> int:
        return self._handle.num_dofs

    @property
    def dofs(self) -> np.ndarray:
        return self._handle.dofs()

    @property
    def state_kind(self) -> str:
        return self._handle.state_kind

    def zero_state(self) -> np.ndarray:
        return self._handle.zero_state()

    def value(self, x: np.ndarray) -> float:
        return self._handle.value(np.asarray(x, dtype=np.float64))

    def gradient(self, x: np.ndarray) -> np.ndarray:
        return self._handle.gradient(np.asarray(x, dtype=np.float64))

    def hessian(self, x: np.ndarray):
        return SparseMatrix(self._handle.hessian(np.asarray(x, dtype=np.float64)))

    def max_step(self, x: np.ndarray, dx: np.ndarray):
        return self._handle.max_step(
            np.asarray(x, dtype=np.float64),
            np.asarray(dx, dtype=np.float64),
        )

    def __repr__(self) -> str:
        return f"QuadraticEnergy({self.num_dofs} DOFs)"


# ---------------------------------------------------------------------------
# Utility: decompose sparse into COO arrays
# ---------------------------------------------------------------------------

def _sparse_to_coo(sparse_or_tuple):
    """Accept PySparseMatrix or a (rows, cols, row_indices, col_indices, values)
    5-tuple, returning (rows, cols, row_indices_list, col_indices_list, values_1d).
    """
    if isinstance(sparse_or_tuple, _core.PySparseMatrix):
        rows, cols = sparse_or_tuple.rows(), sparse_or_tuple.cols()
        ri, ci, vals = sparse_or_tuple.to_coo()
        return (
            rows, cols,
            list(ri), list(ci),
            np.asarray(vals, dtype=np.float64),
        )
    if isinstance(sparse_or_tuple, (list, tuple)) and len(sparse_or_tuple) == 5:
        rows, cols, ri, ci, vals = sparse_or_tuple
        return (
            int(rows), int(cols),
            _int_vector("row_indices", ri),
            _int_vector("col_indices", ci),
            _float_vector("values", vals),
        )
    raise TypeError(
        f"Expected a PySparseMatrix or a 5-tuple (rows, cols, row_indices, "
        f"col_indices, values), got {type(sparse_or_tuple).__name__}"
    )


# ---------------------------------------------------------------------------
# VertexAttachment (E9b)
# ---------------------------------------------------------------------------

class VertexAttachment:
    """Soft pin constraint on selected vertices: coef * ||u_i - target_i||^2.

    The Hessian sparsity template is taken from the sim mesh (Koff).
    A fallback sparse coo input is also accepted.

    Parameters
    ----------
    sim_mesh : SimulationMesh, optional
        Provides numDofs and Koff sparsity template.
    koff : PySparseMatrix or 5-tuple, optional
        Fallback Hessian sparsity template if sim_mesh is not given.
    vertex_indices : ndarray (m,) int64
        Vertex indices to constrain.
    target_positions : ndarray (m*3,) float64
        Flat target positions (x0, y0, z0, x1, y1, z1, ...).
    coeff : float
        Penalty stiffness coefficient.
    is_displacement : bool
        True if the state is displacement (default True).
    """

    def __init__(self, *,
                 sim_mesh=None,
                 koff=None,
                 vertex_indices,
                 target_positions,
                 coeff=1e6,
                 is_displacement=True):
        vtx = np.asarray(vertex_indices, dtype=np.int64, order="C")
        if vtx.ndim != 1:
            raise ValueError("vertex_indices must be 1-D")
        tgt = _float_vector("target_positions", target_positions)
        if len(tgt) != len(vtx) * 3:
            raise ValueError(
                f"target_positions length ({len(tgt)}) must be 3 * "
                f"len(vertex_indices) ({len(vtx) * 3})"
            )

        if sim_mesh is not None:
            # sim_mesh provides DOF count; Koff is built from its DOF structure.
            # For now, use a simple diagonal Koff template of size nDofs.
            nDofs = sim_mesh.num_vertices * 3
            # Build Koff as diagonal identity-like sparsity
            rows = nDofs
            cols = nDofs
            kri = list(range(nDofs))
            kci = list(range(nDofs))
            kvals = [1.0] * nDofs
            rest_positions = np.zeros(nDofs, dtype=np.float64)
        elif koff is not None:
            rows, cols, kri, kci, kvals = _sparse_to_coo(koff)
            nDofs = rows
            rest_positions = np.zeros(nDofs, dtype=np.float64)
        else:
            raise ValueError("Either sim_mesh or koff must be provided")

        self._handle = _core._create_vertex_attachment(
            nDofs, rows, cols, kri, kci, list(kvals),
            rest_positions,
            vtx,
            tgt,
            float(coeff),
            bool(is_displacement),
        )

    @property
    def num_dofs(self) -> int:
        return self._handle.num_dofs

    @property
    def dofs(self) -> np.ndarray:
        return self._handle.dofs()

    @property
    def state_kind(self) -> str:
        return self._handle.state_kind

    def zero_state(self) -> np.ndarray:
        return self._handle.zero_state()

    def value(self, x: np.ndarray) -> float:
        return self._handle.value(np.asarray(x, dtype=np.float64))

    def gradient(self, x: np.ndarray) -> np.ndarray:
        return self._handle.gradient(np.asarray(x, dtype=np.float64))

    def hessian(self, x: np.ndarray):
        return SparseMatrix(self._handle.hessian(np.asarray(x, dtype=np.float64)))

    def max_step(self, x: np.ndarray, dx: np.ndarray):
        return self._handle.max_step(
            np.asarray(x, dtype=np.float64),
            np.asarray(dx, dtype=np.float64),
        )

    def set_targets(self, target_positions):
        """Update target positions."""
        tgt = _float_vector("target_positions", target_positions)
        # Re-create the energy — simple approach for now
        # TODO: add setTargetPositions binding for in-place update
        raise NotImplementedError("set_targets via recreation; bind setTargetPositions to enable")

    def __repr__(self) -> str:
        return f"VertexAttachment({self.num_dofs} DOFs)"


# ---------------------------------------------------------------------------
# EnergySet (E6)
# ---------------------------------------------------------------------------

class EnergySet:
    """Weighted sum of energy terms: total = sum_i weight_i * energy_i.

    Parameters
    ----------
    terms : list of (energy, weight) tuples
        Each energy must have a _handle attribute exposing a
        PyPotentialEnergy.  weight is a float scalar.
    """

    def __init__(self, terms):
        cpp_terms = []
        for i, (energy, weight) in enumerate(terms):
            if not hasattr(energy, "_handle"):
                raise TypeError(
                    f"term {i}: energy must have a _handle attribute"
                )
            cpp_terms.append((energy._handle, float(weight)))
        self._core_handle = _core._create_energy_set(cpp_terms)
        self._handle = self._core_handle.handle

    @property
    def num_dofs(self) -> int:
        return self._core_handle.num_dofs

    @property
    def num_terms(self) -> int:
        return self._core_handle.num_terms

    @property
    def dofs(self) -> np.ndarray:
        return self._handle.dofs()

    @property
    def state_kind(self) -> str:
        return self._handle.state_kind

    def zero_state(self) -> np.ndarray:
        return self._handle.zero_state()

    def value(self, x: np.ndarray) -> float:
        return self._handle.value(np.asarray(x, dtype=np.float64))

    def gradient(self, x: np.ndarray) -> np.ndarray:
        return self._handle.gradient(np.asarray(x, dtype=np.float64))

    def hessian(self, x: np.ndarray):
        return SparseMatrix(self._handle.hessian(np.asarray(x, dtype=np.float64)))

    def max_step(self, x: np.ndarray, dx: np.ndarray):
        return self._handle.max_step(
            np.asarray(x, dtype=np.float64),
            np.asarray(dx, dtype=np.float64),
        )

    def set_weight(self, i: int, w: float) -> None:
        self._core_handle.set_weight(i, float(w))

    def __repr__(self) -> str:
        return self._core_handle.__repr__()
