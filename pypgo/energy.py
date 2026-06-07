"""pypgo.energy — General-purpose energy types for pypgo.

Each Python facade stores its concrete C++ PyXXXX peer in ``_handle``.
Energy peers inherit ``_core.PyPotentialEnergy``, and C++ internals use
``potentialEnergyHandle()`` when a core ``PotentialEnergy`` pointer is required.
Evaluation dispatches through the virtual method so Python users never see
hessianInPlace / hessianAlloc / isHessianTopologyFixed.
"""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo._arrays import float_vector, int_vector
from pypgo.sparse import SparseMatrix, as_coo


# ---------------------------------------------------------------------------
# PotentialEnergy — shared base protocol
# ---------------------------------------------------------------------------

class PotentialEnergy:
    """Read-only handle for any C++ potential energy.

    This is not directly constructible from Python.  All energy types
    (LinearEnergy, QuadraticEnergy, VertexAttachment, EnergySet here, plus
    domain energies such as ``pypgo.fem.DeformationEnergy`` and the
    ``pypgo.contact`` energies) expose the same evaluation interface through
    their ``_handle`` attribute, and ``isinstance(x, PotentialEnergy)``
    checks work for all of them.
    """

    def __init__(self, handle):
        if not isinstance(handle, _core.PyPotentialEnergy):
            raise TypeError(
                f"handle must be a _core.PyPotentialEnergy, got {type(handle).__name__}"
            )
        object.__setattr__(self, "_handle", handle)

    def __setattr__(self, name, value):
        raise AttributeError(f"PotentialEnergy is immutable; cannot set {name!r}")

    def __delattr__(self, name):
        raise AttributeError(f"PotentialEnergy is immutable; cannot delete {name!r}")

    def __repr__(self) -> str:
        return self._handle.__repr__()

    # -- read-only properties --------------------------------------------------

    @property
    def num_dofs(self) -> int:
        return self._handle.num_dofs

    @property
    def dofs(self) -> np.ndarray:
        return self._handle.dofs()

    @property
    def state_kind(self) -> str:
        return self._handle.state_kind

    # -- evaluation ------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# LinearEnergy
# ---------------------------------------------------------------------------

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


class ConstraintPenalty(PotentialEnergy):
    """Soft zero-residual penalty ``0.5 * weight * ||C(x)||^2``."""

    def __init__(self, constraints, weight=1.0):
        from pypgo.constraints import ConstraintFunction

        if not isinstance(constraints, ConstraintFunction):
            raise TypeError("constraints must be a pypgo.constraints.ConstraintFunction")
        object.__setattr__(self, "_constraints", constraints)
        object.__setattr__(self, "weight", float(weight))
        super().__init__(_core._create_constraint_penalty(constraints._handle, self.weight))

    @property
    def constraints(self):
        return self._constraints

    def __repr__(self) -> str:
        return f"ConstraintPenalty({self.num_dofs} DOFs, weight={self.weight:g})"


class ConstraintViolationPenalty(PotentialEnergy):
    """Soft penalty for bound violations in ``lower <= C(x) <= upper``."""

    def __init__(self, bounded, weight=1.0):
        from pypgo.constraints import Bounded

        if not isinstance(bounded, Bounded):
            raise TypeError("bounded must be a pypgo.constraints.Bounded")
        object.__setattr__(self, "_bounded", bounded)
        object.__setattr__(self, "weight", float(weight))
        super().__init__(
            _core._create_constraint_violation_penalty(
                bounded.functions._handle,
                bounded.lower,
                bounded.upper,
                self.weight,
            )
        )

    @property
    def bounded(self):
        return self._bounded

    def __repr__(self) -> str:
        return f"ConstraintViolationPenalty({self.num_dofs} DOFs, weight={self.weight:g})"


# ---------------------------------------------------------------------------
# VertexAttachment (E9b)
# ---------------------------------------------------------------------------

class VertexAttachment(PotentialEnergy):
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
        vtx = int_vector("vertex_indices", vertex_indices)
        tgt = float_vector("target_positions", target_positions)
        if len(tgt) != len(vtx) * 3:
            raise ValueError(
                f"target_positions length ({len(tgt)}) must be 3 * "
                f"len(vertex_indices) ({len(vtx) * 3})"
            )

        if sim_mesh is not None:
            nDofs = sim_mesh.num_vertices * 3
            rows = nDofs
            cols = nDofs
            kri = list(range(nDofs))
            kci = list(range(nDofs))
            kvals = [1.0] * nDofs
            rest_positions = np.zeros(nDofs, dtype=np.float64)
        elif koff is not None:
            rows, cols, kri, kci, kvals = as_coo(koff)
            nDofs = rows
            rest_positions = np.zeros(nDofs, dtype=np.float64)
        else:
            raise ValueError("Either sim_mesh or koff must be provided")

        handle = _core._create_vertex_attachment(
            nDofs, rows, cols, kri, kci, list(kvals),
            rest_positions,
            vtx,
            tgt,
            float(coeff),
            bool(is_displacement),
        )
        super().__init__(handle)
        object.__setattr__(self, "_num_target_dofs", len(tgt))

    def set_targets(self, target_positions):
        """Update target positions."""
        tgt = float_vector("target_positions", target_positions)
        if len(tgt) != self._num_target_dofs:
            raise ValueError(
                f"target_positions length ({len(tgt)}) must match existing "
                f"target length ({self._num_target_dofs})"
            )
        self._handle.set_target_positions(tgt)

    def __repr__(self) -> str:
        return f"VertexAttachment({self.num_dofs} DOFs)"


# ---------------------------------------------------------------------------
# EnergySet (E6)
# ---------------------------------------------------------------------------

class EnergySet(PotentialEnergy):
    """Weighted sum of energy terms: total = sum_i weight_i * energy_i.

    Shares the full :class:`PotentialEnergy` evaluation interface
    (``value``/``gradient``/``hessian``/``max_step``/``dofs``/``state_kind``/
    ``zero_state``); only the term-management surface is added here.

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
        handle = _core._create_energy_set(cpp_terms)
        super().__init__(handle)

    @property
    def num_terms(self) -> int:
        return self._handle.num_terms

    def set_weight(self, i: int, w: float) -> None:
        self._handle.set_weight(i, float(w))

    def __repr__(self) -> str:
        return self._handle.__repr__()


__all__ = [
    "ConstraintPenalty",
    "ConstraintViolationPenalty",
    "EnergySet",
    "LinearEnergy",
    "PotentialEnergy",
    "QuadraticEnergy",
    "VertexAttachment",
]
