"""PotentialEnergy — the shared read-only handle base for all pypgo energies."""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo.sparse import SparseMatrix


class PotentialEnergy:
    """Read-only handle for any C++ potential energy.

    This is not directly constructible from Python.  All energy types
    (LinearEnergy, QuadraticEnergy, VertexAttachment, EnergySet, plus
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
