"""Formulation descriptors — Python class hierarchy mirrors C++ Formulation classes.

Each instance holds a persistent C++ formulation object (_core_obj).  Methods
delegate to C++ virtual dispatch directly.
"""

from __future__ import annotations

import numpy as np

import pypgo._core as _core


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _require_volume_mesh(volume):
    from pypgo.mesh.volume import VolumeMesh

    if not isinstance(volume, VolumeMesh):
        raise TypeError(f"volume must be a VolumeMesh, got {type(volume).__name__}")


# ---------------------------------------------------------------------------
# Formulation hierarchy
# ---------------------------------------------------------------------------


class Formulation:
    """Abstract base — owns a C++ PyFormulation handle."""

    def __init__(self, core_obj) -> None:
        self._core_obj = core_obj

    @property
    def name(self) -> str:
        return self._core_obj.name

    def __repr__(self) -> str:
        return f"{type(self).__name__}()"


class VolumetricFormulation(Formulation):
    """Volumetric formulation with dynamics operators."""

    def mass_matrix(self, volume):
        """Consistent mass matrix."""
        from pypgo.sparse import SparseMatrix

        _require_volume_mesh(volume)
        return SparseMatrix(
            _core.compute_formulation_mass_matrix(volume._core_obj, self._core_obj))

    def body_force(self, volume, acceleration) -> np.ndarray:
        """Generalized body force for a constant 3-vector acceleration."""
        accel = np.asarray(acceleration, dtype=np.float64).reshape(-1)
        if accel.size != 3:
            raise ValueError(f"acceleration must be a 3-vector, got length {accel.size}")
        _require_volume_mesh(volume)
        return np.asarray(
            _core.compute_formulation_body_force(
                volume._core_obj, self._core_obj, accel.tolist()),
            dtype=np.float64,
        )

    def surface_embedding_matrix(self, volume, surface_vertices):
        """Map from simulation DOFs to surface-vertex displacement DOFs."""
        from pypgo.sparse import SparseMatrix

        points = np.asarray(surface_vertices, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(f"surface_vertices must have shape (n, 3), got {points.shape}")
        _require_volume_mesh(volume)
        return SparseMatrix(
            _core.compute_formulation_surface_embedding_matrix(
                volume._core_obj, self._core_obj,
                np.ascontiguousarray(points).reshape(-1).tolist()),
        )


class ShellFormulation(Formulation):
    """Shell formulation — no volumetric dynamics operators."""
    pass


# ---------------------------------------------------------------------------
# Concrete formulations
# ---------------------------------------------------------------------------


class TetP1(VolumetricFormulation):
    def __init__(self) -> None:
        super().__init__(_core.make_tet_p1())


class LinearCubic(VolumetricFormulation):
    def __init__(self) -> None:
        super().__init__(_core.make_linear_cubic())


class TricubicHermite(VolumetricFormulation):
    """Regular-grid tricubic Hermite hex formulation (24 DOFs/vertex)."""

    def __init__(self) -> None:
        super().__init__(_core.make_tricubic_hermite())


class KoiterShell(ShellFormulation):
    def __init__(self) -> None:
        super().__init__(_core.make_koiter_shell())
