"""Formulation descriptors — Python class hierarchy mirrors C++ Formulation classes.

Each instance holds a persistent C++ formulation object (_handle).  Methods
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


def _require_mesh(mesh):
    from pypgo.fem.mesh import SimulationMesh

    if not isinstance(mesh, SimulationMesh):
        raise TypeError(f"mesh must be a SimulationMesh, got {type(mesh).__name__}")


def _element_values(mesh, value, *, name: str) -> np.ndarray:
    """Normalize a scalar or 1-D array to one finite positive value per element."""
    array = np.asarray(value, dtype=np.float64)
    if array.ndim == 0:
        array = np.full(mesh.num_elements, float(array), dtype=np.float64)
    elif array.ndim != 1:
        raise ValueError(
            f"{name} must be a scalar or 1-D array, got shape {array.shape}"
        )
    if array.size != mesh.num_elements:
        raise ValueError(
            f"{name} must contain {mesh.num_elements} element values, "
            f"got {array.size}"
        )
    if not np.all(np.isfinite(array)) or not np.all(array > 0.0):
        raise ValueError(f"{name} must contain finite values > 0")
    return np.ascontiguousarray(array)


# ---------------------------------------------------------------------------
# Formulation hierarchy
# ---------------------------------------------------------------------------


class Formulation:
    """Abstract base — owns a C++ PyFormulation handle."""

    def __init__(self, core_obj) -> None:
        self._handle = core_obj

    @property
    def name(self) -> str:
        return self._handle.name

    def num_basis_functions_per_element(self) -> int:
        """Return the number of interpolation basis functions per element."""
        return self._handle.num_basis_functions_per_element()

    def __repr__(self) -> str:
        return f"{type(self).__name__}()"


class VolumetricFormulation(Formulation):
    """Volumetric formulation with dynamics operators."""

    def mass_matrix(self, mesh, density):
        """Consistent mass matrix from scalar or per-element density (kg/m^3)."""
        from pypgo.sparse import SparseMatrix

        _require_mesh(mesh)
        values = _element_values(mesh, density, name="density")
        return SparseMatrix(
            _core.compute_formulation_mass_matrix(
                mesh._handle, self._handle, values.tolist()
            )
        )

    def body_force(self, mesh, acceleration, density) -> np.ndarray:
        """Generalized body force for a constant 3-vector acceleration."""
        accel = np.asarray(acceleration, dtype=np.float64).reshape(-1)
        if accel.size != 3:
            raise ValueError(f"acceleration must be a 3-vector, got length {accel.size}")
        _require_mesh(mesh)
        values = _element_values(mesh, density, name="density")
        return np.asarray(
            _core.compute_formulation_body_force(
                mesh._handle, self._handle, accel.tolist(), values.tolist()
            ),
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
                volume._handle, self._handle,
                np.ascontiguousarray(points).reshape(-1).tolist()),
        )


class ShellFormulation(Formulation):
    """Shell formulation with lumped mass / body-force operators."""

    def mass_matrix(self, mesh, areal_density):
        """Lumped shell mass matrix from scalar or per-element kg/m^2 values."""
        from pypgo.sparse import SparseMatrix

        _require_mesh(mesh)
        values = _element_values(
            mesh, areal_density, name="areal_density"
        )
        return SparseMatrix(
            _core.compute_shell_formulation_mass_matrix(
                mesh._handle,
                self._handle,
                values.tolist(),
            )
        )

    def body_force(self, mesh, acceleration, areal_density) -> np.ndarray:
        """Lumped shell body force for a constant 3-vector acceleration."""
        accel = np.asarray(acceleration, dtype=np.float64).reshape(-1)
        if accel.size != 3:
            raise ValueError(f"acceleration must be a 3-vector, got length {accel.size}")
        _require_mesh(mesh)
        values = _element_values(
            mesh, areal_density, name="areal_density"
        )
        return np.asarray(
            _core.compute_shell_formulation_body_force(
                mesh._handle,
                self._handle,
                accel.tolist(),
                values.tolist(),
            ),
            dtype=np.float64,
        )


# ---------------------------------------------------------------------------
# Concrete formulations
# ---------------------------------------------------------------------------


class TetLinear(VolumetricFormulation):
    def __init__(self) -> None:
        super().__init__(_core.make_tet_linear())


class CubicLinear(VolumetricFormulation):
    def __init__(self) -> None:
        super().__init__(_core.make_cubic_linear())


class CubicTricubicHermite(VolumetricFormulation):
    """Regular-grid tricubic Hermite hex formulation (24 DOFs/vertex)."""

    def __init__(self) -> None:
        super().__init__(_core.make_cubic_tricubic_hermite())


class KoiterShell(ShellFormulation):
    def __init__(self) -> None:
        super().__init__(_core.make_koiter_shell())
