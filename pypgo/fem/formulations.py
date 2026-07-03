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


def _require_sim_mesh(sim_mesh):
    from pypgo.fem.mesh import SimulationMesh

    if not isinstance(sim_mesh, SimulationMesh):
        raise TypeError(f"sim_mesh must be a SimulationMesh, got {type(sim_mesh).__name__}")


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

    def __repr__(self) -> str:
        return f"{type(self).__name__}()"


class VolumetricFormulation(Formulation):
    """Volumetric formulation with dynamics operators."""

    def mass_matrix(self, sim_mesh, mass_field):
        """Consistent mass matrix; density from a VolumeDensity (kg/m^3) field."""
        from pypgo.sparse import SparseMatrix
        from pypgo.fem.mass import VolumeMassField

        _require_sim_mesh(sim_mesh)
        if not isinstance(mass_field, VolumeMassField):
            raise TypeError(
                f"volumetric mass_matrix expects a VolumeMassField (kg/m^3), got {type(mass_field).__name__}")
        return SparseMatrix(
            _core.compute_formulation_mass_matrix(sim_mesh._handle, self._handle, mass_field._handle))

    def body_force(self, sim_mesh, acceleration, mass_field) -> np.ndarray:
        """Generalized body force for a constant 3-vector acceleration."""
        from pypgo.fem.mass import VolumeMassField

        accel = np.asarray(acceleration, dtype=np.float64).reshape(-1)
        if accel.size != 3:
            raise ValueError(f"acceleration must be a 3-vector, got length {accel.size}")
        _require_sim_mesh(sim_mesh)
        if not isinstance(mass_field, VolumeMassField):
            raise TypeError(
                f"volumetric body_force expects a VolumeMassField (kg/m^3), got {type(mass_field).__name__}")
        return np.asarray(
            _core.compute_formulation_body_force(
                sim_mesh._handle, self._handle, accel.tolist(), mass_field._handle),
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

    def _require_shell_mass_field(self, mass_field):
        from pypgo.fem.mass import ShellMassField

        if not isinstance(mass_field, ShellMassField):
            raise TypeError(
                f"shell formulation expects a ShellMassField (kg/m^2), got {type(mass_field).__name__}")

    def mass_matrix(self, sim_mesh, mass_field):
        """Lumped shell mass matrix."""
        from pypgo.sparse import SparseMatrix

        _require_sim_mesh(sim_mesh)
        self._require_shell_mass_field(mass_field)
        return SparseMatrix(
            _core.compute_shell_formulation_mass_matrix(sim_mesh._handle, self._handle, mass_field._handle))

    def body_force(self, sim_mesh, acceleration, mass_field) -> np.ndarray:
        """Lumped shell body force for a constant 3-vector acceleration."""
        accel = np.asarray(acceleration, dtype=np.float64).reshape(-1)
        if accel.size != 3:
            raise ValueError(f"acceleration must be a 3-vector, got length {accel.size}")
        _require_sim_mesh(sim_mesh)
        self._require_shell_mass_field(mass_field)
        return np.asarray(
            _core.compute_shell_formulation_body_force(
                sim_mesh._handle, self._handle, accel.tolist(), mass_field._handle),
            dtype=np.float64,
        )

    def body_force_parameter_jacobian(self, sim_mesh, acceleration, mass_field):
        """d(body force)/d(elastic parameters) for a parameter-dependent mass field."""
        from pypgo.sparse import SparseMatrix

        accel = np.asarray(acceleration, dtype=np.float64).reshape(-1)
        if accel.size != 3:
            raise ValueError(f"acceleration must be a 3-vector, got length {accel.size}")
        _require_sim_mesh(sim_mesh)
        self._require_shell_mass_field(mass_field)
        return SparseMatrix(
            _core.compute_shell_formulation_body_force_parameter_jacobian(
                sim_mesh._handle, self._handle, accel.tolist(), mass_field._handle))


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
