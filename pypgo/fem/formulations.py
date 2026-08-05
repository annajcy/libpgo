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
        """Consistent mass matrix; density from a VolumeDensity (kg/m^3) field."""
        from pypgo.sparse import SparseMatrix
        from pypgo.fem.mass import VolumeDensity

        _require_mesh(mesh)
        if not isinstance(density, VolumeDensity):
            raise TypeError(
                f"volumetric mass_matrix expects a VolumeDensity (kg/m^3), got {type(density).__name__}")
        return SparseMatrix(
            _core.compute_formulation_mass_matrix(mesh._handle, self._handle, density._handle))

    def body_force(self, mesh, acceleration, density) -> np.ndarray:
        """Generalized body force for a constant 3-vector acceleration."""
        from pypgo.fem.mass import VolumeDensity

        accel = np.asarray(acceleration, dtype=np.float64).reshape(-1)
        if accel.size != 3:
            raise ValueError(f"acceleration must be a 3-vector, got length {accel.size}")
        _require_mesh(mesh)
        if not isinstance(density, VolumeDensity):
            raise TypeError(
                f"volumetric body_force expects a VolumeDensity (kg/m^3), got {type(density).__name__}")
        return np.asarray(
            _core.compute_formulation_body_force(
                mesh._handle, self._handle, accel.tolist(), density._handle),
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

    def _require_shell_areal_density(self, areal_density):
        from pypgo.fem.mass import ShellArealDensity

        if not isinstance(areal_density, ShellArealDensity):
            raise TypeError(
                "shell formulation expects a ShellArealDensity (kg/m^2), "
                f"got {type(areal_density).__name__}"
            )

    @staticmethod
    def _material_state_handle(material_state, *, required=False):
        from pypgo.fem.fields import MaterialState

        if material_state is None:
            if required:
                raise ValueError(
                    "material_state is required for a parameter-dependent Jacobian"
                )
            return None
        if not isinstance(material_state, MaterialState):
            raise TypeError(
                "material_state must be MaterialState, "
                f"got {type(material_state).__name__}"
            )
        return material_state._handle

    def mass_matrix(self, mesh, areal_density, *, material_state=None):
        """Lumped shell mass matrix."""
        from pypgo.sparse import SparseMatrix

        _require_mesh(mesh)
        self._require_shell_areal_density(areal_density)
        return SparseMatrix(
            _core.compute_shell_formulation_mass_matrix(
                mesh._handle,
                self._handle,
                areal_density._handle,
                self._material_state_handle(material_state),
            )
        )

    def body_force(
        self, mesh, acceleration, areal_density, *, material_state=None
    ) -> np.ndarray:
        """Lumped shell body force for a constant 3-vector acceleration."""
        accel = np.asarray(acceleration, dtype=np.float64).reshape(-1)
        if accel.size != 3:
            raise ValueError(f"acceleration must be a 3-vector, got length {accel.size}")
        _require_mesh(mesh)
        self._require_shell_areal_density(areal_density)
        return np.asarray(
            _core.compute_shell_formulation_body_force(
                mesh._handle,
                self._handle,
                accel.tolist(),
                areal_density._handle,
                self._material_state_handle(material_state),
            ),
            dtype=np.float64,
        )

    def body_force_parameter_jacobian(
        self, mesh, acceleration, areal_density, *, material_state
    ):
        """d(body force)/d(elastic parameters) for a parameter-dependent areal density."""
        from pypgo.sparse import SparseMatrix

        accel = np.asarray(acceleration, dtype=np.float64).reshape(-1)
        if accel.size != 3:
            raise ValueError(f"acceleration must be a 3-vector, got length {accel.size}")
        _require_mesh(mesh)
        self._require_shell_areal_density(areal_density)
        return SparseMatrix(
            _core.compute_shell_formulation_body_force_parameter_jacobian(
                mesh._handle,
                self._handle,
                accel.tolist(),
                areal_density._handle,
                self._material_state_handle(material_state, required=True),
            )
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
