"""pypgo.fem — FEM construction surface for deformation energy.

All FEM-specific construction types live here:
  - Formulations:   TetP1, LinearCubic, KoiterShell
  - Elastic laws:   StableNeo, StVK, StVKVolume, LinearElastic, MooneyRivlin, KoiterStVK
  - Parameter fields: VolumetricPlasticity, ShellPlasticity
  - Options:        DeformationOptions

The deformation_energy() factory returns a pypgo.energy.DeformationEnergy.
Do NOT expose FEM construction helpers in pypgo.energy.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import pypgo._core as _core
from pypgo.energy import DeformationEnergy

# ---------------------------------------------------------------------------
# Formulation wrappers
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TetP1:
    """Standard P1 (linear) tetrahedral formulation. 4 nodes, 12 DOFs.

    This is the default formulation for tet meshes.
    """
    name: str = "tet_p1"

    def _to_string(self) -> str:
        return self.name


@dataclass(frozen=True)
class LinearCubic:
    """Trilinear hexahedral formulation.  8 nodes, 24 DOFs.

    Maps to C++ LinearCubicFormulation (internal name: ``hex_trilinear``).
    Cubic meshes MUST pass this explicitly; there is no auto-dispatch for
    cubic topologies.
    """
    name: str = "hex_trilinear"

    def _to_string(self) -> str:
        return self.name


@dataclass(frozen=True)
class KoiterShell:
    """Koiter thin-shell formulation.  6 nodes (triangle + neighbours), 18 DOFs.

    Only valid with shell meshes created via ``SimulationMesh.create_shell()``.
    """
    name: str = "shell_koiter"

    def _to_string(self) -> str:
        return self.name


# ---------------------------------------------------------------------------
# Elastic law wrappers
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class StableNeo:
    """Stable Neo-Hookean material.  Requires ENu payload."""
    _kind: str = "stable_neo"

    def _to_string(self) -> str:
        return self._kind

    def default_field(self, sim_mesh) -> "ParameterField":
        return _make_elastic_field(self, sim_mesh)


@dataclass(frozen=True)
class StVK:
    """Saint Venant-Kirchhoff material.  Requires ENu payload."""
    _kind: str = "stvk"

    def _to_string(self) -> str:
        return self._kind

    def default_field(self, sim_mesh) -> "ParameterField":
        return _make_elastic_field(self, sim_mesh)


@dataclass(frozen=True)
class StVKVolume:
    """Volumetric StVK material.  Requires ENu payload."""
    _kind: str = "stvk_vol"

    def _to_string(self) -> str:
        return self._kind

    def default_field(self, sim_mesh) -> "ParameterField":
        return _make_elastic_field(self, sim_mesh)


@dataclass(frozen=True)
class LinearElastic:
    """Linear elastic material.  Requires ENu payload."""
    _kind: str = "linear"

    def _to_string(self) -> str:
        return self._kind

    def default_field(self, sim_mesh) -> "ParameterField":
        return _make_elastic_field(self, sim_mesh)


@dataclass(frozen=True)
class MooneyRivlin:
    """Mooney-Rivlin hyperelastic material.  Requires Mooney-Rivlin payload.

    Only valid when the volume mesh carries MooneyRivlin material data.
    """
    _kind: str = "mooney_rivlin"

    def _to_string(self) -> str:
        return self._kind

    def default_field(self, sim_mesh) -> "ParameterField":
        return _make_elastic_field(self, sim_mesh)


@dataclass(frozen=True)
class KoiterStVK:
    """StVK material for Koiter shell.  Requires shell ENuh payload."""
    _kind: str = "koiter_stvk"

    def _to_string(self) -> str:
        return self._kind

    def default_field(self, sim_mesh) -> "ParameterField":
        return _make_elastic_field(self, sim_mesh)


# ---------------------------------------------------------------------------
# Plastic parametrization wrappers
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class VolumetricPlasticity:
    """Volumetric deformation-gradient plastic parametrization.

    Parameters
    ----------
    dofs : int
        Number of plastic DOFs: 6 (full symmetric Fp), 3 (diagonal), or 0 (none).
    """
    dofs: int = 6

    def __post_init__(self):
        if self.dofs not in (0, 3, 6):
            raise ValueError(
                f"VolumetricPlasticity dofs must be 0, 3, or 6, got {self.dofs}"
            )

    def _to_string(self) -> str:
        return f"volumetric_dof{self.dofs}"

    def default_field(self, sim_mesh) -> "ParameterField":
        return self.elementwise_field(sim_mesh, values=_default_plastic_values(sim_mesh, self))

    def elementwise_field(self, sim_mesh, values) -> "ParameterField":
        return _make_plastic_field(self, sim_mesh, values)


@dataclass(frozen=True)
class ShellPlasticity:
    """Shell fundamental-forms plastic parametrization.

    Parameters
    ----------
    dofs : int
        Number of plastic DOFs: 1 or 0.
    """
    dofs: int = 1

    def __post_init__(self):
        if self.dofs not in (0, 1):
            raise ValueError(
                f"ShellPlasticity dofs must be 0 or 1, got {self.dofs}"
            )

    def _to_string(self) -> str:
        return f"shell_ff_dof{self.dofs}"

    def default_field(self, sim_mesh) -> "ParameterField":
        return self.elementwise_field(sim_mesh, values=_default_plastic_values(sim_mesh, self))

    def elementwise_field(self, sim_mesh, values) -> "ParameterField":
        return _make_plastic_field(self, sim_mesh, values)


# ---------------------------------------------------------------------------
# Parameter fields
# ---------------------------------------------------------------------------


class ParameterField:
    """Fixed per-element parameter field backed by a C++ owning field.

    Users construct these through material wrappers, e.g.
    ``pf.StableNeo().default_field(sim_mesh)`` or
    ``pf.VolumetricPlasticity(dofs=6).elementwise_field(sim_mesh, values)``.
    """

    def __init__(self, core) -> None:
        if not isinstance(core, _core.PyParameterField):
            raise TypeError(
                f"core must be a PyParameterField, got {type(core).__name__}"
            )
        self._core = core

    @property
    def domain(self) -> str:
        return self._core.domain

    @property
    def model(self) -> str:
        return self._core.model

    @property
    def num_elements(self) -> int:
        return self._core.num_elements

    @property
    def num_channels(self) -> int:
        return self._core.num_channels

    @property
    def values(self) -> np.ndarray:
        return np.asarray(self._core.values(), dtype=np.float64).copy()

    def set_values(self, values) -> None:
        arr = _field_values_array("values", values, self.num_elements, self.num_channels)
        self._core.set_values(arr.ravel())

    def _flat_values(self) -> np.ndarray:
        return np.ascontiguousarray(self.values.ravel(), dtype=np.float64)


def _make_elastic_field(model, sim_mesh) -> ParameterField:
    sim_mesh = _require_sim_mesh(sim_mesh)
    return ParameterField(
        _core._create_elastic_default_field(sim_mesh._core_obj, model._to_string())
    )


def _make_plastic_field(model, sim_mesh, values) -> ParameterField:
    sim_mesh = _require_sim_mesh(sim_mesh)
    arr = _field_values_array("values", values, sim_mesh.num_elements, model.dofs)
    return ParameterField(
        _core._create_plastic_elementwise_field(
            sim_mesh._core_obj,
            model._to_string(),
            arr.ravel(),
        )
    )


def _default_plastic_values(sim_mesh, model) -> np.ndarray:
    sim_mesh = _require_sim_mesh(sim_mesh)
    return np.asarray(
        _core._create_plastic_default_field(
            sim_mesh._core_obj,
            model._to_string(),
        ).values(),
        dtype=np.float64,
    )


def _field_values_array(name, values, num_elements, num_channels):
    arr = np.asarray(values, dtype=np.float64, order="C")
    if arr.ndim == 1:
        expected = num_elements * num_channels
        if arr.size != expected:
            raise ValueError(f"{name} flat size must be {expected}, got {arr.size}")
        arr = arr.reshape((num_elements, num_channels))
    elif arr.ndim != 2:
        raise ValueError(f"{name} must be 1-D or 2-D, got shape {arr.shape}")
    if arr.shape != (num_elements, num_channels):
        raise ValueError(
            f"{name} shape must be {(num_elements, num_channels)}, got {arr.shape}"
        )
    return np.ascontiguousarray(arr, dtype=np.float64)


def _require_sim_mesh(sim_mesh):
    from pypgo.sim import SimulationMesh as _SimulationMesh

    if not isinstance(sim_mesh, _SimulationMesh):
        raise TypeError(
            f"sim_mesh must be a SimulationMesh, got {type(sim_mesh).__name__}"
        )
    return sim_mesh


# ---------------------------------------------------------------------------
# Deformation options
# ---------------------------------------------------------------------------


@dataclass
class DeformationOptions:
    """Options for deformation energy construction.

    Parameters
    ----------
    enforce_spd : bool
        Enforce symmetric-positive-definite Hessian projection (default True).
    enable_material_max_step : bool
        Enable material-aware max-step limiting (default True).
    """
    enforce_spd: bool = True
    enable_material_max_step: bool = True


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------


def deformation_energy(
    sim_mesh,                # SimulationMesh
    formulation=None,        # TetP1 | LinearCubic | KoiterShell | None
    *,
    elastic_field=None,      # ParameterField
    plastic_field=None,      # ParameterField
    options=None,            # DeformationOptions | None
) -> DeformationEnergy:
    """Create a deformation energy from explicit fixed parameter fields.

    Parameters
    ----------
    sim_mesh : SimulationMesh
        Solver-ready simulation mesh.
    formulation : TetP1, LinearCubic, or KoiterShell
        Element formulation.  If None, defaults to TetP1 for tet meshes.
        Cubic and shell meshes REQUIRE an explicit formulation.
    elastic_field : ParameterField
        Fixed elastic parameter field created by an elastic model wrapper.
    plastic_field : ParameterField
        Fixed plastic parameter field created by a plastic model wrapper.
    options : DeformationOptions, optional
        Additional options (SPD enforcement, max-step limiting).

    Returns
    -------
    DeformationEnergy
        An energy object compatible with ``pypgo.energy.EnergySet``.
    """
    from pypgo.sim import SimulationMesh as _SimulationMesh
    if not isinstance(sim_mesh, _SimulationMesh):
        raise TypeError(
            f"sim_mesh must be a SimulationMesh, got {type(sim_mesh).__name__}"
        )

    # Resolve formulation
    if formulation is None:
        if sim_mesh.mesh_type == "tet":
            formulation = TetP1()
        else:
            raise ValueError(
                f"mesh_type '{sim_mesh.mesh_type}' requires an explicit formulation. "
                f"Pass TetP1(), LinearCubic(), or KoiterShell()."
            )
    if not hasattr(formulation, "_to_string"):
        raise TypeError(
            f"formulation must be TetP1(), LinearCubic(), or KoiterShell(), "
            f"got {type(formulation).__name__}"
        )
    # Coarse type check: cubic topo requires LinearCubic
    if sim_mesh.mesh_type == "cubic" and not isinstance(formulation, LinearCubic):
        raise ValueError(
            f"mesh_type 'cubic' requires LinearCubic() formulation, "
            f"got {type(formulation).__name__}"
        )
    if sim_mesh.mesh_type == "shell" and not isinstance(formulation, KoiterShell):
        raise ValueError(
            f"mesh_type 'shell' requires KoiterShell() formulation, "
            f"got {type(formulation).__name__}"
        )

    if elastic_field is None:
        raise ValueError("elastic_field is required")
    if plastic_field is None:
        raise ValueError("plastic_field is required")
    if not isinstance(elastic_field, ParameterField):
        raise TypeError(
            f"elastic_field must be a ParameterField, got {type(elastic_field).__name__}"
        )
    if not isinstance(plastic_field, ParameterField):
        raise TypeError(
            f"plastic_field must be a ParameterField, got {type(plastic_field).__name__}"
        )
    if elastic_field.domain != "elastic":
        raise ValueError("elastic_field must have domain 'elastic'")
    if plastic_field.domain != "plastic":
        raise ValueError("plastic_field must have domain 'plastic'")
    if elastic_field.num_elements != sim_mesh.num_elements:
        raise ValueError("elastic_field element count must match sim_mesh")
    if plastic_field.num_elements != sim_mesh.num_elements:
        raise ValueError("plastic_field element count must match sim_mesh")

    # Resolve options
    if options is None:
        options = DeformationOptions()
    if not isinstance(options, DeformationOptions):
        raise TypeError(
            f"options must be DeformationOptions, got {type(options).__name__}"
        )

    core = _core._create_deformation_energy(
        sim_mesh._core_obj,
        formulation._to_string(),
        elastic_field._core,
        plastic_field._core,
        bool(options.enforce_spd),
        bool(options.enable_material_max_step),
    )
    return DeformationEnergy(core)
