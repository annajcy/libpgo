"""pypgo.fem — FEM construction surface for deformation energy.

All FEM-specific construction types live here:
  - Formulations:   TetP1, LinearCubic, KoiterShell
  - Elastic laws:   StableNeo, StVK, StVKVolume, LinearElastic, MooneyRivlin, KoiterStVK
  - Plastic params: VolumetricPlasticity, ShellPlasticity
  - Options:        DeformationOptions

The deformation_energy() factory returns a pypgo.energy.DeformationEnergy.
Do NOT expose FEM construction helpers in pypgo.energy.
"""

from __future__ import annotations

from dataclasses import dataclass

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


@dataclass(frozen=True)
class StVK:
    """Saint Venant-Kirchhoff material.  Requires ENu payload."""
    _kind: str = "stvk"

    def _to_string(self) -> str:
        return self._kind


@dataclass(frozen=True)
class StVKVolume:
    """Volumetric StVK material.  Requires ENu payload."""
    _kind: str = "stvk_vol"

    def _to_string(self) -> str:
        return self._kind


@dataclass(frozen=True)
class LinearElastic:
    """Linear elastic material.  Requires ENu payload."""
    _kind: str = "linear"

    def _to_string(self) -> str:
        return self._kind


@dataclass(frozen=True)
class MooneyRivlin:
    """Mooney-Rivlin hyperelastic material.  Requires Mooney-Rivlin payload.

    Only valid when the volume mesh carries MooneyRivlin material data.
    """
    _kind: str = "mooney_rivlin"

    def _to_string(self) -> str:
        return self._kind


@dataclass(frozen=True)
class KoiterStVK:
    """StVK material for Koiter shell.  Requires shell ENuh payload."""
    _kind: str = "koiter_stvk"

    def _to_string(self) -> str:
        return self._kind


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
    sim_mesh,               # SimulationMesh
    formulation=None,       # TetP1 | LinearCubic | KoiterShell | None
    elastic=None,           # StableNeo | StVK | StVKVolume | LinearElastic | MooneyRivlin | KoiterStVK
    plastic=None,           # VolumetricPlasticity | ShellPlasticity
    options=None,           # DeformationOptions | None
    plastic_params=None,    # ndarray | None
) -> DeformationEnergy:
    """Create a deformation energy from a simulation mesh.

    Parameters
    ----------
    sim_mesh : SimulationMesh
        Solver-ready simulation mesh.
    formulation : TetP1, LinearCubic, or KoiterShell
        Element formulation.  If None, defaults to TetP1 for tet meshes.
        Cubic and shell meshes REQUIRE an explicit formulation.
    elastic : StableNeo, StVK, StVKVolume, LinearElastic, MooneyRivlin, or KoiterStVK
        Elastic material law.  Required.
    plastic : VolumetricPlasticity or ShellPlasticity
        Plastic parametrization.  Required.
    options : DeformationOptions, optional
        Additional options (SPD enforcement, max-step limiting).
    plastic_params : ndarray, optional
        Initial per-element plastic parameters.  Shape must be
        ``(num_elements, plastic.dofs)`` or flat equivalent.

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

    # Resolve elastic
    if elastic is None:
        raise ValueError("elastic material is required")
    if not hasattr(elastic, "_to_string"):
        raise TypeError(
            f"elastic must be a pypgo.fem elastic wrapper, "
            f"got {type(elastic).__name__}"
        )

    # Resolve plastic
    if plastic is None:
        raise ValueError("plastic parametrization is required")
    if not hasattr(plastic, "_to_string"):
        raise TypeError(
            f"plastic must be a pypgo.fem plastic wrapper, "
            f"got {type(plastic).__name__}"
        )

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
        elastic._to_string(),
        plastic._to_string(),
        bool(options.enforce_spd),
        bool(options.enable_material_max_step),
    )

    energy = DeformationEnergy(core)
    if plastic_params is not None:
        energy.set_plastic_params(plastic_params)
    return energy
