"""pypgo.fem - FEM construction surface for deformation energy.

The deformation model API is state-based: material choices and parameter field
descriptors are bound to one SimulationMesh through DeformationModelState, then
deformation_energy() consumes that state plus a formulation.
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
    name: str = "tet_p1"

    def _to_string(self) -> str:
        return self.name


@dataclass(frozen=True)
class LinearCubic:
    name: str = "hex_trilinear"

    def _to_string(self) -> str:
        return self.name


@dataclass(frozen=True)
class KoiterShell:
    name: str = "shell_koiter"

    def _to_string(self) -> str:
        return self.name


# ---------------------------------------------------------------------------
# Elastic law wrappers
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class StableNeo:
    _kind: str = "stable_neo"

    def _to_string(self) -> str:
        return self._kind


@dataclass(frozen=True)
class StVK:
    _kind: str = "stvk"

    def _to_string(self) -> str:
        return self._kind


@dataclass(frozen=True)
class StVKVolume:
    _kind: str = "stvk_vol"

    def _to_string(self) -> str:
        return self._kind


@dataclass(frozen=True)
class LinearElastic:
    _kind: str = "linear"

    def _to_string(self) -> str:
        return self._kind


@dataclass(frozen=True)
class MooneyRivlin:
    _kind: str = "mooney_rivlin"

    def _to_string(self) -> str:
        return self._kind


@dataclass(frozen=True)
class KoiterStVK:
    _kind: str = "koiter_stvk"

    def _to_string(self) -> str:
        return self._kind


# ---------------------------------------------------------------------------
# Plastic parametrization wrappers
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class VolumetricPlasticity:
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
    dofs: int = 1

    def __post_init__(self):
        if self.dofs not in (0, 1):
            raise ValueError(f"ShellPlasticity dofs must be 0 or 1, got {self.dofs}")

    def _to_string(self) -> str:
        return f"shell_ff_dof{self.dofs}"


# ---------------------------------------------------------------------------
# Parameter field descriptors and state views
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ElementwiseField:
    """Elementwise parameter field descriptor.

    ``values=None`` asks C++ to create model-appropriate default values from the
    mesh/material payload. Otherwise values may be flat or shaped as
    ``(num_elements, num_channels)``.
    """

    values: object = None


@dataclass(frozen=True)
class ConstantField:
    """Constant (mesh-wide shared) parameter field descriptor.

    A single set of ``num_channels`` parameters is shared by every element.
    ``values=None`` asks C++ to seed the shared values from the mesh/material
    payload. Otherwise values may be flat ``(num_channels,)`` or ``(1, num_channels)``.
    """

    values: object = None


class ParameterField:
    """View of a state-owned C++ parameter field."""

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


class DeformationModelState:
    def __init__(self, core) -> None:
        if not isinstance(core, _core.PyDeformationModelState):
            raise TypeError(
                f"core must be a PyDeformationModelState, got {type(core).__name__}"
            )
        self._core = core

    @property
    def elastic_model(self) -> str:
        return self._core.elastic_model

    @property
    def plastic_model(self) -> str:
        return self._core.plastic_model

    @property
    def num_elements(self) -> int:
        return self._core.num_elements

    @property
    def elastic_field(self) -> ParameterField:
        return ParameterField(self._core.elastic_field)

    @property
    def plastic_field(self) -> ParameterField:
        return ParameterField(self._core.plastic_field)

    def set_elastic_values(self, values) -> None:
        field = self.elastic_field
        arr = _field_values_array("values", values, field.num_elements, field.num_channels)
        self._core.set_elastic_values(arr.ravel())

    def set_plastic_values(self, values) -> None:
        field = self.plastic_field
        arr = _field_values_array("values", values, field.num_elements, field.num_channels)
        self._core.set_plastic_values(arr.ravel())


def _field_values_array(name, values, num_elements, num_channels=None):
    arr = np.asarray(values, dtype=np.float64, order="C")
    if arr.ndim == 1:
        if num_channels is not None:
            expected = num_elements * num_channels
            if arr.size != expected:
                raise ValueError(f"{name} flat size must be {expected}, got {arr.size}")
            arr = arr.reshape((num_elements, num_channels))
        return np.ascontiguousarray(arr, dtype=np.float64)
    if arr.ndim != 2:
        raise ValueError(f"{name} must be 1-D or 2-D, got shape {arr.shape}")
    if arr.shape[0] != num_elements:
        raise ValueError(f"{name} first dimension must be {num_elements}, got {arr.shape[0]}")
    if num_channels is not None and arr.shape[1] != num_channels:
        raise ValueError(
            f"{name} shape must be {(num_elements, num_channels)}, got {arr.shape}"
        )
    return np.ascontiguousarray(arr, dtype=np.float64)


def _field_type_string(name, field):
    if isinstance(field, ElementwiseField):
        return "elementwise"
    if isinstance(field, ConstantField):
        return "constant"
    raise TypeError(
        f"{name} must be ElementwiseField or ConstantField, got {type(field).__name__}"
    )


def _field_init_values(name, field, num_elements, num_channels=None):
    if isinstance(field, ConstantField):
        # A constant field stores a single shared set of num_channels parameters.
        if field.values is None:
            return None
        return _field_values_array(name, field.values, 1, num_channels).ravel()
    if isinstance(field, ElementwiseField):
        if field.values is None:
            return None
        return _field_values_array(name, field.values, num_elements, num_channels).ravel()
    raise TypeError(
        f"{name} must be ElementwiseField or ConstantField, got {type(field).__name__}"
    )


def _require_sim_mesh(sim_mesh):
    from pypgo.sim import SimulationMesh as _SimulationMesh

    if not isinstance(sim_mesh, _SimulationMesh):
        raise TypeError(
            f"sim_mesh must be a SimulationMesh, got {type(sim_mesh).__name__}"
        )
    return sim_mesh


def _elastic_value_channels(elastic):
    # Must mirror SimulationMeshMaterial::numElasticParameters (== the elastic
    # model's getNumParameters). The basic 3D materials expose no differentiable
    # elastic parameters, so their optimizable elastic field has 0 channels.
    if isinstance(elastic, KoiterStVK):
        return 5
    if isinstance(elastic, (StableNeo, StVK, LinearElastic, StVKVolume, MooneyRivlin)):
        return 0
    return None


def deformation_model_state(
    sim_mesh,
    *,
    elastic,
    elastic_field,
    plastic,
    plastic_field,
) -> DeformationModelState:
    sim_mesh = _require_sim_mesh(sim_mesh)
    if not hasattr(elastic, "_to_string"):
        raise TypeError(f"elastic must be an elastic material wrapper, got {type(elastic).__name__}")
    if not hasattr(plastic, "_to_string"):
        raise TypeError(f"plastic must be a plastic material wrapper, got {type(plastic).__name__}")

    elastic_values = _field_init_values(
        "elastic_field.values",
        elastic_field,
        sim_mesh.num_elements,
        _elastic_value_channels(elastic),
    )
    plastic_values = _field_init_values(
        "plastic_field.values",
        plastic_field,
        sim_mesh.num_elements,
        plastic.dofs,
    )

    core = _core._create_deformation_model_state(
        sim_mesh._core_obj,
        elastic._to_string(),
        elastic_values,
        plastic._to_string(),
        plastic_values,
        _field_type_string("elastic_field", elastic_field),
        _field_type_string("plastic_field", plastic_field),
    )
    return DeformationModelState(core)


# ---------------------------------------------------------------------------
# Deformation options and factory
# ---------------------------------------------------------------------------


@dataclass
class DeformationOptions:
    enforce_spd: bool = True
    enable_material_max_step: bool = True


def _resolve_formulation(state: DeformationModelState, formulation):
    if formulation is None:
        raise ValueError(
            "formulation is required. Pass TetP1(), LinearCubic(), or KoiterShell()."
        )
    if not hasattr(formulation, "_to_string"):
        raise TypeError(
            f"formulation must be TetP1(), LinearCubic(), or KoiterShell(), "
            f"got {type(formulation).__name__}"
        )
    return formulation


def deformation_energy(
    state,
    formulation=None,
    *,
    options=None,
) -> DeformationEnergy:
    if not isinstance(state, DeformationModelState):
        raise TypeError(
            f"state must be a DeformationModelState, got {type(state).__name__}"
        )
    formulation = _resolve_formulation(state, formulation)

    if options is None:
        options = DeformationOptions()
    if not isinstance(options, DeformationOptions):
        raise TypeError(f"options must be DeformationOptions, got {type(options).__name__}")

    core = _core._create_deformation_energy(
        state._core,
        formulation._to_string(),
        bool(options.enforce_spd),
        bool(options.enable_material_max_step),
    )
    return DeformationEnergy(core)
