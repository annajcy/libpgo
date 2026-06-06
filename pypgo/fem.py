"""pypgo.fem - FEM construction surface for deformation energy.

The deformation model API is state-based: material choices and parameter field
descriptors are bound to one SimulationMesh through DeformationModelState, then
deformation_energy() consumes that state plus a formulation.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import pypgo._core as _core
from pypgo.energy import DeformationEnergy, PlasticMaterialEnergy


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
class TricubicHermite:
    """Regular-grid tricubic Hermite hex formulation.

    Each vertex carries 24 DOFs (8 Hermite modes x 3 coords): the value mode plus the 7 first/second/
    third derivative modes. ``deformation_energy(...).num_dofs`` is therefore ``num_vertices * 24``
    (not ``* 3``). DOF ``vertex*24 + mode*3 + coord`` with mode order
    [value, d/dxi, d/deta, d/dzeta, d2/dxideta, d2/dxidzeta, d2/detadzeta, d3/dxidetadzeta].

    MVP scope: a uniform axis-aligned / affine-parallelepiped hex grid. The element is C1 within
    each cell and the synthesized rest field is exactly affine (rest deformation gradient = I).
    """

    name: str = "hex_tricubic_hermite"

    def _to_string(self) -> str:
        return self.name


@dataclass(frozen=True)
class KoiterShell:
    name: str = "shell_koiter"

    def _to_string(self) -> str:
        return self.name


# ---------------------------------------------------------------------------
# Formulation-aware dynamic operators
# ---------------------------------------------------------------------------


def _require_volume_mesh(volume):
    from pypgo.mesh.volume import VolumeMesh

    if not isinstance(volume, VolumeMesh):
        raise TypeError(f"volume must be a pypgo.mesh.volume.VolumeMesh, got {type(volume).__name__}")


def _formulation_name(formulation) -> str:
    if formulation is None:
        raise ValueError(
            "formulation is required. Pass TetP1(), LinearCubic(), TricubicHermite(), or KoiterShell()."
        )
    if not hasattr(formulation, "_to_string"):
        raise TypeError(
            f"formulation must be TetP1(), LinearCubic(), TricubicHermite(), or KoiterShell(), "
            f"got {type(formulation).__name__}"
        )
    return str(formulation._to_string())


def formulation_mass_matrix(volume, formulation):
    """C++ formulation-aware mass matrix.

    ``TetP1`` and ``LinearCubic`` use the legacy volumetric mass matrix.
    ``TricubicHermite`` uses the C++ consistent ``num_vertices*24`` mass.
    """
    from pypgo.sparse import SparseMatrix

    _require_volume_mesh(volume)
    return SparseMatrix(
        _core.compute_formulation_mass_matrix(volume._core_obj, _formulation_name(formulation))
    )


def body_force(volume, formulation, acceleration) -> np.ndarray:
    """C++ generalized body force for a constant acceleration field."""

    accel = np.asarray(acceleration, dtype=np.float64).reshape(-1)
    if accel.size != 3:
        raise ValueError(f"acceleration must be a 3-vector, got length {accel.size}")

    _require_volume_mesh(volume)
    return np.asarray(
        _core.compute_formulation_body_force(
            volume._core_obj,
            _formulation_name(formulation),
            accel.tolist(),
        ),
        dtype=np.float64,
    )


def surface_embedding_matrix(volume, surface_vertices, formulation):
    """C++ map from formulation simulation DOFs to surface displacement DOFs."""

    from pypgo.sparse import SparseMatrix

    points = np.asarray(surface_vertices, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"surface_vertices must have shape (n, 3), got {points.shape}")

    _require_volume_mesh(volume)
    return SparseMatrix(
        _core.compute_formulation_surface_embedding_matrix(
            volume._core_obj,
            _formulation_name(formulation),
            np.ascontiguousarray(points).reshape(-1).tolist(),
        )
    )


def hermite_vertex_dofs(vertex_ids, *, policy: str = "value") -> np.ndarray:
    """Return Hermite DOF indices for vertex-based boundary conditions."""

    vertices = np.asarray(vertex_ids, dtype=np.int64).reshape(-1)
    return np.asarray(
        _core.hermite_vertex_dofs([int(v) for v in vertices], str(policy)),
        dtype=np.int64,
    )


def hermite_face_dofs(volume, *, axis: str, side: str, policy: str = "all") -> np.ndarray:
    """Return Hermite DOFs on an axis-aligned min/max volume face."""

    _require_volume_mesh(volume)
    if axis not in {"x", "y", "z"}:
        raise ValueError("axis must be 'x', 'y', or 'z'")
    if side not in {"min", "max"}:
        raise ValueError("side must be 'min' or 'max'")

    axis_id = {"x": 0, "y": 1, "z": 2}[axis]
    return np.asarray(
        _core.hermite_face_dofs(volume._core_obj, axis_id, side == "max", str(policy)),
        dtype=np.int64,
    )


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
    def num_value_rows(self) -> int:
        return self._core.num_value_rows

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


def _elastic_value_channels(sim_mesh, elastic):
    return _core._elastic_num_channels(sim_mesh._core_obj, elastic._to_string())


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
        _elastic_value_channels(sim_mesh, elastic),
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
            "formulation is required. Pass TetP1(), LinearCubic(), TricubicHermite(), or KoiterShell()."
        )
    if not hasattr(formulation, "_to_string"):
        raise TypeError(
            f"formulation must be TetP1(), LinearCubic(), TricubicHermite(), or KoiterShell(), "
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


def plastic_material_energy(
    state,
    deformation_energy,
    *,
    fixed_displacement,
) -> PlasticMaterialEnergy:
    """Create a material energy whose optimization variable is the plastic field."""
    if not isinstance(state, DeformationModelState):
        raise TypeError(
            f"state must be a DeformationModelState, got {type(state).__name__}"
        )
    if not isinstance(deformation_energy, DeformationEnergy):
        raise TypeError(
            f"deformation_energy must be a DeformationEnergy, got {type(deformation_energy).__name__}"
        )

    u = np.asarray(fixed_displacement, dtype=np.float64, order="C")
    if u.ndim != 1:
        raise ValueError(f"fixed_displacement must be 1-D, got shape {u.shape}")
    if u.size != deformation_energy.num_dofs:
        raise ValueError(
            f"fixed_displacement size must be {deformation_energy.num_dofs}, got {u.size}"
        )

    handle = _core._create_plastic_material_energy(
        state._core,
        deformation_energy._core,
        u,
    )
    return PlasticMaterialEnergy(
        handle,
        state=state,
        deformation_energy=deformation_energy,
        fixed_displacement=u,
    )
