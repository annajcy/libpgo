"""Deformation energy factory — direct construction from mesh, materials, and fields."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import pypgo._core as _core
from pypgo._utils import float_vector
from pypgo.energy import PotentialEnergy
from pypgo.fem.fields import ConstantField, ElementwiseField, ParameterField
from pypgo.sparse import SparseMatrix


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
    from pypgo.fem.mesh import SimulationMesh as _SimulationMesh

    if not isinstance(sim_mesh, _SimulationMesh):
        raise TypeError(
            f"sim_mesh must be a SimulationMesh, got {type(sim_mesh).__name__}"
        )
    return sim_mesh


def _elastic_value_channels(sim_mesh, elastic):
    from pypgo.fem.elastic import ElasticModel

    if isinstance(elastic, ElasticModel):
        return elastic._handle.num_channels(sim_mesh._handle)
    name = getattr(elastic, "name", None) or elastic._to_string()
    return _core._elastic_num_channels(sim_mesh._handle, name)


# ---------------------------------------------------------------------------
# DeformationEnergy — FEM deformation energy
# ---------------------------------------------------------------------------


class DeformationEnergy(PotentialEnergy):
    """Deformation energy for FEM simulations (tet, cubic, shell).

    Created by ``pypgo.fem.deformation_energy()``, not directly by users.
    This is a **displacement**-kind energy: ``state_kind == "displacement"``.

    Parameters
    ----------
    core : PyDeformationEnergy
        C++ deformation energy wrapper (from ``_core._create_deformation_energy``).

    Properties
    ----------
    rest_position : ndarray (num_vertices, 3) float64
        Rest (undeformed) positions.
    """

    def __init__(self, core):
        if not isinstance(core, _core.PyDeformationEnergy):
            raise TypeError(
                f"core must be a PyDeformationEnergy, got {type(core).__name__}"
            )
        object.__setattr__(self, "_handle", core)
        super().__init__(core)

    @property
    def rest_position(self) -> np.ndarray:
        return np.asarray(self._handle.rest_position(), dtype=np.float64)

    @property
    def num_vertices(self) -> int:
        return self._handle.num_vertices

    @property
    def num_elastic_params(self) -> int:
        """Per-element elastic parameter count."""
        return self._handle.num_elastic_params

    @property
    def num_plastic_params(self) -> int:
        """Per-element plastic parameter count."""
        return self._handle.num_plastic_params

    @property
    def num_elastic_dofs(self) -> int:
        """Total elastic parameter DOFs across all elements."""
        return self._handle.num_elastic_dofs

    @property
    def num_plastic_dofs(self) -> int:
        """Total plastic parameter DOFs across all elements."""
        return self._handle.num_plastic_dofs

    @property
    def elastic_model(self) -> str:
        return self._handle.elastic_model

    @property
    def plastic_model(self) -> str:
        return self._handle.plastic_model

    @property
    def elastic_field(self) -> ParameterField:
        return ParameterField(self._handle.elastic_field)

    @property
    def plastic_field(self) -> ParameterField:
        return ParameterField(self._handle.plastic_field)

    def set_elastic_values(self, values) -> None:
        field = self.elastic_field
        arr = _field_values_array("values", values, field.num_elements, field.num_channels)
        self._handle.set_elastic_values(arr.ravel())

    def set_plastic_values(self, values) -> None:
        field = self.plastic_field
        arr = _field_values_array("values", values, field.num_elements, field.num_channels)
        self._handle.set_plastic_values(arr.ravel())

    def plastic_gradient(self, displacement: np.ndarray) -> np.ndarray:
        u = float_vector("displacement", displacement)
        return np.asarray(self._handle.plastic_gradient(u), dtype=np.float64)

    def plastic_hessian(self, displacement: np.ndarray):
        u = float_vector("displacement", displacement)
        return SparseMatrix(self._handle.plastic_hessian(u))

    def plastic_jacobian(self, displacement: np.ndarray):
        u = float_vector("displacement", displacement)
        return SparseMatrix(self._handle.plastic_jacobian(u))

    def __repr__(self) -> str:
        return f"DeformationEnergy({self.num_dofs} DOFs, state_kind='{self.state_kind}')"


class PlasticMaterialEnergy(PotentialEnergy):
    """Material energy with the plastic field as the optimization variable.

    Created by ``pypgo.fem.plastic_material_energy()``. The displacement is fixed;
    the input state vector is the plastic field's global DOF vector.
    """

    def __init__(self, handle, *, deformation_energy, fixed_displacement):
        object.__setattr__(self, "_deformation_energy", deformation_energy)
        object.__setattr__(
            self,
            "_fixed_displacement",
            np.asarray(fixed_displacement, dtype=np.float64).copy(),
        )
        super().__init__(handle)

    @property
    def deformation_energy(self):
        return self._deformation_energy

    @property
    def fixed_displacement(self) -> np.ndarray:
        return self._fixed_displacement.copy()

    def __repr__(self) -> str:
        return f"PlasticMaterialEnergy({self.num_dofs} DOFs, state_kind='{self.state_kind}')"


# ---------------------------------------------------------------------------
# DeformationOptions
# ---------------------------------------------------------------------------


@dataclass
class DeformationOptions:
    enforce_spd: bool = True
    enable_material_max_step: bool = True


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------


def _resolve_formulation(formulation):
    from pypgo.fem.formulations import Formulation

    if formulation is None:
        raise ValueError(
            "formulation is required. Pass TetLinear(), CubicLinear(), CubicTricubicHermite(), or KoiterShell()."
        )
    if not isinstance(formulation, Formulation):
        raise TypeError(
            f"formulation must be a Formulation, got {type(formulation).__name__}"
        )
    return formulation


# ---------------------------------------------------------------------------
# Energy factories
# ---------------------------------------------------------------------------


def deformation_energy(
    sim_mesh,
    *,
    elastic,
    elastic_field,
    plastic,
    plastic_field,
    formulation=None,
    options=None,
) -> DeformationEnergy:
    sim_mesh = _require_sim_mesh(sim_mesh)
    formulation = _resolve_formulation(formulation)

    from pypgo.fem.elastic import ElasticModel
    from pypgo.fem.plastic import PlasticModel

    if not isinstance(elastic, ElasticModel) and not hasattr(elastic, "name") and not hasattr(elastic, "_to_string"):
        raise TypeError(f"elastic must be an ElasticModel or have 'name'/'_to_string()', got {type(elastic).__name__}")
    if not isinstance(plastic, PlasticModel) and not hasattr(plastic, "name") and not hasattr(plastic, "_to_string"):
        raise TypeError(f"plastic must be a PlasticModel or have 'name'/'_to_string()', got {type(plastic).__name__}")

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

    elastic_name = elastic.name if isinstance(elastic, ElasticModel) else elastic._to_string()
    plastic_name = plastic.name if isinstance(plastic, PlasticModel) else plastic._to_string()

    if options is None:
        options = DeformationOptions()
    if not isinstance(options, DeformationOptions):
        raise TypeError(f"options must be DeformationOptions, got {type(options).__name__}")

    core = _core._create_deformation_energy(
        sim_mesh._handle,
        elastic_name,
        elastic_values,
        plastic_name,
        plastic_values,
        _field_type_string("elastic_field", elastic_field),
        _field_type_string("plastic_field", plastic_field),
        formulation.name,
        bool(options.enforce_spd),
        bool(options.enable_material_max_step),
    )
    return DeformationEnergy(core)


def plastic_material_energy(
    deformation_energy,
    *,
    fixed_displacement,
) -> PlasticMaterialEnergy:
    """Create a material energy whose optimization variable is the plastic field."""
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
        deformation_energy._handle,
        u,
    )
    return PlasticMaterialEnergy(
        handle,
        deformation_energy=deformation_energy,
        fixed_displacement=u,
    )
