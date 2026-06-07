"""Deformation model state and parameter field factories."""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo.fem.fields import ConstantField, ElementwiseField, ParameterField


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------


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
    from pypgo.fem.elastic import ElasticModel
    if isinstance(elastic, ElasticModel):
        return elastic._handle.num_channels(sim_mesh._handle)
    # backward compat: duck-typed object with _to_string() or name
    name = getattr(elastic, "name", None) or elastic._to_string()
    return _core._elastic_num_channels(sim_mesh._handle, name)


# ---------------------------------------------------------------------------
# DeformationModelState
# ---------------------------------------------------------------------------


class DeformationModelState:
    def __init__(self, core) -> None:
        if not isinstance(core, _core.PyDeformationModelState):
            raise TypeError(
                f"core must be a PyDeformationModelState, got {type(core).__name__}"
            )
        self._handle = core

    @property
    def elastic_model(self) -> str:
        return self._handle.elastic_model

    @property
    def plastic_model(self) -> str:
        return self._handle.plastic_model

    @property
    def num_elements(self) -> int:
        return self._handle.num_elements

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


# ---------------------------------------------------------------------------
# deformation_model_state factory
# ---------------------------------------------------------------------------


def deformation_model_state(
    sim_mesh,
    *,
    elastic,
    elastic_field,
    plastic,
    plastic_field,
) -> DeformationModelState:
    sim_mesh = _require_sim_mesh(sim_mesh)

    from pypgo.fem.elastic import ElasticModel

    if not isinstance(elastic, ElasticModel) and not hasattr(elastic, "name") and not hasattr(elastic, "_to_string"):
        raise TypeError(f"elastic must be an ElasticModel or have 'name'/'_to_string()', got {type(elastic).__name__}")
    from pypgo.fem.plastic import PlasticModel

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

    core = _core._create_deformation_model_state(
        sim_mesh._handle,
        elastic_name,
        elastic_values,
        plastic_name,
        plastic_values,
        _field_type_string("elastic_field", elastic_field),
        _field_type_string("plastic_field", plastic_field),
    )
    return DeformationModelState(core)
