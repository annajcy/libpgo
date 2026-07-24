"""Material-parameter layouts, mappings, spaces, and committed state."""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass

import pypgo._core as _core


class ParameterDofLayout:
    """Topology mapping from global parameter DOFs to element-local DOFs."""

    def __init__(self, handle) -> None:
        if not isinstance(handle, _core.PyParameterDofLayout):
            raise TypeError(
                f"handle must be PyParameterDofLayout, got {type(handle).__name__}"
            )
        self._handle = handle


class ElementwiseDofLayout(ParameterDofLayout):
    """One independent parameter-DOF row per element."""

    def __init__(self) -> None:
        super().__init__(_core._make_elementwise_parameter_dof_layout())


class ConstantDofLayout(ParameterDofLayout):
    """One parameter-DOF row shared by every element."""

    def __init__(self) -> None:
        super().__init__(_core._make_constant_parameter_dof_layout())


class MaterialChannelMapping:
    """Differentiable mapping from local DOFs to material channels."""

    def __init__(self, handle) -> None:
        if not isinstance(handle, _core.PyMaterialChannelMapping):
            raise TypeError(
                f"handle must be PyMaterialChannelMapping, got {type(handle).__name__}"
            )
        self._handle = handle


class IdentityMaterialChannelMapping(MaterialChannelMapping):
    """Identity mapping; the local DOFs are the material channels."""

    def __init__(self) -> None:
        super().__init__(_core._make_identity_material_channel_mapping())


@dataclass(frozen=True)
class ParameterFieldDefinition:
    """Immutable pairing of a global DOF layout and channel mapping."""

    layout: ParameterDofLayout
    channel_mapping: MaterialChannelMapping

    def __post_init__(self) -> None:
        if not isinstance(self.layout, ParameterDofLayout):
            raise TypeError("layout must be a ParameterDofLayout")
        if not isinstance(self.channel_mapping, MaterialChannelMapping):
            raise TypeError("channel_mapping must be a MaterialChannelMapping")


class MaterialParameterRef:
    """Semantic reference to one material channel."""

    def __init__(self, handle) -> None:
        if not isinstance(handle, _core.PyMaterialParameterRef):
            raise TypeError(
                f"handle must be PyMaterialParameterRef, got {type(handle).__name__}"
            )
        self._handle = handle

    @property
    def name(self) -> str:
        return self._handle.name

    @property
    def channel(self) -> int:
        return self._handle.channel


class MaterialParameterBlock:
    """Immutable schema for the elastic or plastic parameter block."""

    def __init__(self, handle) -> None:
        if not isinstance(handle, _core.PyMaterialParameterBlock):
            raise TypeError(
                f"handle must be PyMaterialParameterBlock, got {type(handle).__name__}"
            )
        self._handle = handle

    @property
    def channel_names(self) -> tuple[str, ...]:
        return tuple(self._handle.channel_names)

    @property
    def num_channels(self) -> int:
        return self._handle.num_channels

    @property
    def num_local_dofs(self) -> int:
        return self._handle.num_local_dofs

    @property
    def num_global_dofs(self) -> int:
        return self._handle.num_global_dofs

    @property
    def num_value_rows(self) -> int:
        return self._handle.num_value_rows

    def parameter(self, name: str) -> MaterialParameterRef:
        return MaterialParameterRef(self._handle.parameter(str(name)))


class MaterialParameterSpace:
    """Shared immutable elastic/plastic material-parameter schema."""

    def __init__(self, sim_mesh_or_handle, *, elastic=None, plastic=None,
                 elastic_field=None, plastic_field=None) -> None:
        if isinstance(sim_mesh_or_handle, _core.PyMaterialParameterSpace):
            self._handle = sim_mesh_or_handle
            return
        if elastic is None or plastic is None or elastic_field is None or plastic_field is None:
            raise TypeError("MaterialParameterSpace requires sim_mesh, elastic/plastic configs, and field definitions")
        from pypgo.fem.mesh import SimulationMesh
        from pypgo.fem.elastic import ElasticModelConfig
        from pypgo.fem.plastic import PlasticModelConfig
        if not isinstance(sim_mesh_or_handle, SimulationMesh):
            raise TypeError("sim_mesh must be a SimulationMesh")
        if not isinstance(elastic, ElasticModelConfig) or not isinstance(plastic, PlasticModelConfig):
            raise TypeError("elastic and plastic must be model configs")
        if not isinstance(elastic_field, ParameterFieldDefinition) or not isinstance(plastic_field, ParameterFieldDefinition):
            raise TypeError("elastic_field and plastic_field must be ParameterFieldDefinition")
        self._handle = _core._create_material_parameter_space(
            sim_mesh_or_handle._handle, elastic._handle,
            elastic_field.layout._handle, elastic_field.channel_mapping._handle,
            plastic._handle, plastic_field.layout._handle,
            plastic_field.channel_mapping._handle)

    @classmethod
    def _from_handle(cls, handle):
        return cls(handle)

    @property
    def elastic(self) -> MaterialParameterBlock:
        return MaterialParameterBlock(self._handle.elastic)

    @property
    def plastic(self) -> MaterialParameterBlock:
        return MaterialParameterBlock(self._handle.plastic)


def _coerce_global_values(name: str, values, block: MaterialParameterBlock) -> np.ndarray:
    """Validate a global state vector against a block's row/column shape."""
    arr = np.asarray(values, dtype=np.float64, order="C")
    expected = (block.num_value_rows, block.num_local_dofs)
    if arr.ndim == 1:
        if arr.size != block.num_global_dofs:
            raise ValueError(
                f"{name} must contain {block.num_global_dofs} values, got {arr.size}"
            )
        return np.ascontiguousarray(arr, dtype=np.float64)
    if arr.ndim != 2:
        raise ValueError(f"{name} must be 1-D or 2-D, got shape {arr.shape}")
    if arr.shape != expected:
        raise ValueError(f"{name} shape must be {expected}, got {arr.shape}")
    return np.ascontiguousarray(arr.reshape(-1), dtype=np.float64)


class MaterialParameters:
    """Committed material parameter values and their immutable space."""

    def __init__(self, space_or_handle, *, elastic_values=None, plastic_values=None) -> None:
        if isinstance(space_or_handle, _core.PyMaterialParameters):
            self._handle = space_or_handle
            self._space_wrapper = None
            return
        if not isinstance(space_or_handle, MaterialParameterSpace):
            raise TypeError("space must be a MaterialParameterSpace")
        if elastic_values is None or plastic_values is None:
            raise TypeError("elastic_values and plastic_values are required")
        elastic_block = space_or_handle.elastic
        plastic_block = space_or_handle.plastic
        self._handle = _core._create_material_parameters(
            space_or_handle._handle,
            _coerce_global_values("elastic_values", elastic_values, elastic_block),
            _coerce_global_values("plastic_values", plastic_values, plastic_block))
        self._space_wrapper = space_or_handle

    @classmethod
    def _from_handle(cls, handle, space=None):
        result = cls(handle)
        result._space_wrapper = space
        return result

    @classmethod
    def elementwise_defaults(cls, sim_mesh, *, elastic, plastic):
        from pypgo.fem.mesh import SimulationMesh
        from pypgo.fem.elastic import ElasticModelConfig
        from pypgo.fem.plastic import PlasticModelConfig
        if not isinstance(sim_mesh, SimulationMesh):
            raise TypeError("sim_mesh must be a SimulationMesh")
        if not isinstance(elastic, ElasticModelConfig) or not isinstance(plastic, PlasticModelConfig):
            raise TypeError("elastic and plastic must be model configs")
        handle = _core._create_default_material_parameters(
            sim_mesh._handle, elastic._handle, plastic._handle)
        result = cls._from_handle(handle)
        result._config_refs = (elastic, plastic)
        return result

    @property
    def space(self) -> MaterialParameterSpace:
        if self._space_wrapper is None:
            self._space_wrapper = MaterialParameterSpace._from_handle(self._handle.space)
        return self._space_wrapper

    @property
    def elastic_values(self) -> np.ndarray:
        return np.asarray(self._handle.elastic_values, dtype=np.float64).copy()

    @property
    def plastic_values(self) -> np.ndarray:
        return np.asarray(self._handle.plastic_values, dtype=np.float64).copy()

    def set_elastic_values(self, values) -> None:
        self._set_values("elastic", values)

    def set_plastic_values(self, values) -> None:
        self._set_values("plastic", values)

    def same_space(self, other: "MaterialParameters") -> bool:
        if not isinstance(other, MaterialParameters):
            return False
        return bool(self._handle._same_space(other._handle))

    def _set_values(self, kind: str, values) -> None:
        block = getattr(self.space, kind)
        flat = _coerce_global_values(f"{kind}_values", values, block)
        if kind == "elastic":
            self._handle.set_elastic_values(flat)
        else:
            self._handle.set_plastic_values(flat)


__all__ = [
    "ParameterDofLayout",
    "ElementwiseDofLayout",
    "ConstantDofLayout",
    "MaterialChannelMapping",
    "IdentityMaterialChannelMapping",
    "ParameterFieldDefinition",
    "MaterialParameterRef",
    "MaterialParameterBlock",
    "MaterialParameterSpace",
    "MaterialParameters",
]
