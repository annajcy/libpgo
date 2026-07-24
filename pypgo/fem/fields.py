"""Material-parameter layouts, mappings, spaces, and committed state."""

from __future__ import annotations

import numpy as np

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


class ParameterMapping:
    """Differentiable mapping from local DOFs to material channels."""

    def __init__(self, handle) -> None:
        if not isinstance(handle, _core.PyParameterFieldMapping):
            raise TypeError(
                f"handle must be PyParameterFieldMapping, got {type(handle).__name__}"
            )
        self._handle = handle


class IdentityParameterMapping(ParameterMapping):
    """Identity mapping; the local DOFs are the material channels."""

    def __init__(self) -> None:
        super().__init__(_core._make_identity_parameter_field_mapping())


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

    def __init__(self, handle) -> None:
        if not isinstance(handle, _core.PyMaterialParameterSpace):
            raise TypeError(
                f"handle must be PyMaterialParameterSpace, got {type(handle).__name__}"
            )
        self._handle = handle

    @property
    def elastic(self) -> MaterialParameterBlock:
        return MaterialParameterBlock(self._handle.elastic)

    @property
    def plastic(self) -> MaterialParameterBlock:
        return MaterialParameterBlock(self._handle.plastic)


class MaterialParameters:
    """Committed material parameter values and their immutable space."""

    def __init__(self, handle) -> None:
        if not isinstance(handle, _core.PyMaterialParameters):
            raise TypeError(
                f"handle must be PyMaterialParameters, got {type(handle).__name__}"
            )
        self._handle = handle

    @property
    def space(self) -> MaterialParameterSpace:
        return MaterialParameterSpace(self._handle.space)

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
        arr = np.asarray(values, dtype=np.float64, order="C")
        expected = (block.num_value_rows, block.num_local_dofs)
        if arr.size != block.num_global_dofs:
            raise ValueError(
                f"{kind}_values must contain {block.num_global_dofs} values, got {arr.size}"
            )
        if arr.ndim == 2 and arr.shape != expected:
            raise ValueError(f"{kind}_values shape must be {expected}, got {arr.shape}")
        flat = np.ascontiguousarray(arr.reshape(-1), dtype=np.float64)
        if kind == "elastic":
            self._handle.set_elastic_values(flat)
        else:
            self._handle.set_plastic_values(flat)


__all__ = [
    "ParameterDofLayout",
    "ElementwiseDofLayout",
    "ConstantDofLayout",
    "ParameterMapping",
    "IdentityParameterMapping",
    "MaterialParameterRef",
    "MaterialParameterBlock",
    "MaterialParameterSpace",
    "MaterialParameters",
]
