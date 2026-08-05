"""Elementwise physical material values and immutable material state."""

from __future__ import annotations

import numpy as np

import pypgo._core as _core


def _elementwise_values(name, values, num_elements: int, num_channels: int):
    array = np.asarray(values, dtype=np.float64)
    expected = num_elements * num_channels
    if array.size != expected:
        raise ValueError(
            f"{name} must contain {expected} elementwise values, got {array.size}"
        )
    return np.ascontiguousarray(array.reshape(num_elements, num_channels))


class MaterialState:
    """Immutable element-major physical optimizable channel values."""

    def __init__(self, elastic_values, plastic_values, *, _handle=None) -> None:
        if isinstance(_handle, _core.PyMaterialState):
            self._handle = _handle
            return
        self._handle = _core._create_material_state(
            np.ascontiguousarray(elastic_values, dtype=np.float64).reshape(-1),
            np.ascontiguousarray(plastic_values, dtype=np.float64).reshape(-1),
        )

    @classmethod
    def _from_handle(cls, handle):
        return cls((), (), _handle=handle)

    @property
    def elastic_values(self) -> np.ndarray:
        return np.asarray(self._handle.elastic_values, dtype=np.float64).copy()

    @property
    def plastic_values(self) -> np.ndarray:
        return np.asarray(self._handle.plastic_values, dtype=np.float64).copy()

    def with_elastic_values(self, values) -> "MaterialState":
        return MaterialState._from_handle(
            self._handle.with_elastic_values(
                np.ascontiguousarray(values, dtype=np.float64).reshape(-1)))

    def with_plastic_values(self, values) -> "MaterialState":
        return MaterialState._from_handle(
            self._handle.with_plastic_values(
                np.ascontiguousarray(values, dtype=np.float64).reshape(-1)))


class _MaterialDomainBinding:
    def __init__(self, definition, num_elements: int, fixed_values) -> None:
        num_elements = int(num_elements)
        if num_elements < 0:
            raise ValueError("num_elements must be non-negative")
        self.definition = definition
        self.num_elements = num_elements
        self.fixed_values = _elementwise_values(
            "fixed_values", fixed_values, num_elements,
            definition.num_fixed_channels,
        )

    @property
    def num_fixed_channels(self) -> int:
        return self.definition.num_fixed_channels

    @property
    def num_optimizable_channels(self) -> int:
        return self.definition.num_optimizable_channels


class ElasticMaterialBinding(_MaterialDomainBinding):
    """Elastic definition and elementwise fixed physical channels."""

    def __init__(self, definition, num_elements: int, fixed_values) -> None:
        from pypgo.fem.elastic import ElasticModelDefinition
        if not isinstance(definition, ElasticModelDefinition):
            raise TypeError("definition must be an ElasticModelDefinition")
        super().__init__(definition, num_elements, fixed_values)


class PlasticMaterialBinding(_MaterialDomainBinding):
    """Plastic definition and elementwise fixed physical channels."""

    def __init__(self, definition, num_elements: int, fixed_values) -> None:
        from pypgo.fem.plastic import PlasticModelDefinition
        if not isinstance(definition, PlasticModelDefinition):
            raise TypeError("definition must be a PlasticModelDefinition")
        super().__init__(definition, num_elements, fixed_values)


class MaterialFrames:
    """Complete elementwise material coordinate frames."""

    def __init__(self, frames=None, *, _handle=None) -> None:
        if _handle is not None:
            if not isinstance(_handle, _core.PyMaterialFrames):
                raise TypeError("handle must be PyMaterialFrames")
            self._handle = _handle
            return
        values = np.asarray(frames, dtype=np.float64)
        if values.ndim != 3 or values.shape[1:] != (3, 3):
            raise ValueError("frames must have shape (num_elements, 3, 3)")
        if values.shape[0] == 0:
            raise ValueError("frames must contain at least one element")
        self._handle = _core._make_material_frames(
            np.ascontiguousarray(values).reshape(-1).tolist())

    @classmethod
    def _from_handle(cls, handle):
        return cls(_handle=handle)

    @property
    def num_elements(self) -> int:
        return int(self._handle.num_elements)


def material_frames_from_primary_axes(axes) -> MaterialFrames:
    """Construct full deterministic frames from elementwise primary axes."""
    values = np.asarray(axes, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("axes must have shape (num_elements, 3)")
    return MaterialFrames._from_handle(
        _core._make_material_frames_from_primary_axes(
            np.ascontiguousarray(values).reshape(-1).tolist()))


class MaterialBinding:
    """Elastic/plastic definitions, elementwise fixed values, and frames."""

    def __init__(self, elastic, plastic, material_frames=None) -> None:
        if not isinstance(elastic, ElasticMaterialBinding):
            raise TypeError("elastic must be ElasticMaterialBinding")
        if not isinstance(plastic, PlasticMaterialBinding):
            raise TypeError("plastic must be PlasticMaterialBinding")
        if elastic.num_elements != plastic.num_elements:
            raise ValueError("elastic and plastic bindings must share an element count")
        if material_frames is not None and not isinstance(
                material_frames, MaterialFrames):
            raise TypeError("material_frames must be MaterialFrames or None")
        if (material_frames is not None and
                material_frames.num_elements != elastic.num_elements):
            raise ValueError("material frame count must match material domains")
        self._handle = _core._create_material_binding(
            elastic.definition._handle,
            elastic.num_elements,
            elastic.fixed_values.reshape(-1),
            plastic.definition._handle,
            plastic.fixed_values.reshape(-1),
            None if material_frames is None else material_frames._handle,
        )
        self.elastic = elastic
        self.plastic = plastic
        self.material_frames = material_frames

    @property
    def num_elements(self) -> int:
        return self.elastic.num_elements


__all__ = [
    "ElasticMaterialBinding",
    "PlasticMaterialBinding",
    "MaterialBinding",
    "MaterialState",
    "MaterialFrames",
    "material_frames_from_primary_axes",
]
