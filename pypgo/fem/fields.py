"""Fixed and optimizable material fields, schemas, and immutable state."""

from __future__ import annotations

import numpy as np

import pypgo._core as _core


class ParameterLayout:
    """Mapping from global parameter values to element-local parameters."""

    def __init__(self, *, _handle=None) -> None:
        if _handle is None or not isinstance(_handle, _core.PyParameterLayout):
            raise TypeError(
                "ParameterLayout is abstract; use ElementwiseParameterLayout or "
                "ConstantParameterLayout"
            )
        self._handle = _handle

    @classmethod
    def _from_handle(cls, handle):
        wrapper_type = {
            "constant": ConstantParameterLayout,
            "elementwise": ElementwiseParameterLayout,
        }.get(handle._kind, ParameterLayout)
        result = wrapper_type.__new__(wrapper_type)
        ParameterLayout.__init__(result, _handle=handle)
        return result

    @property
    def num_elements(self) -> int:
        return self._handle.num_elements

    @property
    def num_local_parameters(self) -> int:
        return self._handle.num_local_parameters

    @property
    def num_global_parameters(self) -> int:
        return self._handle.num_global_parameters

    @property
    def num_value_rows(self) -> int:
        return self._handle.num_value_rows


class ElementwiseParameterLayout(ParameterLayout):
    """One independent parameter row per element."""

    def __init__(self, num_elements: int, num_local_parameters: int) -> None:
        super().__init__(_handle=_core._make_elementwise_parameter_layout(
            int(num_elements), int(num_local_parameters)))


class ConstantParameterLayout(ParameterLayout):
    """One parameter row shared by every element."""

    def __init__(self, num_elements: int, num_local_parameters: int) -> None:
        super().__init__(_handle=_core._make_constant_parameter_layout(
            int(num_elements), int(num_local_parameters)))


class MaterialChannelMapping:
    """Forward transformation from local parameters to material channels."""

    def __init__(self, *, _handle=None) -> None:
        if _handle is None or not isinstance(_handle, _core.PyMaterialChannelMapping):
            raise TypeError(
                "MaterialChannelMapping is abstract; use a concrete mapping"
            )
        self._handle = _handle

    @classmethod
    def _from_handle(cls, handle):
        wrapper_type = (
            IdentityMaterialChannelMapping
            if handle._kind == "identity"
            else MaterialChannelMapping
        )
        result = wrapper_type.__new__(wrapper_type)
        MaterialChannelMapping.__init__(result, _handle=handle)
        return result

    @property
    def num_parameters(self) -> int:
        return self._handle.num_parameters

    @property
    def num_channels(self) -> int:
        return self._handle.num_channels


class DifferentiableMaterialChannelMapping(MaterialChannelMapping):
    """Differentiable transformation from local parameters to material channels."""

    def __init__(self, *, _handle=None) -> None:
        if _handle is None or not isinstance(
            _handle, _core.PyDifferentiableMaterialChannelMapping
        ):
            raise TypeError(
                "DifferentiableMaterialChannelMapping is abstract; use a concrete mapping"
            )
        super().__init__(_handle=_handle)

    @classmethod
    def _from_handle(cls, handle):
        wrapper_type = (
            IdentityMaterialChannelMapping
            if handle._kind == "identity"
            else DifferentiableMaterialChannelMapping
        )
        result = wrapper_type.__new__(wrapper_type)
        DifferentiableMaterialChannelMapping.__init__(result, _handle=handle)
        return result


class IdentityMaterialChannelMapping(DifferentiableMaterialChannelMapping):
    """Identity mapping; local parameters are the material channels."""

    def __init__(self, num_parameters: int) -> None:
        super().__init__(_handle=_core._make_identity_material_channel_mapping(
            int(num_parameters)))


class OptimizableParameterRef:
    """Semantic reference to one raw optimizable parameter."""

    def __init__(self, *, _handle=None) -> None:
        if _handle is None or not isinstance(_handle, _core.PyOptimizableParameterRef):
            raise TypeError(
                "OptimizableParameterRef objects are created by a parameter field"
            )
        self._handle = _handle

    @classmethod
    def _from_handle(cls, handle):
        return cls(_handle=handle)

    @property
    def name(self) -> str:
        return self._handle.name

    @property
    def parameter_index(self) -> int:
        return self._handle.parameter_index


class OptimizableMaterialChannelRef:
    """Reference to one physical channel produced by a mapping."""

    def __init__(self, *, _handle=None) -> None:
        if _handle is None or not isinstance(
            _handle, _core.PyOptimizableMaterialChannelRef
        ):
            raise TypeError(
                "OptimizableMaterialChannelRef objects are created by a parameterization"
            )
        self._handle = _handle

    @classmethod
    def _from_handle(cls, handle):
        return cls(_handle=handle)

    @property
    def name(self) -> str:
        return self._handle.name

    @property
    def channel_index(self) -> int:
        return self._handle.channel_index


class OptimizableParameterField:
    """Immutable schema for one elastic or plastic optimizable parameter field."""

    def __init__(self, parameter_names, layout=None, mapping=None) -> None:
        if layout is None:
            raise TypeError(
                "OptimizableParameterField requires parameter_names and layout"
            )
        parameter_names = tuple(str(name) for name in parameter_names)
        if mapping is None:
            mapping = IdentityMaterialChannelMapping(len(parameter_names))
        if not isinstance(layout, ParameterLayout) or not isinstance(
            mapping, DifferentiableMaterialChannelMapping
        ):
            raise TypeError("layout and mapping have incompatible types")
        self._handle = _core._create_optimizable_parameter_field(
            list(parameter_names),
            layout._handle, mapping._handle)
        self._layout = layout
        self._mapping = mapping

    @classmethod
    def _from_handle(cls, handle):
        if not isinstance(handle, _core.PyOptimizableParameterField):
            raise TypeError("handle must be PyOptimizableParameterField")
        result = cls.__new__(cls)
        result._handle = handle
        result._layout = None
        result._mapping = None
        return result

    @property
    def parameter_names(self) -> tuple[str, ...]:
        return tuple(self._handle.parameter_names)

    @property
    def num_material_channels(self) -> int:
        return self._handle.num_material_channels

    @property
    def num_elements(self) -> int:
        return self._handle.num_elements

    @property
    def num_local_parameters(self) -> int:
        return self._handle.num_local_parameters

    @property
    def num_global_parameters(self) -> int:
        return self._handle.num_global_parameters

    @property
    def num_value_rows(self) -> int:
        return self._handle.num_value_rows

    @property
    def layout(self) -> ParameterLayout:
        if self._layout is None:
            self._layout = ParameterLayout._from_handle(self._handle.layout)
        return self._layout

    @property
    def mapping(self) -> DifferentiableMaterialChannelMapping:
        if self._mapping is None:
            self._mapping = DifferentiableMaterialChannelMapping._from_handle(
                self._handle.mapping)
        return self._mapping

    def parameter(self, name: str) -> OptimizableParameterRef:
        return OptimizableParameterRef._from_handle(
            self._handle.parameter(str(name)))


def _coerce_global_values(name: str, values, field) -> np.ndarray:
    """Validate a global state vector against a field's row/column shape."""
    arr = np.asarray(values, dtype=np.float64, order="C")
    if arr.ndim == 1:
        if arr.size != field.num_global_parameters:
            raise ValueError(
                f"{name} must contain {field.num_global_parameters} values, got {arr.size}"
            )
        return np.ascontiguousarray(arr, dtype=np.float64)
    if arr.ndim != 2 or arr.shape != (
        field.num_value_rows, field.num_local_parameters
    ):
        raise ValueError(
            f"{name} must be a flat vector of length {field.num_global_parameters} or "
            f"a matrix with shape {(field.num_value_rows, field.num_local_parameters)}"
        )
    return np.ascontiguousarray(arr.reshape(-1), dtype=np.float64)


class MaterialState:
    """Immutable elastic and plastic optimizer-coordinate vectors."""

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


class FixedParameterField:
    """Immutable fixed material values, independent of optimizer parameters."""

    def __init__(self, parameter_names, layout=None, mapping=None) -> None:
        if layout is None:
            raise TypeError(
                "FixedParameterField requires parameter_names and layout"
            )
        parameter_names = tuple(str(name) for name in parameter_names)
        if mapping is None:
            mapping = IdentityMaterialChannelMapping(len(parameter_names))
        if not isinstance(layout, ParameterLayout) or not isinstance(
            mapping, MaterialChannelMapping
        ):
            raise TypeError("layout and mapping have incompatible types")
        self._handle = _core._create_fixed_parameter_field(
            list(parameter_names),
            layout._handle, mapping._handle)
        self._layout = layout
        self._mapping = mapping

    @classmethod
    def _from_handle(cls, handle):
        if not isinstance(handle, _core.PyFixedParameterField):
            raise TypeError("handle must be PyFixedParameterField")
        result = cls.__new__(cls)
        result._handle = handle
        result._layout = None
        result._mapping = None
        return result

    @property
    def parameter_names(self) -> tuple[str, ...]:
        return tuple(self._handle.parameter_names)

    @property
    def num_elements(self) -> int:
        return self._handle.num_elements

    @property
    def num_local_parameters(self) -> int:
        return self._handle.num_local_parameters

    @property
    def num_global_parameters(self) -> int:
        return self._handle.num_global_parameters

    @property
    def num_value_rows(self) -> int:
        return self._handle.num_value_rows

    @property
    def num_material_channels(self) -> int:
        return self._handle.num_material_channels

    @property
    def layout(self) -> ParameterLayout:
        if self._layout is None:
            self._layout = ParameterLayout._from_handle(self._handle.layout)
        return self._layout

    @property
    def mapping(self) -> MaterialChannelMapping:
        if self._mapping is None:
            self._mapping = MaterialChannelMapping._from_handle(self._handle.mapping)
        return self._mapping

class FixedMaterialParameters:
    """One fixed parameter field together with its immutable global values."""

    def __init__(self, field, values) -> None:
        if not isinstance(field, FixedParameterField):
            raise TypeError("field must be a FixedParameterField")
        self.field = field
        self.values = _coerce_global_values("values", values, field)


class _MaterialDomainBinding:
    def __init__(self, definition, fixed, optimizable_field) -> None:
        if not isinstance(fixed, FixedMaterialParameters):
            raise TypeError("fixed must be FixedMaterialParameters")
        if not isinstance(optimizable_field, OptimizableParameterField):
            raise TypeError("optimizable_field must be OptimizableParameterField")
        if fixed.field.num_elements != optimizable_field.num_elements:
            raise ValueError("fixed and optimizable fields must share an element count")
        self.definition = definition
        self.fixed = fixed
        self.optimizable_field = optimizable_field

    @property
    def fixed_channel_names(self) -> tuple[str, ...]:
        return self.definition.fixed_channel_names

    @property
    def optimizable_channel_names(self) -> tuple[str, ...]:
        return self.definition.optimizable_channel_names

    @property
    def num_elements(self) -> int:
        return self.optimizable_field.num_elements


class ElasticMaterialBinding(_MaterialDomainBinding):
    def __init__(self, definition, fixed, optimizable_field) -> None:
        from pypgo.fem.elastic import ElasticModelDefinition
        if not isinstance(definition, ElasticModelDefinition):
            raise TypeError("definition must be an ElasticModelDefinition")
        super().__init__(definition, fixed, optimizable_field)


class PlasticMaterialBinding(_MaterialDomainBinding):
    def __init__(self, definition, fixed, optimizable_field) -> None:
        from pypgo.fem.plastic import PlasticModelDefinition
        if not isinstance(definition, PlasticModelDefinition):
            raise TypeError("definition must be a PlasticModelDefinition")
        super().__init__(definition, fixed, optimizable_field)


class NamedMaterialInputField:
    """One explicitly supplied named spatial input field."""

    def __init__(self, name, channel_names, value_rows, element_to_row):
        values = np.asarray(value_rows, dtype=np.float64)
        if values.ndim != 2:
            raise ValueError("value_rows must be a 2-D array")
        self._handle = _core.PyNamedMaterialInputField(
            str(name),
            [str(value) for value in channel_names],
            values.tolist(),
            [int(value) for value in element_to_row],
        )

    @classmethod
    def _from_handle(cls, handle):
        result = cls.__new__(cls)
        result._handle = handle
        return result

    @property
    def name(self) -> str:
        return self._handle.name

    @property
    def channel_names(self) -> tuple[str, ...]:
        return tuple(self._handle.channel_names)

    @property
    def value_rows(self) -> np.ndarray:
        return np.asarray(self._handle.value_rows, dtype=np.float64)

    @property
    def element_to_row(self) -> np.ndarray:
        return np.asarray(self._handle.element_to_row, dtype=np.int64)


class NamedMaterialInputData:
    """Collection of manual or programmatically generated named inputs."""

    def __init__(self, num_elements, fields):
        fields = tuple(fields)
        if not all(isinstance(field, NamedMaterialInputField) for field in fields):
            raise TypeError("fields must contain NamedMaterialInputField objects")
        self._handle = _core.PyNamedMaterialInputData(
            int(num_elements), [field._handle for field in fields])

    @classmethod
    def _from_handle(cls, handle):
        if not isinstance(handle, _core.PyNamedMaterialInputData):
            raise TypeError("handle must be PyNamedMaterialInputData")
        result = cls.__new__(cls)
        result._handle = handle
        return result

    @property
    def num_elements(self) -> int:
        return int(self._handle.num_elements)

    @property
    def fields(self) -> tuple[NamedMaterialInputField, ...]:
        return tuple(
            NamedMaterialInputField._from_handle(field)
            for field in self._handle.fields
        )


def _projection_field(field):
    if not isinstance(field, (FixedParameterField, OptimizableParameterField)):
        raise TypeError(
            "field must be FixedParameterField or OptimizableParameterField")
    return field


def project_imported_material_inputs(source, field) -> np.ndarray:
    """Project scalar catalog properties into one parameter field."""
    from pypgo.fem.mesh import ImportedMaterialCatalog
    if not isinstance(source, ImportedMaterialCatalog):
        raise TypeError("source must be ImportedMaterialCatalog")
    field = _projection_field(field)
    return np.asarray(
        _core._project_imported_material_inputs(
            source._handle, list(field.parameter_names), field.layout._handle),
        dtype=np.float64,
    ).copy()


def project_named_material_inputs(source, field) -> np.ndarray:
    """Project explicitly supplied named inputs into one parameter field."""
    if not isinstance(source, NamedMaterialInputData):
        raise TypeError("source must be NamedMaterialInputData")
    field = _projection_field(field)
    return np.asarray(
        _core._project_named_material_inputs(
            source._handle, list(field.parameter_names), field.layout._handle),
        dtype=np.float64,
    ).copy()


class MaterialFrameField:
    """Immutable material coordinate frame field bound to a simulation mesh."""

    def __init__(self, *, _handle=None) -> None:
        if _handle is None or not isinstance(_handle, _core.PyMaterialFrameField):
            raise TypeError("MaterialFrameField is abstract; use a concrete frame field")
        self._handle = _handle

    @classmethod
    def _from_handle(cls, handle):
        return cls(_handle=handle)

    @property
    def num_elements(self) -> int:
        return int(self._handle.num_elements)


class GlobalAxesMaterialFrameField(MaterialFrameField):
    def __init__(self, num_elements: int) -> None:
        super().__init__(
            _handle=_core._make_global_axes_material_frame_field(int(num_elements)))


class ConstantMaterialFrameField(MaterialFrameField):
    def __init__(self, num_elements: int, frame) -> None:
        values = np.asarray(frame, dtype=np.float64)
        if values.shape != (3, 3):
            raise ValueError("frame must have shape (3, 3)")
        super().__init__(_handle=_core._make_constant_material_frame_field(
            int(num_elements), np.ascontiguousarray(values).reshape(-1).tolist()))


class ElementwiseMaterialFrameField(MaterialFrameField):
    def __init__(self, frames) -> None:
        values = np.asarray(frames, dtype=np.float64)
        if values.ndim != 3 or values.shape[1:] != (3, 3):
            raise ValueError("frames must have shape (num_elements, 3, 3)")
        super().__init__(_handle=_core._make_elementwise_material_frame_field(
            np.ascontiguousarray(values).reshape(-1).tolist()))


def project_imported_material_frames(
    source, property: str = "rotation"
) -> MaterialFrameField:
    """Convert imported rotation properties into a material frame field."""
    from pypgo.fem.mesh import ImportedMaterialCatalog
    if not isinstance(source, ImportedMaterialCatalog):
        raise TypeError("source must be ImportedMaterialCatalog")
    return MaterialFrameField._from_handle(
        _core._project_imported_material_frame_field(
            source._handle, str(property)))


def material_frames_from_primary_axes(axes) -> MaterialFrameField:
    """Construct full deterministic frames from elementwise primary axes."""
    values = np.asarray(axes, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("axes must have shape (num_elements, 3)")
    return MaterialFrameField._from_handle(
        _core._make_material_frames_from_primary_axes(
            np.ascontiguousarray(values).reshape(-1).tolist()))


class MaterialBinding:
    """Immutable elastic/plastic models, fields, fixed values, and frames."""

    def __init__(self, elastic, plastic, material_frames) -> None:
        if not isinstance(elastic, ElasticMaterialBinding):
            raise TypeError("elastic must be ElasticMaterialBinding")
        if not isinstance(plastic, PlasticMaterialBinding):
            raise TypeError("plastic must be PlasticMaterialBinding")
        if not isinstance(material_frames, MaterialFrameField):
            raise TypeError("material_frames must be a MaterialFrameField")
        if elastic.num_elements != plastic.num_elements:
            raise ValueError("elastic and plastic bindings must share an element count")
        if material_frames.num_elements != elastic.num_elements:
            raise ValueError("material frame count must match material fields")
        self._handle = _core._create_material_binding(
            elastic.definition._handle,
            elastic.fixed.field._handle,
            elastic.fixed.values,
            elastic.optimizable_field._handle,
            plastic.definition._handle,
            plastic.fixed.field._handle,
            plastic.fixed.values,
            plastic.optimizable_field._handle,
            material_frames._handle,
        )
        self.elastic = elastic
        self.plastic = plastic
        self.material_frames = material_frames

    @property
    def num_elements(self) -> int:
        return self.elastic.num_elements


__all__ = [
    "ParameterLayout",
    "ElementwiseParameterLayout",
    "ConstantParameterLayout",
    "MaterialChannelMapping",
    "DifferentiableMaterialChannelMapping",
    "IdentityMaterialChannelMapping",
    "FixedParameterField",
    "FixedMaterialParameters",
    "ElasticMaterialBinding",
    "PlasticMaterialBinding",
    "MaterialBinding",
    "OptimizableParameterRef",
    "OptimizableMaterialChannelRef",
    "OptimizableParameterField",
    "MaterialState",
    "NamedMaterialInputField",
    "NamedMaterialInputData",
    "project_imported_material_inputs",
    "project_named_material_inputs",
    "MaterialFrameField",
    "GlobalAxesMaterialFrameField",
    "ConstantMaterialFrameField",
    "ElementwiseMaterialFrameField",
    "project_imported_material_frames",
    "material_frames_from_primary_axes",
]
