"""Fixed and optimizable material fields, schemas, and committed state."""

from __future__ import annotations

import numpy as np

import pypgo._core as _core


class ParameterLayout:
    """Mapping from global parameter values to element-local parameters."""

    def __init__(self, handle) -> None:
        if not isinstance(handle, _core.PyParameterLayout):
            raise TypeError(
                f"handle must be PyParameterLayout, got {type(handle).__name__}"
            )
        self._handle = handle

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
        super().__init__(_core._make_elementwise_parameter_layout(
            int(num_elements), int(num_local_parameters)))


class ConstantParameterLayout(ParameterLayout):
    """One parameter row shared by every element."""

    def __init__(self, num_elements: int, num_local_parameters: int) -> None:
        super().__init__(_core._make_constant_parameter_layout(
            int(num_elements), int(num_local_parameters)))


class MaterialEvaluator:
    """Forward transformation from local parameters to material channels."""

    def __init__(self, handle) -> None:
        if not isinstance(handle, _core.PyMaterialEvaluator):
            raise TypeError(
                f"handle must be PyMaterialEvaluator, got {type(handle).__name__}"
            )
        self._handle = handle

    @property
    def num_parameters(self) -> int:
        return self._handle.num_parameters

    @property
    def num_channels(self) -> int:
        return self._handle.num_channels


class DifferentiableMaterialEvaluator(MaterialEvaluator):
    """Differentiable transformation from local parameters to material channels."""

    def __init__(self, handle) -> None:
        if not isinstance(handle, _core.PyDifferentiableMaterialEvaluator):
            raise TypeError(
                "handle must be PyDifferentiableMaterialEvaluator, "
                f"got {type(handle).__name__}"
            )
        super().__init__(handle)


class IdentityMaterialEvaluator(DifferentiableMaterialEvaluator):
    """Identity evaluator; local parameters are the material channels."""

    def __init__(self, num_parameters: int) -> None:
        super().__init__(_core._make_identity_material_evaluator(
            int(num_parameters)))


class OptimizableParameterRef:
    """Semantic reference to one raw optimizable parameter."""

    def __init__(self, handle) -> None:
        if not isinstance(handle, _core.PyOptimizableParameterRef):
            raise TypeError(
                f"handle must be PyOptimizableParameterRef, got {type(handle).__name__}"
            )
        self._handle = handle

    @property
    def name(self) -> str:
        return self._handle.name

    @property
    def parameter_index(self) -> int:
        return self._handle.parameter_index


class OptimizableMaterialChannelRef:
    """Reference to one physical channel produced by an evaluator."""

    def __init__(self, handle) -> None:
        if not isinstance(handle, _core.PyOptimizableMaterialChannelRef):
            raise TypeError(
                "handle must be PyOptimizableMaterialChannelRef, "
                f"got {type(handle).__name__}"
            )
        self._handle = handle

    @property
    def name(self) -> str:
        return self._handle.name

    @property
    def channel_index(self) -> int:
        return self._handle.channel_index


class OptimizableParameterField:
    """Immutable schema for one elastic or plastic optimizable parameter field."""

    def __init__(self, handle_or_parameter_names, layout=None, evaluator=None) -> None:
        if isinstance(handle_or_parameter_names, _core.PyOptimizableParameterField):
            self._handle = handle_or_parameter_names
            self._layout = None
            self._evaluator = None
            return
        if layout is None or evaluator is None:
            raise TypeError(
                "OptimizableParameterField requires parameter_names, layout, and evaluator"
            )
        if not isinstance(layout, ParameterLayout) or not isinstance(
            evaluator, DifferentiableMaterialEvaluator
        ):
            raise TypeError("layout and evaluator have incompatible types")
        self._handle = _core._create_optimizable_parameter_field(
            [str(name) for name in handle_or_parameter_names],
            layout._handle, evaluator._handle)
        self._layout = layout
        self._evaluator = evaluator

    @property
    def parameter_names(self) -> tuple[str, ...]:
        return tuple(self._handle.parameter_names)

    @property
    def num_material_channels(self) -> int:
        return self._handle.num_material_channels

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
            self._layout = ParameterLayout(self._handle.layout)
        return self._layout

    @property
    def evaluator(self) -> DifferentiableMaterialEvaluator:
        if self._evaluator is None:
            self._evaluator = DifferentiableMaterialEvaluator(
                self._handle.evaluator)
        return self._evaluator

    def parameter(self, name: str) -> OptimizableParameterRef:
        return OptimizableParameterRef(self._handle.parameter(str(name)))


def _coerce_global_values(name: str, values, field: OptimizableParameterField) -> np.ndarray:
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


class OptimizableParameters:
    """Committed optimizable parameter values exposed by a runtime assignment."""

    def __init__(self, handle) -> None:
        if isinstance(handle, _core.PyOptimizableParameters):
            self._handle = handle
            self._elastic_field_wrapper = None
            self._plastic_field_wrapper = None
            return
        raise TypeError("OptimizableParameters can only be created from a runtime handle")

    @classmethod
    def _from_handle(cls, handle):
        return cls(handle)

    @property
    def elastic_field(self) -> OptimizableParameterField:
        if self._elastic_field_wrapper is None:
            self._elastic_field_wrapper = OptimizableParameterField(
                self._handle.elastic_field)
        return self._elastic_field_wrapper

    @property
    def plastic_field(self) -> OptimizableParameterField:
        if self._plastic_field_wrapper is None:
            self._plastic_field_wrapper = OptimizableParameterField(
                self._handle.plastic_field)
        return self._plastic_field_wrapper

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

    def set_values(self, elastic_values, plastic_values) -> None:
        self._handle.set_values(
            _coerce_global_values(
                "elastic_values", elastic_values, self.elastic_field),
            _coerce_global_values(
                "plastic_values", plastic_values, self.plastic_field))

    def same_fields(self, other: "OptimizableParameters") -> bool:
        if not isinstance(other, OptimizableParameters):
            return False
        return bool(self._handle._same_fields(other._handle))

    def _set_values(self, kind: str, values) -> None:
        field = getattr(self, f"{kind}_field")
        flat = _coerce_global_values(f"{kind}_values", values, field)
        if kind == "elastic":
            self._handle.set_elastic_values(flat)
        else:
            self._handle.set_plastic_values(flat)


class FixedParameterField:
    """Immutable fixed material values, independent of optimizer parameters."""

    def __init__(self, handle_or_parameter_names, layout=None, evaluator=None) -> None:
        if isinstance(handle_or_parameter_names, _core.PyFixedParameterField):
            self._handle = handle_or_parameter_names
            self._layout = None
            self._evaluator = None
            return
        if layout is None or evaluator is None:
            raise TypeError(
                "FixedParameterField requires parameter_names, layout, and evaluator"
            )
        if not isinstance(layout, ParameterLayout) or not isinstance(
            evaluator, MaterialEvaluator
        ):
            raise TypeError("layout and evaluator have incompatible types")
        self._handle = _core._create_fixed_parameter_field(
            [str(name) for name in handle_or_parameter_names],
            layout._handle, evaluator._handle)
        self._layout = layout
        self._evaluator = evaluator

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
            self._layout = ParameterLayout(self._handle.layout)
        return self._layout

    @property
    def evaluator(self) -> MaterialEvaluator:
        if self._evaluator is None:
            self._evaluator = MaterialEvaluator(self._handle.evaluator)
        return self._evaluator

class _MaterialDomainParameterization:
    """Common Python view of one elastic or plastic parameter domain."""

    _definition_type = None

    def __init__(self, handle) -> None:
        self._handle = handle
        self._definition = None
        self._fixed_field = None
        self._optimizable_field = None

    @property
    def definition(self):
        if self._definition is None:
            # This path is only used when a domain wrapper is reconstructed
            # from a C++ handle.  Normal construction retains the concrete
            # definition object supplied by the caller.
            definition_type = self._definition_type
            definition = object.__new__(definition_type)
            definition._handle = self._handle.definition
            self._definition = definition
        return self._definition

    @property
    def fixed_field(self) -> FixedParameterField:
        if self._fixed_field is None:
            self._fixed_field = FixedParameterField(self._handle.fixed_field)
        return self._fixed_field

    @property
    def optimizable_field(self) -> OptimizableParameterField:
        if self._optimizable_field is None:
            self._optimizable_field = OptimizableParameterField(
                self._handle.optimizable_field)
        return self._optimizable_field

    @property
    def fixed_channel_names(self) -> tuple[str, ...]:
        return tuple(self._handle.fixed_channel_names)

    @property
    def optimizable_channel_names(self) -> tuple[str, ...]:
        return tuple(self._handle.optimizable_channel_names)

    def optimizable_channel(self, name: str) -> OptimizableMaterialChannelRef:
        return OptimizableMaterialChannelRef(
            self._handle.optimizable_channel(str(name)))

    @property
    def num_elements(self) -> int:
        return self._handle.num_elements


class ElasticParameterization(_MaterialDomainParameterization):
    """Formal parameterization of one elastic material model."""

    def __init__(self, definition, fixed_field=None, optimizable_field=None) -> None:
        from pypgo.fem.elastic import ElasticModelDefinition
        if isinstance(definition, _core.PyElasticParameterization):
            self._definition_type = ElasticModelDefinition
            super().__init__(definition)
            return
        if not isinstance(definition, ElasticModelDefinition):
            raise TypeError("definition must be an ElasticModelDefinition")
        if not isinstance(fixed_field, FixedParameterField):
            raise TypeError("fixed_field must be FixedParameterField")
        if not isinstance(optimizable_field, OptimizableParameterField):
            raise TypeError("optimizable_field must be OptimizableParameterField")
        super().__init__(_core._create_elastic_parameterization(
            definition._handle, fixed_field._handle, optimizable_field._handle))
        self._definition = definition
        self._fixed_field = fixed_field
        self._optimizable_field = optimizable_field
        self._definition_type = ElasticModelDefinition


class PlasticParameterization(_MaterialDomainParameterization):
    """Formal parameterization of one plastic material model."""

    def __init__(self, definition, fixed_field=None, optimizable_field=None) -> None:
        from pypgo.fem.plastic import PlasticModelDefinition
        if isinstance(definition, _core.PyPlasticParameterization):
            self._definition_type = PlasticModelDefinition
            super().__init__(definition)
            return
        if not isinstance(definition, PlasticModelDefinition):
            raise TypeError("definition must be a PlasticModelDefinition")
        if not isinstance(fixed_field, FixedParameterField):
            raise TypeError("fixed_field must be FixedParameterField")
        if not isinstance(optimizable_field, OptimizableParameterField):
            raise TypeError("optimizable_field must be OptimizableParameterField")
        super().__init__(_core._create_plastic_parameterization(
            definition._handle, fixed_field._handle, optimizable_field._handle))
        self._definition = definition
        self._fixed_field = fixed_field
        self._optimizable_field = optimizable_field
        self._definition_type = PlasticModelDefinition

    @property
    def dofs(self) -> int:
        return self._handle.dofs


class MaterialParameterization:
    """Complete structural definition for elastic and plastic parameters."""

    def __init__(self, elastic, plastic) -> None:
        if not isinstance(elastic, ElasticParameterization):
            raise TypeError("elastic must be ElasticParameterization")
        if not isinstance(plastic, PlasticParameterization):
            raise TypeError("plastic must be PlasticParameterization")
        self._handle = _core._create_material_parameterization(
            elastic._handle, plastic._handle)
        self.elastic = elastic
        self.plastic = plastic

    @property
    def num_elements(self) -> int:
        return self._handle.num_elements


class MaterialParameterData:
    """Pure projected numeric values; it owns no schemas or layouts."""

    def __init__(self, handle=None, *, elastic=None, plastic=None) -> None:
        if isinstance(handle, _core.PyMaterialParameterData):
            self._handle = handle
            return
        if elastic is None or plastic is None or len(elastic) != 2 or len(plastic) != 2:
            raise TypeError("MaterialParameterData requires elastic=(fixed, initial) and plastic=(fixed, initial)")
        self._handle = _core._create_material_parameter_data(
            np.asarray(elastic[0], dtype=np.float64),
            np.asarray(elastic[1], dtype=np.float64),
            np.asarray(plastic[0], dtype=np.float64),
            np.asarray(plastic[1], dtype=np.float64))

    @property
    def elastic_fixed_values(self) -> np.ndarray:
        return np.asarray(self._handle.elastic_fixed_values, dtype=np.float64).copy()

    @property
    def elastic_initial_optimizable_values(self) -> np.ndarray:
        return np.asarray(self._handle.elastic_initial_optimizable_values, dtype=np.float64).copy()

    @property
    def plastic_fixed_values(self) -> np.ndarray:
        return np.asarray(self._handle.plastic_fixed_values, dtype=np.float64).copy()

    @property
    def plastic_initial_optimizable_values(self) -> np.ndarray:
        return np.asarray(self._handle.plastic_initial_optimizable_values, dtype=np.float64).copy()


class MaterialParameterDataProjection:
    """Base class for converting imported values into raw field inputs."""

    def convert(self, source, target: MaterialParameterization) -> MaterialParameterData:
        raise NotImplementedError

    def project(self, source, target: MaterialParameterization) -> MaterialParameterData:
        from pypgo.fem.mesh import SimulationAsset, ImportedMaterialData
        if not isinstance(source, (SimulationAsset, ImportedMaterialData)):
            raise TypeError("source must be SimulationAsset or ImportedMaterialData")
        if not isinstance(target, MaterialParameterization):
            raise TypeError("target must be MaterialParameterization")
        data = self.convert(source, target)
        if not isinstance(data, MaterialParameterData):
            raise TypeError("MaterialParameterDataProjection.convert must return MaterialParameterData")
        _core._validate_material_parameter_data(target._handle, data._handle)
        return data

    def resolve_input(self, source, name: str) -> np.ndarray:
        from pypgo.fem.mesh import SimulationAsset, ImportedMaterialData
        if isinstance(source, SimulationAsset):
            source = source.material_data
        if not isinstance(source, ImportedMaterialData):
            raise TypeError("source must be SimulationAsset or ImportedMaterialData")
        return np.asarray(
            _core._resolve_material_input(source._handle, str(name)),
            dtype=np.float64)

    def pack_element_inputs(self, field, element_local_values) -> np.ndarray:
        if not isinstance(field, (FixedParameterField, OptimizableParameterField)):
            raise TypeError("field must be FixedParameterField or OptimizableParameterField")
        values = np.asarray(element_local_values, dtype=np.float64)
        if values.ndim != 2:
            raise ValueError("element_local_values must have shape (num_elements, num_inputs)")
        packed = _core._pack_material_element_inputs(field._handle.layout, values)
        return np.asarray(packed, dtype=np.float64).reshape(-1)


class NamedChannelMaterialParameterDataProjection(MaterialParameterDataProjection):
    """Default exact-name, no-defaults initialization projection."""

    def convert(self, source, target: MaterialParameterization) -> MaterialParameterData:
        from pypgo.fem.mesh import SimulationAsset, ImportedMaterialData
        if not isinstance(source, (SimulationAsset, ImportedMaterialData)):
            raise TypeError("source must be SimulationAsset or ImportedMaterialData")
        if not isinstance(target, MaterialParameterization):
            raise TypeError("target must be MaterialParameterization")
        if isinstance(source, SimulationAsset):
            handle = _core._project_material_parameter_data(source._handle, target._handle)
        else:
            handle = _core._project_material_parameter_data_from_imported_data(
                source._handle, target._handle)
        return MaterialParameterData(handle)


class MaterialAssignment:
    """Complete material binding for one simulation asset."""

    def __init__(self, asset, parameterization, data, material_frames=None) -> None:
        from pypgo.fem.mesh import SimulationAsset
        if not isinstance(asset, SimulationAsset):
            raise TypeError("asset must be a SimulationAsset")
        if not isinstance(parameterization, MaterialParameterization):
            raise TypeError("parameterization must be a MaterialParameterization")
        if not isinstance(data, MaterialParameterData):
            raise TypeError("data must be MaterialParameterData")
        self._handle = _core._create_material_assignment_from_parameterization(
            asset._handle, parameterization._handle, data._handle)
        self.asset = asset
        self.parameterization = parameterization
        self.data = data
        self.elastic = parameterization.elastic.definition
        self.plastic = parameterization.plastic.definition
        self.elastic_fixed = parameterization.elastic.fixed_field
        self.plastic_fixed = parameterization.plastic.fixed_field
        self.elastic_optimizable = parameterization.elastic.optimizable_field
        self.plastic_optimizable = parameterization.plastic.optimizable_field
        self.optimizable_parameters = OptimizableParameters._from_handle(
            self._handle.optimizable_parameters)


__all__ = [
    "ParameterLayout",
    "ElementwiseParameterLayout",
    "ConstantParameterLayout",
    "MaterialEvaluator",
    "DifferentiableMaterialEvaluator",
    "IdentityMaterialEvaluator",
    "FixedParameterField",
    "ElasticParameterization",
    "PlasticParameterization",
    "MaterialAssignment",
    "OptimizableParameterRef",
    "OptimizableMaterialChannelRef",
    "OptimizableParameterField",
    "OptimizableParameters",
    "MaterialParameterization",
    "MaterialParameterData",
    "MaterialParameterDataProjection",
    "NamedChannelMaterialParameterDataProjection",
]
