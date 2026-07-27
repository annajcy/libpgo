"""Small direct-construction helpers used by Python tests."""

from __future__ import annotations

import numpy as np

import pypgo.fem as fem


def _source_values(asset, name: str) -> np.ndarray:
    aliases = {"E_membrane": "E", "E_bending": "E",
               "nu_membrane": "nu", "nu_bending": "nu",
               "thickness": "h"}
    source_name = aliases.get(name, name)
    data = asset.material_data
    for field in data.fields:
        if source_name in field.channel_names:
            channel = field.channel_names.index(source_name)
            return np.asarray([
                field.value_rows[int(field.element_to_row[i]), channel]
                for i in range(data.num_elements)], dtype=np.float64)
    values = np.empty(data.num_elements, dtype=np.float64)
    for i, material_index in enumerate(data.element_material_indices):
        prop = data.materials[int(material_index)].properties[source_name]
        values[i] = float(np.asarray(prop, dtype=np.float64).reshape(-1)[0])
    return values


def direct_assignment(asset, elastic, plastic, elastic_layout, plastic_layout,
                      elastic_values=None, plastic_values=None):
    def identity_field(field_type, names, layout_type):
        count = len(names)
        return field_type(
            names,
            layout_type(asset.num_elements, count),
            fem.IdentityMaterialEvaluator(count),
        )

    elastic_fixed = fem.FixedParameterField(
        elastic.fixed_channel_names,
        fem.ElementwiseParameterLayout(
            asset.num_elements, len(elastic.fixed_channel_names)),
        fem.IdentityMaterialEvaluator(len(elastic.fixed_channel_names)))
    plastic_fixed = fem.FixedParameterField(
        plastic.fixed_channel_names,
        fem.ElementwiseParameterLayout(
            asset.num_elements, len(plastic.fixed_channel_names)),
        fem.IdentityMaterialEvaluator(len(plastic.fixed_channel_names)))
    elastic_field = identity_field(
        fem.OptimizableParameterField,
        elastic.optimizable_channel_names, elastic_layout)
    plastic_field = identity_field(
        fem.OptimizableParameterField,
        plastic.optimizable_channel_names, plastic_layout)
    parameterization = fem.MaterialParameterization(
        fem.ElasticParameterization(elastic, elastic_fixed, elastic_field),
        fem.PlasticParameterization(plastic, plastic_fixed, plastic_field),
    )

    def fixed_data(definition):
        channels = definition.fixed_channel_names
        if not channels:
            return np.empty(0, dtype=np.float64)
        return np.column_stack([_source_values(asset, name) for name in channels]).reshape(-1)

    def initial_data(values, field):
        if values is not None:
            return np.asarray(values, dtype=np.float64).reshape(-1)
        defaults = {"stretch", "Fx", "Fy", "Fz", "Fxx", "Fyy", "Fzz"}
        rows = field.num_value_rows
        return np.asarray([
            [1.0 if name in defaults else 0.0 for name in field.parameter_names]
            for _ in range(rows)
        ], dtype=np.float64).reshape(-1)

    data = fem.MaterialParameterData(
        elastic=(fixed_data(elastic), initial_data(elastic_values, elastic_field)),
        plastic=(fixed_data(plastic), initial_data(plastic_values, plastic_field)),
    )
    return fem.MaterialAssignment(asset, parameterization, data)
