"""Small direct-construction helpers used by Python tests."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import pypgo.fem as fem


def _catalog_channel_name(name: str) -> str:
    return {"E_membrane": "E", "E_bending": "E",
            "nu_membrane": "nu", "nu_bending": "nu",
            "thickness": "h"}.get(name, name)


@dataclass(frozen=True)
class DirectMaterial:
    mesh: fem.SimulationMesh
    binding: fem.MaterialBinding
    state: fem.MaterialState


def direct_material(source, elastic, plastic, elastic_layout, plastic_layout,
                    elastic_values=None, plastic_values=None):
    if isinstance(source, fem.SimulationImportResult):
        mesh = source.mesh
        material_data = source.material_catalog
    elif isinstance(source, fem.SimulationMesh):
        mesh = source
        material_data = None
    else:
        raise TypeError("source must be SimulationMesh or SimulationImportResult")

    def identity_field(field_type, names, layout_type):
        count = len(names)
        return field_type(
            names,
            layout_type(mesh.num_elements, count),
            fem.IdentityMaterialChannelMapping(count),
        )

    elastic_fixed = fem.FixedParameterField(
        tuple(_catalog_channel_name(name) for name in elastic.fixed_channel_names),
        fem.ElementwiseParameterLayout(
            mesh.num_elements, len(elastic.fixed_channel_names)),
        fem.IdentityMaterialChannelMapping(len(elastic.fixed_channel_names)))
    plastic_fixed = fem.FixedParameterField(
        tuple(_catalog_channel_name(name) for name in plastic.fixed_channel_names),
        fem.ElementwiseParameterLayout(
            mesh.num_elements, len(plastic.fixed_channel_names)),
        fem.IdentityMaterialChannelMapping(len(plastic.fixed_channel_names)))
    elastic_field = identity_field(
        fem.OptimizableParameterField,
        elastic.optimizable_channel_names, elastic_layout)
    plastic_field = identity_field(
        fem.OptimizableParameterField,
        plastic.optimizable_channel_names, plastic_layout)
    def fixed_data(definition, field):
        channels = definition.fixed_channel_names
        if not channels:
            return np.empty(0, dtype=np.float64)
        if material_data is None:
            raise ValueError("fixed material channels require a material catalog")
        return np.asarray(
            fem.project_imported_material_inputs(material_data, field),
            dtype=np.float64,
        ).reshape(-1)

    def initial_data(values, field):
        if values is not None:
            return np.asarray(values, dtype=np.float64).reshape(-1)
        defaults = {"stretch", "Fx", "Fy", "Fz", "Fxx", "Fyy", "Fzz"}
        rows = field.num_value_rows
        return np.asarray([
            [1.0 if name in defaults else 0.0 for name in field.parameter_names]
            for _ in range(rows)
        ], dtype=np.float64).reshape(-1)

    binding = fem.MaterialBinding(
        elastic=fem.ElasticMaterialBinding(
            elastic,
            fem.FixedMaterialParameters(
                elastic_fixed, fixed_data(elastic, elastic_fixed)),
            elastic_field,
        ),
        plastic=fem.PlasticMaterialBinding(
            plastic,
            fem.FixedMaterialParameters(
                plastic_fixed, fixed_data(plastic, plastic_fixed)),
            plastic_field,
        ),
        material_frames=fem.GlobalAxesMaterialFrameField(mesh.num_elements),
    )
    state = fem.MaterialState(
        elastic_values=initial_data(elastic_values, elastic_field),
        plastic_values=initial_data(plastic_values, plastic_field),
    )
    return DirectMaterial(mesh=mesh, binding=binding, state=state)
