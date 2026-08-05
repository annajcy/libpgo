"""Small direct-construction helpers used by Python tests."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import pypgo.fem as fem
from pypgo.mesh.volume import VolumeMesh


_FIXED_ORDERS = {
    "stable_neo": ("E", "nu"),
    "stable_neo_principal_stretch": ("E", "nu"),
    "neo_hookean": ("E", "nu"),
    "stvk": ("E", "nu"),
    "linear": ("E", "nu"),
    "stvk_vol": ("E", "nu", "J"),
    "mooney_rivlin": ("mu01", "mu10", "v1"),
}


@dataclass(frozen=True)
class DirectMaterial:
    mesh: fem.SimulationMesh
    binding: fem.MaterialBinding
    state: fem.MaterialState


def direct_material(source, elastic, plastic, elastic_layout=None,
                    plastic_layout=None, elastic_values=None,
                    plastic_values=None):
    del elastic_layout, plastic_layout
    if isinstance(source, VolumeMesh):
        mesh = fem.SimulationMesh(source)
        veg = source.to_veg_file()
        materials = veg.materials
        assignments = source.element_material_indices
    elif isinstance(source, fem.SimulationMesh):
        mesh = source
        materials = None
        assignments = None
    else:
        raise TypeError("source must be VolumeMesh or SimulationMesh")

    def fixed_data(definition):
        names = _FIXED_ORDERS.get(definition.name, ())
        if len(names) != definition.num_fixed_channels:
            raise ValueError(f"missing fixed-channel order for {definition.name}")
        if not names:
            return np.empty((mesh.num_elements, 0), dtype=np.float64)
        if materials is None:
            raise ValueError("fixed material channels require a VolumeMesh")
        return np.asarray([
            [getattr(materials[int(assignments[element])], name)
             for name in names]
            for element in range(mesh.num_elements)
        ], dtype=np.float64)

    def initial_data(values, definition, plastic_domain=False):
        count = definition.num_optimizable_channels
        if values is not None:
            array = np.asarray(values, dtype=np.float64)
            if array.size == count and mesh.num_elements != 1:
                array = np.broadcast_to(array.reshape(1, count),
                                        (mesh.num_elements, count))
            if array.size != mesh.num_elements * count:
                raise ValueError("material state has the wrong elementwise size")
            return np.ascontiguousarray(array.reshape(-1))
        result = np.zeros((mesh.num_elements, count), dtype=np.float64)
        if plastic_domain:
            if definition.name == "shell_ff_dof1":
                result[:, 0] = 1.0
            elif definition.name == "volumetric_dof3":
                result[:, :] = 1.0
            elif definition.name == "volumetric_dof6":
                result[:, [0, 3, 5]] = 1.0
        return result.reshape(-1)

    binding = fem.MaterialBinding(
        elastic=fem.ElasticMaterialBinding(
            elastic, mesh.num_elements, fixed_data(elastic)),
        plastic=fem.PlasticMaterialBinding(
            plastic, mesh.num_elements, fixed_data(plastic)),
    )
    state = fem.MaterialState(
        elastic_values=initial_data(elastic_values, elastic),
        plastic_values=initial_data(plastic_values, plastic, True),
    )
    return DirectMaterial(mesh=mesh, binding=binding, state=state)
