"""pypgo.fem - FEM construction surface for deformation energy."""

from pypgo.fem.formulations import (
    Formulation,
    KoiterShell,
    LinearCubic,
    ShellFormulation,
    TetP1,
    TricubicHermite,
    VolumetricFormulation,
)
from pypgo.fem.elastic import (
    ElasticModel,
    KoiterStVK,
    LinearElastic,
    MooneyRivlin,
    StableNeo,
    StVK,
    StVKVolume,
)
from pypgo.fem.plastic import (
    PlasticModel,
    ShellPlasticity,
    VolumetricPlasticity,
)
from pypgo.fem.fields import (
    ConstantField,
    ElementwiseField,
    ParameterField,
)
from pypgo.fem.energy import (
    DeformationEnergy,
    DeformationOptions,
    PlasticMaterialEnergy,
    deformation_energy,
    plastic_material_energy,
)
from pypgo.fem.mesh import (
    KoiterStVKShellMaterial,
    SimulationMesh,
    read_shell_config,
    write_shell_config,
)
from pypgo.fem.torch import (
    StaticEquilibriumLayer,
)

__all__ = [
    # Formulations
    "Formulation",
    "KoiterShell",
    "LinearCubic",
    "ShellFormulation",
    "TetP1",
    "TricubicHermite",
    "VolumetricFormulation",
    # Elastic
    "ElasticModel",
    "KoiterStVK",
    "LinearElastic",
    "MooneyRivlin",
    "StableNeo",
    "StVK",
    "StVKVolume",
    # Plastic
    "PlasticModel",
    "ShellPlasticity",
    "VolumetricPlasticity",
    # Fields
    "ConstantField",
    "ElementwiseField",
    "ParameterField",
    # Energy
    "DeformationEnergy",
    "DeformationOptions",
    "PlasticMaterialEnergy",
    "deformation_energy",
    "plastic_material_energy",
    # Mesh
    "KoiterStVKShellMaterial",
    "SimulationMesh",
    "read_shell_config",
    "write_shell_config",
    # Torch
    "StaticEquilibriumLayer",
]
