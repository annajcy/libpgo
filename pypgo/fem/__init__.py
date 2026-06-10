"""pypgo.fem - FEM construction surface for deformation energy."""

from pypgo.fem.formulations import (
    Formulation,
    KoiterShell,
    CubicLinear,
    ShellFormulation,
    TetLinear,
    CubicTricubicHermite,
    VolumetricFormulation,
)
from pypgo.fem.mass import VolumeMassField, VolumeDensity, volume_density_from_veg, ShellMassField, ShellArealDensity, ShellDensityThickness, ShellDensityElasticThickness, SelfWeightGravity
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
    ElasticMaterialEnergy,
    PlasticMaterialEnergy,
    deformation_energy,
    elastic_material_energy,
    plastic_material_energy,
)
from pypgo.fem.mesh import (
    KoiterStVKShellMaterial,
    SimulationMesh,
    read_shell_config,
    write_shell_config,
)
from pypgo.fem.torch import (
    ElasticStaticEquilibriumLayer,
    PlasticStaticEquilibriumLayer,
)

__all__ = [
    # Formulations
    "Formulation",
    "KoiterShell",
    "CubicLinear",
    "ShellFormulation",
    "TetLinear",
    "CubicTricubicHermite",
    "VolumetricFormulation",
    # Mass fields
    "VolumeMassField",
    "VolumeDensity",
    "volume_density_from_veg",
    "ShellMassField",
    "ShellArealDensity",
    "ShellDensityThickness",
    "ShellDensityElasticThickness",
    "SelfWeightGravity",
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
    "ElasticMaterialEnergy",
    "PlasticMaterialEnergy",
    "deformation_energy",
    "elastic_material_energy",
    "plastic_material_energy",
    # Mesh
    "KoiterStVKShellMaterial",
    "SimulationMesh",
    "read_shell_config",
    "write_shell_config",
    # Torch
    "ElasticStaticEquilibriumLayer",
    "PlasticStaticEquilibriumLayer",
]
