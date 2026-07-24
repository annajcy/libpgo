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
from pypgo.fem.mass import VolumeMassField, VolumeDensity, volume_density, ShellMassField, ShellArealDensity, ShellDensityThickness, ShellDensityElasticThickness, SelfWeightGravity
from pypgo.fem.elastic import (
    ElasticModelConfig,
    KoiterStVK,
    LinearElastic,
    MooneyRivlin,
    StableNeo,
    StVK,
    StVKVolume,
)
from pypgo.fem.plastic import (
    PlasticModelConfig,
    ShellPlasticity,
    VolumetricPlasticity,
)
from pypgo.fem.fields import (
    ConstantDofLayout,
    ElementwiseDofLayout,
    IdentityParameterMapping,
    MaterialParameterBlock,
    MaterialParameterRef,
    MaterialParameterSpace,
    MaterialParameters,
    ParameterDofLayout,
    ParameterMapping,
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

_TORCH_EXPORTS = {
    "ElasticStaticEquilibriumLayer",
    "PlasticStaticEquilibriumLayer",
}

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
    "volume_density",
    "ShellMassField",
    "ShellArealDensity",
    "ShellDensityThickness",
    "ShellDensityElasticThickness",
    "SelfWeightGravity",
    # Elastic
    "ElasticModelConfig",
    "KoiterStVK",
    "LinearElastic",
    "MooneyRivlin",
    "StableNeo",
    "StVK",
    "StVKVolume",
    # Plastic
    "PlasticModelConfig",
    "ShellPlasticity",
    "VolumetricPlasticity",
    # Fields
    "ConstantDofLayout",
    "ElementwiseDofLayout",
    "IdentityParameterMapping",
    "MaterialParameterBlock",
    "MaterialParameterRef",
    "MaterialParameterSpace",
    "MaterialParameters",
    "ParameterDofLayout",
    "ParameterMapping",
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


def __getattr__(name: str):
    if name in _TORCH_EXPORTS:
        from pypgo.fem import torch as _torch_module

        value = getattr(_torch_module, name)
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
