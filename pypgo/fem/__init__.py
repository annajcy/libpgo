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
from pypgo.fem.mass import VolumeDensity, volume_density, ShellArealDensity, SelfWeightGravity
from pypgo.fem.elastic import (
    ElasticModelDefinition,
    KoiterStVKDefinition,
    LinearElasticDefinition,
    MooneyRivlinDefinition,
    StableNeoDefinition,
    StVKDefinition,
    StVKVolumeDefinition,
)
from pypgo.fem.plastic import (
    PlasticModelDefinition,
    ShellPlasticityDefinition,
    VolumetricPlasticityDefinition,
)
from pypgo.fem.fields import (
    ConstantParameterLayout,
    ElementwiseParameterLayout,
    IdentityMaterialEvaluator,
    OptimizableParameterField,
    OptimizableParameterRef,
    OptimizableParameters,
    FixedParameterField,
    MaterialAssignment,
    ParameterLayout,
    MaterialEvaluator,
    DifferentiableMaterialEvaluator,
    ElasticParameterization,
    PlasticParameterization,
    MaterialParameterization,
    MaterialParameterData,
    MaterialParameterDataProjection,
    NamedChannelMaterialParameterDataProjection,
)
from pypgo.fem.energy import (
    DeformationEnergy,
    DeformationOptions,
    ElasticMaterialEnergy,
    PlasticMaterialEnergy,
    elastic_material_energy,
    plastic_material_energy,
)
from pypgo.fem.mesh import (
    KoiterStVKShellMaterial,
    ImportedElementSet,
    ImportedMaterialData,
    ImportedMaterialField,
    ImportedMaterialRecord,
    ImportedMaterialRegion,
    SimulationAsset,
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
    # Density fields
    "VolumeDensity",
    "volume_density",
    "ShellArealDensity",
    "SelfWeightGravity",
    # Elastic
    "ElasticModelDefinition",
    "KoiterStVKDefinition",
    "LinearElasticDefinition",
    "MooneyRivlinDefinition",
    "StableNeoDefinition",
    "StVKDefinition",
    "StVKVolumeDefinition",
    # Plastic
    "PlasticModelDefinition",
    "ShellPlasticityDefinition",
    "VolumetricPlasticityDefinition",
    # Fields
    "ConstantParameterLayout",
    "ElementwiseParameterLayout",
    "IdentityMaterialEvaluator",
    "OptimizableParameterField",
    "OptimizableParameterRef",
    "OptimizableParameters",
    "FixedParameterField",
    "MaterialAssignment",
    "ParameterLayout",
    "MaterialEvaluator",
    "DifferentiableMaterialEvaluator",
    "ElasticParameterization",
    "PlasticParameterization",
    "MaterialParameterization",
    "MaterialParameterData",
    "MaterialParameterDataProjection",
    "NamedChannelMaterialParameterDataProjection",
    # Energy
    "DeformationEnergy",
    "DeformationOptions",
    "ElasticMaterialEnergy",
    "PlasticMaterialEnergy",
    "elastic_material_energy",
    "plastic_material_energy",
    # Mesh
    "KoiterStVKShellMaterial",
    "SimulationMesh",
    "SimulationAsset",
    "ImportedMaterialData",
    "ImportedMaterialRecord",
    "ImportedElementSet",
    "ImportedMaterialRegion",
    "ImportedMaterialField",
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
