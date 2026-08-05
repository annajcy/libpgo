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
from pypgo.fem.elastic import (
    ElasticModelDefinition,
    KoiterStVKDefinition,
    LinearElasticDefinition,
    MooneyRivlinDefinition,
    NeoHookeanDefinition,
    StableNeoDefinition,
    StVKDefinition,
    StVKVolumeDefinition,
    SystematicPokingDefinition,
)
from pypgo.fem.plastic import (
    PlasticModelDefinition,
    ShellPlasticityDefinition,
    VolumetricPlasticityDefinition,
)
from pypgo.fem.fields import (
    MaterialState,
    ElasticMaterialBinding,
    PlasticMaterialBinding,
    MaterialBinding,
    MaterialFrames,
    material_frames_from_primary_axes,
)
from pypgo.fem.energy import (
    DeformationEnergyOperator,
    DeformationPotentialEnergy,
    DeformationOptions,
    MaterialVJP,
)
from pypgo.fem.mesh import (
    SimulationMesh,
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
    # Elastic
    "ElasticModelDefinition",
    "KoiterStVKDefinition",
    "LinearElasticDefinition",
    "MooneyRivlinDefinition",
    "NeoHookeanDefinition",
    "StableNeoDefinition",
    "StVKDefinition",
    "StVKVolumeDefinition",
    "SystematicPokingDefinition",
    # Plastic
    "PlasticModelDefinition",
    "ShellPlasticityDefinition",
    "VolumetricPlasticityDefinition",
    # Fields
    "MaterialState",
    "ElasticMaterialBinding",
    "PlasticMaterialBinding",
    "MaterialBinding",
    "MaterialFrames",
    "material_frames_from_primary_axes",
    # Energy
    "DeformationEnergyOperator",
    "DeformationPotentialEnergy",
    "DeformationOptions",
    "MaterialVJP",
    # Mesh
    "SimulationMesh",
]
