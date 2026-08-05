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
from pypgo.fem.mass import VolumeDensity, volume_density, ShellArealDensity
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
    ConstantParameterLayout,
    ElementwiseParameterLayout,
    IdentityMaterialChannelMapping,
    OptimizableParameterField,
    OptimizableParameterRef,
    OptimizableMaterialChannelRef,
    MaterialState,
    FixedParameterField,
    FixedMaterialParameters,
    ElasticMaterialBinding,
    PlasticMaterialBinding,
    MaterialBinding,
    ParameterLayout,
    MaterialChannelMapping,
    DifferentiableMaterialChannelMapping,
    NamedMaterialInputField,
    NamedMaterialInputData,
    MaterialFrameField,
    GlobalAxesMaterialFrameField,
    ConstantMaterialFrameField,
    ElementwiseMaterialFrameField,
    material_frames_from_primary_axes,
    project_imported_material_inputs,
    project_named_material_inputs,
    project_imported_material_frames,
)
from pypgo.fem.energy import (
    DeformationEnergyOperator,
    DeformationPotentialEnergy,
    DeformationOptions,
    MaterialVJP,
)
from pypgo.fem.mesh import (
    ImportedElementSet,
    ImportedMaterialCatalog,
    ImportedMaterialRecord,
    ImportedMaterialRegion,
    SimulationImportResult,
    SimulationMesh,
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
    "ConstantParameterLayout",
    "ElementwiseParameterLayout",
    "IdentityMaterialChannelMapping",
    "OptimizableParameterField",
    "OptimizableParameterRef",
    "OptimizableMaterialChannelRef",
    "MaterialState",
    "FixedParameterField",
    "FixedMaterialParameters",
    "ElasticMaterialBinding",
    "PlasticMaterialBinding",
    "MaterialBinding",
    "ParameterLayout",
    "MaterialChannelMapping",
    "DifferentiableMaterialChannelMapping",
    "NamedMaterialInputField",
    "NamedMaterialInputData",
    "project_imported_material_inputs",
    "project_named_material_inputs",
    "MaterialFrameField",
    "GlobalAxesMaterialFrameField",
    "ConstantMaterialFrameField",
    "ElementwiseMaterialFrameField",
    "material_frames_from_primary_axes",
    "project_imported_material_frames",
    # Energy
    "DeformationEnergyOperator",
    "DeformationPotentialEnergy",
    "DeformationOptions",
    "MaterialVJP",
    # Mesh
    "SimulationMesh",
    "SimulationImportResult",
    "ImportedMaterialCatalog",
    "ImportedMaterialRecord",
    "ImportedElementSet",
    "ImportedMaterialRegion",
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
