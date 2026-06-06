"""pypgo.fem - FEM construction surface for deformation energy.

The deformation model API is state-based: material choices and parameter field
descriptors are bound to one SimulationMesh through DeformationModelState, then
deformation_energy() consumes that state plus a formulation.
"""

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
from pypgo.fem.state import (
    DeformationModelState,
    deformation_model_state,
)
from pypgo.fem.energy import (
    DeformationOptions,
    deformation_energy,
    plastic_material_energy,
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
    # State
    "DeformationModelState",
    "deformation_model_state",
    # Energy
    "DeformationOptions",
    "deformation_energy",
    "plastic_material_energy",
]
