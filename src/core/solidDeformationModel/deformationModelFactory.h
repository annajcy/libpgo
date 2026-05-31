/*
author: Bohan Wang
copyright to USC
*/

#pragma once

#include "deformationModelManager.h"
#include "EigenSupport.h"

#include "formulations/formulation.h"
#include "simulationMesh.h"
#include "deformationModelEnergy.h"

#include <memory>

namespace pgo
{
namespace VolumetricMeshes
{
class VolumetricMesh;
}  // namespace VolumetricMeshes

namespace SolidDeformationModel
{

struct DeformationModelOptions
{
  bool enforceSPD = true;
  bool enableMaterialMaxStep = true;
  // Per-element assembler weights; empty means all ones.
  EigenSupport::VXd elementWeights;
};

// Build a SimulationMesh from a volumetric mesh.
std::unique_ptr<SimulationMesh> makeSimulationMesh(const VolumetricMeshes::VolumetricMesh &mesh);

// Build a ready-to-use deformation energy from an existing SimulationMesh.
// The returned energy owns its rest position and initial material parameters.
std::shared_ptr<DeformationModelEnergy> makeDeformationEnergy(
  const SimulationMesh &mesh,
  const Formulation &formulation,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  const DeformationModelOptions &opts = {});

}  // namespace SolidDeformationModel
}  // namespace pgo
