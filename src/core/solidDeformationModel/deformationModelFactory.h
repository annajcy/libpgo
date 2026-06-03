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

class DeformationModelState;

struct DeformationModelOptions
{
  bool enforceSPD = true;
  bool enableMaterialMaxStep = true;
  // Per-element assembler weights; empty means all ones.
  EigenSupport::VXd elementWeights;
};

// Build a SimulationMesh from a volumetric mesh.
std::unique_ptr<SimulationMesh> makeSimulationMesh(const VolumetricMeshes::VolumetricMesh &mesh);

std::shared_ptr<DeformationModelEnergy> makeDeformationEnergy(
  std::shared_ptr<DeformationModelState> state,
  const Formulation &formulation,
  const DeformationModelOptions &opts = {});

}  // namespace SolidDeformationModel
}  // namespace pgo
