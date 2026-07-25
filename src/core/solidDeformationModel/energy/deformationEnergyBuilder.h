/*
author: Bohan Wang
copyright to USC
*/

#pragma once

#include "deformation/deformationModelManager.h"
#include "material/fields/materialParameterBuilder.h"
#include "material/fields/materialFrameField.h"
#include "EigenSupport.h"

#include "formulations/formulation/formulation.h"
#include "energy/deformationModelEnergy.h"

#include <memory>

namespace pgo
{
namespace SolidDeformationModel
{

class SimulationMesh;

struct DeformationModelOptions
{
  bool projectHessianPSD = true;
  bool enableMaterialMaxStep = true;
  // Per-element assembler weights; empty means all ones.
  EigenSupport::VXd elementWeights;
};

std::shared_ptr<DeformationModelEnergy> makeDeformationEnergy(
  std::shared_ptr<const SimulationMesh> mesh,
  std::shared_ptr<const ElasticModelConfig> elastic,
  std::shared_ptr<const PlasticModelConfig> plastic,
  std::shared_ptr<MaterialParameters> materialParameters,
  std::shared_ptr<const MaterialFrameField> materialFrames,
  const Formulation &formulation,
  const DeformationModelOptions &opts = {});

std::shared_ptr<DeformationModelEnergy> makeDeformationEnergy(
  std::shared_ptr<const SimulationMesh> mesh,
  std::shared_ptr<const ElasticModelConfig> elastic,
  std::shared_ptr<const PlasticModelConfig> plastic,
  const Formulation &formulation,
  const DeformationModelOptions &opts = {});

}  // namespace SolidDeformationModel
}  // namespace pgo
