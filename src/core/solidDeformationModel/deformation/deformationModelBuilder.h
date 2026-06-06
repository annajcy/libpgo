/*
author: Bohan Wang
copyright to USC
*/

#pragma once

#include "deformation/deformationModelManager.h"
#include "EigenSupport.h"

#include "formulations/formulation.h"
#include "deformation/deformationModelEnergy.h"

#include <memory>

namespace pgo
{
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

std::shared_ptr<DeformationModelEnergy> makeDeformationEnergy(
  std::shared_ptr<DeformationModelState> state,
  const Formulation &formulation,
  const DeformationModelOptions &opts = {});

}  // namespace SolidDeformationModel
}  // namespace pgo
