/*
author: Bohan Wang
copyright to USC
*/

#pragma once

#include "deformation/deformationModelManager.h"
#include "material/fields/materialParameterFieldInit.h"
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
  bool enforceSPD = true;
  bool enableMaterialMaxStep = true;
  // Per-element assembler weights; empty means all ones.
  EigenSupport::VXd elementWeights;
};

std::shared_ptr<DeformationModelEnergy> makeDeformationEnergy(
  std::shared_ptr<const SimulationMesh> mesh,
  DeformationModelElasticMaterial elastic,
  ElasticFieldInit elasticField,
  DeformationModelPlasticMaterial plastic,
  PlasticFieldInit plasticField,
  const Formulation &formulation,
  const DeformationModelOptions &opts = {});

std::shared_ptr<DeformationModelEnergy> makeDeformationEnergy(
  std::shared_ptr<const SimulationMesh> mesh,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  const Formulation &formulation,
  const DeformationModelOptions &opts = {});

}  // namespace SolidDeformationModel
}  // namespace pgo
