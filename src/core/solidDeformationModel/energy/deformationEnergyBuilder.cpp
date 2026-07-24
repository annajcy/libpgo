/*
author: Bohan Wang
copyright to USC
*/
#include "energy/deformationEnergyBuilder.h"

#include "simulation/simulationMesh.h"
#include "deformation/deformationModelAssembler.h"
#include "material/fields/materialParameterFieldInit.h"
#include "pgoLogging.h"

#include <utility>

namespace pgo::SolidDeformationModel
{
namespace ES = pgo::EigenSupport;

std::shared_ptr<DeformationModelEnergy> makeDeformationEnergy(
  std::shared_ptr<const SimulationMesh> mesh,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  DeformationModelInputs inputs,
  const Formulation &formulation,
  const DeformationModelOptions &opts)
{
  if (!mesh)
    throw std::invalid_argument("makeDeformationEnergy: mesh must be non-null.");
  const int nele = mesh->getNumElements();

  SPDLOG_LOGGER_INFO(pgo::Logging::lgr(), "Building deformation energy with formulation: {}",
    formulation.getName());

  if (!inputs.elasticParameters)
    throw std::invalid_argument(
      "makeDeformationEnergy: elasticParameters must be non-null.");
  if (!inputs.plasticParameters)
    throw std::invalid_argument(
      "makeDeformationEnergy: plasticParameters must be non-null.");
  if (!inputs.materialFrames)
    throw std::invalid_argument(
      "makeDeformationEnergy: materialFrames must be non-null.");

  auto manager = std::make_shared<DeformationModelManager>(
    mesh, elastic, plastic, formulation,
    opts.enforceSPD ? 1 : 0,
    inputs.materialFrames);

  ES::VXd elementWeights = opts.elementWeights;
  if (elementWeights.size() == 0)
    elementWeights = ES::VXd::Ones(nele);
  else if (static_cast<int>(elementWeights.size()) != nele)
    throw std::invalid_argument("makeDeformationEnergy: elementWeights size does not match the element count.");

  auto assembler = std::make_unique<DeformationModelAssembler>(
    std::move(manager), formulation,
    std::move(inputs.elasticParameters),
    std::move(inputs.plasticParameters),
    elementWeights.data());

  auto energy = std::make_shared<DeformationModelEnergy>(
    std::move(assembler), 0, opts.enableMaterialMaxStep);

  return energy;
}

std::shared_ptr<DeformationModelEnergy> makeDeformationEnergy(
  std::shared_ptr<const SimulationMesh> mesh,
  DeformationModelElasticMaterial elastic,
  ElasticFieldInit elasticField,
  DeformationModelPlasticMaterial plastic,
  PlasticFieldInit plasticField,
  const Formulation &formulation,
  const DeformationModelOptions &opts)
{
  if (!mesh)
    throw std::invalid_argument("makeDeformationEnergy: mesh must be non-null.");
  DeformationModelInputs inputs;
  inputs.elasticParameters =
    createElasticParameterField(*mesh, elastic, std::move(elasticField));
  inputs.plasticParameters =
    createPlasticParameterField(*mesh, plastic, std::move(plasticField));
  inputs.materialFrames =
    makeGlobalAxesMaterialFrameField(mesh->getNumElements());
  return makeDeformationEnergy(
    std::move(mesh), elastic, plastic,
    std::move(inputs), formulation, opts);
}

std::shared_ptr<DeformationModelEnergy> makeDeformationEnergy(
  std::shared_ptr<const SimulationMesh> mesh,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  const Formulation &formulation,
  const DeformationModelOptions &opts)
{
  return makeDeformationEnergy(
    std::move(mesh), elastic, ElasticFieldInit{}, plastic, PlasticFieldInit{}, formulation, opts);
}

}  // namespace pgo::SolidDeformationModel
