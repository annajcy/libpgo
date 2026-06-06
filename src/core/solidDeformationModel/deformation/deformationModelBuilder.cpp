/*
author: Bohan Wang
copyright to USC
*/
#include "deformation/deformationModelBuilder.h"

#include "simulation/simulationMesh.h"
#include "deformation/deformationModelState.h"
#include "deformation/deformationModelAssembler.h"
#include "pgoLogging.h"

#include <utility>

namespace pgo::SolidDeformationModel
{
namespace ES = pgo::EigenSupport;

std::shared_ptr<DeformationModelEnergy> makeDeformationEnergy(
  std::shared_ptr<DeformationModelState> state,
  const Formulation &formulation,
  const DeformationModelOptions &opts)
{
  if (!state)
    throw std::invalid_argument("makeDeformationEnergy: state must be non-null.");
  const SimulationMesh &mesh = *state->mesh();
  const int nele = mesh.getNumElements();

  SPDLOG_LOGGER_INFO(pgo::Logging::lgr(), "Building deformation energy with formulation: {} and deformation model state",
    formulation.getName());

  auto manager = std::make_unique<DeformationModelManager>(
    std::move(state), formulation,
    opts.enforceSPD ? 1 : 0,
    /*elementFiberDirections=*/nullptr,
    /*vertexFiberDirections=*/nullptr);

  ES::VXd elementWeights = opts.elementWeights;
  if (elementWeights.size() == 0)
    elementWeights = ES::VXd::Ones(nele);
  else if (static_cast<int>(elementWeights.size()) != nele)
    throw std::invalid_argument("makeDeformationEnergy: elementWeights size does not match the element count.");

  auto assembler = std::make_unique<DeformationModelAssembler>(std::move(manager), elementWeights.data());

  auto energy = std::make_shared<DeformationModelEnergy>(
    std::move(assembler), 0, opts.enableMaterialMaxStep);

  return energy;
}

}  // namespace pgo::SolidDeformationModel
