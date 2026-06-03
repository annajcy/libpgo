/*
author: Bohan Wang
copyright to USC
*/
#include "deformationModelFactory.h"

#include "simulationMesh.h"
#include "deformationModelState.h"
#include "cubicMesh.h"
#include "tetMesh.h"
#include "volumetricMesh.h"

#include "deformationModelAssembler.h"
#include "pgoLogging.h"

#include <utility>

namespace pgo::SolidDeformationModel
{
namespace ES = pgo::EigenSupport;

std::unique_ptr<SimulationMesh> makeSimulationMesh(const VolumetricMeshes::VolumetricMesh &mesh)
{
  std::unique_ptr<SimulationMesh> result;
  switch (mesh.getElementType()) {
  case VolumetricMeshes::VolumetricMesh::TET: {
    const auto *tet = dynamic_cast<const VolumetricMeshes::TetMesh *>(&mesh);
    if (!tet)
      throw std::invalid_argument("makeSimulationMesh: element type is TET but object is not a TetMesh.");
    result = loadTetMesh(tet);
    break;
  }
  case VolumetricMeshes::VolumetricMesh::CUBIC: {
    const auto *cubic = dynamic_cast<const VolumetricMeshes::CubicMesh *>(&mesh);
    if (!cubic)
      throw std::invalid_argument("makeSimulationMesh: element type is CUBIC but object is not a CubicMesh.");
    result = loadCubicMesh(cubic);
    break;
  }
  default:
    throw std::invalid_argument("makeSimulationMesh: unsupported volumetric element type.");
  }

  if (!result)
    throw std::runtime_error("makeSimulationMesh: failed to create SimulationMesh from volumetric mesh.");

  return result;
}

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
