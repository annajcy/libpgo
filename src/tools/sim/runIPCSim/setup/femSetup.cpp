#include "setup/femSetup.h"

#include "deformationModelFactory.h"
#include "deformationModelAssembler.h"
#include "tetMesh.h"
#include "cubicMesh.h"

#include <stdexcept>
#include <utility>

namespace pgo::RunSim
{
InitializedVolumetricSimulation initializeVolumetricSimulation(
  const VolumetricMeshes::VolumetricMesh &volumetricMesh,
  SolidDeformationModel::DeformationModelElasticMaterial elasticMat,
  SolidDeformationModel::DeformationModelPlasticMaterial plasticMat,
  bool enableMaterialMaxStep)
{
  using namespace pgo::SolidDeformationModel;

  DeformationModelOptions opts;
  opts.enforceSPD = true;
  opts.enableMaterialMaxStep = enableMaterialMaxStep;

  auto simMesh = makeSimulationMesh(volumetricMesh);
  if (!simMesh)
    throw std::runtime_error("initializeVolumetricSimulation: failed to create SimulationMesh.");

  std::shared_ptr<DeformationModelEnergy> energy;

  switch (simMesh->getElementType()) {
  case SimulationMeshType::TET: {
    energy = makeDeformationEnergy(*simMesh, P1TetFormulation{}, elasticMat, plasticMat, opts);
    break;
  }
  case SimulationMeshType::CUBIC: {
    energy = makeDeformationEnergy(*simMesh, LinearCubicFormulation{}, elasticMat, plasticMat, opts);
    break;
  }
  default:
    throw std::invalid_argument("initializeVolumetricSimulation: unsupported volumetric element type.");
  }

  InitializedVolumetricSimulation initialized;
  initialized.elasticEnergy = energy;
  initialized.plasticity = energy->assembler().getDeformationModelManager().getPlasticGlobalParams();
  initialized.restPosition = energy->getRestPosition();
  initialized.simulationMesh = std::move(simMesh);
  return initialized;
}
}  // namespace pgo::RunSim
