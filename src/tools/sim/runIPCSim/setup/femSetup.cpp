#include "setup/femSetup.h"

#include "deformationModelFactory.h"
#include "deformationModelState.h"
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

  std::shared_ptr<const SimulationMesh> simMesh(makeSimulationMesh(volumetricMesh).release());
  if (!simMesh)
    throw std::runtime_error("initializeVolumetricSimulation: failed to create SimulationMesh.");

  std::shared_ptr<DeformationModelEnergy> energy;
  auto state = DeformationModelState::create(
    simMesh,
    elasticMat,
    ElasticFieldInit{},
    plasticMat,
    PlasticFieldInit{});

  switch (simMesh->getElementType()) {
  case SimulationMeshType::TET: {
    energy = makeDeformationEnergy(state, P1TetFormulation{}, opts);
    break;
  }
  case SimulationMeshType::CUBIC: {
    energy = makeDeformationEnergy(state, LinearCubicFormulation{}, opts);
    break;
  }
  default:
    throw std::invalid_argument("initializeVolumetricSimulation: unsupported volumetric element type.");
  }

  InitializedVolumetricSimulation initialized;
  initialized.elasticEnergy = energy;
  initialized.plasticity = energy->assembler().getDeformationModelManager().getPlasticParameterSnapshot();
  initialized.restPosition = energy->getRestPosition();
  initialized.simulationMesh = simMesh;
  return initialized;
}
}  // namespace pgo::RunSim
