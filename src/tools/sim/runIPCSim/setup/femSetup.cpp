#include "setup/femSetup.h"

#include "deformationModelFactory.h"
#include "deformationModelAssembler.h"
#include "factories/elasticModelFactory.h"
#include "factories/plasticModelFactory.h"
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
  auto elasticField = ElasticModelFactory::createDefaultField(*simMesh, elasticMat);
  auto plasticField = PlasticModelFactory::createDefaultField(*simMesh, plasticMat);

  switch (simMesh->getElementType()) {
  case SimulationMeshType::TET: {
    energy = makeDeformationEnergy(*simMesh, P1TetFormulation{}, elasticField, plasticField, opts);
    break;
  }
  case SimulationMeshType::CUBIC: {
    energy = makeDeformationEnergy(*simMesh, LinearCubicFormulation{}, elasticField, plasticField, opts);
    break;
  }
  default:
    throw std::invalid_argument("initializeVolumetricSimulation: unsupported volumetric element type.");
  }

  InitializedVolumetricSimulation initialized;
  initialized.elasticEnergy = energy;
  initialized.plasticity = energy->assembler().getDeformationModelManager().getPlasticParameterSnapshot();
  initialized.restPosition = energy->getRestPosition();
  initialized.simulationMesh = std::move(simMesh);
  return initialized;
}
}  // namespace pgo::RunSim
