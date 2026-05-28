#include "setup/femSetup.h"

#include "deformationModelFactory.h"
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

  DeformationModelBundle bundle;

  switch (simMesh->getElementType()) {
  case SimulationMeshType::TET: {
    bundle = makeTetDeformationModel(*simMesh, TetP1{}, elasticMat, plasticMat, opts);
    break;
  }
  case SimulationMeshType::CUBIC: {
    bundle = makeCubicDeformationModel(*simMesh, HexTrilinear{}, elasticMat, plasticMat, opts);
    break;
  }
  default:
    throw std::invalid_argument("initializeVolumetricSimulation: unsupported volumetric element type.");
  }

  InitializedVolumetricSimulation initialized;
  initialized.elasticEnergy = std::move(bundle.energy);
  initialized.plasticity = std::move(bundle.plasticParams);
  initialized.restPosition = std::move(bundle.restPosition);
  initialized.simulationMesh = std::move(simMesh);
  return initialized;
}
}  // namespace pgo::RunSim
