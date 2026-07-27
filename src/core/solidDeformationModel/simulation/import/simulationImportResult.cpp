#include "simulation/import/simulationImportResult.h"
#include "simulation/simulationMesh.h"

#include <stdexcept>

namespace pgo::SolidDeformationModel
{

SimulationImportResult::SimulationImportResult(
  std::shared_ptr<const SimulationMesh> mesh,
  ImportedMaterialCatalog materialCatalog):
  mesh_(std::move(mesh)),
  materialCatalog_(std::move(materialCatalog))
{
  if (!mesh_)
    throw std::invalid_argument("SimulationImportResult requires a non-null mesh.");
  if (materialCatalog_.numElements() != mesh_->getNumElements())
    throw std::invalid_argument(
      "SimulationImportResult material catalog element count does not match the mesh.");
}

}  // namespace pgo::SolidDeformationModel
