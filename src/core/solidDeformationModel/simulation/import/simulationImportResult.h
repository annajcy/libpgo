#pragma once

#include "material/import/importedMaterialCatalog.h"

#include <memory>

namespace pgo::SolidDeformationModel
{

class SimulationMesh;

class SimulationImportResult final
{
public:
  SimulationImportResult(
    std::shared_ptr<const SimulationMesh> mesh,
    ImportedMaterialCatalog materialCatalog);

  const std::shared_ptr<const SimulationMesh> &mesh() const { return mesh_; }
  const ImportedMaterialCatalog &materialCatalog() const
  {
    return materialCatalog_;
  }

private:
  std::shared_ptr<const SimulationMesh> mesh_;
  ImportedMaterialCatalog materialCatalog_;
};

}  // namespace pgo::SolidDeformationModel
