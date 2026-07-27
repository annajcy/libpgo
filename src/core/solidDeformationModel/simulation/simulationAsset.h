#pragma once

#include "simulation/importedMaterial.h"

#include <memory>
#include <stdexcept>
#include <string>

namespace pgo::SolidDeformationModel
{

class SimulationMesh;

class SimulationAsset final
{
public:
  SimulationAsset(
    std::shared_ptr<const SimulationMesh> mesh,
    ImportedMaterialData materialData);

  const std::shared_ptr<const SimulationMesh> &mesh() const { return mesh_; }
  const ImportedMaterialData &materialData() const { return materialData_; }

private:
  std::shared_ptr<const SimulationMesh> mesh_;
  ImportedMaterialData materialData_;
};

}  // namespace pgo::SolidDeformationModel
