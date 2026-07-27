#include "simulation/simulationAsset.h"
#include "simulation/simulationMesh.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace pgo::SolidDeformationModel
{

SimulationAsset::SimulationAsset(
  std::shared_ptr<const SimulationMesh> mesh,
  ImportedMaterialData materialData):
  mesh_(std::move(mesh)),
  materialData_(std::move(materialData))
{
  if (!mesh_)
    throw std::invalid_argument("SimulationAsset requires a non-null mesh.");
  if (materialData_.numElements() != mesh_->getNumElements())
    throw std::invalid_argument(
      "SimulationAsset material data element count does not match the mesh.");
}

void ImportedMaterialField::validate() const
{
  if (channelNames_.size() != static_cast<std::size_t>(valueRows_.cols()))
    throw std::invalid_argument(
      "ImportedMaterialField channel count does not match value rows.");
  std::unordered_set<std::string> names;
  for (const auto &name : channelNames_) {
    if (name.empty() || !names.insert(name).second)
      throw std::invalid_argument(
        "ImportedMaterialField channel names must be non-empty and unique.");
  }
  if (!valueRows_.allFinite())
    throw std::invalid_argument("ImportedMaterialField values must be finite.");
  for (const int row : elementToRow_) {
    if (row < -1 || row >= valueRows_.rows())
      throw std::invalid_argument(
        "ImportedMaterialField element-to-row index is out of range.");
  }
}

void ImportedMaterialData::validate() const
{
  if (numElements_ < 0)
    throw std::invalid_argument("ImportedMaterialData element count must be non-negative.");

  for (const auto &material : materials_) {
    if (material.name.empty() || material.family.empty())
      throw std::invalid_argument(
        "ImportedMaterialData material name and family must be non-empty.");
    for (const auto &[name, value] : material.properties) {
      if (name.empty())
        throw std::invalid_argument(
          "ImportedMaterialData property names must be non-empty.");
      if (const auto *scalar = std::get_if<double>(&value)) {
        if (!std::isfinite(*scalar))
          throw std::invalid_argument(
            "ImportedMaterialData scalar properties must be finite.");
      }
      else {
        const auto &vector = std::get<std::vector<double>>(value);
        if (!std::all_of(vector.begin(), vector.end(),
              [](double entry) { return std::isfinite(entry); }))
          throw std::invalid_argument(
            "ImportedMaterialData vector properties must be finite.");
      }
    }
  }

  for (const auto &set : sets_) {
    if (set.name.empty())
      throw std::invalid_argument("ImportedMaterialData set names must be non-empty.");
    for (const int element : set.elements) {
      if (element < 0 || element >= numElements_)
        throw std::invalid_argument(
          "ImportedMaterialData set contains an out-of-range element.");
    }
  }

  for (const auto &region : regions_) {
    if (region.materialIndex < 0 ||
        region.materialIndex >= static_cast<int>(materials_.size()) ||
        region.setIndex < 0 ||
        region.setIndex >= static_cast<int>(sets_.size()))
      throw std::invalid_argument(
        "ImportedMaterialData region references an out-of-range material or set.");
  }

  for (const auto &field : fields_) {
    if (field.numElements() != numElements_)
      throw std::invalid_argument(
        "ImportedMaterialData field element count does not match the data domain.");
  }
}

std::vector<int> ImportedMaterialData::elementMaterialIndices() const
{
  std::vector<int> result(static_cast<std::size_t>(numElements_), -1);
  for (const auto &region : regions_) {
    for (const int element : sets_[static_cast<std::size_t>(region.setIndex)].elements)
      result[static_cast<std::size_t>(element)] = region.materialIndex;
  }
  return result;
}

}  // namespace pgo::SolidDeformationModel
