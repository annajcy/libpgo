#include "material/import/importedMaterialCatalog.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace pgo::SolidDeformationModel
{

void ImportedMaterialCatalog::validate() const
{
  if (numElements_ < 0)
    throw std::invalid_argument(
      "ImportedMaterialCatalog element count must be non-negative.");

  for (const auto &material : materials_) {
    if (material.name.empty() || material.family.empty())
      throw std::invalid_argument(
        "ImportedMaterialCatalog material name and family must be non-empty.");
    for (const auto &[name, value] : material.properties) {
      if (name.empty())
        throw std::invalid_argument(
          "ImportedMaterialCatalog property names must be non-empty.");
      if (const auto *scalar = std::get_if<double>(&value)) {
        if (!std::isfinite(*scalar))
          throw std::invalid_argument(
            "ImportedMaterialCatalog scalar properties must be finite.");
      }
      else {
        const auto &vector = std::get<std::vector<double>>(value);
        if (!std::all_of(
              vector.begin(), vector.end(),
              [](double entry) { return std::isfinite(entry); }))
          throw std::invalid_argument(
            "ImportedMaterialCatalog vector properties must be finite.");
      }
    }
  }

  for (const auto &set : sets_) {
    if (set.name.empty())
      throw std::invalid_argument(
        "ImportedMaterialCatalog set names must be non-empty.");
    for (const int element : set.elements) {
      if (element < 0 || element >= numElements_)
        throw std::invalid_argument(
          "ImportedMaterialCatalog set contains an out-of-range element.");
    }
  }

  for (const auto &region : regions_) {
    if (region.materialIndex < 0 ||
        region.materialIndex >= static_cast<int>(materials_.size()) ||
        region.setIndex < 0 ||
        region.setIndex >= static_cast<int>(sets_.size()))
      throw std::invalid_argument(
        "ImportedMaterialCatalog region references an out-of-range material or set.");
  }
}

std::vector<int> ImportedMaterialCatalog::elementMaterialIndices() const
{
  std::vector<int> result(static_cast<std::size_t>(numElements_), -1);
  for (const auto &region : regions_) {
    for (const int element :
      sets_[static_cast<std::size_t>(region.setIndex)].elements)
      result[static_cast<std::size_t>(element)] = region.materialIndex;
  }
  return result;
}

}  // namespace pgo::SolidDeformationModel
