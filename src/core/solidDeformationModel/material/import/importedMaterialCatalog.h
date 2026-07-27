#pragma once

#include <span>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace pgo::SolidDeformationModel
{

/// A value copied from an import format without assigning it a constitutive
/// meaning.  Scalar properties cover the material constants used by the
/// built-in Vega materials; vector properties cover structured values such as
/// an orthotropic material frame.
using ImportedPropertyValue = std::variant<double, std::vector<double>>;

struct ImportedMaterialRecord
{
  std::string name;
  std::string family;
  std::unordered_map<std::string, ImportedPropertyValue> properties;
};

struct ImportedElementSet
{
  std::string name;
  std::vector<int> elements;
};

struct ImportedMaterialRegion
{
  int materialIndex = -1;
  int setIndex = -1;
};

/// Lossless material table and region assignment imported from a source mesh.
/// This type deliberately does not contain manually supplied spatial fields.
class ImportedMaterialCatalog final
{
public:
  ImportedMaterialCatalog() = default;
  ImportedMaterialCatalog(
    int numElements,
    std::vector<ImportedMaterialRecord> materials,
    std::vector<ImportedElementSet> sets,
    std::vector<ImportedMaterialRegion> regions):
    numElements_(numElements),
    materials_(std::move(materials)),
    sets_(std::move(sets)),
    regions_(std::move(regions))
  {
    validate();
  }

  int numElements() const { return numElements_; }
  std::span<const ImportedMaterialRecord> materials() const { return materials_; }
  std::span<const ImportedElementSet> sets() const { return sets_; }
  std::span<const ImportedMaterialRegion> regions() const { return regions_; }

  /// Resolve material assignment using the source format's ordered-region
  /// semantics.  Later regions overwrite earlier regions.
  std::vector<int> elementMaterialIndices() const;

private:
  void validate() const;

  int numElements_ = 0;
  std::vector<ImportedMaterialRecord> materials_;
  std::vector<ImportedElementSet> sets_;
  std::vector<ImportedMaterialRegion> regions_;
};

}  // namespace pgo::SolidDeformationModel
