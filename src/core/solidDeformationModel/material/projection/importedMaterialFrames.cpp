#include "material/projection/importedMaterialFrames.h"

#include <stdexcept>
#include <variant>
#include <vector>

namespace pgo::SolidDeformationModel
{

std::shared_ptr<const MaterialFrameField>
projectImportedMaterialFrames(
  const ImportedMaterialCatalog &data,
  std::string property)
{
  const auto assignments = data.elementMaterialIndices();
  std::vector<MaterialFrame> frames;
  frames.reserve(static_cast<std::size_t>(data.numElements()));
  bool hasRotation = false;
  for (int element = 0; element < data.numElements(); ++element) {
    MaterialFrame frame = MaterialFrame::Identity();
    const int materialIndex = assignments[static_cast<std::size_t>(element)];
    if (materialIndex >= 0) {
      const auto &properties =
        data.materials()[static_cast<std::size_t>(materialIndex)].properties;
      const auto iter = properties.find(property);
      if (iter != properties.end()) {
        const auto *values =
          std::get_if<std::vector<double>>(&iter->second);
        if (!values || values->size() != 9)
          throw std::invalid_argument(
            "Imported material rotation must be a vector of nine values.");
        for (int row = 0; row < 3; ++row)
          for (int col = 0; col < 3; ++col)
            frame(row, col) =
              (*values)[static_cast<std::size_t>(row * 3 + col)];
        validateMaterialFrame(frame);
        hasRotation = true;
      }
    }
    frames.push_back(frame);
  }
  if (!hasRotation)
    return std::make_shared<const GlobalAxesMaterialFrameField>(
      data.numElements());
  return std::make_shared<const ElementwiseMaterialFrameField>(
    std::move(frames));
}

}  // namespace pgo::SolidDeformationModel
