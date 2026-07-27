#pragma once

#include "material/import/importedMaterialCatalog.h"
#include "material/frame/materialFrameField.h"

#include <memory>
#include <string>

namespace pgo::SolidDeformationModel
{

/// One-shot conversion of imported rotation properties into a runtime frame
/// field. Frame import is a single policy-free operation, so it is a helper
/// rather than a projection class hierarchy.
std::shared_ptr<const MaterialFrameField> projectImportedMaterialFrames(
  const ImportedMaterialCatalog &source,
  std::string property = "rotation");

}  // namespace pgo::SolidDeformationModel
