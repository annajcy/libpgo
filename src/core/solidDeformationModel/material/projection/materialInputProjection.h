#pragma once

#include "material/data/namedMaterialInputData.h"
#include "material/import/importedMaterialCatalog.h"
#include "material/model/materialSchema.h"
#include "material/parameterization/parameterLayout.h"

namespace pgo::SolidDeformationModel
{

/// Project scalar properties from an imported material catalog into a
/// parameter layout.
EigenSupport::VXd projectImportedMaterialInputs(
  const ImportedMaterialCatalog &catalog,
  const ParameterInputSchema &schema,
  const ParameterLayout &layout);

/// Project explicitly supplied named spatial inputs into a parameter layout.
EigenSupport::VXd projectNamedMaterialInputs(
  const NamedMaterialInputData &inputs,
  const ParameterInputSchema &schema,
  const ParameterLayout &layout);

}  // namespace pgo::SolidDeformationModel
