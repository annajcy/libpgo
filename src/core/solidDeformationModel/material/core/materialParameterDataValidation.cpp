#include "materialParameterDataValidation.h"

#include "materialParameterData.h"
#include "materialParameterization.h"

#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace
{
template<class ModelDefinition>
void validateBlock(
  const MaterialDomainParameterization<ModelDefinition> &parameterization,
  const MaterialParameterBlockData &data)
{
  if (data.fixedValues.size() !=
    parameterization.fixedField()->layout().numGlobalParameters())
    throw std::invalid_argument(
      "MaterialParameterData fixed value count does not match its field layout.");
  if (data.initialOptimizableValues.size() !=
    parameterization.optimizableField()->layout().numGlobalParameters())
    throw std::invalid_argument(
      "MaterialParameterData optimizable value count does not match its field layout.");
  if (!data.fixedValues.allFinite() ||
    !data.initialOptimizableValues.allFinite())
    throw std::invalid_argument(
      "MaterialParameterData values must be finite.");
}
}  // namespace

void validateMaterialParameterData(
  const MaterialParameterization &parameterization,
  const MaterialParameterData &data)
{
  validateBlock(parameterization.elastic(), data.elastic);
  validateBlock(parameterization.plastic(), data.plastic);
}

}  // namespace pgo::SolidDeformationModel
