#include "material/parameterization/materialParameterization.h"

#include "material/data/materialParameterData.h"

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
    throw std::invalid_argument("MaterialParameterData values must be finite.");
}
}  // namespace

void MaterialParameterization::validate(const MaterialParameterData &data) const
{
  validateBlock(elastic_, data.elastic);
  validateBlock(plastic_, data.plastic);
}

}  // namespace pgo::SolidDeformationModel
