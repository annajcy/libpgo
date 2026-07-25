#pragma once

#include "material/core/materialParameters.h"
#include "material/elastic/elasticModel.h"
#include "material/plastic/plasticModel.h"

#include <memory>

namespace pgo
{
namespace SolidDeformationModel
{

class SimulationMesh;

std::shared_ptr<const MaterialParameterSpace> makeMaterialParameterSpace(
  const ElasticModelConfig &elastic,
  std::shared_ptr<const ParameterDofLayout> elasticDofLayout,
  std::shared_ptr<const MaterialChannelMapping> elasticMapping,
  const PlasticModelConfig &plastic,
  std::shared_ptr<const ParameterDofLayout> plasticDofLayout,
  std::shared_ptr<const MaterialChannelMapping> plasticMapping);

std::shared_ptr<MaterialParameters> makeMaterialParameters(
  std::shared_ptr<const MaterialParameterSpace> space,
  EigenSupport::VXd elasticValues,
  EigenSupport::VXd plasticValues);

std::shared_ptr<MaterialParameters> makeDefaultMaterialParameters(
  const SimulationMesh &mesh,
  const ElasticModelConfig &elastic,
  const PlasticModelConfig &plastic);

}  // namespace SolidDeformationModel
}  // namespace pgo
