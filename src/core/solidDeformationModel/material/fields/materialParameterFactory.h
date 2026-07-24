#pragma once

#include "material/fields/materialParameters.h"
#include "material/elastic/elasticModel.h"
#include "material/plastic/plasticModel.h"

#include <memory>
#include <optional>

namespace pgo
{
namespace SolidDeformationModel
{

class SimulationMesh;

std::shared_ptr<MaterialParameters> makeMaterialParameters(
  const SimulationMesh &mesh,
  const ElasticModelConfig &elastic,
  std::shared_ptr<const ParameterDofLayout> elasticDofLayout,
  std::shared_ptr<const ParameterFieldMapping> elasticMapping,
  std::optional<EigenSupport::VXd> elasticValues,
  const PlasticModelConfig &plastic,
  std::shared_ptr<const ParameterDofLayout> plasticDofLayout,
  std::shared_ptr<const ParameterFieldMapping> plasticMapping,
  std::optional<EigenSupport::VXd> plasticValues);

std::shared_ptr<MaterialParameters> makeDefaultMaterialParameters(
  const SimulationMesh &mesh,
  const ElasticModelConfig &elastic,
  const PlasticModelConfig &plastic);

}  // namespace SolidDeformationModel
}  // namespace pgo
