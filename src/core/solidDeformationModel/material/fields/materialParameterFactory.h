#pragma once

#include "material/fields/materialParameters.h"
#include "material/materialTypes.h"

#include <memory>
#include <optional>

namespace pgo
{
namespace SolidDeformationModel
{

class SimulationMesh;

std::shared_ptr<MaterialParameters> makeMaterialParameters(
  const SimulationMesh &mesh,
  DeformationModelElasticMaterial elastic,
  std::unique_ptr<const ParameterDofLayout> elasticDofLayout,
  std::unique_ptr<const ParameterFieldMapping> elasticMapping,
  std::optional<EigenSupport::VXd> elasticValues,
  DeformationModelPlasticMaterial plastic,
  std::unique_ptr<const ParameterDofLayout> plasticDofLayout,
  std::unique_ptr<const ParameterFieldMapping> plasticMapping,
  std::optional<EigenSupport::VXd> plasticValues);

std::shared_ptr<MaterialParameters> makeDefaultMaterialParameters(
  const SimulationMesh &mesh,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic);

}  // namespace SolidDeformationModel
}  // namespace pgo
