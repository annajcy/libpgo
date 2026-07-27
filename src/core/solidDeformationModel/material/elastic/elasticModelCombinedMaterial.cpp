#include "material/elastic/elasticModelCombinedMaterial.h"
#include "material/elastic/elasticModelInvariantBasedMaterial.h"
#include "material/elastic/invariantBasedMaterialStVK.h"
#include "material/elastic/elasticModelVolumeMaterial.h"
#include <stdexcept>
#include <string>

namespace pgo::SolidDeformationModel {
namespace {
// This model has no optimization channels.
}

MaterialChannelSchema StVKVolumeDefinition::optimizableChannelSchema() const { return {}; }
MaterialChannelSchema StVKVolumeDefinition::fixedChannelSchema() const { static constexpr std::array<std::string_view, 3> names{"E", "nu", "J"}; return MaterialChannelSchema(names); }
std::unique_ptr<ElasticModel> StVKVolumeDefinition::createModel(std::span<const double> values, const MaterialFrame &) const
{
  if (values.size() != 3) throw std::invalid_argument("stvk_vol requires fixed channels E, nu, J");
  return std::make_unique<ElasticModelCombinedMaterial<2>>(
    std::make_unique<ElasticModelInvariantBasedMaterial>(
      std::make_unique<InvariantBasedMaterialStVK>(values[0], values[1], values[2])),
    std::make_unique<ElasticModelVolumeMaterial>(values[2]));
}
}  // namespace pgo::SolidDeformationModel
