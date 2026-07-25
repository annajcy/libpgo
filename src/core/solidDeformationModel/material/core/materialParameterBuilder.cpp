#include "materialParameterBuilder.h"

#include "simulation/simulationMesh.h"

#include <cstddef>
#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace ES = EigenSupport;

namespace
{
std::span<double> elementChannelSpan(
  ES::VXd &values, int element, int numChannels)
{
  if (numChannels == 0)
    return {};
  const auto offset = static_cast<std::ptrdiff_t>(element) * numChannels;
  return std::span<double>(values.data() + offset, static_cast<std::size_t>(numChannels));
}
}  // namespace

std::shared_ptr<const MaterialParameterSpace> makeMaterialParameterSpace(
  const ElasticModelConfig &elastic,
  std::shared_ptr<const ParameterDofLayout> elasticDofLayout,
  std::shared_ptr<const MaterialChannelMapping> elasticMapping,
  const PlasticModelConfig &plastic,
  std::shared_ptr<const ParameterDofLayout> plasticDofLayout,
  std::shared_ptr<const MaterialChannelMapping> plasticMapping)
{
  if (!elasticDofLayout || !elasticMapping || !plasticDofLayout || !plasticMapping)
    throw std::invalid_argument("makeMaterialParameterSpace requires non-null layouts and channel mappings");
  const auto elasticNames = elastic.parameterChannelNames();
  const auto plasticNames = plastic.parameterChannelNames();
  const int elasticChannels = static_cast<int>(elasticNames.size());
  const int plasticChannels = static_cast<int>(plasticNames.size());
  if (elasticMapping->numChannels() != elasticChannels || elasticMapping->numInputDofs() != elasticDofLayout->numLocalDofs())
    throw std::invalid_argument("elastic parameter field definition does not match the config/layout");
  if (plasticMapping->numChannels() != plasticChannels || plasticMapping->numInputDofs() != plasticDofLayout->numLocalDofs())
    throw std::invalid_argument("plastic parameter field definition does not match the config/layout");
  if (elasticDofLayout->numElements() != plasticDofLayout->numElements())
    throw std::invalid_argument("elastic and plastic layouts must have the same element count");
  std::vector<std::string> elasticChannelNames;
  elasticChannelNames.reserve(elasticNames.size());
  for (const auto name : elasticNames) elasticChannelNames.emplace_back(name);
  std::vector<std::string> plasticChannelNames;
  plasticChannelNames.reserve(plasticNames.size());
  for (const auto name : plasticNames) plasticChannelNames.emplace_back(name);
  return std::make_shared<MaterialParameterSpace>(
    MaterialParameterField::create(std::move(elasticChannelNames),
      std::move(elasticDofLayout), std::move(elasticMapping)),
    MaterialParameterField::create(std::move(plasticChannelNames),
      std::move(plasticDofLayout), std::move(plasticMapping)));
}

std::shared_ptr<MaterialParameters> makeMaterialParameters(
  std::shared_ptr<const MaterialParameterSpace> space,
  EigenSupport::VXd elasticValues,
  EigenSupport::VXd plasticValues)
{
  if (!space)
    throw std::invalid_argument("makeMaterialParameters requires a parameter space");
  if (elasticValues.size() != space->elastic().dofLayout().numGlobalDofs() ||
    plasticValues.size() != space->plastic().dofLayout().numGlobalDofs())
    throw std::invalid_argument("material parameter values do not match the space global DOF shapes");
  return std::make_shared<MaterialParameters>(
    std::move(space), std::move(elasticValues), std::move(plasticValues));
}

std::shared_ptr<MaterialParameters> makeDefaultMaterialParameters(
  const SimulationMesh &mesh,
  const ElasticModelConfig &elastic,
  const PlasticModelConfig &plastic)
{
  const int numElements = mesh.getNumElements();
  const int numElasticChannels = static_cast<int>(elastic.parameterChannelNames().size());
  const int numPlasticChannels = static_cast<int>(plastic.parameterChannelNames().size());
  auto space = makeMaterialParameterSpace(
    elastic,
    std::make_shared<ElementwiseParameterDofLayout>(numElements, numElasticChannels),
    std::make_shared<IdentityMaterialChannelMapping>(numElasticChannels),
    plastic,
    std::make_shared<ElementwiseParameterDofLayout>(numElements, numPlasticChannels),
    std::make_shared<IdentityMaterialChannelMapping>(numPlasticChannels));

  ES::VXd elasticValues(static_cast<Eigen::Index>(numElements) * numElasticChannels);
  ES::VXd plasticValues(static_cast<Eigen::Index>(numElements) * numPlasticChannels);
  for (int element = 0; element < numElements; element++) {
    elastic.initializeDefaultElementChannels(
      mesh, element, elementChannelSpan(elasticValues, element, numElasticChannels));
    plastic.initializeDefaultElementChannels(
      mesh, element, elementChannelSpan(plasticValues, element, numPlasticChannels));
  }
  return makeMaterialParameters(std::move(space), std::move(elasticValues), std::move(plasticValues));
}

}  // namespace pgo::SolidDeformationModel
