#include "materialParameterFactory.h"

#include "simulation/simulationMesh.h"

#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace ES = EigenSupport;

namespace
{
ES::VXd defaultValues(
  const SimulationMesh &mesh,
  const auto &config,
  const ParameterDofLayout &layout,
  const ParameterFieldMapping &mapping)
{
  const int channels = mapping.numChannels();
  if (mapping.numInputDofs() != channels)
    throw std::invalid_argument(
      "makeMaterialParameters requires explicit values for a non-identity mapping.");

  ES::VXd perElement(static_cast<Eigen::Index>(mesh.getNumElements()) * channels);
  for (int element = 0; element < mesh.getNumElements(); element++) {
    config.initializeDefaultParameters(
      mesh, element,
      std::span<double>(perElement.data() + static_cast<std::ptrdiff_t>(element) * channels, channels));
  }
  return layout.globalValuesFromElementDefaults(
    std::span<const double>(perElement.data(), perElement.size()));
}

void validateLayout(const char *name, const ParameterDofLayout &layout, int numElements)
{
  if (layout.numElements() != numElements)
    throw std::invalid_argument(
      std::string("makeMaterialParameters ") + name +
      " layout element count does not match the mesh.");
}
}  // namespace

std::shared_ptr<MaterialParameters> makeMaterialParameters(
  const SimulationMesh &mesh,
  const ElasticModelConfig &elastic,
  std::shared_ptr<const ParameterDofLayout> elasticDofLayout,
  std::shared_ptr<const ParameterFieldMapping> elasticMapping,
  std::optional<ES::VXd> elasticValues,
  const PlasticModelConfig &plastic,
  std::shared_ptr<const ParameterDofLayout> plasticDofLayout,
  std::shared_ptr<const ParameterFieldMapping> plasticMapping,
  std::optional<ES::VXd> plasticValues)
{
  if (!elasticDofLayout || !elasticMapping || !plasticDofLayout || !plasticMapping)
    throw std::invalid_argument("makeMaterialParameters requires non-null layouts and mappings.");

  const int numElements = mesh.getNumElements();
  validateLayout("elastic", *elasticDofLayout, numElements);
  validateLayout("plastic", *plasticDofLayout, numElements);
  const auto elasticSpec = elastic.parameterSpec();
  const auto plasticSpec = plastic.parameterSpec();
  if (elasticMapping->numChannels() != static_cast<int>(elasticSpec.channelNames.size()))
    throw std::invalid_argument("elastic mapping channel count does not match the model config.");
  if (plasticMapping->numChannels() != static_cast<int>(plasticSpec.channelNames.size()))
    throw std::invalid_argument("plastic mapping channel count does not match the model config.");

  if (!elasticValues)
    elasticValues = defaultValues(mesh, elastic, *elasticDofLayout, *elasticMapping);
  if (!plasticValues)
    plasticValues = defaultValues(mesh, plastic, *plasticDofLayout, *plasticMapping);

  MaterialParameterBlock elasticBlock(
    elasticSpec.channelNames, std::move(elasticDofLayout), std::move(elasticMapping));
  MaterialParameterBlock plasticBlock(
    plasticSpec.channelNames, std::move(plasticDofLayout), std::move(plasticMapping));
  auto space = std::make_shared<MaterialParameterSpace>(
    std::move(elasticBlock), std::move(plasticBlock));
  return std::make_shared<MaterialParameters>(
    std::move(space), std::move(*elasticValues), std::move(*plasticValues));
}

std::shared_ptr<MaterialParameters> makeDefaultMaterialParameters(
  const SimulationMesh &mesh,
  const ElasticModelConfig &elastic,
  const PlasticModelConfig &plastic)
{
  const int numElements = mesh.getNumElements();
  const int numElasticChannels = static_cast<int>(elastic.parameterSpec().channelNames.size());
  const int numPlasticChannels = static_cast<int>(plastic.parameterSpec().channelNames.size());
  return makeMaterialParameters(
    mesh,
    elastic,
    std::make_shared<ElementwiseParameterDofLayout>(numElements, numElasticChannels),
    std::make_shared<IdentityParameterFieldMapping>(numElasticChannels),
    std::nullopt,
    plastic,
    std::make_shared<ElementwiseParameterDofLayout>(numElements, numPlasticChannels),
    std::make_shared<IdentityParameterFieldMapping>(numPlasticChannels),
    std::nullopt);
}

}  // namespace pgo::SolidDeformationModel
