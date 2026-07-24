#include "materialParameterFactory.h"

#include "material/elastic/elasticModelFactory.h"
#include "material/plastic/plasticModelFactory.h"
#include "material/plastic/plasticModel.h"
#include "simulation/simulationMesh.h"

#include <stdexcept>
#include <vector>

namespace pgo::SolidDeformationModel
{
namespace ES = EigenSupport;
namespace
{
ES::VXd defaultElasticValues(
  const SimulationMesh &mesh,
  DeformationModelElasticMaterial material,
  const ParameterDofLayout &layout,
  const ParameterFieldMapping &mapping)
{
  const int channels = mapping.numChannels();
  if (dynamic_cast<const IdentityParameterFieldMapping *>(&mapping) == nullptr ||
    mapping.numInputDofs() != channels) {
    throw std::invalid_argument(
      "makeMaterialParameters requires explicit elastic values for a non-identity mapping.");
  }

  ES::VXd perElement =
    ElasticModelFactory::initializeDefaultElasticParams(mesh, material, channels);
  if (layout.kind() == ParameterDofLayoutKind::ELEMENTWISE)
    return perElement;
  if (layout.kind() == ParameterDofLayoutKind::CONSTANT)
    return channels == 0 ? ES::VXd() : ES::VXd(perElement.head(channels));
  throw std::invalid_argument("makeMaterialParameters received an unsupported elastic layout.");
}

ES::VXd defaultPlasticValues(
  const SimulationMesh &mesh,
  DeformationModelPlasticMaterial material,
  const ParameterDofLayout &layout,
  const ParameterFieldMapping &mapping)
{
  const int channels = mapping.numChannels();
  if (dynamic_cast<const IdentityParameterFieldMapping *>(&mapping) == nullptr ||
    mapping.numInputDofs() != channels) {
    throw std::invalid_argument(
      "makeMaterialParameters requires explicit plastic values for a non-identity mapping.");
  }

  const int numElements = mesh.getNumElements();
  std::vector<std::unique_ptr<PlasticModel>> ownedModels(numElements);
  std::vector<PlasticModel *> models(numElements);
  for (int ele = 0; ele < numElements; ele++) {
    ownedModels[ele] = PlasticModelFactory::create(material, MaterialFrame::Identity());
    models[ele] = ownedModels[ele].get();
  }
  ES::VXd perElement =
    PlasticModelFactory::initializeDefaultPlasticParams(
      numElements, channels, models.data());
  if (layout.kind() == ParameterDofLayoutKind::ELEMENTWISE)
    return perElement;
  if (layout.kind() == ParameterDofLayoutKind::CONSTANT)
    return channels == 0 ? ES::VXd() : ES::VXd(perElement.head(channels));
  throw std::invalid_argument("makeMaterialParameters received an unsupported plastic layout.");
}

void validateLayout(
  const char *name,
  const ParameterDofLayout &layout,
  int numElements)
{
  if (layout.numElements() != numElements)
    throw std::invalid_argument(
      std::string("makeMaterialParameters ") + name +
      " layout element count does not match the mesh.");
}
}  // namespace

std::shared_ptr<MaterialParameters> makeMaterialParameters(
  const SimulationMesh &mesh,
  DeformationModelElasticMaterial elastic,
  std::unique_ptr<const ParameterDofLayout> elasticDofLayout,
  std::unique_ptr<const ParameterFieldMapping> elasticMapping,
  std::optional<ES::VXd> elasticValues,
  DeformationModelPlasticMaterial plastic,
  std::unique_ptr<const ParameterDofLayout> plasticDofLayout,
  std::unique_ptr<const ParameterFieldMapping> plasticMapping,
  std::optional<ES::VXd> plasticValues)
{
  if (!elasticDofLayout || !elasticMapping ||
    !plasticDofLayout || !plasticMapping) {
    throw std::invalid_argument(
      "makeMaterialParameters requires non-null layouts and mappings.");
  }

  const int numElements = mesh.getNumElements();
  validateLayout("elastic", *elasticDofLayout, numElements);
  validateLayout("plastic", *plasticDofLayout, numElements);

  MaterialParameterSpec elasticSpec =
    ElasticModelFactory::parameterSpec(mesh, elastic);
  MaterialParameterSpec plasticSpec =
    PlasticModelFactory::parameterSpec(plastic);

  if (elasticMapping->numChannels() !=
    static_cast<int>(elasticSpec.channelNames.size())) {
    throw std::invalid_argument(
      "makeMaterialParameters elastic mapping channel count does not match the material model.");
  }
  if (plasticMapping->numChannels() !=
    static_cast<int>(plasticSpec.channelNames.size())) {
    throw std::invalid_argument(
      "makeMaterialParameters plastic mapping channel count does not match the material model.");
  }

  if (!elasticValues)
    elasticValues = defaultElasticValues(
      mesh, elastic, *elasticDofLayout, *elasticMapping);
  if (!plasticValues)
    plasticValues = defaultPlasticValues(
      mesh, plastic, *plasticDofLayout, *plasticMapping);

  MaterialParameterBlock elasticBlock(
    MaterialParameterBlockKind::ELASTIC,
    std::move(elasticSpec.modelId),
    std::move(elasticSpec.channelNames),
    std::move(elasticDofLayout),
    std::move(elasticMapping));
  MaterialParameterBlock plasticBlock(
    MaterialParameterBlockKind::PLASTIC,
    std::move(plasticSpec.modelId),
    std::move(plasticSpec.channelNames),
    std::move(plasticDofLayout),
    std::move(plasticMapping));
  auto space = std::make_shared<MaterialParameterSpace>(
    std::move(elasticBlock), std::move(plasticBlock));
  return std::make_shared<MaterialParameters>(
    std::move(space), std::move(*elasticValues), std::move(*plasticValues));
}

std::shared_ptr<MaterialParameters> makeDefaultMaterialParameters(
  const SimulationMesh &mesh,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic)
{
  const int numElements = mesh.getNumElements();
  const int numElasticChannels =
    static_cast<int>(ElasticModelFactory::parameterSpec(mesh, elastic).channelNames.size());
  const int numPlasticChannels =
    static_cast<int>(PlasticModelFactory::parameterSpec(plastic).channelNames.size());
  return makeMaterialParameters(
    mesh,
    elastic,
    std::make_unique<ElementwiseParameterDofLayout>(
      numElements, numElasticChannels),
    std::make_unique<IdentityParameterFieldMapping>(numElasticChannels),
    std::nullopt,
    plastic,
    std::make_unique<ElementwiseParameterDofLayout>(
      numElements, numPlasticChannels),
    std::make_unique<IdentityParameterFieldMapping>(numPlasticChannels),
    std::nullopt);
}

}  // namespace pgo::SolidDeformationModel
