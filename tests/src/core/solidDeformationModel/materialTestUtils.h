#pragma once

#include "energy/deformationModelEnergy.h"
#include "material/runtime/materialAssignment.h"
#include "material/parameterization/materialChannelMapping.h"
#include "material/frame/materialFrameField.h"
#include "material/data/materialParameterData.h"
#include "material/data/namedMaterialInputData.h"
#include "material/parameterization/materialParameterization.h"
#include "material/runtime/optimizableParameters.h"
#include "material/parameterization/parameterLayout.h"
#include "simulation/import/simulationImportResult.h"
#include "simulation/simulationMesh.h"

#include <algorithm>
#include <memory>
#include <numeric>
#include <span>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace pgo::SolidDeformationModel::TestUtils
{

inline std::shared_ptr<const SimulationImportResult> shareAsset(
  std::unique_ptr<SimulationImportResult> asset)
{
  return std::shared_ptr<const SimulationImportResult>(std::move(asset));
}

inline std::shared_ptr<const SimulationImportResult> shareAsset(
  std::shared_ptr<const SimulationMesh> mesh,
  ImportedMaterialCatalog materialData)
{
  return std::make_shared<const SimulationImportResult>(
    std::move(mesh), std::move(materialData));
}

inline ImportedMaterialCatalog uniformImportedMaterialCatalog(
  int numElements,
  std::vector<std::string> channelNames,
  std::vector<double> channelValues,
  std::string materialName = "test_uniform_material")
{
  ImportedMaterialRecord material;
  material.name = std::move(materialName);
  material.family = "test";
  for (int channel = 0;
       channel < static_cast<int>(channelValues.size());
       ++channel)
    material.properties.emplace(
      std::move(channelNames[static_cast<std::size_t>(channel)]),
      channelValues[static_cast<std::size_t>(channel)]);
  std::vector<int> elements(static_cast<std::size_t>(numElements));
  std::iota(elements.begin(), elements.end(), 0);
  return ImportedMaterialCatalog(
    numElements, {std::move(material)},
    {ImportedElementSet{"all", std::move(elements)}},
    {ImportedMaterialRegion{0, 0}});
}

inline std::shared_ptr<const FixedParameterField> emptyFixedField(int numElements)
{
  return std::make_shared<const FixedParameterField>(
    ParameterInputSchema{},
    std::make_shared<const ElementwiseParameterLayout>(numElements, 0),
    std::make_shared<const IdentityMaterialChannelMapping>(0));
}

inline std::shared_ptr<const SimulationImportResult> makeAsset(
  std::shared_ptr<const SimulationMesh> mesh,
  std::vector<std::string> importedChannelNames,
  std::vector<double> importedValues)
{
  const int numElements = mesh->getNumElements();
  const int numChannels = static_cast<int>(importedChannelNames.size());
  std::vector<ImportedMaterialRecord> materials;
  std::vector<ImportedElementSet> sets;
  std::vector<ImportedMaterialRegion> regions;
  materials.reserve(static_cast<std::size_t>(numElements));
  sets.reserve(static_cast<std::size_t>(numElements));
  regions.reserve(static_cast<std::size_t>(numElements));
  for (int element = 0; element < numElements; ++element)
  {
    ImportedMaterialRecord material;
    material.name = "test_" + std::to_string(element);
    material.family = "test";
    for (int channel = 0; channel < numChannels; ++channel)
      material.properties.emplace(
        importedChannelNames[static_cast<std::size_t>(channel)],
        importedValues[
          static_cast<std::size_t>(element) * numChannels + channel]);
    materials.push_back(std::move(material));
    sets.push_back(ImportedElementSet{
      "element_" + std::to_string(element), {element}});
    regions.push_back(ImportedMaterialRegion{element, element});
  }
  return std::make_shared<const SimulationImportResult>(
    std::move(mesh), ImportedMaterialCatalog(
      numElements, std::move(materials), std::move(sets),
      std::move(regions)));
}

inline std::shared_ptr<const SimulationImportResult> makeENuAsset(
  std::shared_ptr<const SimulationMesh> mesh,
  double E, double nu, double compressionRatio = 10000.0)
{
  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(mesh->getNumElements()) * 3);
  for (int element = 0; element < mesh->getNumElements(); ++element) {
    values.push_back(E);
    values.push_back(nu);
    values.push_back(compressionRatio);
  }
  return makeAsset(
    std::move(mesh), {"E", "nu", "J"}, std::move(values));
}

inline std::shared_ptr<const SimulationImportResult> makeENuhAsset(
  std::shared_ptr<const SimulationMesh> mesh,
  double E, double nu, double thickness,
  double compressionRatio = 10000.0)
{
  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(mesh->getNumElements()) * 4);
  for (int element = 0; element < mesh->getNumElements(); ++element) {
    values.push_back(E);
    values.push_back(nu);
    values.push_back(thickness);
    values.push_back(compressionRatio);
  }
  return makeAsset(
    std::move(mesh), {"E", "nu", "h", "J"}, std::move(values));
}

inline double importedValue(
  const SimulationImportResult &asset, int element, std::string_view requested)
{
  const auto assignments = asset.materialCatalog().elementMaterialIndices();
  if (element >= 0 && element < static_cast<int>(assignments.size()) &&
      assignments[static_cast<std::size_t>(element)] >= 0) {
    const auto &material = asset.materialCatalog().materials()[static_cast<std::size_t>(
      assignments[static_cast<std::size_t>(element)])];
    const auto iter = material.properties.find(std::string(requested));
    if (iter != material.properties.end()) {
      if (const auto *value = std::get_if<double>(&iter->second))
        return *value;
    }
  }
  if (requested == "E_membrane" || requested == "E_bending")
    return importedValue(asset, element, "E");
  if (requested == "nu_membrane" || requested == "nu_bending")
    return importedValue(asset, element, "nu");
  if (requested == "thickness")
    return importedValue(asset, element, "h");
  throw std::invalid_argument(
    "test asset cannot initialize optimizable channel " +
    std::string(requested));
}

inline double defaultPlasticValue(std::string_view channel)
{
  if (channel == "stretch" || channel == "Fx" || channel == "Fy" ||
    channel == "Fz" || channel == "Fxx" || channel == "Fyy" ||
    channel == "Fzz")
    return 1.0;
  return 0.0;
}

inline ParameterInputSchema identityParameterSchema(
  const MaterialChannelSchema &channelSchema)
{
  const auto names = channelSchema.channelNames();
  return ParameterInputSchema(
    std::vector<std::string>(names.begin(), names.end()));
}

inline std::shared_ptr<OptimizableParameters> makeDefaultOptimizableParameters(
  const SimulationImportResult &asset,
  const ElasticModelDefinition &elastic,
  const PlasticModelDefinition &plastic)
{
  const int numElements = asset.mesh()->getNumElements();
  const MaterialChannelSchema elasticSchema =
    elastic.optimizableChannelSchema();
  const MaterialChannelSchema plasticSchema =
    plastic.optimizableChannelSchema();
  const int numElastic = elasticSchema.numChannels();
  const int numPlastic = plasticSchema.numChannels();

  auto elasticField = std::make_shared<const OptimizableParameterField>(
    identityParameterSchema(elasticSchema),
    std::make_shared<ElementwiseParameterLayout>(numElements, numElastic),
    std::make_shared<IdentityMaterialChannelMapping>(numElastic));
  auto plasticField = std::make_shared<const OptimizableParameterField>(
    identityParameterSchema(plasticSchema),
    std::make_shared<ElementwiseParameterLayout>(numElements, numPlastic),
    std::make_shared<IdentityMaterialChannelMapping>(numPlastic));
  EigenSupport::VXd elasticValues(numElements * numElastic);
  const auto elasticNames = elasticSchema.channelNames();
  for (int element = 0; element < numElements; ++element)
    for (int channel = 0; channel < numElastic; ++channel)
      elasticValues[element * numElastic + channel] =
        importedValue(asset, element, elasticNames[channel]);

  EigenSupport::VXd plasticValues(numElements * numPlastic);
  const auto plasticNames = plasticSchema.channelNames();
  for (int element = 0; element < numElements; ++element)
    for (int channel = 0; channel < numPlastic; ++channel)
      plasticValues[element * numPlastic + channel] =
        defaultPlasticValue(plasticNames[channel]);

  return std::make_shared<OptimizableParameters>(
    std::move(elasticField), std::move(plasticField),
    std::move(elasticValues), std::move(plasticValues));
}

inline std::shared_ptr<const MaterialAssignment> makeMaterialAssignment(
  std::shared_ptr<const SimulationImportResult> asset,
  std::shared_ptr<const ElasticModelDefinition> elastic,
  std::shared_ptr<const PlasticModelDefinition> plastic,
  std::shared_ptr<OptimizableParameters> parameters = {},
  std::shared_ptr<const MaterialFrameField> frames = {})
{
  if (!asset || !elastic || !plastic)
    throw std::invalid_argument(
      "test material assignment requires asset and model definitions");

  const int numElements = asset->mesh()->getNumElements();
  auto makeField = [&](const auto &definition) {
    const auto schema = definition.fixedChannelSchema();
    const auto names = schema.channelNames();
    std::vector<std::string> inputNames(names.begin(), names.end());
    return std::make_shared<const FixedParameterField>(
      ParameterInputSchema(std::move(inputNames)),
      std::make_shared<const ElementwiseParameterLayout>(
        numElements, schema.numChannels()),
      std::make_shared<const IdentityMaterialChannelMapping>(schema.numChannels()));
  };
  auto elasticFixed = makeField(*elastic);
  auto plasticFixed = makeField(*plastic);
  if (!parameters)
    parameters = makeDefaultOptimizableParameters(
      *asset, *elastic, *plastic);
  if (!frames)
    frames = std::make_shared<const GlobalAxesMaterialFrameField>(
      asset->mesh()->getNumElements());

  auto elasticOptimizable = parameters->elasticFieldHandle();
  auto plasticOptimizable = parameters->plasticFieldHandle();
  auto parameterization = std::make_shared<const MaterialParameterization>(
    ElasticParameterization(elastic, elasticFixed, elasticOptimizable),
    PlasticParameterization(plastic, plasticFixed, plasticOptimizable));
  MaterialParameterData data;
  const auto projectFixed = [&](const auto &field) {
    EigenSupport::VXd values(field->layout().numGlobalParameters());
    const auto names = field->inputSchema().parameterNames();
    for (int element = 0; element < numElements; ++element)
      for (int channel = 0; channel < static_cast<int>(names.size()); ++channel)
        values[field->layout().globalParameter(element, channel)] =
          importedValue(*asset, element, names[static_cast<std::size_t>(channel)]);
    return values;
  };
  data.elastic.fixedValues = projectFixed(elasticFixed);
  data.plastic.fixedValues = projectFixed(plasticFixed);
  data.elastic.initialOptimizableValues = parameters->elasticSnapshot();
  data.plastic.initialOptimizableValues = parameters->plasticSnapshot();
  return std::make_shared<const MaterialAssignment>(
    asset->mesh(), std::move(parameterization),
    std::make_shared<const MaterialParameterData>(std::move(data)),
    std::move(frames));
}

inline std::shared_ptr<DeformationModelEnergy> makeTestEnergy(
  std::shared_ptr<const SimulationImportResult> asset,
  const Formulation &formulation,
  std::shared_ptr<const ElasticModelDefinition> elastic,
  std::shared_ptr<const PlasticModelDefinition> plastic,
  std::shared_ptr<OptimizableParameters> parameters = {},
  std::shared_ptr<const MaterialFrameField> frames = {},
  const DeformationModelOptions &options = {})
{
  return std::make_shared<DeformationModelEnergy>(
    makeMaterialAssignment(
      std::move(asset), std::move(elastic), std::move(plastic),
      std::move(parameters), std::move(frames)),
    formulation, options);
}

}  // namespace pgo::SolidDeformationModel::TestUtils
