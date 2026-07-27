#pragma once

#include "energy/deformationModelEnergy.h"
#include "material/core/materialAssignment.h"
#include "material/core/materialEvaluator.h"
#include "material/core/materialFrameField.h"
#include "material/core/materialParameterData.h"
#include "material/core/materialParameterization.h"
#include "material/core/optimizableParameters.h"
#include "material/core/parameterLayout.h"
#include "simulation/simulationAsset.h"
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

inline std::shared_ptr<const SimulationAsset> shareAsset(
  std::unique_ptr<SimulationAsset> asset)
{
  return std::shared_ptr<const SimulationAsset>(std::move(asset));
}

inline std::shared_ptr<const FixedParameterField> emptyFixedField(int numElements)
{
  return std::make_shared<const FixedParameterField>(
    ParameterSchema{},
    std::make_shared<const ElementwiseParameterLayout>(numElements, 0),
    std::make_shared<const IdentityMaterialEvaluator>(0));
}

inline std::shared_ptr<const SimulationAsset> makeAsset(
  std::shared_ptr<const SimulationMesh> mesh,
  std::vector<std::string> importedChannelNames,
  std::vector<double> importedValues)
{
  const int numElements = mesh->getNumElements();
  const int numChannels = static_cast<int>(importedChannelNames.size());
  EigenSupport::MXd rows(numElements, numChannels);
  for (int element = 0; element < numElements; ++element)
    for (int channel = 0; channel < numChannels; ++channel)
      rows(element, channel) = importedValues[
        static_cast<std::size_t>(element) * numChannels + channel];
  std::vector<int> elementToRow(static_cast<std::size_t>(numElements));
  std::iota(elementToRow.begin(), elementToRow.end(), 0);
  return std::make_shared<const SimulationAsset>(
    std::move(mesh), ImportedMaterialData(
      numElements, {}, {}, {},
      {ImportedMaterialField(
        std::move(importedChannelNames), std::move(rows),
        std::move(elementToRow), "test_field")}));
}

inline std::shared_ptr<const SimulationAsset> makeENuAsset(
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

inline std::shared_ptr<const SimulationAsset> makeENuhAsset(
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
  const SimulationAsset &asset, int element, std::string_view requested)
{
  for (const auto &field : asset.materialData().fields()) {
    const auto names = field.channelNames();
    const int row = field.rowForElement(element);
    for (int channel = 0; channel < static_cast<int>(names.size()); ++channel)
      if (names[channel] == requested)
        return field.valueRows()(row, channel);
  }
  const auto assignments = asset.materialData().elementMaterialIndices();
  if (element >= 0 && element < static_cast<int>(assignments.size()) &&
      assignments[static_cast<std::size_t>(element)] >= 0) {
    const auto &material = asset.materialData().materials()[static_cast<std::size_t>(
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

inline ParameterSchema identityParameterSchema(
  const MaterialChannelSchema &channelSchema)
{
  const auto names = channelSchema.channelNames();
  return ParameterSchema(
    std::vector<std::string>(names.begin(), names.end()));
}

inline std::shared_ptr<OptimizableParameters> makeDefaultOptimizableParameters(
  const SimulationAsset &asset,
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
    std::make_shared<IdentityMaterialEvaluator>(numElastic));
  auto plasticField = std::make_shared<const OptimizableParameterField>(
    identityParameterSchema(plasticSchema),
    std::make_shared<ElementwiseParameterLayout>(numElements, numPlastic),
    std::make_shared<IdentityMaterialEvaluator>(numPlastic));
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
  std::shared_ptr<const SimulationAsset> asset,
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
      ParameterSchema(std::move(inputNames)),
      std::make_shared<const ElementwiseParameterLayout>(
        numElements, schema.numChannels()),
      std::make_shared<const IdentityMaterialEvaluator>(schema.numChannels()));
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
    const auto names = field->parameterSchema().parameterNames();
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
  std::shared_ptr<const SimulationAsset> asset,
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
