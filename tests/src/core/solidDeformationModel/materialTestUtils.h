#pragma once

#include "energy/deformationEnergyOperator.h"
#include "energy/deformationPotentialEnergy.h"
#include "material/runtime/materialBinding.h"
#include "material/frame/materialFrames.h"
#include "material/runtime/materialState.h"
#include "simulation/simulationMesh.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace pgo::SolidDeformationModel::TestUtils
{

struct TestAsset
{
  std::shared_ptr<const SimulationMesh> simulationMesh;
  std::unordered_map<std::string, EigenSupport::VXd> elementChannels;

  const std::shared_ptr<const SimulationMesh> &mesh() const
  {
    return simulationMesh;
  }
};

inline std::shared_ptr<const TestAsset> makeAsset(
  std::shared_ptr<const SimulationMesh> mesh,
  std::vector<std::string> channelNames,
  std::vector<double> elementMajorValues)
{
  if (!mesh)
    throw std::invalid_argument("test asset requires a mesh");
  const int numElements = mesh->getNumElements();
  const int numChannels = static_cast<int>(channelNames.size());
  if (static_cast<int>(elementMajorValues.size()) !=
      numElements * numChannels)
    throw std::invalid_argument("test material channel data has wrong length");
  TestAsset asset{std::move(mesh), {}};
  for (int channel = 0; channel < numChannels; ++channel) {
    EigenSupport::VXd values(numElements);
    for (int element = 0; element < numElements; ++element)
      values[element] = elementMajorValues[
        static_cast<std::size_t>(element) * numChannels + channel];
    asset.elementChannels.emplace(
      std::move(channelNames[static_cast<std::size_t>(channel)]),
      std::move(values));
  }
  return std::make_shared<const TestAsset>(std::move(asset));
}

inline std::shared_ptr<const TestAsset> makeENuAsset(
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

inline std::shared_ptr<const TestAsset> makeENuhAsset(
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
  const TestAsset &asset, int element, std::string_view requested)
{
  const auto iter = asset.elementChannels.find(std::string(requested));
  if (iter != asset.elementChannels.end() && element >= 0 &&
      element < iter->second.size())
    return iter->second[element];
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

inline std::shared_ptr<const TestAsset> shareAsset(
  std::shared_ptr<const SimulationMesh> mesh)
{
  return makeENuAsset(std::move(mesh), 1e6, 0.45);
}

inline double defaultPlasticValue(std::string_view channel)
{
  if (channel == "stretch" || channel == "Fx" || channel == "Fy" ||
    channel == "Fz" || channel == "Fxx" || channel == "Fyy" ||
    channel == "Fzz")
    return 1.0;
  return 0.0;
}

inline std::vector<std::string> fixedChannelNamesForTest(
  std::string_view id)
{
  if (id == "stable_neo" || id == "stable_neo_principal_stretch" ||
    id == "neo_hookean" || id == "stvk" || id == "linear")
    return {"E", "nu"};
  if (id == "stvk_vol" || id == "inv_stvk")
    return {"E", "nu", "J"};
  if (id == "mooney_rivlin")
    return {"mu01", "mu10", "v1"};
  if (id == "volume")
    return {"J"};
  if (id == "hill_stable_neo")
    return {"E", "nu", "Eact", "gamma", "lo"};
  if (id == "hill_stvk" || id == "hill_stvk_vol")
    return {"E", "nu", "J", "Eact", "gamma", "lo"};
  return {};
}

inline std::vector<std::string> optimizableChannelNamesForTest(
  const ElasticModelDefinition &definition)
{
  if (definition.id() == "koiter_stvk")
    return {"E_membrane", "nu_membrane", "E_bending", "nu_bending", "thickness"};
  if (definition.id() == "koiter_fabric")
    return {"mu0", "k1_4", "k2_4", "k1_6", "k2_6", "ks",
      "alpha", "kappa11", "kappa22", "kappa12", "I8_0", "h"};
  if (definition.id() == "hill_stable_neo" ||
    definition.id() == "hill_stvk" || definition.id() == "hill_stvk_vol")
    return {"activation"};
  if (definition.id() == "systematic_poking") {
    std::vector<std::string> names;
    for (int i = 0; i + 1 < definition.numOptimizableChannels(); ++i)
      names.push_back("f_dd_" + std::to_string(i));
    names.push_back("lambda");
    return names;
  }
  return {};
}

inline std::vector<std::string> optimizableChannelNamesForTest(
  const PlasticModelDefinition &definition)
{
  if (definition.id() == "shell_ff_dof1") return {"stretch"};
  if (definition.id() == "volumetric_dof3") return {"Fx", "Fy", "Fz"};
  if (definition.id() == "volumetric_dof6")
    return {"Fxx", "Fxy", "Fxz", "Fyy", "Fyz", "Fzz"};
  return {};
}

inline std::shared_ptr<MaterialState> makeDefaultMaterialState(
  const TestAsset &asset,
  const ElasticModelDefinition &elastic,
  const PlasticModelDefinition &plastic)
{
  const int numElements = asset.mesh()->getNumElements();
  const int numElastic = elastic.numOptimizableChannels();
  const int numPlastic = plastic.numOptimizableChannels();
  EigenSupport::VXd elasticValues(numElements * numElastic);
  const auto elasticNames = optimizableChannelNamesForTest(elastic);
  for (int element = 0; element < numElements; ++element)
    for (int channel = 0; channel < numElastic; ++channel)
      elasticValues[element * numElastic + channel] =
        importedValue(asset, element, elasticNames[channel]);

  EigenSupport::VXd plasticValues(numElements * numPlastic);
  const auto plasticNames = optimizableChannelNamesForTest(plastic);
  for (int element = 0; element < numElements; ++element)
    for (int channel = 0; channel < numPlastic; ++channel)
      plasticValues[element * numPlastic + channel] =
        defaultPlasticValue(plasticNames[channel]);

  return std::make_shared<MaterialState>(
    std::move(elasticValues), std::move(plasticValues));
}

struct MaterialBindingAndState
{
  std::shared_ptr<const MaterialBinding> binding;
  std::shared_ptr<MaterialState> state;
};

inline MaterialBindingAndState makeMaterialBinding(
  std::shared_ptr<const TestAsset> asset,
  std::shared_ptr<const ElasticModelDefinition> elastic,
  std::shared_ptr<const PlasticModelDefinition> plastic,
  std::shared_ptr<MaterialState> state = {},
  std::optional<MaterialFrames> frames = std::nullopt)
{
  if (!asset || !elastic || !plastic)
    throw std::invalid_argument(
      "test material binding requires asset and model definitions");

  const int numElements = asset->mesh()->getNumElements();
  auto projectFixed = [&](const auto &definition) {
    const auto names = fixedChannelNamesForTest(definition.id());
    EigenSupport::VXd values(numElements * names.size());
    for (int element = 0; element < numElements; ++element)
      for (int channel = 0; channel < static_cast<int>(names.size()); ++channel)
        values[element * names.size() + channel] =
          importedValue(
            *asset, element, names[static_cast<std::size_t>(channel)]);
    return values;
  };
  if (!state)
    state = makeDefaultMaterialState(*asset, *elastic, *plastic);
  auto binding = std::make_shared<const MaterialBinding>(
    ElasticMaterialBinding(
      elastic, numElements, projectFixed(*elastic)),
    PlasticMaterialBinding(
      plastic, numElements, projectFixed(*plastic)),
    std::move(frames));
  return { std::move(binding), std::move(state) };
}

inline std::shared_ptr<DeformationPotentialEnergy> makeTestEnergy(
  std::shared_ptr<const TestAsset> asset,
  const Formulation &formulation,
  std::shared_ptr<const ElasticModelDefinition> elastic,
  std::shared_ptr<const PlasticModelDefinition> plastic,
  std::shared_ptr<MaterialState> parameters = {},
  std::optional<MaterialFrames> frames = std::nullopt,
  const DeformationModelOptions &options = {})
{
  auto result = makeMaterialBinding(
    asset, std::move(elastic), std::move(plastic),
    std::move(parameters), std::move(frames));
  auto energyOperator = std::make_shared<DeformationEnergyOperator>(
    *asset->mesh(), *result.binding, formulation, options);
  return std::make_shared<DeformationPotentialEnergy>(
    std::move(energyOperator), *result.state);
}

}  // namespace pgo::SolidDeformationModel::TestUtils
