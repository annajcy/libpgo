#include "materialParameterDataProjection.h"

#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace pgo::SolidDeformationModel
{
namespace
{
std::vector<double> resolveNamedInputImpl(
  const ImportedMaterialData &source,
  std::string_view channel)
{
  const int numElements = source.numElements();
  int fieldIndex = -1;
  int fieldColumn = -1;
  for (int i = 0; i < static_cast<int>(source.fields().size()); ++i) {
    const auto &field = source.fields()[static_cast<std::size_t>(i)];
    const auto names = field.channelNames();
    for (int c = 0; c < static_cast<int>(names.size()); ++c) {
      if (names[static_cast<std::size_t>(c)] != channel)
        continue;
      if (fieldIndex >= 0)
        throw std::invalid_argument(
          "ImportedMaterialData channel '" + std::string(channel) +
          "' is ambiguous across spatial fields.");
      fieldIndex = i;
      fieldColumn = c;
    }
  }

  const auto materialIndices = source.elementMaterialIndices();
  bool hasMaterialProperty = false;
  for (const auto &material : source.materials()) {
    if (material.properties.find(std::string(channel)) != material.properties.end()) {
      if (fieldIndex >= 0)
        throw std::invalid_argument(
          "ImportedMaterialData channel '" + std::string(channel) +
          "' is ambiguous.");
      hasMaterialProperty = true;
    }
  }

  if (fieldIndex >= 0) {
    const auto &field = source.fields()[static_cast<std::size_t>(fieldIndex)];
    std::vector<double> values(static_cast<std::size_t>(numElements));
    for (int element = 0; element < numElements; ++element) {
      const int row = field.rowForElement(element);
      if (row < 0)
        throw std::invalid_argument(
          "ImportedMaterialData field is missing channel '" +
          std::string(channel) + "' for an element.");
      values[static_cast<std::size_t>(element)] =
        field.valueRows()(row, fieldColumn);
    }
    return values;
  }

  if (!hasMaterialProperty)
    throw std::invalid_argument(
      "ImportedMaterialData is missing required channel '" +
      std::string(channel) + "'.");

  std::vector<double> values(static_cast<std::size_t>(numElements));
  for (int element = 0; element < numElements; ++element) {
    const int materialIndex = materialIndices[static_cast<std::size_t>(element)];
    if (materialIndex < 0)
      throw std::invalid_argument(
        "ImportedMaterialData has no material assignment for element " +
        std::to_string(element) + ".");
    const auto &properties = source.materials()[static_cast<std::size_t>(materialIndex)].properties;
    const auto iter = properties.find(std::string(channel));
    if (iter == properties.end())
      throw std::invalid_argument(
        "ImportedMaterialData material assignment is missing channel '" +
        std::string(channel) + "'.");
    if (const auto *scalar = std::get_if<double>(&iter->second)) {
      values[static_cast<std::size_t>(element)] = *scalar;
    }
    else {
      const auto &vector = std::get<std::vector<double>>(iter->second);
      if (vector.size() != 1)
        throw std::invalid_argument(
          "ImportedMaterialData channel '" + std::string(channel) +
          "' is vector-valued and cannot initialize a scalar parameter.");
      values[static_cast<std::size_t>(element)] = vector.front();
    }
  }
  return values;
}

}  // namespace

std::vector<double> MaterialParameterDataProjection::resolveNamedInput(
  const ImportedMaterialData &source,
  std::string_view name)
{
  return resolveNamedInputImpl(source, name);
}

EigenSupport::VXd MaterialParameterDataProjection::packElementInputs(
  const ParameterLayout &layout,
  std::span<const double> elementLocalValues)
{
  const int numElements = layout.numElements();
  const int numLocalParameters = layout.numLocalParameters();
  const std::size_t expectedSize = static_cast<std::size_t>(numElements) *
    static_cast<std::size_t>(numLocalParameters);
  if (elementLocalValues.size() != expectedSize)
    throw std::invalid_argument(
      "MaterialParameterDataProjection element input count does not match the layout.");

  EigenSupport::VXd result = EigenSupport::VXd::Constant(
    layout.numGlobalParameters(), std::numeric_limits<double>::quiet_NaN());
  for (int element = 0; element < numElements; ++element) {
    for (int local = 0; local < numLocalParameters; ++local) {
      const double value = elementLocalValues[
        static_cast<std::size_t>(element) * numLocalParameters + local];
      if (!std::isfinite(value))
        throw std::invalid_argument(
          "MaterialParameterDataProjection values must be finite.");
      const int global = layout.globalParameter(element, local);
      if (std::isfinite(result[global]) &&
        std::abs(result[global] - value) > 1e-12 *
          std::max({1.0, std::abs(result[global]), std::abs(value)}))
        throw std::invalid_argument(
          "MaterialParameterDataProjection cannot reduce differing elementwise values to a shared parameter.");
      result[global] = value;
    }
  }
  for (Eigen::Index i = 0; i < result.size(); ++i) {
    if (!std::isfinite(result[i]))
      throw std::invalid_argument(
        "MaterialParameterDataProjection left a parameter value uninitialized.");
  }
  return result;
}

namespace
{
EigenSupport::VXd projectInputs(
  const ImportedMaterialData &source,
  std::span<const std::string> inputNames,
  const ParameterLayout &layout)
{
  if (source.numElements() != layout.numElements())
    throw std::invalid_argument(
      "MaterialParameterDataProjection source and target element counts differ.");
  if (layout.numLocalParameters() != static_cast<int>(inputNames.size()))
    throw std::invalid_argument(
      "MaterialParameterDataProjection field schema does not match its layout.");

  const int numElements = layout.numElements();
  const int numInputs = layout.numLocalParameters();
  std::vector<double> elementLocalValues(
    static_cast<std::size_t>(numElements) * numInputs);
  for (int input = 0; input < numInputs; ++input) {
    const auto values = MaterialParameterDataProjection::resolveNamedInput(
      source, inputNames[static_cast<std::size_t>(input)]);
    for (int element = 0; element < numElements; ++element)
      elementLocalValues[
        static_cast<std::size_t>(element) * numInputs + input] =
        values[static_cast<std::size_t>(element)];
  }
  return MaterialParameterDataProjection::packElementInputs(
    layout, elementLocalValues);
}
}  // namespace

MaterialParameterData NamedChannelMaterialParameterDataProjection::convert(
  const ImportedMaterialData &source,
  const MaterialParameterization &target) const
{
  MaterialParameterData result;
  const auto projectFixed = [&](const auto &domain) {
    const auto &field = *domain.fixedField();
    const auto names = field.parameterSchema().parameterNames();
    return projectInputs(source, names, domain.fixedField()->layout());
  };
  const auto projectOptimizable = [&](const auto &domain) {
    const auto names =
      domain.optimizableField()->parameterSchema().parameterNames();
    return projectInputs(
      source, names, domain.optimizableField()->layout());
  };
  result.elastic.fixedValues = projectFixed(target.elastic());
  result.elastic.initialOptimizableValues = projectOptimizable(target.elastic());
  result.plastic.fixedValues = projectFixed(target.plastic());
  result.plastic.initialOptimizableValues = projectOptimizable(target.plastic());
  return result;
}

}  // namespace pgo::SolidDeformationModel
