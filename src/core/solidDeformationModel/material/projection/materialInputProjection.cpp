#include "material/projection/materialInputProjection.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace pgo::SolidDeformationModel
{
namespace
{

EigenSupport::VXd packElementInputs(
  const ParameterLayout &layout,
  std::span<const double> elementLocalValues)
{
  const int numElements = layout.numElements();
  const int numLocalParameters = layout.numLocalParameters();
  const std::size_t expectedSize = static_cast<std::size_t>(numElements) *
    static_cast<std::size_t>(numLocalParameters);
  if (elementLocalValues.size() != expectedSize)
    throw std::invalid_argument(
      "Material input count does not match the parameter layout.");

  EigenSupport::VXd result = EigenSupport::VXd::Constant(
    layout.numGlobalParameters(), std::numeric_limits<double>::quiet_NaN());
  for (int element = 0; element < numElements; ++element) {
    for (int local = 0; local < numLocalParameters; ++local) {
      const double value = elementLocalValues[
        static_cast<std::size_t>(element) * numLocalParameters + local];
      if (!std::isfinite(value))
        throw std::invalid_argument("Material input values must be finite.");
      const int global = layout.globalParameter(element, local);
      if (std::isfinite(result[global]) &&
        std::abs(result[global] - value) > 1e-12 *
          std::max({1.0, std::abs(result[global]), std::abs(value)}))
        throw std::invalid_argument(
          "Differing element values cannot initialize a shared parameter.");
      result[global] = value;
    }
  }
  for (Eigen::Index i = 0; i < result.size(); ++i) {
    if (!std::isfinite(result[i]))
      throw std::invalid_argument(
        "Material input projection left a parameter uninitialized.");
  }
  return result;
}

template<class Resolver>
EigenSupport::VXd projectInputs(
  int numSourceElements,
  const ParameterInputSchema &schema,
  const ParameterLayout &layout,
  Resolver &&resolve)
{
  const auto inputNames = schema.parameterNames();
  if (numSourceElements != layout.numElements())
    throw std::invalid_argument(
      "Material input source and target element counts differ.");
  if (layout.numLocalParameters() != static_cast<int>(inputNames.size()))
    throw std::invalid_argument(
      "Material input schema does not match its parameter layout.");

  const int numElements = layout.numElements();
  const int numInputs = layout.numLocalParameters();
  std::vector<double> elementLocalValues(
    static_cast<std::size_t>(numElements) * numInputs);
  for (int input = 0; input < numInputs; ++input) {
    const auto values =
      resolve(inputNames[static_cast<std::size_t>(input)]);
    for (int element = 0; element < numElements; ++element)
      elementLocalValues[
        static_cast<std::size_t>(element) * numInputs + input] =
        values[static_cast<std::size_t>(element)];
  }
  return packElementInputs(layout, elementLocalValues);
}

std::vector<double> resolveImportedInput(
  const ImportedMaterialCatalog &catalog,
  std::string_view channel)
{
  bool hasProperty = false;
  for (const auto &material : catalog.materials()) {
    if (material.properties.contains(std::string(channel)))
      hasProperty = true;
  }
  if (!hasProperty)
    throw std::invalid_argument(
      "Imported material catalog is missing channel '" +
      std::string(channel) + "'.");

  const auto assignments = catalog.elementMaterialIndices();
  std::vector<double> values(static_cast<std::size_t>(catalog.numElements()));
  for (int element = 0; element < catalog.numElements(); ++element) {
    const int materialIndex = assignments[static_cast<std::size_t>(element)];
    if (materialIndex < 0)
      throw std::invalid_argument(
        "Imported material catalog has no assignment for element " +
        std::to_string(element) + ".");
    const auto &properties =
      catalog.materials()[static_cast<std::size_t>(materialIndex)].properties;
    const auto iter = properties.find(std::string(channel));
    if (iter == properties.end())
      throw std::invalid_argument(
        "Imported material assignment is missing channel '" +
        std::string(channel) + "'.");
    const auto *scalar = std::get_if<double>(&iter->second);
    if (!scalar)
      throw std::invalid_argument(
        "Imported material channel '" + std::string(channel) +
        "' is vector-valued and cannot initialize a scalar parameter.");
    values[static_cast<std::size_t>(element)] = *scalar;
  }
  return values;
}

std::vector<double> resolveNamedInput(
  const NamedMaterialInputData &inputs,
  std::string_view channel)
{
  const NamedMaterialInputField *matchedField = nullptr;
  int matchedColumn = -1;
  for (const auto &field : inputs.fields()) {
    const auto names = field.channelNames();
    for (int column = 0; column < static_cast<int>(names.size()); ++column) {
      if (names[static_cast<std::size_t>(column)] != channel)
        continue;
      if (matchedField)
        throw std::invalid_argument(
          "Named material input channel '" + std::string(channel) +
          "' is ambiguous across fields.");
      matchedField = &field;
      matchedColumn = column;
    }
  }
  if (!matchedField)
    throw std::invalid_argument(
      "Named material input data is missing channel '" +
      std::string(channel) + "'.");

  std::vector<double> values(static_cast<std::size_t>(inputs.numElements()));
  for (int element = 0; element < inputs.numElements(); ++element) {
    const int row = matchedField->rowForElement(element);
    if (row < 0)
      throw std::invalid_argument(
        "Named material input field is missing channel '" +
        std::string(channel) + "' for an element.");
    values[static_cast<std::size_t>(element)] =
      matchedField->valueRows()(row, matchedColumn);
  }
  return values;
}

}  // namespace

EigenSupport::VXd projectImportedMaterialInputs(
  const ImportedMaterialCatalog &catalog,
  const ParameterInputSchema &schema,
  const ParameterLayout &layout)
{
  return projectInputs(
    catalog.numElements(), schema, layout,
    [&](std::string_view name) {
      return resolveImportedInput(catalog, name);
    });
}

EigenSupport::VXd projectNamedMaterialInputs(
  const NamedMaterialInputData &inputs,
  const ParameterInputSchema &schema,
  const ParameterLayout &layout)
{
  return projectInputs(
    inputs.numElements(), schema, layout,
    [&](std::string_view name) {
      return resolveNamedInput(inputs, name);
    });
}

}  // namespace pgo::SolidDeformationModel
