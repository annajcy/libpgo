#include "parameterLayout.h"

#include <algorithm>
#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace
{
void validateShape(int numElements, int numLocalParameters)
{
  if (numElements < 0)
    throw std::invalid_argument("ParameterLayout requires a non-negative element count.");
  if (numLocalParameters < 0)
    throw std::invalid_argument(
      "ParameterLayout requires a non-negative local parameter count.");
}

void validateElement(int element, int numElements)
{
  if (element < 0 || element >= numElements)
    throw std::out_of_range("ParameterLayout element index is out of range.");
}

void validateLocalParameter(int localParameter, int numLocalParameters)
{
  if (localParameter < 0 || localParameter >= numLocalParameters)
    throw std::out_of_range(
      "ParameterLayout local parameter index is out of range.");
}

void validateGatherSpans(
  std::span<const double> globalParameters,
  std::span<double> localParameters,
  int numGlobalParameters,
  int numLocalParameters)
{
  if (globalParameters.size() !=
    static_cast<std::size_t>(numGlobalParameters))
    throw std::invalid_argument(
      "ParameterLayout global parameter count does not match the layout.");
  if (localParameters.size() != static_cast<std::size_t>(numLocalParameters))
    throw std::invalid_argument(
      "ParameterLayout local parameter count does not match the layout.");
}
}  // namespace

ConstantParameterLayout::ConstantParameterLayout(
  int numElements, int numLocalParameters):
  numElements_(numElements), numLocalParameters_(numLocalParameters)
{
  validateShape(numElements, numLocalParameters);
}

int ConstantParameterLayout::globalParameter(
  int element, int localParameter) const
{
  validateElement(element, numElements_);
  validateLocalParameter(localParameter, numLocalParameters_);
  return localParameter;
}

void ConstantParameterLayout::gather(
  int element,
  std::span<const double> globalParameters,
  std::span<double> localParameters) const
{
  validateElement(element, numElements_);
  validateGatherSpans(
    globalParameters, localParameters,
    numGlobalParameters(), numLocalParameters_);
  std::copy(
    globalParameters.begin(), globalParameters.end(), localParameters.begin());
}

ElementwiseParameterLayout::ElementwiseParameterLayout(
  int numElements, int numLocalParameters):
  numElements_(numElements), numLocalParameters_(numLocalParameters)
{
  validateShape(numElements, numLocalParameters);
}

int ElementwiseParameterLayout::globalParameter(
  int element, int localParameter) const
{
  validateElement(element, numElements_);
  validateLocalParameter(localParameter, numLocalParameters_);
  return element * numLocalParameters_ + localParameter;
}

void ElementwiseParameterLayout::gather(
  int element,
  std::span<const double> globalParameters,
  std::span<double> localParameters) const
{
  validateElement(element, numElements_);
  validateGatherSpans(
    globalParameters, localParameters,
    numGlobalParameters(), numLocalParameters_);
  const auto offset =
    static_cast<std::size_t>(element) * numLocalParameters_;
  std::copy_n(
    globalParameters.begin() + static_cast<std::ptrdiff_t>(offset),
    numLocalParameters_, localParameters.begin());
}

}  // namespace pgo::SolidDeformationModel
