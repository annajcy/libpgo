#include "material/parameterization/parameterLayout.h"

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

void ParameterLayout::gather(
  int element,
  std::span<const double> globalParameters,
  std::span<double> localParameters) const
{
  validateElement(element, numElements());
  validateGatherSpans(
    globalParameters, localParameters,
    numGlobalParameters(), numLocalParameters());
  for (int local = 0; local < numLocalParameters(); ++local) {
    const int global = globalParameter(element, local);
    if (global < 0 || global >= numGlobalParameters())
      throw std::logic_error(
        "ParameterLayout returned an out-of-range global parameter.");
    localParameters[static_cast<std::size_t>(local)] =
      globalParameters[static_cast<std::size_t>(global)];
  }
}

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

}  // namespace pgo::SolidDeformationModel
