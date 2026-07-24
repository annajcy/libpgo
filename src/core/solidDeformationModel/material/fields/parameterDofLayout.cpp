#include "parameterDofLayout.h"

#include <algorithm>
#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace
{
void validateShape(int numElements, int numLocalDofs)
{
  if (numElements < 0)
    throw std::invalid_argument("ParameterDofLayout requires a non-negative element count.");
  if (numLocalDofs < 0)
    throw std::invalid_argument("ParameterDofLayout requires a non-negative local DOF count.");
}

void validateElement(int element, int numElements)
{
  if (element < 0 || element >= numElements)
    throw std::out_of_range("ParameterDofLayout element index is out of range.");
}

void validateLocalDof(int localDof, int numLocalDofs)
{
  if (localDof < 0 || localDof >= numLocalDofs)
    throw std::out_of_range("ParameterDofLayout local DOF index is out of range.");
}

void validateGatherSpans(
  std::span<const double> globalValues,
  std::span<double> localDofValues,
  int numGlobalDofs,
  int numLocalDofs)
{
  if (globalValues.size() != static_cast<std::size_t>(numGlobalDofs))
    throw std::invalid_argument("ParameterDofLayout global value count does not match the layout.");
  if (localDofValues.size() != static_cast<std::size_t>(numLocalDofs))
    throw std::invalid_argument("ParameterDofLayout local value count does not match the layout.");
}
}  // namespace

ConstantParameterDofLayout::ConstantParameterDofLayout(
  int numElements, int numLocalDofs):
  numElements_(numElements), numLocalDofs_(numLocalDofs)
{
  validateShape(numElements, numLocalDofs);
}

int ConstantParameterDofLayout::globalDof(int element, int localDof) const
{
  validateElement(element, numElements_);
  validateLocalDof(localDof, numLocalDofs_);
  return localDof;
}

void ConstantParameterDofLayout::gather(
  int element,
  std::span<const double> globalValues,
  std::span<double> localDofValues) const
{
  validateElement(element, numElements_);
  validateGatherSpans(globalValues, localDofValues, numGlobalDofs(), numLocalDofs_);
  std::copy(globalValues.begin(), globalValues.end(), localDofValues.begin());
}

EigenSupport::VXd ConstantParameterDofLayout::globalValuesFromElementDefaults(
  std::span<const double> elementValues) const
{
  const std::size_t expected = static_cast<std::size_t>(numElements_) * numLocalDofs_;
  if (elementValues.size() != expected)
    throw std::invalid_argument("ConstantParameterDofLayout default value count does not match the layout.");
  if (numLocalDofs_ == 0)
    return EigenSupport::VXd();
  EigenSupport::VXd result(numLocalDofs_);
  std::copy_n(elementValues.begin(), numLocalDofs_, result.data());
  return result;
}

ElementwiseParameterDofLayout::ElementwiseParameterDofLayout(
  int numElements, int numLocalDofs):
  numElements_(numElements), numLocalDofs_(numLocalDofs)
{
  validateShape(numElements, numLocalDofs);
}

int ElementwiseParameterDofLayout::globalDof(int element, int localDof) const
{
  validateElement(element, numElements_);
  validateLocalDof(localDof, numLocalDofs_);
  return element * numLocalDofs_ + localDof;
}

void ElementwiseParameterDofLayout::gather(
  int element,
  std::span<const double> globalValues,
  std::span<double> localDofValues) const
{
  validateElement(element, numElements_);
  validateGatherSpans(globalValues, localDofValues, numGlobalDofs(), numLocalDofs_);
  const auto offset = static_cast<std::size_t>(element) * numLocalDofs_;
  std::copy_n(globalValues.begin() + static_cast<std::ptrdiff_t>(offset),
    numLocalDofs_, localDofValues.begin());
}

EigenSupport::VXd ElementwiseParameterDofLayout::globalValuesFromElementDefaults(
  std::span<const double> elementValues) const
{
  const std::size_t expected = static_cast<std::size_t>(numElements_) * numLocalDofs_;
  if (elementValues.size() != expected)
    throw std::invalid_argument("ElementwiseParameterDofLayout default value count does not match the layout.");
  EigenSupport::VXd result(static_cast<Eigen::Index>(elementValues.size()));
  if (!elementValues.empty())
    std::copy(elementValues.begin(), elementValues.end(), result.data());
  return result;
}

}  // namespace pgo::SolidDeformationModel
