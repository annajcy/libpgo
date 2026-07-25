#include "materialChannelMapping.h"

#include <algorithm>
#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace
{
void validateIdentityInput(
  int numChannels,
  std::span<const double> localDofValues,
  std::span<double> materialValues)
{
  if (localDofValues.size() != static_cast<std::size_t>(numChannels))
    throw std::invalid_argument("IdentityMaterialChannelMapping local DOF count mismatch.");
  if (materialValues.size() != static_cast<std::size_t>(numChannels))
    throw std::invalid_argument("IdentityMaterialChannelMapping material channel count mismatch.");
}
}  // namespace

IdentityMaterialChannelMapping::IdentityMaterialChannelMapping(int numChannels):
  numChannels_(numChannels)
{
  if (numChannels < 0)
    throw std::invalid_argument("IdentityMaterialChannelMapping requires a non-negative channel count.");
}

void IdentityMaterialChannelMapping::evaluate(
  int, int,
  std::span<const double> localDofValues,
  std::span<double> materialValues) const
{
  validateIdentityInput(numChannels_, localDofValues, materialValues);
  std::copy(localDofValues.begin(), localDofValues.end(), materialValues.begin());
}

void IdentityMaterialChannelMapping::evaluateJacobian(
  int, int,
  std::span<const double> localDofValues,
  double *output) const
{
  if (localDofValues.size() != static_cast<std::size_t>(numChannels_))
    throw std::invalid_argument("IdentityMaterialChannelMapping local DOF count mismatch.");
  if (numChannels_ > 0 && output == nullptr)
    throw std::invalid_argument("IdentityMaterialChannelMapping requires a Jacobian output buffer.");
  if (numChannels_ > 0)
    std::fill(
      output,
      output + static_cast<std::ptrdiff_t>(numChannels_) * numChannels_,
      0.0);
  for (int i = 0; i < numChannels_; i++)
    output[static_cast<std::size_t>(i) * numChannels_ + i] = 1.0;
}

void IdentityMaterialChannelMapping::evaluateHessians(
  int, int,
  std::span<const double> localDofValues,
  double *output) const
{
  if (localDofValues.size() != static_cast<std::size_t>(numChannels_))
    throw std::invalid_argument("IdentityMaterialChannelMapping local DOF count mismatch.");
  const std::size_t count =
    static_cast<std::size_t>(numChannels_) * numChannels_ * numChannels_;
  if (count > 0 && output == nullptr)
    throw std::invalid_argument("IdentityMaterialChannelMapping requires a Hessian output buffer.");
  if (count > 0)
    std::fill(output, output + static_cast<std::ptrdiff_t>(count), 0.0);
}

}  // namespace pgo::SolidDeformationModel
