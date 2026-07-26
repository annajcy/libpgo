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
  EigenSupport::RefMatXd jacobian) const
{
  if (localDofValues.size() != static_cast<std::size_t>(numChannels_))
    throw std::invalid_argument("IdentityMaterialChannelMapping local DOF count mismatch.");
  if (jacobian.rows() != numChannels_ || jacobian.cols() != numChannels_)
    throw std::invalid_argument("IdentityMaterialChannelMapping Jacobian size mismatch.");
  jacobian.setIdentity();
}

void IdentityMaterialChannelMapping::evaluateHessians(
  int, int,
  std::span<const double> localDofValues,
  std::span<EigenSupport::MXd> channelHessians) const
{
  if (localDofValues.size() != static_cast<std::size_t>(numChannels_))
    throw std::invalid_argument("IdentityMaterialChannelMapping local DOF count mismatch.");
  if (channelHessians.size() != static_cast<std::size_t>(numChannels_))
    throw std::invalid_argument("IdentityMaterialChannelMapping Hessian size mismatch.");
  for (EigenSupport::MXd &hessian : channelHessians) {
    if (hessian.rows() != numChannels_ || hessian.cols() != numChannels_)
      throw std::invalid_argument("IdentityMaterialChannelMapping Hessian shape mismatch.");
    hessian.setZero();
  }
}

}  // namespace pgo::SolidDeformationModel
