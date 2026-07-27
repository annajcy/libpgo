#include "material/parameterization/materialChannelMapping.h"

#include <algorithm>
#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace
{
void validateIdentityInput(
  int numChannels,
  std::span<const double> parameters,
  std::span<double> channels)
{
  if (parameters.size() != static_cast<std::size_t>(numChannels))
    throw std::invalid_argument("IdentityMaterialChannelMapping parameter count mismatch.");
  if (channels.size() != static_cast<std::size_t>(numChannels))
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
  std::span<const double> parameters,
  std::span<double> channels) const
{
  validateIdentityInput(numChannels_, parameters, channels);
  std::copy(parameters.begin(), parameters.end(), channels.begin());
}

void IdentityMaterialChannelMapping::evaluateJacobian(
  int, int,
  std::span<const double> parameters,
  EigenSupport::RefMatXd jacobian) const
{
  if (parameters.size() != static_cast<std::size_t>(numChannels_))
    throw std::invalid_argument("IdentityMaterialChannelMapping parameter count mismatch.");
  if (jacobian.rows() != numChannels_ || jacobian.cols() != numChannels_)
    throw std::invalid_argument("IdentityMaterialChannelMapping Jacobian size mismatch.");
  jacobian.setIdentity();
}

void IdentityMaterialChannelMapping::evaluateHessians(
  int, int,
  std::span<const double> parameters,
  std::span<EigenSupport::MXd> channelHessians) const
{
  if (parameters.size() != static_cast<std::size_t>(numChannels_))
    throw std::invalid_argument("IdentityMaterialChannelMapping parameter count mismatch.");
  if (channelHessians.size() != static_cast<std::size_t>(numChannels_))
    throw std::invalid_argument("IdentityMaterialChannelMapping Hessian size mismatch.");
  for (EigenSupport::MXd &hessian : channelHessians) {
    if (hessian.rows() != numChannels_ || hessian.cols() != numChannels_)
      throw std::invalid_argument("IdentityMaterialChannelMapping Hessian shape mismatch.");
    hessian.setZero();
  }
}

}  // namespace pgo::SolidDeformationModel
