#pragma once

#include "EigenSupport.h"

#include <span>

namespace pgo
{
namespace SolidDeformationModel
{

class MaterialChannelMapping
{
public:
  virtual ~MaterialChannelMapping() = default;

  virtual int numInputDofs() const = 0;
  virtual int numChannels() const = 0;
  virtual bool isAffine() const = 0;

  virtual void evaluate(
    int element,
    int quadrature,
    std::span<const double> localDofValues,
    std::span<double> materialValues) const = 0;

  // jacobian(channel, localDof) = d materialValues[channel] /
  //                                d localDofValues[localDof].
  virtual void evaluateJacobian(
    int element,
    int quadrature,
    std::span<const double> localDofValues,
    EigenSupport::RefMatXd jacobian) const = 0;

  // channelHessians[channel](i, j) is the second derivative of the
  // corresponding material channel with respect to local DOFs i and j.
  virtual void evaluateHessians(
    int element,
    int quadrature,
    std::span<const double> localDofValues,
    std::span<EigenSupport::MXd> channelHessians) const = 0;
};

class IdentityMaterialChannelMapping final : public MaterialChannelMapping
{
public:
  explicit IdentityMaterialChannelMapping(int numChannels);

  int numInputDofs() const override { return numChannels_; }
  int numChannels() const override { return numChannels_; }
  bool isAffine() const override { return true; }

  void evaluate(
    int element,
    int quadrature,
    std::span<const double> localDofValues,
    std::span<double> materialValues) const override;
  void evaluateJacobian(
    int element,
    int quadrature,
    std::span<const double> localDofValues,
    EigenSupport::RefMatXd jacobian) const override;
  void evaluateHessians(
    int element,
    int quadrature,
    std::span<const double> localDofValues,
    std::span<EigenSupport::MXd> channelHessians) const override;

private:
  int numChannels_ = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
