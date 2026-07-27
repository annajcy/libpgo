#pragma once

#include "EigenSupport.h"

#include <span>

namespace pgo
{
namespace SolidDeformationModel
{

/// Forward-only transformation from element-local parameter values to the
/// physical material channels consumed by a model definition.
class MaterialChannelMapping
{
public:
  virtual ~MaterialChannelMapping() = default;
  virtual int numInputs() const = 0;
  virtual int numChannels() const = 0;

  virtual void evaluate(
    int element,
    int quadrature,
    std::span<const double> parameters,
    std::span<double> channels) const = 0;
};

/// Differentiable transformation used by optimizable parameter fields.
///
/// For channels = f(parameters), the Jacobian has shape
/// [numChannels(), numInputs()] and stores
/// jacobian(channel, parameter) = d channels[channel] / d parameters[parameter].
/// channelHessians[channel] has shape
/// [numInputs(), numInputs()] and stores the Hessian of that channel.
class DifferentiableMaterialChannelMapping : public MaterialChannelMapping
{
public:
  virtual ~DifferentiableMaterialChannelMapping() = default;
  virtual bool isAffine() const = 0;

  virtual void evaluateJacobian(
    int element,
    int quadrature,
    std::span<const double> parameters,
    EigenSupport::RefMatXd jacobian) const = 0;

  virtual void evaluateHessians(
    int element,
    int quadrature,
    std::span<const double> parameters,
    std::span<EigenSupport::MXd> channelHessians) const = 0;
};

class IdentityMaterialChannelMapping final : public DifferentiableMaterialChannelMapping
{
public:
  explicit IdentityMaterialChannelMapping(int numChannels);

  int numInputs() const override { return numChannels_; }
  int numChannels() const override { return numChannels_; }
  bool isAffine() const override { return true; }

  void evaluate(
    int element,
    int quadrature,
    std::span<const double> parameters,
    std::span<double> channels) const override;
  void evaluateJacobian(
    int element,
    int quadrature,
    std::span<const double> parameters,
    EigenSupport::RefMatXd jacobian) const override;
  void evaluateHessians(
    int element,
    int quadrature,
    std::span<const double> parameters,
    std::span<EigenSupport::MXd> channelHessians) const override;

private:
  int numChannels_ = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
