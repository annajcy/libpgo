#pragma once

#include "EigenSupport.h"

#include <span>

namespace pgo
{
namespace SolidDeformationModel
{

/// Forward-only transformation from element-local parameter values to the
/// physical material channels consumed by a model definition.
class MaterialEvaluator
{
public:
  virtual ~MaterialEvaluator() = default;
  virtual int numParameters() const = 0;
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
/// [numChannels(), numParameters()] and stores
/// jacobian(channel, parameter) = d channels[channel] / d parameters[parameter].
/// channelHessians[channel] has shape
/// [numParameters(), numParameters()] and stores the Hessian of that channel.
class DifferentiableMaterialEvaluator : public MaterialEvaluator
{
public:
  virtual ~DifferentiableMaterialEvaluator() = default;
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

class IdentityMaterialEvaluator final : public DifferentiableMaterialEvaluator
{
public:
  explicit IdentityMaterialEvaluator(int numChannels);

  int numParameters() const override { return numChannels_; }
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
