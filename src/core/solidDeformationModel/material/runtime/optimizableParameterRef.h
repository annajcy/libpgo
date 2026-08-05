#pragma once

#include "material/runtime/materialBinding.h"
#include "material/runtime/materialState.h"

#include <memory>
#include <string>
#include <string_view>

namespace pgo::SolidDeformationModel
{

/// Runtime reference to one raw optimizer coordinate in an optimizable field.
class OptimizableParameterRef
{
public:
  OptimizableParameterRef() = default;
  OptimizableParameterRef(
    std::shared_ptr<const OptimizableParameterField> field,
    int parameterIndex);
  OptimizableParameterRef(
    std::shared_ptr<const OptimizableParameterField> field,
    std::string_view parameterName);

  const OptimizableParameterField &field() const;
  std::shared_ptr<const OptimizableParameterField> fieldHandle() const
  {
    return field_;
  }
  int parameterIndex() const { return parameterIndex_; }
  std::string_view name() const;

  double value(
    int element,
    int quadrature,
    std::span<const double> globalValues) const;
  double value(
    int element,
    int quadrature,
    std::span<const double> globalValues,
    MaterialStateEvaluationScratch &scratch) const;

  void localDerivative(
    int element,
    int quadrature,
    std::span<const double> globalValues,
    EigenSupport::RefVecXd output) const;
  void localDerivative(
    int element,
    int quadrature,
    std::span<const double> globalValues,
    MaterialStateEvaluationScratch &scratch,
    EigenSupport::RefVecXd output) const;

private:
  std::shared_ptr<const OptimizableParameterField> field_;
  int parameterIndex_ = -1;
};

/// Runtime reference to one physical material channel produced by an
/// optimizable field's differentiable channel mapping.
class OptimizableMaterialChannelRef
{
public:
  OptimizableMaterialChannelRef() = default;
  template<class ModelDefinition>
  OptimizableMaterialChannelRef(
    const MaterialDomainBinding<ModelDefinition> &binding,
    std::string_view channelName):
    OptimizableMaterialChannelRef(
      binding.optimizableField(),
      binding.definition()->optimizableChannelSchema(),
      channelName)
  {
  }

  const OptimizableParameterField &field() const;
  std::shared_ptr<const OptimizableParameterField> fieldHandle() const
  {
    return field_;
  }
  int channelIndex() const { return channelIndex_; }
  std::string_view name() const;

  double value(
    int element,
    int quadrature,
    std::span<const double> globalValues) const;
  double value(
    int element,
    int quadrature,
    std::span<const double> globalValues,
    MaterialStateEvaluationScratch &scratch) const;

  void localDerivative(
    int element,
    int quadrature,
    std::span<const double> globalValues,
    MaterialStateEvaluationScratch &scratch,
    EigenSupport::RefVecXd output) const;
  void localHessian(
    int element,
    int quadrature,
    std::span<const double> globalValues,
    EigenSupport::RefMatXd output) const;
  OptimizableMaterialChannelRef(
    std::shared_ptr<const OptimizableParameterField> field,
    MaterialChannelSchema outputSchema,
    std::string_view channelName);

private:
  std::shared_ptr<const OptimizableParameterField> field_;
  std::string channelName_;
  int channelIndex_ = -1;
};

}  // namespace pgo::SolidDeformationModel
