#pragma once

#include "materialEvaluator.h"
#include "materialSchema.h"
#include "parameterLayout.h"
#include "EigenSupport.h"

#include <memory>
#include <span>
#include <string_view>

namespace pgo::SolidDeformationModel
{

class OptimizableParameterEvaluationView;
struct OptimizableParameterEvaluationScratch;
class OptimizableParameterRef;

/// Immutable fixed parameterization structure. Numerical values are supplied
/// separately by MaterialParameterData.
class FixedParameterField final
{
public:
  FixedParameterField(
    ParameterSchema parameterSchema,
    std::shared_ptr<const ParameterLayout> layout,
    std::shared_ptr<const MaterialEvaluator> evaluator);

  const ParameterSchema &parameterSchema() const { return parameterSchema_; }
  const ParameterLayout &layout() const { return *layout_; }
  const MaterialEvaluator &evaluator() const { return *evaluator_; }
  const std::shared_ptr<const ParameterLayout> &layoutHandle() const
  {
    return layout_;
  }
  const std::shared_ptr<const MaterialEvaluator> &evaluatorHandle() const
  {
    return evaluator_;
  }

  int numElements() const { return layout_->numElements(); }
  int numLocalParameters() const { return parameterSchema_.numParameters(); }
  int numGlobalParameters() const { return layout_->numGlobalParameters(); }
  int numMaterialChannels() const { return evaluator_->numChannels(); }

  void evaluate(
    int element,
    int quadrature,
    std::span<const double> parameterValues,
    std::span<double> materialChannels) const;

private:
  ParameterSchema parameterSchema_;
  std::shared_ptr<const ParameterLayout> layout_;
  std::shared_ptr<const MaterialEvaluator> evaluator_;
};

/// Immutable schema for one elastic or plastic optimizable parameter field.
/// Copies share the same immutable state and therefore preserve field identity.
class OptimizableParameterField final
{
public:
  OptimizableParameterField(
    ParameterSchema parameterSchema,
    std::shared_ptr<const ParameterLayout> layout,
    std::shared_ptr<const DifferentiableMaterialEvaluator> evaluator);

  const ParameterSchema &parameterSchema() const
  {
    return state_->parameterSchema;
  }
  const ParameterLayout &layout() const { return *state_->layout; }
  const DifferentiableMaterialEvaluator &evaluator() const
  {
    return *state_->evaluator;
  }
  const std::shared_ptr<const ParameterLayout> &layoutHandle() const
  {
    return state_->layout;
  }
  const std::shared_ptr<const DifferentiableMaterialEvaluator> &
  evaluatorHandle() const
  {
    return state_->evaluator;
  }

  int numElements() const { return layout().numElements(); }
  int numLocalParameters() const { return parameterSchema().numParameters(); }
  int numGlobalParameters() const { return layout().numGlobalParameters(); }
  int numMaterialChannels() const { return evaluator().numChannels(); }

  OptimizableParameterRef parameter(std::string_view name) const;
  bool sharesStateWith(const OptimizableParameterField &other) const
  {
    return state_ == other.state_;
  }

private:
  struct State
  {
    ParameterSchema parameterSchema;
    std::shared_ptr<const ParameterLayout> layout;
    std::shared_ptr<const DifferentiableMaterialEvaluator> evaluator;
  };

  std::shared_ptr<const State> state_;
};

/// Reference to one raw optimizer coordinate in an optimizable field.
class OptimizableParameterRef
{
public:
  OptimizableParameterRef() = default;
  OptimizableParameterRef(
    std::shared_ptr<const OptimizableParameterField> field,
    int parameterIndex);

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
    const OptimizableParameterEvaluationView &state) const;
  double value(
    int element,
    int quadrature,
    const OptimizableParameterEvaluationView &state,
    OptimizableParameterEvaluationScratch &scratch) const;

  void localDerivative(
    int element,
    int quadrature,
    const OptimizableParameterEvaluationView &state,
    EigenSupport::RefVecXd output) const;
  void localDerivative(
    int element,
    int quadrature,
    const OptimizableParameterEvaluationView &state,
    OptimizableParameterEvaluationScratch &scratch,
    EigenSupport::RefVecXd output) const;

private:
  std::shared_ptr<const OptimizableParameterField> field_;
  int parameterIndex_ = -1;
};

/// Reference to one physical material channel produced by an optimizable
/// field's differentiable evaluator.
class OptimizableMaterialChannelRef
{
public:
  OptimizableMaterialChannelRef() = default;
  OptimizableMaterialChannelRef(
    std::shared_ptr<const OptimizableParameterField> field,
    MaterialChannelSchema outputSchema,
    int channelIndex);

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
    const OptimizableParameterEvaluationView &state) const;
  double value(
    int element,
    int quadrature,
    const OptimizableParameterEvaluationView &state,
    OptimizableParameterEvaluationScratch &scratch) const;

  void localDerivative(
    int element,
    int quadrature,
    const OptimizableParameterEvaluationView &state,
    OptimizableParameterEvaluationScratch &scratch,
    EigenSupport::RefVecXd output) const;
  void localHessian(
    int element,
    int quadrature,
    const OptimizableParameterEvaluationView &state,
    EigenSupport::RefMatXd output) const;

private:
  std::shared_ptr<const OptimizableParameterField> field_;
  MaterialChannelSchema outputSchema_;
  int channelIndex_ = -1;
};

}  // namespace pgo::SolidDeformationModel
