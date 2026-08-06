#include "solver/newton/newtonTerminationPolicy.h"

#include <stdexcept>

namespace pgo::NonlinearOptimization {

NewtonTerminationDecision NewtonTerminationPolicy::afterStep(const NewtonIterationContext &) const {
  return NewtonTerminationDecision::Continue;
}

AbsoluteNewtonTerminationPolicy::AbsoluteNewtonTerminationPolicy(double absTolerance)
  : absTolerance_(absTolerance)
{
  if (!(absTolerance_ > 0.0))
    throw std::invalid_argument(
      "AbsoluteNewtonTerminationPolicy absTolerance must be positive");
}

NewtonTerminationInfo AbsoluteNewtonTerminationPolicy::evaluate(
    const NewtonIterationContext &ctx) const {
  if (ctx.gradMaxNorm < absTolerance_)
    return { NewtonTerminationDecision::Converged,
      NewtonConvergenceReason::AbsoluteGradient, absTolerance_ };
  return {};
}

RelativeNewtonTerminationPolicy::RelativeNewtonTerminationPolicy(double relativeTolerance)
  : relativeTolerance_(relativeTolerance)
{
  if (!(relativeTolerance_ > 0.0))
    throw std::invalid_argument(
      "RelativeNewtonTerminationPolicy relativeTolerance must be positive");
}

NewtonTerminationInfo RelativeNewtonTerminationPolicy::evaluate(
    const NewtonIterationContext &ctx) const {
  const double threshold = ctx.lambda0 * relativeTolerance_;
  if (ctx.gradMaxNorm < threshold)
    return { NewtonTerminationDecision::Converged,
      NewtonConvergenceReason::RelativeGradient, threshold };
  return {};
}

HybridNewtonTerminationPolicy::HybridNewtonTerminationPolicy(
  double absTolerance, double relativeTolerance)
  : absTolerance_(absTolerance), relativeTolerance_(relativeTolerance)
{
  if (!(absTolerance_ > 0.0))
    throw std::invalid_argument(
      "HybridNewtonTerminationPolicy absTolerance must be positive");
  if (!(relativeTolerance_ > 0.0))
    throw std::invalid_argument(
      "HybridNewtonTerminationPolicy relativeTolerance must be positive");
}

NewtonTerminationInfo HybridNewtonTerminationPolicy::evaluate(
    const NewtonIterationContext &ctx) const {
  const double relThreshold = ctx.lambda0 * relativeTolerance_;
  if (ctx.gradMaxNorm < absTolerance_)
    return { NewtonTerminationDecision::Converged,
      NewtonConvergenceReason::AbsoluteGradient, absTolerance_ };
  if (ctx.gradMaxNorm < relThreshold)
    return { NewtonTerminationDecision::Converged,
      NewtonConvergenceReason::RelativeGradient, relThreshold };
  return {};
}

}  // namespace pgo::NonlinearOptimization
