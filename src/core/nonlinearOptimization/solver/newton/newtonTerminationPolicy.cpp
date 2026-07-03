#include "solver/newton/newtonTerminationPolicy.h"
namespace pgo::NonlinearOptimization {
constexpr double kRelTolFactor = 1e-5;

NewtonTerminationDecision NewtonTerminationPolicy::afterStep(const NewtonIterationContext &) const {
  return NewtonTerminationDecision::Continue;
}

NewtonTerminationDecision FixedNewtonTerminationPolicy::beforeLinearSolve(
    const NewtonIterationContext &ctx) const {
  const double relThreshold = ctx.lambda0 * kRelTolFactor;
  const bool absConverged = ctx.gradMaxNorm < ctx.epsilon;
  const bool relConverged = ctx.gradMaxNorm < relThreshold;
  if (absConverged || relConverged) return NewtonTerminationDecision::Converged;
  return NewtonTerminationDecision::Continue;
}
}  // namespace pgo::NonlinearOptimization
