#pragma once

#include "solver/common/solveDiagnostics.h"

#include <limits>

namespace pgo::NonlinearOptimization {

// Default tolerances used by the NewtonSolver's built-in policy when no
// explicit termination policy is supplied.
inline constexpr double kDefaultAbsoluteTolerance = 1e-6;
inline constexpr double kDefaultRelativeToleranceFactor = 1e-5;

enum class NewtonTerminationDecision { Continue, Converged, Failed };

struct NewtonIterationContext {
  int iteration = 0;
  double energy = 0.0;
  double gradMaxNorm = 0.0;
  double gradNorm = 0.0;
  double lambda0 = 0.0;
  double lambdaScale = 0.0;
  // Post-step fields (NaN when evaluating the current iterate):
  double acceptedAlpha = std::numeric_limits<double>::quiet_NaN();
  double acceptedEnergy = std::numeric_limits<double>::quiet_NaN();
  double acceptedStepMaxNorm = std::numeric_limits<double>::quiet_NaN();
};

// Outcome of one termination-policy evaluation: whether to stop, and when
// stopping, which convergence condition triggered and at what threshold.
struct NewtonTerminationInfo {
  NewtonTerminationDecision decision = NewtonTerminationDecision::Continue;
  NewtonConvergenceReason reason = NewtonConvergenceReason::TerminationPolicy;
  double threshold = std::numeric_limits<double>::quiet_NaN();
};

class NewtonTerminationPolicy {
public:
  virtual ~NewtonTerminationPolicy() = default;
  // Decide whether the current iterate is converged (or failed) and, if so,
  // report the reason and threshold for solver diagnostics.
  virtual NewtonTerminationInfo evaluate(const NewtonIterationContext &ctx) const = 0;
  virtual NewtonTerminationDecision afterStep(const NewtonIterationContext &ctx) const;
};

// Stop when ||grad||_max < absTolerance.
class AbsoluteNewtonTerminationPolicy final : public NewtonTerminationPolicy {
public:
  explicit AbsoluteNewtonTerminationPolicy(double absTolerance);
  NewtonTerminationInfo evaluate(const NewtonIterationContext &ctx) const override;

private:
  double absTolerance_;
};

// Stop when ||grad||_max < lambda0 * relativeTolerance.
class RelativeNewtonTerminationPolicy final : public NewtonTerminationPolicy {
public:
  explicit RelativeNewtonTerminationPolicy(double relativeTolerance);
  NewtonTerminationInfo evaluate(const NewtonIterationContext &ctx) const override;

private:
  double relativeTolerance_;
};

// Stop when either the absolute or the relative threshold is met.
class HybridNewtonTerminationPolicy final : public NewtonTerminationPolicy {
public:
  HybridNewtonTerminationPolicy(double absTolerance, double relativeTolerance);
  NewtonTerminationInfo evaluate(const NewtonIterationContext &ctx) const override;

private:
  double absTolerance_;
  double relativeTolerance_;
};

}  // namespace pgo::NonlinearOptimization
