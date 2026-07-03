#pragma once
#include <limits>
namespace pgo::NonlinearOptimization {

enum class NewtonTerminationDecision { Continue, Converged, Failed };

struct NewtonIterationContext {
  int iteration = 0;
  double energy = 0.0;
  double gradMaxNorm = 0.0;
  double gradNorm = 0.0;
  double lambda0 = 0.0;
  double lambdaScale = 0.0;
  double epsilon = 0.0;
  // Post-step fields (NaN for beforeLinearSolve):
  double acceptedAlpha = std::numeric_limits<double>::quiet_NaN();
  double acceptedEnergy = std::numeric_limits<double>::quiet_NaN();
  double acceptedStepMaxNorm = std::numeric_limits<double>::quiet_NaN();
};

class NewtonTerminationPolicy {
public:
  virtual ~NewtonTerminationPolicy() = default;
  virtual NewtonTerminationDecision beforeLinearSolve(const NewtonIterationContext &ctx) const = 0;
  virtual NewtonTerminationDecision afterStep(const NewtonIterationContext &ctx) const;
};

class FixedNewtonTerminationPolicy final : public NewtonTerminationPolicy {
public:
  NewtonTerminationDecision beforeLinearSolve(const NewtonIterationContext &ctx) const override;
};

}  // namespace pgo::NonlinearOptimization
