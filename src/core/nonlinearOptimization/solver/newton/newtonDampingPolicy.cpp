#include "solver/newton/newtonDampingPolicy.h"
namespace pgo::NonlinearOptimization {
double NoDampingPolicy::dampingForIteration(const NewtonIterationContext &) const { return 0.0; }
double FixedDampingPolicy::dampingForIteration(const NewtonIterationContext &ctx) const {
  return params_.dampingScale * ctx.lambdaScale * ctx.lambda0;
}
}  // namespace pgo::NonlinearOptimization
