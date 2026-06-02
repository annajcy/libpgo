#include "optimizationService.h"

#include "optimizationBackend.h"

namespace pgo::NonlinearOptimization
{

OptimizationResult minimize(
  const OptimizationProblem &problem,
  EigenSupport::ConstRefVecXd x0,
  const NewtonOptions &options)
{
  return NewtonOptimizationBackend(options).solve(problem, x0);
}

}  // namespace pgo::NonlinearOptimization
