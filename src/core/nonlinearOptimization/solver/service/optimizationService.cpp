#include "solver/service/optimizer.h"

namespace pgo::NonlinearOptimization::Optimization
{

OptimizationResult minimize(
  Optimizer &optimizer,
  const OptimizationProblem &problem,
  EigenSupport::ConstRefVecXd x0)
{
  return optimizer.solve(problem, x0);
}

}  // namespace pgo::NonlinearOptimization::Optimization
