#pragma once

#include "solver/service/optimizationProblem.h"
#include "solver/service/optimizationResult.h"

namespace pgo::NonlinearOptimization::Optimization
{

class Optimizer
{
public:
  virtual ~Optimizer() = default;

  virtual OptimizationResult solve(
    const OptimizationProblem &problem,
    EigenSupport::ConstRefVecXd x0) = 0;
};

OptimizationResult minimize(
  Optimizer &optimizer,
  const OptimizationProblem &problem,
  EigenSupport::ConstRefVecXd x0);

}  // namespace pgo::NonlinearOptimization::Optimization
