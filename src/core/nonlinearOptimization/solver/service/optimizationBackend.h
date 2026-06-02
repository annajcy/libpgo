#pragma once

#include "solver/service/optimizationService.h"

namespace pgo::NonlinearOptimization
{

class OptimizationBackend
{
public:
  virtual ~OptimizationBackend() = default;

  virtual OptimizationResult solve(
    const OptimizationProblem &problem,
    EigenSupport::ConstRefVecXd x0) = 0;
};

class NewtonOptimizationBackend final : public OptimizationBackend
{
public:
  explicit NewtonOptimizationBackend(NewtonOptions options);

  OptimizationResult solve(
    const OptimizationProblem &problem,
    EigenSupport::ConstRefVecXd x0) override;

private:
  NewtonOptions options_;
};

}  // namespace pgo::NonlinearOptimization
