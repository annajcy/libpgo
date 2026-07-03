#pragma once

#include "solver/service/optimizer.h"

namespace pgo::NonlinearOptimization::Optimization
{

class IpoptOptimizer final : public Optimizer
{
public:
  struct Options
  {
    int maxIterations = 1000;
    double tolerance = 1e-8;
    int printLevel = 5;
  };

  IpoptOptimizer();
  explicit IpoptOptimizer(Options options);

  OptimizationResult solve(
    const OptimizationProblem &problem,
    EigenSupport::ConstRefVecXd x0) override;

private:
  Options options_;
};

}  // namespace pgo::NonlinearOptimization::Optimization
