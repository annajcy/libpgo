#pragma once

#include "solver/service/optimizer.h"

#include <string>

namespace pgo::NonlinearOptimization::Optimization
{

class KnitroOptimizer final : public Optimizer
{
public:
  struct Options
  {
    int maxIterations = 1000;
    double optimalityTolerance = 1e-8;
    double feasibilityTolerance = -1.0;
    int verbose = 0;
    std::string configFilename;
    int parallelEval = 0;
  };

  KnitroOptimizer();
  explicit KnitroOptimizer(Options options);

  OptimizationResult solve(
    const OptimizationProblem &problem,
    EigenSupport::ConstRefVecXd x0) override;

private:
  Options options_;
};

}  // namespace pgo::NonlinearOptimization::Optimization
