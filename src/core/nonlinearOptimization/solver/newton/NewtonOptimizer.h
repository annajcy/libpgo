#pragma once

#include "solver/newton/newtonDampingPolicy.h"
#include "solver/newton/newtonLineSearchPolicy.h"
#include "solver/newton/newtonSparseSolverBackend.h"
#include "solver/newton/newtonTerminationPolicy.h"
#include "solver/service/optimizer.h"

#include <memory>

namespace pgo::NonlinearOptimization::Optimization
{

class NewtonOptimizer final : public Optimizer
{
public:
  struct Options
  {
    int maxIterations = 50;
    double gradientTolerance = 1e-6;
    int verbose = 0;
    // Concrete (immutable) line-search policy handle; null selects a default Backtracking policy.
    std::shared_ptr<const NewtonLineSearchPolicy> lineSearch;
    // Newton diagonal damping policy; null selects NoDampingPolicy.
    std::shared_ptr<const NewtonDampingPolicy> damping;
    // Newton termination policy; null selects FixedNewtonTerminationPolicy.
    std::shared_ptr<const NewtonTerminationPolicy> termination;
    // Sparse linear-solver selector; null picks Auto (best available backend).
    std::shared_ptr<const NewtonSparseSolverSelector> sparseSolver;
  };

  NewtonOptimizer();
  explicit NewtonOptimizer(Options options);

  OptimizationResult solve(
    const OptimizationProblem &problem,
    EigenSupport::ConstRefVecXd x0) override;

private:
  Options options_;
};

}  // namespace pgo::NonlinearOptimization::Optimization
