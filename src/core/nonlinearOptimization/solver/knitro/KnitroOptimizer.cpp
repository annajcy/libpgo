#include "solver/knitro/KnitroOptimizer.h"

#include "solver/service/optimizerUtils.h"

#if defined(PGO_HAS_KNITRO)
#  include "solver/knitro/knitroProblem.h"
#  include "solver/knitro/knitroSolverWrapper.h"
#endif

#include <memory>
#include <stdexcept>
#include <utility>

namespace pgo::NonlinearOptimization::Optimization
{

KnitroOptimizer::KnitroOptimizer():
  KnitroOptimizer(Options())
{
}

KnitroOptimizer::KnitroOptimizer(Options options):
  options_(std::move(options))
{
}

OptimizationResult KnitroOptimizer::solve(
  const OptimizationProblem &problem,
  EigenSupport::ConstRefVecXd x0)
{
#if !defined(PGO_HAS_KNITRO)
  (void)problem;
  (void)x0;
  throw std::runtime_error("KnitroOptimizer requested but PGO_HAS_KNITRO is disabled");
#else
  if (!problem.objective) {
    throw std::invalid_argument("OptimizationProblem objective must not be null");
  }
  if (options_.maxIterations < 0 || options_.optimalityTolerance < 0.0 || options_.verbose < 0 ||
    options_.parallelEval < 0 || options_.numThreads < 0) {
    throw std::invalid_argument("KnitroOptimizer options must be non-negative");
  }
  if (problem.constraints.size() > 1) {
    throw std::invalid_argument("KnitroOptimizer currently supports at most one constraint block");
  }

  const int numDofs = problem.objective->getNumDOFs();
  if (x0.size() != numDofs) {
    throw std::invalid_argument("Initial state size must match objective DOFs");
  }

  EigenSupport::VXd x = x0;
  MaterializedBounds xb = materializeBounds(problem.variableBounds, numDofs, -KnitroProblem::inf(), KnitroProblem::inf());

  std::unique_ptr<KnitroProblem> knitroProblem;
  if (problem.constraints.empty()) {
    knitroProblem = std::make_unique<KnitroProblem>(problem.objective);
  }
  else {
    const ConstraintBlock &block = problem.constraints.front();
    if (!block.functions) {
      throw std::invalid_argument("ConstraintBlock functions must not be null");
    }
    MaterializedBounds cb = materializeBounds(
      block.bounds,
      block.functions->getNumConstraints(),
      -KnitroProblem::inf(),
      KnitroProblem::inf());
    knitroProblem = std::make_unique<KnitroProblem>(problem.objective, block.functions);
    knitroProblem->setConstraintsRange(cb.lower, cb.upper);
  }

  knitroProblem->setInit(x);
  knitroProblem->setRange(xb.lower, xb.upper);

  KnitroSolverWrapper solver(knitroProblem.get());
  if (!options_.configFilename.empty()) {
    solver.setConfigFile(options_.configFilename.c_str());
  }
  if (options_.maxIterations > 0) {
    solver.setMaxIter(options_.maxIterations);
  }
  if (options_.optimalityTolerance > 0.0) {
    solver.setOptTol(options_.optimalityTolerance);
  }
  if (options_.feasibilityTolerance > 0.0) {
    solver.setFeasTol(options_.feasibilityTolerance);
  }
  solver.setVerbose(options_.verbose);
  solver.enableMultiEvaluation(options_.parallelEval);
  if (options_.numThreads > 0) {
    solver.setNumThreads(options_.numThreads);
  }
  solver.init();

  const int rawStatus = solver.solve();

  OptimizationResult out;
  out.solver = makeKnitroSolverResult(rawStatus);
  out.x = Eigen::Map<const EigenSupport::VXd>(solver.getx(), knitroProblem->getn());
  out.finalObjective = problem.objective->func(out.x);
  if (!problem.constraints.empty()) {
    out.multipliers = Eigen::Map<const EigenSupport::VXd>(solver.getlambda(), knitroProblem->getm());
    out.constraintValues = Eigen::Map<const EigenSupport::VXd>(solver.getg(), knitroProblem->getm());
  }
  return out;
#endif
}

}  // namespace pgo::NonlinearOptimization::Optimization
