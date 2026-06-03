#include "solver/ipopt/IpoptOptimizer.h"

#include "solver/service/optimizerUtils.h"

#if defined(PGO_HAS_IPOPT)
#  include "solver/ipopt/IpoptProblem.h"
#  include "solver/ipopt/IpoptSolverWrapper.h"
#endif

#include <memory>
#include <stdexcept>
#include <utility>

namespace pgo::NonlinearOptimization::Optimization
{

IpoptOptimizer::IpoptOptimizer():
  IpoptOptimizer(Options())
{
}

IpoptOptimizer::IpoptOptimizer(Options options):
  options_(std::move(options))
{
}

OptimizationResult IpoptOptimizer::solve(
  const OptimizationProblem &problem,
  EigenSupport::ConstRefVecXd x0)
{
#if !defined(PGO_HAS_IPOPT)
  (void)problem;
  (void)x0;
  throw std::runtime_error("IpoptOptimizer requested but PGO_HAS_IPOPT is disabled");
#else
  if (!problem.objective) {
    throw std::invalid_argument("OptimizationProblem objective must not be null");
  }
  if (options_.maxIterations < 0 || options_.tolerance < 0.0 || options_.printLevel < 0) {
    throw std::invalid_argument("IpoptOptimizer options must be non-negative");
  }
  if (problem.constraints.size() > 1) {
    throw std::invalid_argument("IpoptOptimizer currently supports at most one constraint block");
  }

  const int numDofs = problem.objective->getNumDOFs();
  if (x0.size() != numDofs) {
    throw std::invalid_argument("Initial state size must match objective DOFs");
  }

  EigenSupport::VXd x = x0;
  MaterializedBounds xb = materializeBounds(problem.variableBounds, numDofs, -IpoptProblem::inf(), IpoptProblem::inf());

  Ipopt::SmartPtr<IpoptProblem> ipoptProblem;
  if (problem.constraints.empty()) {
    ipoptProblem = new IpoptProblem(problem.objective);
  }
  else {
    const ConstraintBlock &block = problem.constraints.front();
    if (!block.functions) {
      throw std::invalid_argument("ConstraintBlock functions must not be null");
    }
    MaterializedBounds cb = materializeBounds(
      block.bounds,
      block.functions->getNumConstraints(),
      -IpoptProblem::inf(),
      IpoptProblem::inf());
    ipoptProblem = new IpoptProblem(problem.objective, block.functions);
    ipoptProblem->setConstraintsRange(cb.lower, cb.upper);
  }

  ipoptProblem->setInit(x);
  ipoptProblem->setRange(xb.lower, xb.upper);

  IpoptSolverWrapper solver(ipoptProblem);
  solver.setMaxIter(options_.maxIterations);
  solver.setTol(options_.tolerance);
  solver.setVerbose(options_.printLevel);
  if (!problem.constraints.empty() && problem.constraints.front().functions->isLinear()) {
    solver.setLinearConstraints(true);
  }

  const int initRet = solver.init();
  if (initRet != 0) {
    OptimizationResult out;
    out.x = std::move(x);
    out.solver = makeSolverResult(SolveStatus::ExternalSolverFailure, initRet);
    return out;
  }

  const int rawStatus = solver.solve();

  OptimizationResult out;
  out.solver = makeIpoptSolverResult(rawStatus);
  out.x = ipoptProblem->getFinalx();
  out.finalObjective = problem.objective->func(out.x);
  if (!problem.constraints.empty()) {
    out.multipliers = ipoptProblem->getFinalLambda();
    out.constraintValues = ipoptProblem->getFinalg();
  }
  return out;
#endif
}

}  // namespace pgo::NonlinearOptimization::Optimization
