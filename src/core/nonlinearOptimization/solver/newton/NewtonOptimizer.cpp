#include "solver/newton/NewtonOptimizer.h"

#include "energy/evaluationStateAwareEnergy.h"
#include "solver/newton/NewtonSolver.h"
#include "solver/service/optimizerUtils.h"

#include <cmath>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <utility>

namespace pgo::NonlinearOptimization::Optimization
{
namespace
{
using hclock = std::chrono::steady_clock;

double secondsBetween(const hclock::time_point &start, const hclock::time_point &end)
{
  return std::chrono::duration<double>(end - start).count();
}
}  // namespace

NewtonOptimizer::NewtonOptimizer():
  NewtonOptimizer(Options())
{
}

NewtonOptimizer::NewtonOptimizer(Options options):
  options_(std::move(options))
{
}

OptimizationResult NewtonOptimizer::solve(
  const OptimizationProblem &problem,
  EigenSupport::ConstRefVecXd x0)
{
  const hclock::time_point optimizerStart = hclock::now();
  if (!problem.objective) {
    throw std::invalid_argument("OptimizationProblem objective must not be null");
  }
  if (options_.maxIterations < 0) {
    throw std::invalid_argument("NewtonOptimizer maxIterations must be non-negative");
  }
  if (options_.gradientTolerance < 0.0) {
    throw std::invalid_argument("NewtonOptimizer gradientTolerance must be non-negative");
  }
  if (options_.verbose < 0) {
    throw std::invalid_argument("NewtonOptimizer verbose must be non-negative");
  }
  if (!problem.constraints.empty()) {
    throw std::invalid_argument("NewtonOptimizer does not support nonlinear constraints");
  }

  const int numDofs = problem.objective->getNumDOFs();
  if (x0.size() != numDofs) {
    throw std::invalid_argument("Initial state size must match objective DOFs");
  }

  FixedDofsFromBounds fixed = extractFixedDofsFromVariableBounds(problem.variableBounds, numDofs);
  if (fixed.hasGeneralBounds) {
    throw std::invalid_argument("NewtonOptimizer only supports equality variable bounds");
  }

  EigenSupport::VXd x = x0;
  for (size_t k = 0; k < fixed.dofs.size(); ++k) {
    x[fixed.dofs[k]] = fixed.values[static_cast<Eigen::Index>(k)];
  }

  NewtonSolver::SolverParam sp;
  sp.sst = NewtonSolver::SST_SUBITERATION_LINE_SEARCH;
  sp.lineSearch = options_.lineSearch;
  sp.damping = options_.damping;
  sp.termination = options_.termination;
  sp.sparseSolver = options_.sparseSolver;
  sp.threading = options_.threading;

  const double *fixedValues = fixed.values.size() > 0 ? fixed.values.data() : nullptr;
  const hclock::time_point setupStart = hclock::now();
  NewtonSolver solver(x.data(), sp, problem.objective, fixed.dofs, fixedValues);
  const hclock::time_point setupEnd = hclock::now();
  SolverResult solverResult = solver.solve(
    x.data(),
    options_.maxIterations,
    options_.gradientTolerance,
    options_.verbose);

  OptimizationResult result;
  result.solver = std::move(solverResult);
  result.x = std::move(x);

  double finalObjective = std::numeric_limits<double>::quiet_NaN();
  const hclock::time_point finalEvaluationStart = hclock::now();
  auto evaluateFinalObjective = [&] {
    if (const auto *aware = dynamic_cast<const EvaluationStateAwareEnergy *>(problem.objective.get()))
      aware->prepareEvaluationState(result.x);
    finalObjective = problem.objective->func(result.x);
  };
  if (options_.threading)
    options_.threading->executeEvaluation(evaluateFinalObjective);
  else
    evaluateFinalObjective();
  const hclock::time_point finalEvaluationEnd = hclock::now();
  result.solver.diagnostics.threadingEvaluationPhaseCalls += 1;
  result.solver.diagnostics.threadingEvaluationPhaseSeconds +=
    secondsBetween(finalEvaluationStart, finalEvaluationEnd);
  result.solver.diagnostics.finalObjectiveSeconds =
    secondsBetween(finalEvaluationStart, finalEvaluationEnd);

  if (std::isfinite(finalObjective)) {
    result.finalObjective = finalObjective;
  }

  const NewtonSolver::CleanupMetrics cleanup = solver.closeLinearSolver();
  result.solver.diagnostics.linearSolverCleanupSeconds = cleanup.wallSeconds;
  result.solver.diagnostics.threadingLinearSolverPhaseCalls += cleanup.linearSolverPhaseCalls;
  result.solver.diagnostics.threadingLinearSolverPhaseSeconds += cleanup.linearSolverPhaseSeconds;
  result.solver.diagnostics.optimizerPreparationSeconds =
    secondsBetween(optimizerStart, setupStart);
  result.solver.diagnostics.newtonSolverSetupSeconds =
    secondsBetween(setupStart, setupEnd);
  result.solver.diagnostics.optimizerTotalSeconds =
    secondsBetween(optimizerStart, hclock::now());

  return result;
}

}  // namespace pgo::NonlinearOptimization::Optimization
