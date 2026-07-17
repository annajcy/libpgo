#include "core.h"

#include "eigen_numpy.h"
#include "solver/newton/NewtonOptimizer.h"
#include "solver/service/optimizationProblem.h"
#include "solver/service/optimizerUtils.h"
#include "solver/service/optimizationResult.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cmath>

namespace nb = nanobind;
using namespace pgo;
namespace NOO = NonlinearOptimization::Optimization;

// ── Helpers ────────────────────────────────────────────────────────────────

namespace
{

nb::object finiteOrNone(double value)
{
  return std::isfinite(value) ? nb::object(nb::float_(value)) : nb::none();
}

nb::dict newtonIterationTraceToDict(const NonlinearOptimization::NewtonIterationTrace &trace)
{
  nb::dict out;
  out["iteration"] = trace.iteration;
  out["energy_before"] = finiteOrNone(trace.energyBefore);
  out["energy_after"] = finiteOrNone(trace.energyAfter);
  out["energy_delta"] = finiteOrNone(trace.energyDelta);
  out["grad_norm_before"] = finiteOrNone(trace.gradNormBefore);
  out["grad_norm_after"] = finiteOrNone(trace.gradNormAfter);
  out["grad_max_before"] = finiteOrNone(trace.gradMaxBefore);
  out["grad_max_after"] = finiteOrNone(trace.gradMaxAfter);
  out["grad_reduction_ratio"] = finiteOrNone(trace.gradReductionRatio);
  out["grad_dot_dx"] = finiteOrNone(trace.gradDotDx);
  out["raw_step_norm"] = finiteOrNone(trace.rawStepNorm);
  out["raw_step_max_norm"] = finiteOrNone(trace.rawStepMaxNorm);
  out["accepted_alpha"] = finiteOrNone(trace.acceptedAlpha);
  out["accepted_step_norm"] = finiteOrNone(trace.acceptedStepNorm);
  out["accepted_step_max_norm"] = finiteOrNone(trace.acceptedStepMaxNorm);
  out["line_search_iterations"] = trace.lineSearchIterations;
  out["line_search_status"] = trace.lineSearchStatus;
  out["min_feasible_alpha"] = finiteOrNone(trace.minFeasibleAlpha);
  out["feasible_alpha"] = finiteOrNone(trace.feasibleAlpha);
  out["line_search_alpha"] = finiteOrNone(trace.lineSearchAlpha);
  out["contact_clamp_count"] = trace.contactClampCount;
  out["material_clamp_count"] = trace.materialClampCount;
  out["damping_value"] = finiteOrNone(trace.dampingValue);
  out["hessian_rows"] = trace.hessianRows;
  out["hessian_cols"] = trace.hessianCols;
  out["hessian_nnz"] = trace.hessianNnz;
  out["symbolic_rebuilt"] = trace.symbolicRebuilt;
  out["func_grad_hessian_seconds"] = finiteOrNone(trace.funcGradHessianSeconds);
  out["symbolic_analyze_seconds"] = finiteOrNone(trace.symbolicAnalyzeSeconds);
  out["factorize_seconds"] = finiteOrNone(trace.factorizeSeconds);
  out["solve_seconds"] = finiteOrNone(trace.solveSeconds);
  out["line_search_seconds"] = finiteOrNone(trace.lineSearchSeconds);
  out["iteration_wall_seconds"] = finiteOrNone(trace.iterationWallSeconds);
  out["low_value"] = trace.lowValue;
  return out;
}

nb::dict diagnosticsToDict(const NonlinearOptimization::SolveDiagnostics &diagnostics)
{
  nb::dict out;
  out["min_feasible_alpha"] = diagnostics.minFeasibleAlpha;
  out["min_line_search_alpha"] = diagnostics.minLineSearchAlpha;
  out["min_effective_alpha"] = diagnostics.minEffectiveAlpha;
  out["material_clamp_count"] = diagnostics.clampCounts[static_cast<int>(NonlinearOptimization::StepSource::Material)];
  out["contact_clamp_count"] = diagnostics.clampCounts[static_cast<int>(NonlinearOptimization::StepSource::Contact)];
  out["final_gradient_norm"] = diagnostics.hasFinalGradientStats ? nb::object(nb::float_(diagnostics.finalGradientNorm)) : nb::none();
  out["final_gradient_max_norm"] = diagnostics.hasFinalGradientStats ? nb::object(nb::float_(diagnostics.finalGradientMaxNorm)) : nb::none();
  out["last_grad_dot_dx"] = diagnostics.lastGradDotDx;
  out["last_raw_step_max_norm"] = diagnostics.lastRawStepMaxNorm;
  out["last_raw_step_norm"] = diagnostics.lastRawStepNorm;
  out["last_accepted_step_max_norm"] = diagnostics.lastAcceptedStepMaxNorm;
  out["last_line_search_iterations"] = diagnostics.lastLineSearchIterations;
  out["max_line_search_iterations"] = diagnostics.maxLineSearchIterations;
  out["total_line_search_iterations"] = diagnostics.totalLineSearchIterations;
  out["last_current_energy"] = diagnostics.lastCurrentEnergy;
  out["last_accepted_energy"] = diagnostics.lastAcceptedEnergy;
  out["last_energy_delta"] = diagnostics.lastEnergyDelta;
  out["last_damping_value"] = diagnostics.lastDampingValue;
  out["last_active_system_nnz"] = diagnostics.lastActiveSystemNnz;
  out["last_active_system_rows"] = diagnostics.lastActiveSystemRows;
  out["last_active_system_cols"] = diagnostics.lastActiveSystemCols;
  out["linear_solver_symbolic_rebuild_count"] = diagnostics.linearSolverSymbolicRebuildCount;
  out["linear_solver_symbolic_reuse_count"] = diagnostics.linearSolverSymbolicReuseCount;
  out["newton_convergence_reason"] = static_cast<int>(diagnostics.newtonConvergenceReason);
  out["newton_convergence_reason_name"] = NonlinearOptimization::newtonConvergenceReasonName(diagnostics.newtonConvergenceReason);
  out["newton_convergence_threshold"] = finiteOrNone(diagnostics.newtonConvergenceThreshold);
  out["newton_low_value_iteration_count"] = diagnostics.newtonLowValueIterationCount;
  out["newton_tiny_step_count"] = diagnostics.newtonTinyStepCount;
  out["newton_small_alpha_count"] = diagnostics.newtonSmallAlphaCount;
  out["newton_symbolic_rebuild_count"] = diagnostics.newtonSymbolicRebuildCount;
  out["newton_worst_progress_iteration"] = diagnostics.newtonWorstProgressIteration;
  out["newton_worst_progress_ratio"] = diagnostics.newtonWorstProgressRatio;
  out["newton_total_factorize_seconds"] = diagnostics.newtonTotalFactorizeSeconds;
  out["newton_total_solve_seconds"] = diagnostics.newtonTotalSolveSeconds;
  out["threading_evaluation_phase_calls"] = diagnostics.threadingEvaluationPhaseCalls;
  out["threading_linear_solver_phase_calls"] = diagnostics.threadingLinearSolverPhaseCalls;
  out["threading_evaluation_phase_seconds"] = diagnostics.threadingEvaluationPhaseSeconds;
  out["threading_linear_solver_phase_seconds"] = diagnostics.threadingLinearSolverPhaseSeconds;
  nb::list iterations;
  for (const NonlinearOptimization::NewtonIterationTrace &trace : diagnostics.newtonIterations)
    iterations.append(newtonIterationTraceToDict(trace));
  out["newton_iterations"] = iterations;
  return out;
}

}  // namespace

// ── PyOptimizationProblem ───────────────────────────────────────────────────

void PyOptimizationProblem::setObjective(std::shared_ptr<const NO::PotentialEnergy> coreEnergy)
{
  if (!coreEnergy)
    throw nb::type_error("objective must not be None");
  objectiveOwner_ = std::move(coreEnergy);
  problem_.objective = objectiveOwner_;
}

std::shared_ptr<PyOptimizationProblem> createOptimizationProblem(
  std::shared_ptr<const NO::PotentialEnergy> objective)
{
  auto problem = std::make_shared<PyOptimizationProblem>();
  problem->setObjective(std::move(objective));
  return problem;
}

std::shared_ptr<PyOptimizationProblem> createOptimizationProblemFromPeer(
  PyPotentialEnergy &objective)
{
  return createOptimizationProblem(objective.potentialEnergyHandle());
}

void PyOptimizationProblem::setVariableBounds(
  nb::ndarray<nb::numpy, const double> lower,
  bool hasLower,
  nb::ndarray<nb::numpy, const double> upper,
  bool hasUpper)
{
  if (hasLower) {
    problem_.variableBounds.lower = python::ndarrayToVectorXd(lower);
  }
  else {
    problem_.variableBounds.lower.reset();
  }
  if (hasUpper) {
    problem_.variableBounds.upper = python::ndarrayToVectorXd(upper);
  }
  else {
    problem_.variableBounds.upper.reset();
  }
}

void PyOptimizationProblem::fixVariables(
  std::vector<int> dofs,
  nb::ndarray<nb::numpy, const double> values,
  int numDofs)
{
  auto valueVec = python::ndarrayToVectorXd(values);
  NOO::fixVariables(problem_, dofs, valueVec, numDofs);
}

// ── PyNewtonOptimizerOptions → core Options ────────────────────────────────

NOO::NewtonOptimizer::Options makeNewtonOptions(const PyNewtonOptimizerOptions &options)
{
  NOO::NewtonOptimizer::Options opts;
  opts.maxIterations = options.maxIterations;
  opts.gradientTolerance = options.gradientTolerance;
  opts.verbose = options.verbose;
  // Null handles let the core defaults kick in.
  if (options.lineSearch)
    opts.lineSearch = options.lineSearch->handle();
  if (options.damping)
    opts.damping = options.damping->handle();
  if (options.termination)
    opts.termination = options.termination->handle();
  if (options.sparseSolver)
    opts.sparseSolver = options.sparseSolver->handle();
  opts.threading = options.threading;
  return opts;
}

// ── PyOptimizer ─────────────────────────────────────────────────────────────

nb::dict PyOptimizer::solve(
  const PyOptimizationProblem &problem,
  nb::ndarray<nb::numpy, const double> x0)
{
  auto x0Map = python::ndarrayToVectorMapXd(x0);
  NOO::OptimizationResult result;
  {
    nb::gil_scoped_release release;
    result = asOptimizer().solve(problem.handle(), x0Map);
  }
  return optimizationResultToDict(std::move(result));
}

// ── PyNewtonOptimizer ─────────────────────────────────────────────────────

PyNewtonOptimizer::PyNewtonOptimizer(PyNewtonOptimizerOptions options): optimizer_(makeNewtonOptions(options))
{
}

// ── Result → dict ──────────────────────────────────────────────────────────

nb::dict optimizationResultToDict(NOO::OptimizationResult result)
{
  const auto &solver = result.solver;
  nb::dict out;
  out["x"] = python::vectorXdToNdarray(std::move(result.x));
  out["status"] = static_cast<int>(solver.status);
  out["converged"] = solver.converged();
  out["iterations"] = solver.iterations;
  out["raw_status_code"] = solver.rawStatusCode;
  out["final_objective"] = result.finalObjective.has_value() ? nb::object(nb::float_(*result.finalObjective)) : nb::none();
  out["final_gradient_norm"] = solver.hasFinalGradientStats ? nb::object(nb::float_(solver.finalGradientNorm)) : nb::none();
  out["final_gradient_max_norm"] = solver.hasFinalGradientStats ? nb::object(nb::float_(solver.finalGradientMaxNorm)) : nb::none();
  out["diagnostics"] = diagnosticsToDict(solver.diagnostics);
  return out;
}
