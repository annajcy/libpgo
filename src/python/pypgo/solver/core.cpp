#include "core.h"

#include "eigen_numpy.h"
#include "solver/newton/NewtonOptimizer.h"
#include "solver/service/optimizationProblem.h"
#include "solver/service/optimizerUtils.h"
#include "solver/service/optimizationResult.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

namespace nb = nanobind;
using namespace pgo;
namespace NOO = NonlinearOptimization::Optimization;

// ── Helpers ────────────────────────────────────────────────────────────────

namespace {

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
  } else {
    problem_.variableBounds.lower.reset();
  }
  if (hasUpper) {
    problem_.variableBounds.upper = python::ndarrayToVectorXd(upper);
  } else {
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
  opts.damping = options.damping;
  // Null handles let the core defaults (Backtracking / Auto) kick in.
  if (options.lineSearch)
    opts.lineSearch = options.lineSearch->handle();
  if (options.sparseSolver)
    opts.sparseSolver = options.sparseSolver->handle();
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

PyNewtonOptimizer::PyNewtonOptimizer(PyNewtonOptimizerOptions options)
  : optimizer_(makeNewtonOptions(options))
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
