#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "eigen_numpy.h"
#include "energy_core.h"
#include "solver/newton/NewtonOptimizer.h"
#include "solver/service/optimizerUtils.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace nb = nanobind;
using namespace pgo;

namespace
{

NonlinearOptimization::NewtonLineSearchKind parseLineSearch(const std::string &name)
{
  if (name == "golden")
    return NonlinearOptimization::NewtonLineSearchKind::Golden;
  if (name == "brents")
    return NonlinearOptimization::NewtonLineSearchKind::Brents;
  if (name == "backtrack")
    return NonlinearOptimization::NewtonLineSearchKind::Backtrack;
  if (name == "simple")
    return NonlinearOptimization::NewtonLineSearchKind::Simple;
  throw nb::value_error("unknown line_search; expected 'golden', 'brents', 'backtrack', or 'simple'");
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
  return out;
}

nb::dict resultToDict(NonlinearOptimization::Optimization::OptimizationResult result)
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

NonlinearOptimization::Optimization::NewtonOptimizer::Options makeNewtonOptimizerOptions(
  int maxIterations,
  double gradientTolerance,
  bool damping,
  const std::string &lineSearch,
  int verbose,
  int sparseSolverKind)
{
  NonlinearOptimization::Optimization::NewtonOptimizer::Options options;
  options.maxIterations = maxIterations;
  options.gradientTolerance = gradientTolerance;
  options.verbose = verbose;
  options.damping = damping;
  options.lineSearch = parseLineSearch(lineSearch);
  options.sparseSolver.kind = static_cast<NonlinearOptimization::NewtonSparseSolverKind>(sparseSolverKind);
  return options;
}

nb::dict solveWithNewtonOptimizer(
  std::shared_ptr<PyPotentialEnergy> energy,
  nb::ndarray<nb::numpy, const double> x0,
  nb::ndarray<nb::numpy, const double> lower,
  bool hasLower,
  nb::ndarray<nb::numpy, const double> upper,
  bool hasUpper,
  int maxIterations,
  double gradientTolerance,
  bool damping,
  const std::string &lineSearch,
  int verbose,
  int sparseSolverKind)
{
  auto x0Map = python::ndarrayToVectorMapXd(x0);

  NonlinearOptimization::Optimization::OptimizationProblem problem;
  problem.objective = energy->handle_;
  if (hasLower) {
    problem.variableBounds.lower = python::ndarrayToVectorXd(lower);
  }
  if (hasUpper) {
    problem.variableBounds.upper = python::ndarrayToVectorXd(upper);
  }

  NonlinearOptimization::Optimization::NewtonOptimizer optimizer(
    makeNewtonOptimizerOptions(maxIterations, gradientTolerance, damping, lineSearch, verbose, sparseSolverKind));

  NonlinearOptimization::Optimization::OptimizationResult result;
  {
    nb::gil_scoped_release release;
    result = optimizer.solve(problem, x0Map);
  }

  return resultToDict(std::move(result));
}

nb::dict solveNewton(
  std::shared_ptr<PyPotentialEnergy> energy,
  nb::ndarray<nb::numpy, const double> x0,
  std::vector<int> fixedDofs,
  nb::ndarray<nb::numpy, const double> fixedValues,
  bool hasFixedValues,
  int maxIter,
  double tol,
  bool damping,
  const std::string &lineSearch,
  int verbose,
  int sparseSolverKind)
{
  auto x0Map = python::ndarrayToVectorMapXd(x0);

  NonlinearOptimization::Optimization::OptimizationProblem problem;
  problem.objective = energy->handle_;
  if (!fixedDofs.empty() || hasFixedValues) {
    EigenSupport::VXd values = hasFixedValues ? python::ndarrayToVectorXd(fixedValues) : EigenSupport::VXd::Zero(static_cast<Eigen::Index>(fixedDofs.size()));
    if (!hasFixedValues) {
      for (size_t i = 0; i < fixedDofs.size(); ++i) {
        values[static_cast<Eigen::Index>(i)] = x0Map[fixedDofs[i]];
      }
    }
    NonlinearOptimization::Optimization::fixVariables(problem, fixedDofs, values, static_cast<int>(x0Map.size()));
  }

  NonlinearOptimization::Optimization::NewtonOptimizer optimizer(
    makeNewtonOptimizerOptions(maxIter, tol, damping, lineSearch, verbose, sparseSolverKind));

  NonlinearOptimization::Optimization::OptimizationResult result;
  {
    nb::gil_scoped_release release;
    result = optimizer.solve(problem, x0Map);
  }

  return resultToDict(std::move(result));
}

}  // namespace

void init_solver_bindings(nb::module_ &m)
{
  m.def("_newton_optimizer_solve", &solveWithNewtonOptimizer,
    nb::arg("energy"),
    nb::arg("x0"),
    nb::arg("lower"),
    nb::arg("has_lower"),
    nb::arg("upper"),
    nb::arg("has_upper"),
    nb::arg("max_iterations"),
    nb::arg("gradient_tolerance"),
    nb::arg("damping"),
    nb::arg("line_search"),
    nb::arg("verbose"),
    nb::arg("sparse_solver_kind"));

  m.def("_solve_newton", &solveNewton,
    nb::arg("energy"),
    nb::arg("x0"),
    nb::arg("fixed_dofs"),
    nb::arg("fixed_values"),
    nb::arg("has_fixed_values"),
    nb::arg("max_iter"),
    nb::arg("tol"),
    nb::arg("damping"),
    nb::arg("line_search"),
    nb::arg("verbose"),
    nb::arg("sparse_solver_kind"));
}
