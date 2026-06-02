#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "eigen_numpy.h"
#include "energy_core.h"
#include "optimizationService.h"

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

NonlinearOptimization::LineSearchMethod parseLineSearch(const std::string &name)
{
  if (name == "golden")
    return NonlinearOptimization::LineSearchMethod::Golden;
  if (name == "brents")
    return NonlinearOptimization::LineSearchMethod::Brents;
  if (name == "backtrack")
    return NonlinearOptimization::LineSearchMethod::Backtrack;
  if (name == "simple")
    return NonlinearOptimization::LineSearchMethod::Simple;
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
  int verbose)
{
  auto x0Map = python::ndarrayToVectorMapXd(x0);

  NonlinearOptimization::OptimizationProblem problem;
  problem.energy = energy->handle_;

  if (!fixedDofs.empty() || hasFixedValues) {
    NonlinearOptimization::FixedVariables fixed;
    fixed.dofs = std::move(fixedDofs);
    if (hasFixedValues) {
      fixed.values = python::ndarrayToVectorXd(fixedValues);
    }
    problem.fixedVariables = std::move(fixed);
  }

  NonlinearOptimization::NewtonOptions options;
  options.control.maxIterations = maxIter;
  options.control.tolerance = tol;
  options.control.verbose = verbose;
  options.damping = damping;
  options.lineSearch = parseLineSearch(lineSearch);

  NonlinearOptimization::OptimizationResult result;
  {
    nb::gil_scoped_release release;
    result = NonlinearOptimization::minimize(problem, x0Map, options);
  }

  const auto &solver = result.solver;
  nb::dict out;
  out["x"] = python::vectorXdToNdarray(std::move(result.x));
  out["status"] = static_cast<int>(solver.status);
  out["converged"] = solver.converged();
  out["iterations"] = solver.iterations;
  out["raw_status_code"] = solver.rawStatusCode;
  out["final_objective"] = result.hasFinalObjective ? nb::object(nb::float_(result.finalObjective)) : nb::none();
  out["final_gradient_norm"] = solver.hasFinalGradientStats ? nb::object(nb::float_(solver.finalGradientNorm)) : nb::none();
  out["final_gradient_max_norm"] = solver.hasFinalGradientStats ? nb::object(nb::float_(solver.finalGradientMaxNorm)) : nb::none();
  out["diagnostics"] = diagnosticsToDict(solver.diagnostics);
  return out;
}

}  // namespace

void init_solver_bindings(nb::module_ &m)
{
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
    nb::arg("verbose"));
}
