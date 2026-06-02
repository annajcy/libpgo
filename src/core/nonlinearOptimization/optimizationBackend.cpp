#include "optimizationBackend.h"

#include "NewtonSolver.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace pgo::NonlinearOptimization
{
namespace
{

void validateControl(const SolverControl &control)
{
  if (control.maxIterations < 0) {
    throw std::invalid_argument("Newton maxIterations must be non-negative");
  }
  if (control.tolerance < 0.0) {
    throw std::invalid_argument("Newton tolerance must be non-negative");
  }
  if (control.verbose < 0) {
    throw std::invalid_argument("Newton verbose must be non-negative");
  }
}

void validateRejectedFutureFields(const OptimizationProblem &problem, int numDofs)
{
  if (problem.bounds.has_value()) {
    const auto &bounds = *problem.bounds;
    if (bounds.lower.size() != numDofs || bounds.upper.size() != numDofs) {
      throw std::invalid_argument("BoxBounds lower/upper must match problem size");
    }
    throw std::invalid_argument("Newton backend does not support bounds");
  }

  if (problem.constraints.has_value()) {
    const auto &constraints = *problem.constraints;
    if (!constraints.functions) {
      throw std::invalid_argument("NonlinearConstraints functions must not be null");
    }
    throw std::invalid_argument("Newton backend does not support nonlinear constraints");
  }
}

FixedVariables normalizeFixedVariables(
  const std::optional<FixedVariables> &input,
  EigenSupport::ConstRefVecXd x0)
{
  FixedVariables normalized;
  if (!input.has_value() || input->dofs.empty()) {
    return normalized;
  }

  const auto &fixed = *input;
  if (fixed.values.has_value() &&
      fixed.values->size() != static_cast<Eigen::Index>(fixed.dofs.size())) {
    throw std::invalid_argument("FixedVariables values size must match dofs size");
  }

  std::vector<std::pair<int, double>> pairs;
  pairs.reserve(fixed.dofs.size());
  for (size_t i = 0; i < fixed.dofs.size(); ++i) {
    const int dof = fixed.dofs[i];
    if (dof < 0 || dof >= x0.size()) {
      throw std::invalid_argument("Fixed variable dof is out of range");
    }
    const double value = fixed.values.has_value() ? (*fixed.values)[static_cast<Eigen::Index>(i)] : x0[dof];
    pairs.emplace_back(dof, value);
  }

  std::sort(pairs.begin(), pairs.end(),
    [](const auto &a, const auto &b) { return a.first < b.first; });
  for (size_t i = 1; i < pairs.size(); ++i) {
    if (pairs[i - 1].first == pairs[i].first) {
      throw std::invalid_argument("Fixed variable dofs must be unique");
    }
  }

  normalized.dofs.reserve(pairs.size());
  EigenSupport::VXd values(static_cast<Eigen::Index>(pairs.size()));
  for (size_t i = 0; i < pairs.size(); ++i) {
    normalized.dofs.push_back(pairs[i].first);
    values[static_cast<Eigen::Index>(i)] = pairs[i].second;
  }
  normalized.values = std::move(values);
  return normalized;
}

}  // namespace

NewtonOptimizationBackend::NewtonOptimizationBackend(NewtonOptions options):
  options_(std::move(options))
{
}

OptimizationResult NewtonOptimizationBackend::solve(
  const OptimizationProblem &problem,
  EigenSupport::ConstRefVecXd x0)
{
  if (!problem.energy) {
    throw std::invalid_argument("OptimizationProblem energy must not be null");
  }

  const int numDofs = problem.energy->getNumDOFs();
  if (x0.size() != numDofs) {
    throw std::invalid_argument("Initial state size must match problem energy DOFs");
  }

  validateControl(options_.control);
  validateRejectedFutureFields(problem, numDofs);

  FixedVariables fixed = normalizeFixedVariables(problem.fixedVariables, x0);

  EigenSupport::VXd x = x0;

  NewtonSolver::SolverParam sp;
  sp.addDamping = options_.damping ? 1 : 0;
  sp.sst = NewtonSolver::SST_SUBITERATION_LINE_SEARCH;
  sp.lineSearch = options_.lineSearch;

  const double *fixedValuesPtr = fixed.values.has_value() ? fixed.values->data() : nullptr;
  NewtonSolver solver(x.data(), sp, problem.energy, fixed.dofs, fixedValuesPtr, options_.sparseSolver);
  SolverResult solverResult = solver.solve(
    x.data(),
    options_.control.maxIterations,
    options_.control.tolerance,
    options_.control.verbose);

  OptimizationResult result;
  result.solver = std::move(solverResult);
  result.x = std::move(x);

  const double finalObjective = problem.energy->func(result.x);
  if (std::isfinite(finalObjective)) {
    result.hasFinalObjective = true;
    result.finalObjective = finalObjective;
  }

  return result;
}

}  // namespace pgo::NonlinearOptimization
