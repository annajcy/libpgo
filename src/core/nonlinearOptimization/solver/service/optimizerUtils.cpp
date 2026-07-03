#include "solver/service/optimizerUtils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace pgo::NonlinearOptimization::Optimization
{
namespace
{

void validateBoundVector(const std::optional<EigenSupport::VXd> &v, int size, const char *name)
{
  if (v.has_value() && v->size() != size) {
    throw std::invalid_argument(std::string(name) + " size must match target dimension");
  }
}

}  // namespace

MaterializedBounds materializeBounds(
  const Bounds &bounds,
  int size,
  double negativeInfinity,
  double positiveInfinity)
{
  validateBoundVector(bounds.lower, size, "lower bounds");
  validateBoundVector(bounds.upper, size, "upper bounds");

  MaterializedBounds out;
  out.lower = bounds.lower.value_or(EigenSupport::VXd::Constant(size, negativeInfinity));
  out.upper = bounds.upper.value_or(EigenSupport::VXd::Constant(size, positiveInfinity));

  for (int i = 0; i < size; ++i) {
    if (out.lower[i] > out.upper[i]) {
      throw std::invalid_argument("lower bounds must be <= upper bounds");
    }
  }

  return out;
}

FixedDofsFromBounds extractFixedDofsFromVariableBounds(
  const Bounds &bounds,
  int numDofs)
{
  validateBoundVector(bounds.lower, numDofs, "variable lower bounds");
  validateBoundVector(bounds.upper, numDofs, "variable upper bounds");

  FixedDofsFromBounds out;
  std::vector<double> values;

  for (int i = 0; i < numDofs; ++i) {
    const bool hasLower = bounds.lower.has_value() && std::isfinite((*bounds.lower)[i]);
    const bool hasUpper = bounds.upper.has_value() && std::isfinite((*bounds.upper)[i]);
    if (!hasLower && !hasUpper) {
      continue;
    }

    const std::optional<double> lo = hasLower ? std::optional<double>((*bounds.lower)[i]) : std::nullopt;
    const std::optional<double> hi = hasUpper ? std::optional<double>((*bounds.upper)[i]) : std::nullopt;
    if (lo.has_value() && hi.has_value() && *lo > *hi) {
      throw std::invalid_argument("variable lower bounds must be <= upper bounds");
    }

    if (lo.has_value() && hi.has_value() && *lo == *hi) {
      out.dofs.push_back(i);
      values.push_back(*lo);
    }
    else {
      out.hasGeneralBounds = true;
    }
  }

  out.values.resize(static_cast<Eigen::Index>(values.size()));
  for (Eigen::Index i = 0; i < out.values.size(); ++i) {
    out.values[i] = values[static_cast<size_t>(i)];
  }

  return out;
}

void fixVariables(
  OptimizationProblem &problem,
  std::span<const int> dofs,
  EigenSupport::ConstRefVecXd values,
  int numDofs)
{
  if (values.size() != static_cast<Eigen::Index>(dofs.size())) {
    throw std::invalid_argument("fixed values size must match fixed dofs size");
  }

  validateBoundVector(problem.variableBounds.lower, numDofs, "variable lower bounds");
  validateBoundVector(problem.variableBounds.upper, numDofs, "variable upper bounds");

  if (!problem.variableBounds.lower.has_value()) {
    problem.variableBounds.lower = EigenSupport::VXd::Constant(numDofs, -std::numeric_limits<double>::infinity());
  }
  if (!problem.variableBounds.upper.has_value()) {
    problem.variableBounds.upper = EigenSupport::VXd::Constant(numDofs, std::numeric_limits<double>::infinity());
  }

  std::vector<int> sorted(dofs.begin(), dofs.end());
  std::sort(sorted.begin(), sorted.end());
  if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end()) {
    throw std::invalid_argument("fixed dofs must be unique");
  }

  for (size_t i = 0; i < dofs.size(); ++i) {
    const int dof = dofs[i];
    if (dof < 0 || dof >= numDofs) {
      throw std::invalid_argument("fixed dof out of range");
    }

    (*problem.variableBounds.lower)[dof] = values[static_cast<Eigen::Index>(i)];
    (*problem.variableBounds.upper)[dof] = values[static_cast<Eigen::Index>(i)];
  }
}

}  // namespace pgo::NonlinearOptimization::Optimization
