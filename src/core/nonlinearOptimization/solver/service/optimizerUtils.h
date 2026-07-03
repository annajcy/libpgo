#pragma once

#include "solver/service/optimizationProblem.h"

#include <span>

namespace pgo::NonlinearOptimization::Optimization
{

struct MaterializedBounds
{
  EigenSupport::VXd lower;
  EigenSupport::VXd upper;
};

MaterializedBounds materializeBounds(
  const Bounds &bounds,
  int size,
  double negativeInfinity,
  double positiveInfinity);

struct FixedDofsFromBounds
{
  std::vector<int> dofs;
  EigenSupport::VXd values;
  bool hasGeneralBounds = false;
};

FixedDofsFromBounds extractFixedDofsFromVariableBounds(
  const Bounds &bounds,
  int numDofs);

void fixVariables(
  OptimizationProblem &problem,
  std::span<const int> dofs,
  EigenSupport::ConstRefVecXd values,
  int numDofs);

}  // namespace pgo::NonlinearOptimization::Optimization
