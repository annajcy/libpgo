#pragma once

#include "EigenSupport.h"
#include "solver/common/solverResult.h"

#include <optional>

namespace pgo::NonlinearOptimization::Optimization
{

struct OptimizationResult
{
  SolverResult solver;
  EigenSupport::VXd x;
  std::optional<double> finalObjective;
  std::optional<EigenSupport::VXd> multipliers;
  std::optional<EigenSupport::VXd> constraintValues;
};

}  // namespace pgo::NonlinearOptimization::Optimization
