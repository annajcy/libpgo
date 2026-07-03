#pragma once

#include "EigenSupport.h"
#include "constraints/constraintFunctions.h"
#include "energy/potentialEnergy.h"

#include <optional>
#include <vector>

namespace pgo::NonlinearOptimization::Optimization
{

struct Bounds
{
  std::optional<EigenSupport::VXd> lower;
  std::optional<EigenSupport::VXd> upper;
};

struct ConstraintBlock
{
  ConstraintFunctions_const_p functions;
  Bounds bounds;
};

struct OptimizationProblem
{
  PotentialEnergy_const_p objective;
  Bounds variableBounds;
  std::vector<ConstraintBlock> constraints;
};

}  // namespace pgo::NonlinearOptimization::Optimization
