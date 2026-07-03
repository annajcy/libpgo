#pragma once

#include "dynamicState.h"
#include "dynamicStepOptions.h"
#include "common/stageResidual.h"
#include "EigenDef.h"
#include "solver/service/optimizationResult.h"
#include "solver/service/optimizationService.h"

#include <vector>

namespace pgo
{
namespace Simulation
{

DynamicState makeZeroState(int n);
DynamicStepRequest makeZeroRequest(int n);

void dispatchBeginStep(const DynamicProblem &problem, const DynamicState &state);

EigenSupport::VXd resolveFixedValues(
  const DynamicProblem &problem,
  const DynamicState &state,
  const DynamicStepRequest &request);

NonlinearOptimization::Optimization::OptimizationResult solveStageProblem(
  const ImplicitStageProblem &stage,
  const std::vector<int> &fixedDofs,
  const EigenSupport::VXd &fixedValues,
  NonlinearOptimization::Optimization::Optimizer &optimizer);

}  // namespace Simulation
}  // namespace pgo
