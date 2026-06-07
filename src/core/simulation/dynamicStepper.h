/*
  DynamicStepper: the unified dynamic time-stepping service.

  One public step() interface drives implicit backward Euler (single stage) and
  TRBDF2 (two stages). The immutable DynamicProblem is supplied at construction;
  per step only a DynamicStepRequest (external force, optional fixed values) is
  passed. Each stepper holds its persistent stage residual(s) and updates them
  in place. See Tasks T7.

  Contact note: beginStep is dispatched to every StepAwareEnergy term. Contact
  active sets are prepared at nonlinear evaluation points by the solver.
*/

#pragma once

#include "dynamicState.h"
#include "dynamicStepOptions.h"
#include "solver/common/solverResult.h"
#include "solver/service/optimizationService.h"

#include <memory>
#include <vector>

namespace pgo
{
namespace Simulation
{

struct DynamicStepResult
{
  DynamicState state;
  NonlinearOptimization::SolverResult solver;
  std::vector<NonlinearOptimization::SolverResult> stageResults;
  bool accepted = false;
};

class DynamicStepper
{
public:
  virtual ~DynamicStepper() = default;
  virtual DynamicStepResult step(
    const DynamicState &state,
    const DynamicStepRequest &request,
    NonlinearOptimization::Optimization::Optimizer &optimizer) = 0;
  virtual int numDofs() const = 0;
};

}  // namespace Simulation
}  // namespace pgo
