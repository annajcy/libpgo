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
#include "stageResidual.h"
#include "energySet.h"
#include "implicitEulerStageBuilder.h"
#include "trbdf2StageBuilder.h"
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

class ImplicitEulerStepper final : public DynamicStepper
{
public:
  explicit ImplicitEulerStepper(DynamicProblem problem);
  DynamicStepResult step(
    const DynamicState &state,
    const DynamicStepRequest &request,
    NonlinearOptimization::Optimization::Optimizer &optimizer) override;
  int numDofs() const override { return n_; }

  // Expose the stage EnergySet for legacy getInternalEnergy() / max-step tests.
  NonlinearOptimization::EnergySet_const_p getStageEnergy() const { return stageHandle_.energySet; }

private:
  DynamicProblem problem_;
  int n_;
  ImplicitEulerStageBuilder builder_;
  StageResidualHandle stageHandle_;
};

class TRBDF2Stepper final : public DynamicStepper
{
public:
  explicit TRBDF2Stepper(DynamicProblem problem, double gamma = 0.5);
  DynamicStepResult step(
    const DynamicState &state,
    const DynamicStepRequest &request,
    NonlinearOptimization::Optimization::Optimizer &optimizer) override;
  int numDofs() const override { return n_; }

  // Expose stage energies for legacy tests.
  NonlinearOptimization::EnergySet_const_p getStage1Energy() const { return stage1Handle_.energySet; }
  NonlinearOptimization::EnergySet_const_p getStage2Energy() const { return stage2Handle_.energySet; }

private:
  DynamicProblem problem_;
  int n_;
  TRBDF2Coefficients coeffs_;
  bool singleStage_;
  TRBDF2StageBuilder builder_;
  StageResidualHandle stage1Handle_;
  StageResidualHandle stage2Handle_;
};

}  // namespace Simulation
}  // namespace pgo
