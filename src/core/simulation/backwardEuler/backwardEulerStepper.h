#pragma once

#include "backwardEuler/backwardEulerStageBuilder.h"
#include "common/stageResidual.h"
#include "dynamicStepper.h"

namespace pgo
{
namespace Simulation
{

class BackwardEulerStepper final : public DynamicStepper
{
public:
  explicit BackwardEulerStepper(DynamicProblem problem);
  DynamicStepResult step(
    const DynamicState &state,
    const DynamicStepRequest &request,
    NonlinearOptimization::Optimization::Optimizer &optimizer) override;
  int numDofs() const override { return n_; }

  // Expose the stage EnergySet for max-step tests.
  NonlinearOptimization::EnergySet_const_p getStageEnergy() const { return stageHandle_.energySet; }

private:
  DynamicProblem problem_;
  int n_;
  BackwardEulerStageBuilder builder_;
  StageResidualHandle stageHandle_;
};

}  // namespace Simulation
}  // namespace pgo
