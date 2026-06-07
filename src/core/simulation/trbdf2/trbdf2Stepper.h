#pragma once

#include "common/stageResidual.h"
#include "dynamicStepper.h"
#include "trbdf2/trbdf2StageBuilder.h"

namespace pgo
{
namespace Simulation
{

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
