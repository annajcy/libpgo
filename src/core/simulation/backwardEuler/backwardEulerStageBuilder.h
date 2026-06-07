/*
  Backward Euler stage builder.

  Pure coefficient computation for the single IBE stage:
    A   = M/h² + D/h
    l   = -(f_ext + M v/h + A u)
  and the post-solve state update. No EnergySet ownership — the stepper holds
  the persistent EnergySet and updates it via prepareStageResidual. See Task T5.
*/

#pragma once

#include "dynamicState.h"
#include "dynamicStepOptions.h"
#include "EigenDef.h"

namespace pgo
{
namespace Simulation
{

struct IBEStageCoefficients
{
  EigenSupport::SpMatD A;
  EigenSupport::VXd linear;
  EigenSupport::VXd initialGuess;
};

class BackwardEulerStageBuilder
{
public:
  IBEStageCoefficients compute(
    const DynamicState &state,
    const DynamicProblem &problem,
    const DynamicStepRequest &request,
    const EigenSupport::SpMatD &damping) const;
};

DynamicState updateBackwardEulerState(
  const DynamicState &state,
  EigenSupport::ConstRefVecXd solution,
  double timestep);

}  // namespace Simulation
}  // namespace pgo
