/*
  TRBDF2 two-stage builder.

  Pure coefficient computation matching the TRBDF2 formulas
  (updateCoeffs / updateA1 / updateb1 / updateA2 / updateb2) plus the
  intermediate and final state updates. See Task T6.
*/

#pragma once

#include "dynamicState.h"
#include "dynamicStepOptions.h"
#include "EigenDef.h"

namespace pgo
{
namespace Simulation
{

struct TRBDF2Coefficients
{
  double gamma = 0.5;
  double alpha = 0.0;
  double beta[8] = {};
};

// Throws std::invalid_argument unless 0 < gamma <= 1 and timestep > 0.
TRBDF2Coefficients computeTRBDF2Coefficients(double gamma, double timestep);

struct TRBDF2StageCoefficients
{
  EigenSupport::SpMatD A;
  EigenSupport::VXd linear;
  EigenSupport::VXd initialGuess;
};

struct TRBDF2IntermediateState
{
  EigenSupport::VXd uy;
  EigenSupport::VXd vy;
  EigenSupport::VXd ay;
};

class TRBDF2StageBuilder
{
public:
  TRBDF2StageCoefficients computeStage1(
    const DynamicState &state,
    const DynamicProblem &problem,
    const DynamicStepRequest &request,
    const EigenSupport::SpMatD &damping,
    const TRBDF2Coefficients &coeffs) const;

  TRBDF2IntermediateState updateAfterStage1(
    const DynamicState &state,
    EigenSupport::ConstRefVecXd stage1Solution,
    const TRBDF2Coefficients &coeffs) const;

  TRBDF2StageCoefficients computeStage2(
    const DynamicState &state,
    const TRBDF2IntermediateState &intermediate,
    const DynamicProblem &problem,
    const DynamicStepRequest &request,
    const EigenSupport::SpMatD &damping,
    const TRBDF2Coefficients &coeffs) const;

  DynamicState updateAfterStage2(
    const DynamicState &state,
    const TRBDF2IntermediateState &intermediate,
    EigenSupport::ConstRefVecXd stage2Solution,
    const TRBDF2Coefficients &coeffs) const;
};

}  // namespace Simulation
}  // namespace pgo
