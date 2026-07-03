#include "trbdf2StageBuilder.h"

#include <cmath>
#include <stdexcept>

namespace pgo
{
namespace Simulation
{

TRBDF2Coefficients computeTRBDF2Coefficients(double gamma, double timestep)
{
  if (!(gamma > 0.0) || gamma > 1.0 || !std::isfinite(gamma))
    throw std::invalid_argument("computeTRBDF2Coefficients: gamma must be in (0, 1]");
  if (!(timestep > 0.0) || !std::isfinite(timestep))
    throw std::invalid_argument("computeTRBDF2Coefficients: timestep must be positive");

  const double y = gamma;
  const double h = timestep;

  TRBDF2Coefficients c;
  c.gamma = y;
  c.alpha = 2.0 / (y * h);

  c.beta[0] = (2.0 - y) / (y * (1.0 - y) * (1.0 - y) * h * h);
  c.beta[1] = -c.beta[0];
  c.beta[2] = (1.0 - y) / (y * h);
  c.beta[3] = -1.0 / (y * (1.0 - y) * h);
  c.beta[4] = (2.0 - y) * (2.0 - y) / ((1.0 - y) * (1.0 - y) * h * h);
  c.beta[5] = 1.0 / (y * (1.0 - y) * h);
  c.beta[6] = -c.beta[5];
  c.beta[7] = (2.0 - y) / ((1.0 - y) * h);

  return c;
}

TRBDF2StageCoefficients TRBDF2StageBuilder::computeStage1(
  const DynamicState &state,
  const DynamicProblem &problem,
  const DynamicStepRequest &request,
  const EigenSupport::SpMatD &damping,
  const TRBDF2Coefficients &coeffs) const
{
  const EigenSupport::SpMatD &M = problem.mass;
  const double alpha = coeffs.alpha;

  TRBDF2StageCoefficients out;

  // A1 = α²M + αD
  out.A = (alpha * alpha) * M + alpha * damping;

  // linear1 = -(2αM v + M a + D v + f_ext) - A1 u
  out.linear =
    -(2.0 * alpha * (M * state.velocity) + M * state.acceleration + damping * state.velocity + request.externalForce)
    - out.A * state.displacement;

  out.initialGuess = state.displacement;
  return out;
}

TRBDF2IntermediateState TRBDF2StageBuilder::updateAfterStage1(
  const DynamicState &state,
  EigenSupport::ConstRefVecXd stage1Solution,
  const TRBDF2Coefficients &coeffs) const
{
  const double alpha = coeffs.alpha;
  const EigenSupport::VXd du = stage1Solution - state.displacement;

  TRBDF2IntermediateState mid;
  mid.uy = stage1Solution;
  mid.vy = alpha * du - state.velocity;
  mid.ay = (alpha * alpha) * du - 2.0 * alpha * state.velocity - state.acceleration;
  return mid;
}

TRBDF2StageCoefficients TRBDF2StageBuilder::computeStage2(
  const DynamicState &state,
  const TRBDF2IntermediateState &intermediate,
  const DynamicProblem &problem,
  const DynamicStepRequest &request,
  const EigenSupport::SpMatD &damping,
  const TRBDF2Coefficients &coeffs) const
{
  const EigenSupport::SpMatD &M = problem.mass;
  const double *beta = coeffs.beta;

  TRBDF2StageCoefficients out;

  // A2 = β4 M + β7 D
  out.A = beta[4] * M + beta[7] * damping;

  // linear2 = M(β0 u + β1 uy + β2 v + β3 vy) + D(β5 u + β6 uy) - f_ext - A2 u
  const EigenSupport::VXd massTerm =
    beta[0] * state.displacement + beta[1] * intermediate.uy + beta[2] * state.velocity + beta[3] * intermediate.vy;
  const EigenSupport::VXd dampTerm =
    beta[5] * state.displacement + beta[6] * intermediate.uy;

  out.linear = M * massTerm + damping * dampTerm - request.externalForce - out.A * state.displacement;

  out.initialGuess = intermediate.uy;
  return out;
}

DynamicState TRBDF2StageBuilder::updateAfterStage2(
  const DynamicState &state,
  const TRBDF2IntermediateState &intermediate,
  EigenSupport::ConstRefVecXd stage2Solution,
  const TRBDF2Coefficients &coeffs) const
{
  const double *beta = coeffs.beta;
  const EigenSupport::VXd du = stage2Solution - state.displacement;

  DynamicState next;
  next.displacement = stage2Solution;
  next.velocity = beta[5] * state.displacement + beta[6] * intermediate.uy + beta[7] * du;
  next.acceleration =
    beta[0] * state.displacement + beta[1] * intermediate.uy + beta[2] * state.velocity + beta[3] * intermediate.vy + beta[4] * du;
  next.timestepId = state.timestepId + 1;
  next.time = state.time + (2.0 / (coeffs.alpha * coeffs.gamma));  // = state.time + timestep
  return next;
}

}  // namespace Simulation
}  // namespace pgo
