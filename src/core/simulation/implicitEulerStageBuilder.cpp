#include "implicitEulerStageBuilder.h"

namespace pgo
{
namespace Simulation
{

IBEStageCoefficients ImplicitEulerStageBuilder::compute(
  const DynamicState &state,
  const DynamicProblem &problem,
  const DynamicStepRequest &request,
  const EigenSupport::SpMatD &damping) const
{
  const double h = problem.timestep;
  const EigenSupport::SpMatD &M = problem.mass;

  IBEStageCoefficients coeffs;

  // A = M/h² + D/h. The union pattern (M ∪ D) is state-independent.
  coeffs.A = (1.0 / (h * h)) * M + (1.0 / h) * damping;

  // linear = -(f_ext + M v/h + A u)
  coeffs.linear = -(request.externalForce + (1.0 / h) * (M * state.velocity) + coeffs.A * state.displacement);

  coeffs.initialGuess = state.displacement;
  return coeffs;
}

DynamicState updateImplicitEulerState(
  const DynamicState &state,
  EigenSupport::ConstRefVecXd solution,
  double timestep)
{
  DynamicState next;
  next.displacement = solution;
  next.velocity = (solution - state.displacement) / timestep;
  next.acceleration = (next.velocity - state.velocity) / timestep;
  next.timestepId = state.timestepId + 1;
  next.time = state.time + timestep;
  return next;
}

}  // namespace Simulation
}  // namespace pgo
