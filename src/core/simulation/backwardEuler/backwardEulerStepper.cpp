#include "backwardEuler/backwardEulerStepper.h"

#include "common/dynamicStepperUtils.h"
#include "common/rayleighDampingAssembly.h"

#include <utility>

namespace pgo
{
namespace Simulation
{
namespace NO = pgo::NonlinearOptimization;
namespace NOO = pgo::NonlinearOptimization::Optimization;
namespace ES = pgo::EigenSupport;

BackwardEulerStepper::BackwardEulerStepper(DynamicProblem problem)
  : problem_(std::move(problem))
{
  n_ = (int)problem_.mass.rows();
  validateDynamicProblem(problem_, n_);

  const DynamicState zero = makeZeroState(n_);
  const DynamicStepRequest zeroReq = makeZeroRequest(n_);
  const ES::SpMatD D = assembleRayleighDamping(problem_.persistentTerms, problem_.mass, zero.displacement);
  const IBEStageCoefficients coeffs = builder_.compute(zero, problem_, zeroReq, D);

  stageHandle_ = initStageResidual(coeffs.A, ES::VXd::Zero(n_), problem_.persistentTerms);
}

DynamicStepResult BackwardEulerStepper::step(
  const DynamicState &state,
  const DynamicStepRequest &request,
  NOO::Optimizer &optimizer)
{
  validateDynamicState(state, n_);
  validateDynamicStepRequest(request, problem_, n_);

  dispatchBeginStep(problem_, state);

  const ES::SpMatD D = assembleRayleighDamping(problem_.persistentTerms, problem_.mass, state.displacement);
  const IBEStageCoefficients coeffs = builder_.compute(state, problem_, request, D);
  const ImplicitStageProblem stage = prepareStageResidual(stageHandle_, coeffs.A, coeffs.linear, coeffs.initialGuess);

  const ES::VXd fixedValues = resolveFixedValues(problem_, state, request);
  const NOO::OptimizationResult res = solveStageProblem(stage, problem_.fixedDofs, fixedValues, optimizer);

  DynamicStepResult out;
  out.solver = res.solver;
  out.stageResults.push_back(res.solver);
  out.accepted = NO::acceptsDynamicSolveStatus(res.solver.status);
  out.state = out.accepted ? updateBackwardEulerState(state, res.x, problem_.timestep) : state;
  return out;
}

}  // namespace Simulation
}  // namespace pgo
