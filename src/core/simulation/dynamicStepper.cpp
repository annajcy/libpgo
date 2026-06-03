#include "dynamicStepper.h"
#include "rayleighDampingAssembly.h"
#include "stepAwareEnergy.h"

#include <utility>

namespace pgo
{
namespace Simulation
{
namespace NO = pgo::NonlinearOptimization;
namespace NOO = pgo::NonlinearOptimization::Optimization;
namespace ES = pgo::EigenSupport;

namespace
{
DynamicState makeZeroState(int n)
{
  DynamicState s;
  s.displacement = ES::VXd::Zero(n);
  s.velocity = ES::VXd::Zero(n);
  s.acceleration = ES::VXd::Zero(n);
  return s;
}

DynamicStepRequest makeZeroRequest(int n)
{
  DynamicStepRequest r;
  r.externalForce = ES::VXd::Zero(n);
  return r;
}

// Dispatch beginStep to every step-aware persistent energy. Contact active-set
// refresh (StatefulContactEnergy) plugs in here once contact_api_refactor lands.
void dispatchBeginStep(const DynamicProblem &problem, const DynamicState &state)
{
  NO::StepState ss;
  ss.time = state.time;
  ss.timestep = problem.timestep;
  ss.previousX = &state.displacement;

  for (const ImplicitModelTerm &term : problem.persistentTerms) {
    if (auto *aware = dynamic_cast<NO::StepAwareEnergy *>(term.energy.get()))
      aware->beginStep(ss);
  }
}

ES::VXd resolveFixedValues(const DynamicProblem &problem, const DynamicState &state, const DynamicStepRequest &request)
{
  if (request.fixedValues)
    return *request.fixedValues;

  ES::VXd values((Eigen::Index)problem.fixedDofs.size());
  for (size_t k = 0; k < problem.fixedDofs.size(); k++)
    values[(Eigen::Index)k] = state.displacement[problem.fixedDofs[k]];
  return values;
}

NOO::OptimizationResult solveStageProblem(
  const ImplicitStageProblem &stage,
  const std::vector<int> &fixedDofs,
  const ES::VXd &fixedValues,
  NOO::Optimizer &optimizer)
{
  NOO::OptimizationProblem problem;
  problem.objective = stage.energy;

  ES::VXd x0 = stage.initialGuess;
  if (!fixedDofs.empty()) {
    NOO::fixVariables(problem, fixedDofs, fixedValues, static_cast<int>(x0.size()));

    // Pin the initial guess on fixed DOFs so the solver starts feasible.
    for (size_t k = 0; k < fixedDofs.size(); k++)
      x0[fixedDofs[k]] = fixedValues[(Eigen::Index)k];
  }

  return optimizer.solve(problem, x0);
}
}  // namespace

// ── Implicit backward Euler ─────────────────────────────────────────────

ImplicitEulerStepper::ImplicitEulerStepper(DynamicProblem problem)
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

DynamicStepResult ImplicitEulerStepper::step(
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
  out.state = out.accepted ? updateImplicitEulerState(state, res.x, problem_.timestep) : state;
  return out;
}

// ── TRBDF2 ──────────────────────────────────────────────────────────────

TRBDF2Stepper::TRBDF2Stepper(DynamicProblem problem, double gamma)
  : problem_(std::move(problem))
{
  n_ = (int)problem_.mass.rows();
  validateDynamicProblem(problem_, n_);

  coeffs_ = computeTRBDF2Coefficients(gamma, problem_.timestep);
  singleStage_ = (gamma >= 1.0 - 1e-9);

  const DynamicState zero = makeZeroState(n_);
  const DynamicStepRequest zeroReq = makeZeroRequest(n_);
  const ES::SpMatD D = assembleRayleighDamping(problem_.persistentTerms, problem_.mass, zero.displacement);

  const TRBDF2StageCoefficients c1 = builder_.computeStage1(zero, problem_, zeroReq, D, coeffs_);
  stage1Handle_ = initStageResidual(c1.A, ES::VXd::Zero(n_), problem_.persistentTerms);

  if (!singleStage_) {
    TRBDF2IntermediateState midZero;
    midZero.uy = ES::VXd::Zero(n_);
    midZero.vy = ES::VXd::Zero(n_);
    midZero.ay = ES::VXd::Zero(n_);
    const TRBDF2StageCoefficients c2 = builder_.computeStage2(zero, midZero, problem_, zeroReq, D, coeffs_);
    stage2Handle_ = initStageResidual(c2.A, ES::VXd::Zero(n_), problem_.persistentTerms);
  }
}

DynamicStepResult TRBDF2Stepper::step(
  const DynamicState &state,
  const DynamicStepRequest &request,
  NOO::Optimizer &optimizer)
{
  validateDynamicState(state, n_);
  validateDynamicStepRequest(request, problem_, n_);

  dispatchBeginStep(problem_, state);

  const ES::SpMatD D = assembleRayleighDamping(problem_.persistentTerms, problem_.mass, state.displacement);
  const ES::VXd fixedValues = resolveFixedValues(problem_, state, request);

  // Stage 1 (trapezoidal rule).
  const TRBDF2StageCoefficients c1 = builder_.computeStage1(state, problem_, request, D, coeffs_);
  const ImplicitStageProblem stage1 = prepareStageResidual(stage1Handle_, c1.A, c1.linear, c1.initialGuess);
  const NOO::OptimizationResult res1 = solveStageProblem(stage1, problem_.fixedDofs, fixedValues, optimizer);

  DynamicStepResult out;
  out.stageResults.push_back(res1.solver);

  if (!NO::acceptsDynamicSolveStatus(res1.solver.status)) {
    out.solver = res1.solver;
    out.accepted = false;
    out.state = state;
    return out;
  }

  const TRBDF2IntermediateState mid = builder_.updateAfterStage1(state, res1.x, coeffs_);

  if (singleStage_) {
    // gamma == 1: the trapezoidal intermediate state is the final state.
    DynamicState next;
    next.displacement = mid.uy;
    next.velocity = mid.vy;
    next.acceleration = mid.ay;
    next.timestepId = state.timestepId + 1;
    next.time = state.time + problem_.timestep;

    out.solver = res1.solver;
    out.accepted = true;
    out.state = std::move(next);
    return out;
  }

  // Stage 2 (BDF2).
  const TRBDF2StageCoefficients c2 = builder_.computeStage2(state, mid, problem_, request, D, coeffs_);
  const ImplicitStageProblem stage2 = prepareStageResidual(stage2Handle_, c2.A, c2.linear, c2.initialGuess);
  const NOO::OptimizationResult res2 = solveStageProblem(stage2, problem_.fixedDofs, fixedValues, optimizer);

  out.stageResults.push_back(res2.solver);
  out.solver = res2.solver;

  if (!NO::acceptsDynamicSolveStatus(res2.solver.status)) {
    out.accepted = false;
    out.state = state;
    return out;
  }

  out.accepted = true;
  out.state = builder_.updateAfterStage2(state, mid, res2.x, coeffs_);
  return out;
}

}  // namespace Simulation
}  // namespace pgo
