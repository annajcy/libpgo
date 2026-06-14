#include "common/dynamicStepperUtils.h"

#include "stepAwareEnergy.h"
#include "solver/service/optimizerUtils.h"

namespace pgo
{
namespace Simulation
{
namespace NO = pgo::NonlinearOptimization;
namespace NOO = pgo::NonlinearOptimization::Optimization;
namespace ES = pgo::EigenSupport;

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

void dispatchBeginStep(const DynamicProblem &problem, const DynamicState &state)
{
  NO::StepState ss;
  ss.time = state.time;
  ss.timestep = problem.timestep;
  ss.currentX = &state.displacement;
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

}  // namespace Simulation
}  // namespace pgo
