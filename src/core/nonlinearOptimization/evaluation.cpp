#include "evaluation.h"

#include "evaluationStateAwareEnergy.h"

#include <stdexcept>

namespace pgo::NonlinearOptimization
{
namespace
{

void validateStateSize(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x)
{
  if (x.size() != energy.getNumDOFs()) {
    throw std::invalid_argument("State size mismatch: energy has " +
      std::to_string(energy.getNumDOFs()) + " DOFs, but x has size " +
      std::to_string(x.size()));
  }
}

void prepareEvaluationStateIfNeeded(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x)
{
  if (const auto *aware = dynamic_cast<const EvaluationStateAwareEnergy *>(&energy))
    aware->prepareEvaluationState(x);
}

}  // namespace

double evaluateValue(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x)
{
  validateStateSize(energy, x);
  prepareEvaluationStateIfNeeded(energy, x);
  return energy.func(x);
}

EigenSupport::VXd evaluateGradient(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x)
{
  validateStateSize(energy, x);
  prepareEvaluationStateIfNeeded(energy, x);
  EigenSupport::VXd grad(energy.getNumDOFs());
  energy.gradient(x, grad);
  return grad;
}

EigenSupport::SpMatD evaluateHessian(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x)
{
  validateStateSize(energy, x);
  prepareEvaluationStateIfNeeded(energy, x);
  EigenSupport::SpMatD H;
  energy.hessian(x, H);
  return H;
}

StepConstraint evaluateMaxStep(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx, StepConstraintSink *sink)
{
  validateStateSize(energy, x);
  prepareEvaluationStateIfNeeded(energy, x);
  return energy.computeMaxStepLimit(x, dx, sink);
}

std::vector<int> dofsOf(const PotentialEnergy &energy)
{
  std::vector<int> dofs;
  energy.getDOFs(dofs);
  return dofs;
}

}  // namespace pgo::NonlinearOptimization
