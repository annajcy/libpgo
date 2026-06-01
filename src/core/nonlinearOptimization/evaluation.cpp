#include "evaluation.h"

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

}  // namespace

double evaluateValue(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x)
{
  validateStateSize(energy, x);
  return energy.func(x);
}

EigenSupport::VXd evaluateGradient(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x)
{
  validateStateSize(energy, x);
  EigenSupport::VXd grad(energy.getNumDOFs());
  energy.gradient(x, grad);
  return grad;
}

EigenSupport::SpMatD evaluateHessian(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x)
{
  validateStateSize(energy, x);
  EigenSupport::SpMatD H;
  energy.hessian(x, H);
  return H;
}

MaxStepResult evaluateMaxStep(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx)
{
  validateStateSize(energy, x);
  return energy.computeMaxStepLimit(x, dx);
}

std::vector<int> dofsOf(const PotentialEnergy &energy)
{
  std::vector<int> dofs;
  energy.getDOFs(dofs);
  return dofs;
}

}  // namespace pgo::NonlinearOptimization
