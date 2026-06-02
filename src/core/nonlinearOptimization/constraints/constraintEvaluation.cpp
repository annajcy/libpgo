/*
author: Bohan Wang
copyright to USC
*/

#include "constraintEvaluation.h"

#include <cstring>
#include <stdexcept>

using namespace pgo;
using namespace pgo::NonlinearOptimization;

namespace
{
void validateX(const ConstraintFunctions &constraints, EigenSupport::ConstRefVecXd x)
{
  if (x.size() != constraints.getNumDOFs()) {
    throw std::invalid_argument("Constraint state size mismatch");
  }
}

void validateMultipliers(const ConstraintFunctions &constraints, EigenSupport::ConstRefVecXd multipliers)
{
  if (multipliers.size() != constraints.getNumConstraints()) {
    throw std::invalid_argument("Constraint multiplier size mismatch");
  }
}
}  // namespace

EigenSupport::VXd pgo::NonlinearOptimization::evaluateConstraintValues(
  const ConstraintFunctions &constraints,
  EigenSupport::ConstRefVecXd x)
{
  validateX(constraints, x);
  EigenSupport::VXd values(constraints.getNumConstraints());
  constraints.func(x, values);
  return values;
}

EigenSupport::SpMatD pgo::NonlinearOptimization::evaluateConstraintJacobian(
  const ConstraintFunctions &constraints,
  EigenSupport::ConstRefVecXd x)
{
  validateX(constraints, x);
  EigenSupport::SpMatD jac;
  constraints.createJacobian(jac);
  constraints.jacobian(x, jac);
  return jac;
}

EigenSupport::SpMatD pgo::NonlinearOptimization::evaluateConstraintHessian(
  const ConstraintFunctions &constraints,
  EigenSupport::ConstRefVecXd x,
  EigenSupport::ConstRefVecXd multipliers)
{
  validateX(constraints, x);
  validateMultipliers(constraints, multipliers);
  EigenSupport::SpMatD hess;
  constraints.hessianAlloc(hess);
  if (hess.valuePtr()) {
    memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());
  }
  constraints.hessianInPlace(x, multipliers, hess);
  return hess;
}
