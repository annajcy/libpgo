/*
author: Bohan Wang
copyright to USC
*/

#pragma once

#include "constraintFunctions.h"

namespace pgo
{
namespace NonlinearOptimization
{
EigenSupport::VXd evaluateConstraintValues(
  const ConstraintFunctions &constraints,
  EigenSupport::ConstRefVecXd x);

EigenSupport::SpMatD evaluateConstraintJacobian(
  const ConstraintFunctions &constraints,
  EigenSupport::ConstRefVecXd x);

EigenSupport::SpMatD evaluateConstraintHessian(
  const ConstraintFunctions &constraints,
  EigenSupport::ConstRefVecXd x,
  EigenSupport::ConstRefVecXd multipliers);
}  // namespace NonlinearOptimization
}  // namespace pgo
