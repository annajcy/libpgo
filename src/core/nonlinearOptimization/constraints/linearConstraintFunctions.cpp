/*
author: Bohan Wang
copyright to USC
*/

#include "linearConstraintFunctions.h"
#include "EigenSupport.h"

#include <stdexcept>
#include <utility>

using namespace pgo;
using namespace pgo::NonlinearOptimization;

LinearConstraintFunctions::LinearConstraintFunctions(ES::SpMatD C, ES::VXd offset):
  ConstraintFunctions(static_cast<int>(C.cols())), jacConst_(std::move(C)), offset_(std::move(offset))
{
  if (offset_.size() != jacConst_.rows()) {
    throw std::invalid_argument("LinearConstraintFunctions offset size must match C rows");
  }
  jacConst_.makeCompressed();
  lambdahZero_.resize(jacConst_.cols(), jacConst_.cols());
}

void LinearConstraintFunctions::setOffset(ES::VXd offset)
{
  if (offset.size() != jacConst_.rows()) {
    throw std::invalid_argument("LinearConstraintFunctions offset size must match C rows");
  }
  offset_ = std::move(offset);
}

void LinearConstraintFunctions::func(ES::ConstRefVecXd x, ES::RefVecXd g) const
{
  EigenSupport::mv(jacConst_, x, g);
  g.noalias() += offset_;
}

void LinearConstraintFunctions::jacobian(ES::ConstRefVecXd, ES::SpMatD &jac) const
{
  memcpy(jac.valuePtr(), jacConst_.valuePtr(), jacConst_.nonZeros() * sizeof(double));
}

void LinearConstraintFunctions::hessianInPlace(ES::ConstRefVecXd, ES::ConstRefVecXd, ES::SpMatD &hess) const
{
  if (hess.valuePtr())
    memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());
}

void LinearConstraintFunctions::hessianVector(ES::ConstRefVecXd, ES::ConstRefVecXd, ES::ConstRefVecXd, ES::RefVecXd hessVec) const
{
  hessVec.setZero();
}
