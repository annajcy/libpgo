/*
author: Bohan Wang
copyright to USC
*/

#pragma once

#include "constraintFunctions.h"

#include "EigenSupport.h"

#include <memory>
#include <vector>

namespace pgo
{
namespace ES = EigenSupport;

namespace NonlinearOptimization
{
class LinearConstraintFunctions : public ConstraintFunctions
{
public:
  LinearConstraintFunctions(EigenSupport::SpMatD C, EigenSupport::VXd offset);

  void setOffset(EigenSupport::VXd offset);
  void setd(EigenSupport::ConstRefVecXd newd) { setOffset(EigenSupport::VXd(newd)); }

  virtual void func(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd g) const override;
  virtual void jacobian(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &jac) const override;
  virtual void hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd lambda, EigenSupport::SpMatD &hess) const override;
  virtual void hessianVector(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd lambda, EigenSupport::ConstRefVecXd vec, EigenSupport::RefVecXd hessVec) const override;

  virtual void createJacobian(EigenSupport::SpMatD &jac) const override { jac = jacConst_; }
  virtual void hessianAlloc(EigenSupport::SpMatD &hess) const override { hess = lambdahZero_; }
  virtual int getNumConstraints() const override { return (int)jacConst_.rows(); }

  virtual const EigenSupport::SpMatD &getlambdaHessianTemplate() const override { return lambdahZero_; }
  virtual const EigenSupport::SpMatD &getJacobianTemplate() const override { return jacConst_; }
  virtual int getNNZJacobian() const override { return (int)jacConst_.nonZeros(); }

  virtual bool isQuadratic() const override { return false; }
  virtual bool isLinear() const override { return true; }
  virtual bool hasHessianVector() const override { return true; }

protected:
  EigenSupport::SpMatD jacConst_;
  EigenSupport::SpMatD lambdahZero_;
  EigenSupport::VXd offset_;
};

typedef std::shared_ptr<LinearConstraintFunctions> LinearConstraintFunctions_p;
typedef std::shared_ptr<const LinearConstraintFunctions> LinearConstraintFunctions_const_p;

}  // namespace NonlinearOptimization
}  // namespace pgo
