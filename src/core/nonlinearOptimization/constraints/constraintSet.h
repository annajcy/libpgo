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
namespace NonlinearOptimization
{
class ConstraintSet : public ConstraintFunctions
{
public:
  struct Term
  {
    ConstraintFunctions_const_p functions;
  };

  ConstraintSet(int numDofs, std::vector<Term> terms);
  virtual ~ConstraintSet();

  int numTerms() const { return (int)terms_.size(); }
  const Term &term(int index) const { return terms_.at(index); }

  virtual void func(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd g) const override;
  virtual void jacobian(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &jac) const override;
  virtual void hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd lambda, EigenSupport::SpMatD &hess) const override;
  virtual void hessianVector(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd lambda, EigenSupport::ConstRefVecXd vec, EigenSupport::RefVecXd hessVec) const override;

  virtual bool isLinear() const override { return isLinear_; }
  virtual bool isQuadratic() const override { return false; }
  virtual bool hasHessianVector() const override { return hasHessianVectorRoutine_; }

protected:
  std::vector<Term> terms_;
  std::vector<EigenSupport::IDX> rowOffsets_;
  mutable std::vector<EigenSupport::SpMatD> childJacobianBuffers_;
  mutable std::vector<EigenSupport::SpMatD> childHessianBuffers_;
  std::vector<EigenSupport::SpMatI> jacobianMappings_;
  std::vector<EigenSupport::SpMatI> hessianMappings_;
  bool isLinear_ = true;
  bool hasHessianVectorRoutine_ = true;
};

typedef std::shared_ptr<ConstraintSet> ConstraintSet_p;
typedef std::shared_ptr<const ConstraintSet> ConstraintSet_const_p;
}  // namespace NonlinearOptimization
}  // namespace pgo
