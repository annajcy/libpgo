/*
author: Bohan Wang
copyright to USC
*/

#pragma once

#include "energy/potentialEnergy.h"
#include "constraintFunctions.h"

#include <memory>

namespace pgo
{
namespace ES = EigenSupport;

namespace NonlinearOptimization
{
class PotentialEnergyConstraintFunctions : public PotentialEnergy
{
public:
  PotentialEnergyConstraintFunctions(int nAll, std::shared_ptr<const ConstraintFunctions> cnstt);
  virtual ~PotentialEnergyConstraintFunctions();

  virtual double func(EigenSupport::ConstRefVecXd x) const override;
  virtual void gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const override;
  virtual void hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const override;

  virtual void hessianAlloc(EigenSupport::SpMatD &hess) const override { hess = hessAll; }
  virtual void getDOFs(std::vector<int> &dofs) const override { dofs = allDOFs; }
  virtual int getNumDOFs() const override { return (int)allDOFs.size(); }


protected:
  std::shared_ptr<const ConstraintFunctions> cnstt;
  std::vector<int> allDOFs;

  mutable EigenSupport::VXd g;
  mutable EigenSupport::SpMatD jac, lambdaHessian, JTJ;
  EigenSupport::SpMatD hessAll;
  EigenSupport::SymbolicMmData *mmData;
  EigenSupport::SpMatI JTJMapping, lambdaHessMapping;
};

class PotentialEnergyBoundedConstraintFunctions : public PotentialEnergy
{
public:
  PotentialEnergyBoundedConstraintFunctions(int nAll, std::shared_ptr<const ConstraintFunctions> cnstt,
    EigenSupport::VXd lower, EigenSupport::VXd upper);
  virtual ~PotentialEnergyBoundedConstraintFunctions();

  virtual double func(EigenSupport::ConstRefVecXd x) const override;
  virtual void gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const override;
  virtual void hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const override;

  virtual void hessianAlloc(EigenSupport::SpMatD &hess) const override { hess = hessAll; }
  virtual void getDOFs(std::vector<int> &dofs) const override { dofs = allDOFs; }
  virtual int getNumDOFs() const override { return (int)allDOFs.size(); }

protected:
  void computeViolation(EigenSupport::ConstRefVecXd x) const;

  std::shared_ptr<const ConstraintFunctions> cnstt;
  EigenSupport::VXd lower, upper;
  std::vector<int> allDOFs;

  mutable EigenSupport::VXd values, residual, activeMask;
  mutable EigenSupport::SpMatD jac, lambdaHessian, weightedJTJ;
  EigenSupport::SpMatD hessAll;
  EigenSupport::SymbolicMmData *mmData = nullptr;
  EigenSupport::SpMatI weightedJTJMapping, lambdaHessMapping;
};

}  // namespace NonlinearOptimization
}  // namespace pgo
