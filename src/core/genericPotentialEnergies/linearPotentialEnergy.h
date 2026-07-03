/*
author: Bohan Wang
copyright to USC, MIT
*/

#pragma once

#include "energy/potentialEnergy.h"

#include <numeric>
#include <vector>

namespace pgo
{
namespace PredefinedPotentialEnergies
{
class LinearPotentialEnergy : public NonlinearOptimization::PotentialEnergy
{
public:
  // b^T x
  explicit LinearPotentialEnergy(EigenSupport::VXd b);

  void setDOFs(const std::vector<int> &dofs);

  virtual double func(EigenSupport::ConstRefVecXd x) const override { return x.dot(b_); }
  virtual void gradient(EigenSupport::ConstRefVecXd, EigenSupport::RefVecXd grad) const override { grad = b_; }
  virtual void hessianInPlace(EigenSupport::ConstRefVecXd, EigenSupport::SpMatD &) const override {}

  virtual void hessianAlloc(EigenSupport::SpMatD &hess) const override { hess.resize(allDOFs.size(), allDOFs.size()); }
  virtual void getDOFs(std::vector<int> &dofs) const override { dofs = allDOFs; }
  virtual int getNumDOFs() const override { return (int)allDOFs.size(); }

  virtual int isQuadratic() const override { return 0; }
  virtual int hasHessianVector() const override { return 0; }


protected:
  std::vector<int> allDOFs;
  EigenSupport::VXd b_;
};
}  // namespace NonlinearOptimization
}  // namespace pgo