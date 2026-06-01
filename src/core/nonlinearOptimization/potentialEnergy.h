/*
author: Bohan Wang
copyright to USC
*/

#pragma once

#include "EigenDef.h"
#include "solveDiagnostics.h"

#include <memory>

namespace pgo
{
namespace NonlinearOptimization
{

enum class EnergyStateKind
{
  Generic,
  Displacement,
};

class PotentialEnergy
{
public:
  PotentialEnergy() {}
  virtual ~PotentialEnergy() {}

  virtual double func(EigenSupport::ConstRefVecXd x) const = 0;
  virtual void gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const = 0;
  virtual void hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const = 0;
  virtual void hessianVector(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd vec, EigenSupport::RefVecXd hessVec) const;

  virtual double func_grad(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const { gradient(x, grad); return func(x); }
  virtual double func_grad_hessian(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad, EigenSupport::SpMatD &hess) const { gradient(x, grad), hessianInPlace(x, hess); return func(x); }
  virtual void gradient_hessian(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad, EigenSupport::SpMatD &hess) const { gradient(x, grad); hessian(x, hess); }

  virtual void hessian(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const;
  virtual void hessianAlloc(EigenSupport::SpMatD &hess) const = 0;

  virtual EnergyStateKind stateKind() const { return EnergyStateKind::Generic; }

  virtual void getDOFs(std::vector<int> &dofs) const = 0;
  virtual int getNumDOFs() const = 0;

  virtual int isQuadratic() const { return 0; }
  virtual int hasHessianVector() const { return 0; }
  virtual int hasHessian() const { return 1; }
  virtual int isHessianTopologyFixed() const { return 1; }

  // Largest feasible step along dx before a barrier (FEM element inversion, contact
  // CCD, ...) is violated. Energies without such a barrier keep the default.
  virtual MaxStepResult computeMaxStepLimit(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx) const { return MaxStepResult::unconstrained(); }
};

typedef std::shared_ptr<PotentialEnergy> PotentialEnergy_p;
typedef std::shared_ptr<const PotentialEnergy> PotentialEnergy_const_p;

inline void PotentialEnergy::hessianVector(EigenSupport::ConstRefVecXd, EigenSupport::ConstRefVecXd, EigenSupport::RefVecXd) const
{
}

inline void PotentialEnergy::hessian(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const
{
  hessianAlloc(hess);
  hessianInPlace(x, hess);
}

}  // namespace NonlinearOptimization
}  // namespace pgo
