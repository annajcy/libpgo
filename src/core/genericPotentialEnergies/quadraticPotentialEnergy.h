/*
author: Bohan Wang
copyright to USC, MIT
*/

#pragma once

#include "energy/potentialEnergy.h"

#include <memory>
#include <optional>
#include <vector>

namespace pgo
{
namespace PredefinedPotentialEnergies
{
class QuadraticPotentialEnergyCache;

// 1/2 x^T A x + b^T x + c
//
// For least-squares forms (e.g. 1/2 ||Ax + b||^2), use the factory
// functions below — they precompute A^T A / A^T W A into this same
// internal representation.

class QuadraticPotentialEnergy : public NonlinearOptimization::PotentialEnergy
{
public:
  explicit QuadraticPotentialEnergy(EigenSupport::SpMatD A);
  QuadraticPotentialEnergy(EigenSupport::SpMatD A, EigenSupport::VXd b);

  void setDOFs(const std::vector<int> &dofs);

  // In-place updaters for time-integrator stage residuals (D2):
  // the stage energy ½xᵀAx + lᵀx is constructed once and updated each step
  // without rebuilding the owning EnergySet's Hessian template.
  //
  // setLinearTerm replaces lᵀx (the b_ term); cheap, called every step.
  // setAValues overwrites the quadratic values in place and REQUIRES the same
  // sparsity pattern as the A passed to the constructor (only the scalar values
  // change when the timestep changes); the Hessian topology stays fixed.
  void setLinearTerm(EigenSupport::VXd b);
  void setAValues(const EigenSupport::SpMatD &A);

  virtual double func(EigenSupport::ConstRefVecXd x) const override;
  virtual void gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const override;
  virtual void hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const override;
  virtual void hessianVector(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd vec, EigenSupport::RefVecXd hVec) const override;

  virtual void hessianAlloc(EigenSupport::SpMatD &hess) const override { hess = A_; }
  virtual void getDOFs(std::vector<int> &dofs) const override { dofs = allDOFs; }
  virtual int getNumDOFs() const override { return (int)allDOFs.size(); }

  void gradientComponent(EigenSupport::SpMatD *A, EigenSupport::VXd *b) const;

  virtual int isQuadratic() const override { return 1; }
  virtual int hasHessianVector() const override { return 1; }

protected:
  EigenSupport::SpMatD A_;
  std::optional<EigenSupport::VXd> b_;
  double c = 0.0;

  std::vector<int> allDOFs;
  std::shared_ptr<QuadraticPotentialEnergyCache> cache;

  friend std::shared_ptr<QuadraticPotentialEnergy>
    makeLeastSquaresEnergy(EigenSupport::SpMatD A, EigenSupport::VXd b);
  friend std::shared_ptr<QuadraticPotentialEnergy>
    makeLeastSquaresEnergy(EigenSupport::SpMatD A, EigenSupport::VXd b, const double *W);
};

// ── Least-squares factory functions ─────────────────────────────

// 1/2 ||A x||^2  =  1/2 x^T (A^T A) x
std::shared_ptr<QuadraticPotentialEnergy>
  makeLeastSquaresEnergy(EigenSupport::SpMatD A);

// 1/2 ||A x + b||^2  =  1/2 x^T (A^T A) x + b^T A x + 1/2 b^T b
std::shared_ptr<QuadraticPotentialEnergy>
  makeLeastSquaresEnergy(EigenSupport::SpMatD A, EigenSupport::VXd b);

// 1/2 (A x)^T W (A x)  =  1/2 x^T (A^T W A) x   (W = diag weights)
std::shared_ptr<QuadraticPotentialEnergy>
  makeLeastSquaresEnergy(EigenSupport::SpMatD A, const double *W);

// 1/2 (A x + b)^T W (A x + b)   (W = diag weights)
std::shared_ptr<QuadraticPotentialEnergy>
  makeLeastSquaresEnergy(EigenSupport::SpMatD A, EigenSupport::VXd b, const double *W);

}  // namespace PredefinedPotentialEnergies
}  // namespace pgo
