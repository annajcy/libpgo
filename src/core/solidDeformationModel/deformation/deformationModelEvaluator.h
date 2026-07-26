/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "EigenSupport.h"

#include <span>
#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{

class UnsupportedDeformationDiagnosticError : public std::logic_error
{
public:
  using std::logic_error::logic_error;
};

class DeformationModelEvaluator
{
public:
  virtual ~DeformationModelEvaluator() = default;

  virtual void prepare(
    std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams) = 0;

  virtual double compute_E() const = 0;
  virtual void compute_dE_dx(EigenSupport::RefVecXd grad) const = 0;
  virtual void compute_d2E_dx2(EigenSupport::RefMatXd hess) const = 0;

  virtual void compute_d2E_dudp(
    EigenSupport::RefMatXd hess, int materialLocation = -1) const = 0;
  virtual void compute_d2E_dude(
    EigenSupport::RefMatXd hess, int materialLocation = -1) const = 0;

  virtual void compute_dE_dp(
    EigenSupport::RefVecXd grad, int materialLocation = -1) const = 0;
  virtual void compute_d2E_dp2(
    EigenSupport::RefMatXd hess, int materialLocation = -1) const = 0;
  virtual void compute_dE_de(
    EigenSupport::RefVecXd grad, int materialLocation = -1) const = 0;
  virtual void compute_d2E_de2(
    EigenSupport::RefMatXd hess, int materialLocation = -1) const = 0;
  virtual void compute_d2E_dpde(
    EigenSupport::RefMatXd hess, int materialLocation = -1) const = 0;

  virtual int computeVonMisesStress(
    std::span<double>, int) const
  {
    throw UnsupportedDeformationDiagnosticError(
      "Von Mises stress is not implemented by this deformation model.");
  }
  virtual int computeMaxStrain(
    std::span<double>, int) const
  {
    throw UnsupportedDeformationDiagnosticError(
      "Maximum strain is not implemented by this deformation model.");
  }

protected:
  void markPrepared() const { prepared_ = true; }
  void markUnprepared() const { prepared_ = false; }
  void ensurePrepared() const
  {
    if (!prepared_)
      throw std::logic_error("Deformation model evaluator has not been prepared.");
  }

private:
  mutable bool prepared_ = false;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
