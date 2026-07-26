/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "EigenSupport.h"

#include <limits>
#include <memory>
#include <span>
#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{

class DeformationModelCacheData
{
public:
  DeformationModelCacheData() {}
  virtual ~DeformationModelCacheData() {}

  bool isPrepared() const { return prepared_; }
  void markPrepared() { prepared_ = true; }
  void markUnprepared() { prepared_ = false; }

private:
  bool prepared_ = false;
};

class UnsupportedDeformationDiagnosticError : public std::logic_error
{
public:
  using std::logic_error::logic_error;
};

class DeformationModel
{
public:
  struct LocalMaxStepResult
  {
    double alpha = 1.0;
    bool illegalInitialState = false;
    double phi0 = std::numeric_limits<double>::quiet_NaN();
    double eps = 0.0;
    int locationId = -1;
  };

  DeformationModel() = default;
  virtual ~DeformationModel() {}

  typedef DeformationModelCacheData CacheData;

  virtual std::unique_ptr<CacheData> allocateCacheData() const = 0;
  virtual bool isCacheDataCompatible(const CacheData &cacheData) const = 0;
  virtual void prepareData(std::span<const double> x,
    std::span<const double> elasticParams, std::span<const double> plasticParams,
    CacheData &cacheData) const = 0;

  // Optional diagnostics return the number of samples written to output.
  // Implementations must return a value in [1, capacity].
  virtual int computeVonMisesStress(
    const CacheData &, std::span<double>, int) const
  {
    throw UnsupportedDeformationDiagnosticError(
      "Von Mises stress is not implemented by this deformation model.");
  }
  virtual int computeMaxStrain(
    const CacheData &, std::span<double>, int) const
  {
    throw UnsupportedDeformationDiagnosticError(
      "Maximum strain is not implemented by this deformation model.");
  }

  virtual double computeEnergy(const CacheData &cacheData) const = 0;
  virtual void compute_dE_dx(const CacheData &cacheData,
    EigenSupport::RefVecXd grad) const = 0;
  virtual void compute_d2E_dx2(const CacheData &cacheData,
    EigenSupport::RefMatXd hess) const = 0;

  // Parameter-derivative notation: p = plastic DOFs, e = elastic DOFs.
  // The local position coordinate has dx/du = I, so mixed x-parameter and
  // displacement-parameter derivatives are identical.
  virtual void compute_d2E_dudp(
    const CacheData &cacheData, EigenSupport::RefMatXd hess,
    int materialLocation = -1) const = 0;
  virtual void compute_d2E_dude(
    const CacheData &cacheData, EigenSupport::RefMatXd hess,
    int materialLocation = -1) const = 0;

  virtual void compute_dE_dp(
    const CacheData &, EigenSupport::RefVecXd, int = -1) const
  {
    throw std::logic_error(
      "DeformationModel::compute_dE_dp is not implemented by this model.");
  }
  virtual void compute_d2E_dp2(
    const CacheData &, EigenSupport::RefMatXd, int = -1) const
  {
    throw std::logic_error(
      "DeformationModel::compute_d2E_dp2 is not implemented by this model.");
  }
  virtual void compute_dE_de(
    const CacheData &, EigenSupport::RefVecXd, int = -1) const
  {
    throw std::logic_error(
      "DeformationModel::compute_dE_de is not implemented by this model.");
  }
  virtual void compute_d2E_de2(
    const CacheData &, EigenSupport::RefMatXd, int = -1) const
  {
    throw std::logic_error(
      "DeformationModel::compute_d2E_de2 is not implemented by this model.");
  }
  virtual void compute_d2E_dpde(
    const CacheData &, EigenSupport::RefMatXd, int = -1) const
  {
    throw std::logic_error(
      "DeformationModel::compute_d2E_dpde is not implemented by this model.");
  }

  virtual void setProjectHessianPSD(bool enable) = 0;

  virtual int getNumElasticParameters() const = 0;
  virtual int getNumPlasticParameters() const = 0;
  virtual void defaultPlasticParams(std::span<double> params) const { (void)params; }
  virtual int getNumVertices() const = 0;
  virtual int getNumDOFs() const = 0;
  virtual LocalMaxStepResult computeLocalMaxStepSize(std::span<const double> x_local,
    std::span<const double> dx_local) const
  {
    (void)x_local;
    (void)dx_local;
    return LocalMaxStepResult{};
  }

  // advanced routines
  virtual int getNumMaterialLocations() const { return 1; }

protected:
  int numMaterialLocations = 1;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
