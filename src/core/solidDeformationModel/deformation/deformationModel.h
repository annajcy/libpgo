/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include <limits>
#include <memory>
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
  void prepareData(const double *x, CacheData *cacheData) const
  {
    prepareData(x, nullptr, nullptr, cacheData);
  }
  virtual void prepareData(const double *x, const double *elasticParams,
    const double *plasticParams, CacheData *cacheData) const = 0;

  // Optional diagnostics return the number of samples written to output.
  // Implementations must return a value in [1, capacity].
  virtual int computeVonMisesStress(
    const CacheData *, double *, int) const
  {
    throw UnsupportedDeformationDiagnosticError(
      "Von Mises stress is not implemented by this deformation model.");
  }
  virtual int computeMaxStrain(
    const CacheData *, double *, int) const
  {
    throw UnsupportedDeformationDiagnosticError(
      "Maximum strain is not implemented by this deformation model.");
  }

  virtual double computeEnergy(const CacheData *cacheData) const = 0;
  virtual void compute_dE_dx(const CacheData *cacheData, double *grad) const = 0;
  virtual void compute_d2E_dx2(const CacheData *cacheData, double *hess) const = 0;

  // Parameter-derivative notation: p = plastic DOFs, e = elastic DOFs.
  // The local position coordinate has dx/du = I, so mixed x-parameter and
  // displacement-parameter derivatives are identical.
  virtual void compute_d2E_dudp(
    const CacheData *cacheData, double *hess,
    int materialLocation = -1) const = 0;
  virtual void compute_d2E_dude(
    const CacheData *cacheData, double *hess,
    int materialLocation = -1) const = 0;

  virtual void compute_dE_dp(
    const CacheData *, double *, int = -1) const {}
  virtual void compute_d2E_dp2(
    const CacheData *, double *, int = -1) const {}
  virtual void compute_dE_de(
    const CacheData *, double *, int = -1) const {}
  virtual void compute_d2E_de2(
    const CacheData *, double *, int = -1) const {}
  virtual void compute_d2E_dpde(
    const CacheData *, double *, int = -1) const {}

  virtual void enableSPD(int enable) { (void)enable; }

  // virtual void compute_d3E_dx3(const CacheData *cacheData, double *tensor) const = 0;
  // virtual void compute_d3E_dxdadx(const CacheData *cacheData, double *tensor) const = 0;
  // virtual void compute_d3E_dxdada(const CacheData *cacheData, double *tensor) const = 0;

  // inline static double d3E_dx3_ijk(const double *tensor, int i, int j, int k, int dim) { return tensor[k * dim * dim + j * dim + i]; }
  // inline static double &d3E_dx3_ijk(double *tensor, int i, int j, int k, int dim) { return tensor[k * dim * dim + j * dim + i]; }

  virtual int getNumElasticParameters() const = 0;
  virtual int getNumPlasticParameters() const = 0;
  virtual void defaultPlasticParams(double *params) const { (void)params; }
  virtual int getNumVertices() const = 0;
  virtual int getNumDOFs() const = 0;
  virtual LocalMaxStepResult computeLocalMaxStepSize(const double *x_local, const double *dx_local) const
  {
    (void)x_local;
    (void)dx_local;
    return LocalMaxStepResult{};
  }

  // advanced routines
  virtual int getNumMaterialLocations() const { return 1; }
  // virtual void computeF(const double *x, int materialLocationIDs, double F[9]) const {}
  // virtual void computeP(const CacheData *cacheDataBase, int materialLocationIDs, double P[9]) const {}
  // virtual void computedPdF(const CacheData *cacheDataBase, int materialLocationIDs, double dPdF[81]) const {}
  // virtual void computedFdx(const CacheData *cacheDataBase, int materialLocationIDs, double *dFdx) const {}

protected:
  int numMaterialLocations = 1;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
