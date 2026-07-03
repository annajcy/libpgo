/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include <limits>
#include <memory>

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

  virtual void vonMisesStress(const CacheData *, int &, double *) const {};
  virtual void maxStrain(const CacheData *, int &, double *) const {};

  virtual double computeEnergy(const CacheData *cacheData) const = 0;
  virtual void compute_dE_dx(const CacheData *cacheData, double *grad) const = 0;
  virtual void compute_d2E_dx2(const CacheData *cacheData, double *hess) const = 0;

  virtual void compute_d2E_dxda(const CacheData *cacheData, double *hess) const = 0;
  virtual void compute_d2E_dxdb(const CacheData *cacheData, double *hess) const = 0;

  virtual void compute_dE_da(const CacheData *cacheData, double *grad) const {}
  virtual void compute_d2E_da2(const CacheData *cacheData, double *hess) const {}
  virtual void compute_dE_db(const CacheData *cacheData, double *grad) const {}
  virtual void compute_d2E_db2(const CacheData *cacheData, double *hess) const {}
  virtual void compute_d2E_dadb(const CacheData *cacheData, double *hess) const {}

  virtual void enableSPD(int enable) { (void)enable; }

  // virtual void compute_d3E_dx3(const CacheData *cacheData, double *tensor) const = 0;
  // virtual void compute_d3E_dxdadx(const CacheData *cacheData, double *tensor) const = 0;
  // virtual void compute_d3E_dxdada(const CacheData *cacheData, double *tensor) const = 0;

  // inline static double d3E_dx3_ijk(const double *tensor, int i, int j, int k, int dim) { return tensor[k * dim * dim + j * dim + i]; }
  // inline static double &d3E_dx3_ijk(double *tensor, int i, int j, int k, int dim) { return tensor[k * dim * dim + j * dim + i]; }

  virtual int getNumElasticParameters() const = 0;
  virtual int getNumPlasticParameters() const = 0;
  virtual void defaultPlasticParams(double *params) const { (void)params; }
  virtual void setPlasticFiberAxes(const double *R) { (void)R; }
  virtual bool isPlasticIdentityTransform() const { return false; }

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
