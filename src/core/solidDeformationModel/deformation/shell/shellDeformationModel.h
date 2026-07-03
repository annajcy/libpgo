#pragma once

#include "deformation/deformationModel.h"
#include "deformation/shell/shellElementMapping.h"
#include "material/elastic/elasticModel2DFundamentalForms.h"
#include "material/plastic/plasticModel2DFundamentalForms.h"
#include "shellDeformationModelCacheData.h"
#include "EigenSupport.h"

#include <memory>

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

// ShellDeformationModel — generic shell element facade.
// Works with any ShellElementMapping implementation.

class ShellDeformationModel : public DeformationModel
{
public:
  using CacheData = ShellDeformationModelCacheData;
  using DeformationModel::prepareData;

  ShellDeformationModel(std::unique_ptr<ShellElementMapping> mapping,
    std::unique_ptr<ElasticModel2DFundamentalForms> elasticModel,
    std::unique_ptr<PlasticModel2DFundamentalForms> plasticModel);

  std::unique_ptr<DeformationModelCacheData> allocateCacheData() const override;
  bool isCacheDataCompatible(const DeformationModelCacheData &cacheData) const override;

  void prepareData(const double *x, const double *elasticParams,
    const double *plasticParams,
    DeformationModelCacheData *cacheDataBase) const override;

  double computeEnergy(const DeformationModelCacheData *cacheDataBase) const override;

  void compute_dE_dx(const DeformationModelCacheData *cacheDataBase,
    double *grad) const override;

  void compute_d2E_dx2(const DeformationModelCacheData *cacheDataBase,
    double *hess) const override;

  void compute_d2E_dxda(const DeformationModelCacheData *cacheDataBase,
    double *hess) const override;

  void compute_d2E_dxdb(const DeformationModelCacheData *cacheDataBase,
    double *hess) const override;

  void compute_dE_da(const DeformationModelCacheData *cacheDataBase,
    double *grad) const override;

  void compute_d2E_da2(const DeformationModelCacheData *cacheDataBase,
    double *hess) const override;

  void compute_dE_db(const DeformationModelCacheData *cacheDataBase,
    double *grad) const override;

  void compute_d2E_db2(const DeformationModelCacheData *cacheDataBase,
    double *hess) const override;

  void compute_d2E_dadb(const DeformationModelCacheData *cacheDataBase,
    double *hess) const override;

  void vonMisesStress(const DeformationModelCacheData *cacheData,
    int &nPt, double *stresses) const override;

  void enableSPD(int enable) override;
  int getNumElasticParameters() const override { return numElasticParams_; }
  int getNumPlasticParameters() const override { return numPlasticParams_; }
  void defaultPlasticParams(double *params) const override;
  bool isPlasticIdentityTransform() const override;

  int getNumVertices() const override;
  int getNumDOFs() const override;

  LocalMaxStepResult computeLocalMaxStepSize(
    const double *x_local, const double *dx_local) const override;

private:
  std::unique_ptr<ShellElementMapping> elementMapping_;
  std::unique_ptr<ElasticModel2DFundamentalForms> elastic2D_;
  std::unique_ptr<PlasticModel2DFundamentalForms> plastic2D_;
  int enableSPD_ = 0;

  int numPlasticParams_ = 0;
  int numElasticParams_ = 0;

  const CacheData *cacheData(const DeformationModelCacheData *cacheDataBase) const;
  CacheData *cacheData(DeformationModelCacheData *cacheDataBase) const;
  double computeEnergyWithParams(const CacheData &cacheData,
    const double *plasticParams, const double *elasticParams) const;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
