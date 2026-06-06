#pragma once

#include "../../deformation/deformationModel.h"
#include "../kernels/shellKernel.h"
#include "shellDeformationModelCacheData.h"
#include "EigenSupport.h"

#include <memory>

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

// ShellDeformationModel — generic shell element facade.
// Works with any ShellKernel implementation.

class ShellDeformationModel : public DeformationModel
{
public:
  using CacheData = ShellDeformationModelCacheData;

  ShellDeformationModel(int ele, std::unique_ptr<ShellKernel> kernel,
    std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel,
    const ParameterField *elasticParams, const ParameterField *plasticParams);

  std::unique_ptr<DeformationModelCacheData> allocateCacheData() const override;

  void prepareData(const double *x,
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

  void enableSPD(int enable) override;

  int getNumVertices() const override;
  int getNumDOFs() const override;

  LocalMaxStepResult computeLocalMaxStepSize(
    const double *x_local, const double *dx_local) const override;

private:
  std::unique_ptr<ShellKernel> kernel_;
  ElasticModel2DFundamentalForms *elasticModel_ = nullptr;
  PlasticModel2DFundamentalForms *plasticModel_ = nullptr;
  int enableSPD_ = 0;

  int ele_ = -1;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
