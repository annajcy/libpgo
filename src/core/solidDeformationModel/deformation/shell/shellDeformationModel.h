#pragma once

#include "deformation/deformationModel.h"
#include "deformation/shell/shellElementMapping.h"
#include "material/elastic/elasticModel2DFundamentalForms.h"
#include "material/plastic/plasticModel2DFundamentalForms.h"
#include "shellDeformationModelCacheData.h"
#include "EigenSupport.h"

#include <memory>
#include <span>

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

  ShellDeformationModel(std::unique_ptr<ShellElementMapping> mapping,
    std::unique_ptr<ElasticModel2DFundamentalForms> elasticModel,
    std::unique_ptr<PlasticModel2DFundamentalForms> plasticModel);

  std::unique_ptr<DeformationModelCacheData> allocateCacheData() const override;
  bool isCacheDataCompatible(const DeformationModelCacheData &cacheData) const override;

  void prepareData(std::span<const double> x, std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    DeformationModelCacheData &cacheDataBase) const override;

  double computeEnergy(const DeformationModelCacheData &cacheDataBase) const override;

  void compute_dE_dx(const DeformationModelCacheData &cacheDataBase,
    EigenSupport::RefVecXd grad) const override;

  void compute_d2E_dx2(const DeformationModelCacheData &cacheDataBase,
    EigenSupport::RefMatXd hess) const override;

  void compute_d2E_dudp(const DeformationModelCacheData &cacheDataBase,
    EigenSupport::RefMatXd hess, int materialLocation = -1) const override;

  void compute_d2E_dude(const DeformationModelCacheData &cacheDataBase,
    EigenSupport::RefMatXd hess, int materialLocation = -1) const override;

  void compute_dE_dp(const DeformationModelCacheData &cacheDataBase,
    EigenSupport::RefVecXd grad, int materialLocation = -1) const override;

  void compute_d2E_dp2(const DeformationModelCacheData &cacheDataBase,
    EigenSupport::RefMatXd hess, int materialLocation = -1) const override;

  void compute_dE_de(const DeformationModelCacheData &cacheDataBase,
    EigenSupport::RefVecXd grad, int materialLocation = -1) const override;

  void compute_d2E_de2(const DeformationModelCacheData &cacheDataBase,
    EigenSupport::RefMatXd hess, int materialLocation = -1) const override;

  void compute_d2E_dpde(const DeformationModelCacheData &cacheDataBase,
    EigenSupport::RefMatXd hess, int materialLocation = -1) const override;

  int computeVonMisesStress(
    const DeformationModelCacheData &cacheData,
    std::span<double> stresses, int capacity) const override;

  void setProjectHessianPSD(bool enable) override;
  int getNumElasticParameters() const override { return numElasticParams_; }
  int getNumPlasticParameters() const override { return numPlasticParams_; }
  void defaultPlasticParams(std::span<double> params) const override;

  int getNumVertices() const override;
  int getNumDOFs() const override;

  LocalMaxStepResult computeLocalMaxStepSize(
    std::span<const double> x_local, std::span<const double> dx_local) const override;

private:
  std::unique_ptr<ShellElementMapping> elementMapping_;
  std::unique_ptr<ElasticModel2DFundamentalForms> elastic2D_;
  std::unique_ptr<PlasticModel2DFundamentalForms> plastic2D_;
  bool projectHessianPSD_ = false;

  int numPlasticParams_ = 0;
  int numElasticParams_ = 0;

  const CacheData &cacheData(const DeformationModelCacheData &cacheDataBase) const;
  CacheData &cacheData(DeformationModelCacheData &cacheDataBase) const;
  double computeEnergyWithParams(const CacheData &cacheData,
    std::span<const double> plasticParams, std::span<const double> elasticParams) const;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
