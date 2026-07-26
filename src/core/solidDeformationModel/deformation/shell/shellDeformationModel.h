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

class ShellDeformationModel;
class ShellDeformationModelEvaluator;

// ShellDeformationModel — generic shell element facade.
// Works with any ShellElementMapping implementation.

class ShellDeformationModel : public DeformationModel
{
public:
  ShellDeformationModel(std::unique_ptr<ShellElementMapping> mapping,
    std::unique_ptr<ElasticModel2DFundamentalForms> elasticModel,
    std::unique_ptr<PlasticModel2DFundamentalForms> plasticModel,
    DeformationModelConstructionOptions options = {});

  std::unique_ptr<DeformationModelEvaluator> createEvaluator() const override;

private:
  friend class ShellDeformationModelEvaluator;

  void prepareData(std::span<const double> x, std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    ShellDeformationModelCacheData &cacheDataBase) const;

  double compute_E(const ShellDeformationModelCacheData &cacheDataBase) const;

  void compute_dE_dx(const ShellDeformationModelCacheData &cacheDataBase,
    EigenSupport::RefVecXd grad) const;

  void compute_d2E_dx2(const ShellDeformationModelCacheData &cacheDataBase,
    EigenSupport::RefMatXd hess) const;

  void compute_d2E_dudp(const ShellDeformationModelCacheData &cacheDataBase,
    EigenSupport::RefMatXd hess, int materialLocation = -1) const;

  void compute_d2E_dude(const ShellDeformationModelCacheData &cacheDataBase,
    EigenSupport::RefMatXd hess, int materialLocation = -1) const;

  void compute_dE_dp(const ShellDeformationModelCacheData &cacheDataBase,
    EigenSupport::RefVecXd grad, int materialLocation = -1) const;

  void compute_d2E_dp2(const ShellDeformationModelCacheData &cacheDataBase,
    EigenSupport::RefMatXd hess, int materialLocation = -1) const;

  void compute_dE_de(const ShellDeformationModelCacheData &cacheDataBase,
    EigenSupport::RefVecXd grad, int materialLocation = -1) const;

  void compute_d2E_de2(const ShellDeformationModelCacheData &cacheDataBase,
    EigenSupport::RefMatXd hess, int materialLocation = -1) const;

  void compute_d2E_dpde(const ShellDeformationModelCacheData &cacheDataBase,
    EigenSupport::RefMatXd hess, int materialLocation = -1) const;

  int computeVonMisesStress(
    const ShellDeformationModelCacheData &cacheData,
    std::span<double> stresses, int capacity) const;

public:
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

  const ShellDeformationModelCacheData &cacheData(const ShellDeformationModelCacheData &cacheDataBase) const;
  ShellDeformationModelCacheData &cacheData(ShellDeformationModelCacheData &cacheDataBase) const;
  double compute_E(const ShellDeformationModelCacheData &cacheData,
    std::span<const double> plasticParams, std::span<const double> elasticParams) const;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
