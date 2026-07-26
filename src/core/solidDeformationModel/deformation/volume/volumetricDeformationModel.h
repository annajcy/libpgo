#pragma once

#include "deformation/deformationModel.h"
#include "formulations/shapeFunction/shapeFunction.h"
#include "deformation/volume/volumetricElementMapping.h"
#include "formulations/quadrature/quadrature.h"
#include "material/elastic/elasticModel3DDeformationGradient.h"
#include "material/plastic/plasticModel3DDeformationGradient.h"
#include "volumetricDeformationModelCacheData.h"

#include "EigenSupport.h"

#include <memory>
#include <span>

namespace pgo
{
namespace SolidDeformationModel
{

class VolumetricDeformationModel;
class VolumetricDeformationModelEvaluator;

class VolumetricDeformationModel : public DeformationModel
{
public:
  using M3xN = Eigen::Matrix<double, 3, Eigen::Dynamic>;
  using M9xNDOF = Eigen::Matrix<double, 9, Eigen::Dynamic>;

  VolumetricDeformationModel(VolumetricElementMapping &&mapping,
    std::unique_ptr<ElasticModel3DDeformationGradient> elasticModel,
    std::unique_ptr<PlasticModel3DDeformationGradient> plasticModel,
    DeformationModelConstructionOptions options = {});

  std::unique_ptr<DeformationModelEvaluator> createEvaluator() const override;

private:
  friend class VolumetricDeformationModelEvaluator;

  void prepareData(std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    VolumetricDeformationModelCacheData &cacheDataBase) const;

  // DeformationModels.

  double compute_E(const VolumetricDeformationModelCacheData &cacheData) const;
  void compute_dE_dx(const VolumetricDeformationModelCacheData &cacheData, EigenSupport::RefVecXd grad) const;
  void compute_d2E_dx2(const VolumetricDeformationModelCacheData &cacheData, EigenSupport::RefMatXd hess) const;
  void compute_d2E_dudp(
    const VolumetricDeformationModelCacheData &cacheData, EigenSupport::RefMatXd hess,
    int materialLocation = -1) const;
  void compute_d2E_dude(
    const VolumetricDeformationModelCacheData &cacheData, EigenSupport::RefMatXd hess,
    int materialLocation = -1) const;

  void compute_dE_dp(
    const VolumetricDeformationModelCacheData &cacheData, EigenSupport::RefVecXd grad,
    int materialLocation = -1) const;
  void compute_d2E_dp2(
    const VolumetricDeformationModelCacheData &cacheData, EigenSupport::RefMatXd hess,
    int materialLocation = -1) const;
  void compute_dE_de(
    const VolumetricDeformationModelCacheData &cacheData, EigenSupport::RefVecXd grad,
    int materialLocation = -1) const;
  void compute_d2E_de2(
    const VolumetricDeformationModelCacheData &cacheData, EigenSupport::RefMatXd hess,
    int materialLocation = -1) const;
  void compute_d2E_dpde(
    const VolumetricDeformationModelCacheData &cacheData, EigenSupport::RefMatXd hess,
    int materialLocation = -1) const;

public:
  int getNumElasticParameters() const override { return numElasticParams_; }
  int getNumPlasticParameters() const override { return numPlasticParams_; }
  void defaultPlasticParams(std::span<double> params) const override;
  int getNumVertices() const override { return numNodes_; }
  int getNumDOFs() const override { return localDofs_; }
  int getNumMaterialLocations() const override { return numQuadPts_; }

  LocalMaxStepResult computeLocalMaxStepSize(std::span<const double> x_local,
    std::span<const double> dx_local) const override;

  ES::M3d compute_F(std::span<const double> x, int materialLocationID) const;

private:
  ES::M3d compute_Fe(const VolumetricDeformationModelCacheData &cacheData, int materialLocationID) const;
  ES::M3d compute_P(const VolumetricDeformationModelCacheData &cacheData, int materialLocationID) const;
  ES::M9d compute_dP_dF(const VolumetricDeformationModelCacheData &cacheData, int materialLocationID) const;
  void compute_dF_dx(const VolumetricDeformationModelCacheData &cacheData, int materialLocationID,
    EigenSupport::RefMatXd dFdxOut) const;
  void computeForceFromP(const VolumetricDeformationModelCacheData &cacheData, int materialLocationID,
    const ES::M3d &P, EigenSupport::RefVecXd f) const;

  int computeVonMisesStress(
    const VolumetricDeformationModelCacheData &cacheData,
    std::span<double> stresses, int capacity) const;
  int computeMaxStrain(
    const VolumetricDeformationModelCacheData &cacheData,
    std::span<double> strains, int capacity) const;

public:
  const VolumetricElementMapping &mapping() const { return elementMapping_; }

  static SpectralState computeSpectralState(const ES::M3d &Fe);

private:
  int numNodes_ = 0;
  int numQuadPts_ = 0;
  int localDofs_ = 0;

  VolumetricElementMapping elementMapping_;
  std::unique_ptr<ElasticModel3DDeformationGradient> elasticModel_;
  std::unique_ptr<PlasticModel3DDeformationGradient> plasticModel_;

  int numPlasticParams_ = 0;
  int numElasticParams_ = 0;
  bool projectHessianPSD_ = false;

  std::span<const double> elasticParams(const VolumetricDeformationModelCacheData &cacheData, int q) const;

  double compute_dV_dai(double weightDetJ, double ddetA_dai) const;
  double compute_d2V_daidaj(double weightDetJ, double d2detA_daidaj) const;
  ES::M3d compute_dFe_dai(
    const ES::M3d &Fref, const ES::M3d &dAInvdai) const;
  ES::M3d compute_d2Fe_dai_daj(
    const ES::M3d &Fref, const ES::M3d &dAInvdaidaj) const;
  ES::M3d compute_dP_dai(
    const ES::M9d &dPdF, const ES::M3d &dFdai) const;
  double compute_dpsi_dai(const ES::M3d &Fref, const ES::M3d &dAInv_dai, const ES::M3d &P) const;
  double compute_d2psi_dai_daj(const ES::M3d &Fref, const ES::M3d &dAInv_dai,
    const ES::M3d &dAInv_daj, const ES::M3d &d2AInv_dai_daj,
    const ES::M3d &P, const ES::M9d &dPdF) const;
  void compute_d2Fe_dx_dai(const ES::M3d &dAInvdai, const M9xNDOF &rest_dF_dx, M9xNDOF &d2Fdudai) const;
  void computeCurrent_dF_dx(const M9xNDOF &rest_dF_dx, const ES::M3d &FpInv, M9xNDOF &dFdx) const;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
