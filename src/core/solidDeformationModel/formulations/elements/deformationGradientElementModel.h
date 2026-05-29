#pragma once

#include "../../deformationModel.h"
#include "../../elasticModel3DDeformationGradient.h"
#include "../../plasticModel3DDeformationGradient.h"
#include "../basis/basis.h"
#include "../quadrature/quadrature.h"
#include "../kernels/deformationGradientKernel.h"
#include "deformationGradientElementModelCacheData.h"
#include "parameterizedMaterialBlock.h"

#include "EigenSupport.h"

namespace pgo
{
namespace SolidDeformationModel
{

class DeformationGradientElementModel : public DeformationModel
{
public:
  using M3xN = Eigen::Matrix<double, 3, Eigen::Dynamic>;
  using M9xNDOF = Eigen::Matrix<double, 9, Eigen::Dynamic>;

  // Block-based constructor (production path).
  DeformationGradientElementModel(int ele, const double *restPositions,
    const Basis &basis, const Quadrature &quadrature,
    const ElasticBlock &elasticBlock, const PlasticBlock &plasticBlock);

  // Compatibility constructor (test/legacy path).
  DeformationGradientElementModel(const double *restPositions,
    const Basis &basis, const Quadrature &quadrature,
    ElasticModel *elasticModel, PlasticModel *plasticModel);

  // DeformationModel overrides.
  std::unique_ptr<DeformationModelCacheData> allocateCacheData() const override;
  void prepareData(const double *x, DeformationModelCacheData *cacheData) const override;

  double computeEnergy(const DeformationModelCacheData *cacheData) const override;
  void compute_dE_dx(const DeformationModelCacheData *cacheData, double *grad) const override;
  void compute_d2E_dx2(const DeformationModelCacheData *cacheData, double *hess) const override;
  void compute_d2E_dxda(const DeformationModelCacheData *cacheData, double *hess) const override;
  void compute_d2E_dxdb(const DeformationModelCacheData *cacheData, double *hess) const override;

  void compute_dE_da(const DeformationModelCacheData *cacheData, double *grad) const override;
  void compute_d2E_da2(const DeformationModelCacheData *cacheData, double *hess) const override;
  void compute_dE_db(const DeformationModelCacheData *cacheData, double *grad) const override;
  void compute_d2E_db2(const DeformationModelCacheData *cacheData, double *hess) const override;
  void compute_d2E_dadb(const DeformationModelCacheData *cacheData, double *hess) const override;

  void enableSPD(int enable) override;
  int getNumVertices() const override { return numNodes_; }
  int getNumDOFs() const override { return localDofs_; }
  int getNumMaterialLocations() const override { return numQuadPts_; }

  LocalMaxStepResult computeLocalMaxStepSize(const double *x_local, const double *dx_local) const override;

  void computeF(const double *x, int materialLocationID, double F[9]) const;
  void computeFe(const DeformationModelCacheData *cacheData, int materialLocationID, double F[9]) const;
  void computeP(const DeformationModelCacheData *cacheData, int materialLocationID, double POut[9]) const;
  void computedPdF(const DeformationModelCacheData *cacheData, int materialLocationID, double dPdFOut[81]) const;
  void computedFdx(const DeformationModelCacheData *cacheData, int materialLocationID, double *dFdxOut) const;
  void computeForceFromP(const DeformationModelCacheData *cacheData, int materialLocationID,
    const double P[9], double f[/*localDofs_*/]) const;

  void vonMisesStress(const DeformationModelCacheData *cacheData,
    int &nPt, double *stresses) const override;
  void maxStrain(const DeformationModelCacheData *cacheData,
    int &nPt, double *stresses) const override;

  const DeformationGradientKernel &kernel() const { return kernel_; }

  static void computeSVD(const ES::M3d &Fe, ES::M3d &U, ES::M3d &V, ES::V3d &S);

private:
  int numNodes_ = 0;
  int numQuadPts_ = 0;
  int localDofs_ = 0;

  DeformationGradientKernel kernel_;
  const ElasticModel3DDeformationGradient *elasticModel_ = nullptr;
  const PlasticModel3DDeformationGradient *plasticModel_ = nullptr;

  int ele_ = -1;
  ElasticBlock elasticBlock_;
  PlasticBlock plasticBlock_;

  int numPlasticParams_ = 0;
  int numElasticParams_ = 0;

  ES::VXd compatZeroPlasticParams_;
  ES::VXd compatZeroElasticParams_;

  const double *elasticParamsPtr(const DeformationModelCacheData *cacheData) const;

  double compute_dV_dai(double weightDetJ, double ddetA_dai) const;
  double compute_d2V_daidaj(double weightDetJ, double d2detA_daidaj) const;
  void compute_dFe_dai(const ES::M3d &Fref, const ES::M3d &dAInvdai, ES::M3d &dFdai) const;
  void compute_d2Fe_dai_daj(const ES::M3d &Fref, const ES::M3d &dAInvdaidaj, ES::M3d &d2Fdaidaj) const;
  void compute_dP_dai(const ES::M9d &dPdF, const ES::M3d &dFdai, ES::M3d &dPdai) const;
  double compute_dpsi_dai(const ES::M3d &Fref, const ES::M3d &dAInv_dai, const ES::M3d &P) const;
  double compute_d2psi_dai_daj(const ES::M3d &Fref, const ES::M3d &dAInv_dai,
    const ES::M3d &dAInv_daj, const ES::M3d &d2AInv_dai_daj,
    const ES::M3d &P, const ES::M9d &dPdF) const;
  void compute_d2Fe_dx_dai(const ES::M3d &dAInvdai, const M9xNDOF &rest_dFdx, M9xNDOF &d2Fdudai) const;
  void computeCurrent_dFdx(const M9xNDOF &rest_dFdx, const ES::M3d &FpInv, M9xNDOF &dFdx) const;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
