#pragma once

#include "../../deformationModel.h"
#include "../../elasticModel3DDeformationGradient.h"
#include "../../plasticModel3DDeformationGradient.h"
#include "../../materialMaxStepPolynomialUtils.h"
#include "../kernels/deformationGradientKernel.h"

#include "EigenSupport.h"

#include <array>
#include <memory>
#include <stdexcept>
#include <vector>

namespace ES = pgo::EigenSupport;

namespace pgo
{
namespace SolidDeformationModel
{

// DeformationGradientElementModel<Kernel>
//
// Complete per-element deformation energy model that combines:
//   - formulation kinematics from Kernel
//   - ElasticModel3DDeformationGradient for material response
//   - PlasticModel3DDeformationGradient for plastic deformation
//
// Implements the DeformationModel virtual interface.
// Supports 1 (tet) or 8 (hex) quadrature/material locations.

template<class Kernel>
class DeformationGradientElementModel : public DeformationModel
{
public:
  static constexpr int numNodes = Kernel::numNodes;
  static constexpr int localDofs = Kernel::localDofs;
  static constexpr int numQuadPts = Kernel::numQuadPts;

  using M3xN = typename Kernel::M3xN;
  using M9xNDOF = typename Kernel::M9xNDOF;

  DeformationGradientElementModel(const double restPositions[localDofs],
    ElasticModel *elasticModel, PlasticModel *plasticModel);

  // DeformationModel overrides (virtual methods).
  DeformationModelCacheData *allocateCacheData() const override;
  void freeCacheData(DeformationModelCacheData *data) const override;
  void prepareData(const double *x, const double *param, const double *materialParam,
    DeformationModelCacheData *cacheData) const override;

  double computeEnergy(const DeformationModelCacheData *cacheData) const override;
  void compute_dE_dx(const DeformationModelCacheData *cacheData, double *grad) const override;
  void compute_d2E_dx2(const DeformationModelCacheData *cacheData, double *hess) const override;
  void compute_d2E_dxda(const DeformationModelCacheData *cacheData, double *hess) const override;
  void compute_d2E_dxdb(const DeformationModelCacheData *cacheData, double *hess) const override;

  // Non-virtual methods matching legacy element-model API.
  void compute_dE_da(const DeformationModelCacheData *cacheData, double *grad) const;
  void compute_d2E_da2(const DeformationModelCacheData *cacheData, double *hess) const;
  void compute_dE_db(const DeformationModelCacheData *cacheData, double *grad) const;
  void compute_d2E_db2(const DeformationModelCacheData *cacheData, double *hess) const;
  void compute_d2E_dadb(const DeformationModelCacheData *cacheData, double *hess) const;

  void enableSPD(int enable) override;
  int getNumVertices() const override { return numNodes; }
  int getNumDOFs() const override { return localDofs; }
  int getNumMaterialLocations() const override { return numQuadPts; }

  LocalMaxStepResult computeLocalMaxStepSize(const double *x_local, const double *dx_local) const override;

  // Legacy element-model query methods (non-virtual in base).
  void computeF(const double *x, int materialLocationID, double F[9]) const;
  void computeFe(const DeformationModelCacheData *cacheData, int materialLocationID, double F[9]) const;
  void computeP(const DeformationModelCacheData *cacheData, int materialLocationID, double POut[9]) const;
  void computedPdF(const DeformationModelCacheData *cacheData, int materialLocationID, double dPdFOut[81]) const;
  void computedFdx(const DeformationModelCacheData *cacheData, int materialLocationID, double *dFdxOut) const;
  void computeForceFromP(const DeformationModelCacheData *cacheData, int materialLocationID, const double P[9], double f[localDofs]) const;

  void vonMisesStress(const DeformationModelCacheData *cacheData, int &nPt, double *stresses) const override;
  void maxStrain(const DeformationModelCacheData *cacheData, int &nPt, double *stresses) const override;

  // Access the kernel (for tests and construction-time introspection).
  const Kernel &kernel() const { return kernel_; }

  // Static helper for SVD with sign correction (matching legacy convention).
  static void computeSVD(const ES::M3d &Fe, ES::M3d &U, ES::M3d &V, ES::V3d &S);

private:
  Kernel kernel_;
  const ElasticModel3DDeformationGradient *elasticModel_ = nullptr;
  const PlasticModel3DDeformationGradient *plasticModel_ = nullptr;

  int numPlasticParams_ = 0;
  int numElasticParams_ = 0;

  // Plastic-model helper methods (shared with legacy code).
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

// ============================================================
// Cache data, per-element
// ============================================================

template<class Kernel>
struct DeformationGradientElementModelCacheData : public DeformationModelCacheData
{
  static constexpr int numQuadPts = Kernel::numQuadPts;
  static constexpr int localDofs = Kernel::localDofs;

  using M3xN = typename Kernel::M3xN;
  using M9xNDOF = typename Kernel::M9xNDOF;

  int numPlasticParams = 0;
  int numElasticParams = 0;

  // Current positions.
  M3xN x;

  // Plastic state.
  ES::M3d Fp = ES::M3d::Identity();
  ES::M3d FpInv = ES::M3d::Identity();
  double detFp = 1.0;

  ES::VXd plasticParam;
  ES::VXd ddetA_da;
  ES::MXd d2detA_da2;
  std::vector<ES::M3d> dAInv_dai;
  std::vector<ES::M3d> d2AInv_dai_daj;

  // Per-quadrature-point data.
  std::array<ES::M3d, numQuadPts> Fref;
  std::array<ES::M3d, numQuadPts> Fe;
  std::array<ES::M3d, numQuadPts> U, V;
  std::array<ES::V3d, numQuadPts> S;
  std::array<M9xNDOF, numQuadPts> dFdx;
  std::array<M3xN, numQuadPts> Bm;

  ES::VXd materialParam;

  DeformationGradientElementModelCacheData(int np, int ne):
    numPlasticParams(np),
    numElasticParams(ne),
    plasticParam(ES::VXd::Zero(np)),
    ddetA_da(ES::VXd::Zero(np)),
    d2detA_da2(ES::MXd::Zero(np, np)),
    dAInv_dai(np, ES::M3d::Zero()),
    d2AInv_dai_daj(np * np, ES::M3d::Zero()),
    materialParam(ES::VXd::Zero(ne))
  {}

  ES::M3d &d2AInv(int i, int j) { return d2AInv_dai_daj[i * numPlasticParams + j]; }
  const ES::M3d &d2AInv(int i, int j) const { return d2AInv_dai_daj[i * numPlasticParams + j]; }
};

// ============================================================
// DeformationGradientElementModel implementation
// ============================================================

template<class Kernel>
DeformationGradientElementModel<Kernel>::DeformationGradientElementModel(
  const double restPositions[localDofs],
  ElasticModel *elasticModel, PlasticModel *plasticModel):
  DeformationModel(elasticModel, plasticModel),
  kernel_(restPositions)
{
  elasticModel_ = dynamic_cast<const ElasticModel3DDeformationGradient *>(elasticModel);
  plasticModel_ = dynamic_cast<const PlasticModel3DDeformationGradient *>(plasticModel);

  if (elasticModel_ == nullptr) {
    throw std::invalid_argument(
      "DeformationGradientElementModel requires ElasticModel3DDeformationGradient.");
  }
  if (plasticModel_ == nullptr) {
    throw std::invalid_argument(
      "DeformationGradientElementModel requires PlasticModel3DDeformationGradient.");
  }

  numPlasticParams_ = plasticModel_->getNumParameters();
  numElasticParams_ = elasticModel_->getNumParameters();
}

template<class Kernel>
DeformationModelCacheData *DeformationGradientElementModel<Kernel>::allocateCacheData() const
{
  return new DeformationGradientElementModelCacheData<Kernel>(numPlasticParams_, numElasticParams_);
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::freeCacheData(DeformationModelCacheData *data) const
{
  delete data;
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::prepareData(
  const double *x, const double *param, const double *materialParam, DeformationModelCacheData *cacheDataBase) const
{
  using CD = DeformationGradientElementModelCacheData<Kernel>;
  CD *cd = static_cast<CD *>(cacheDataBase);

  // Load current positions.
  for (int vi = 0; vi < numNodes; vi++) {
    cd->x.col(vi) = ES::V3d(x[vi * 3 + 0], x[vi * 3 + 1], x[vi * 3 + 2]);
  }

  // Plastic state.
  plasticModel_->computeA(param, cd->Fp.data());
  plasticModel_->computeAInv(param, cd->FpInv.data());
  cd->detFp = plasticModel_->compute_detA(param);

  for (int i = 0; i < numPlasticParams_; i++) {
    cd->plasticParam[i] = param[i];
  }

  if (numPlasticParams_ > 0) {
    plasticModel_->compute_ddetA_da(cd->plasticParam.data(), cd->ddetA_da.data(), numPlasticParams_);
    plasticModel_->compute_d2detA_da2(cd->plasticParam.data(), cd->d2detA_da2.data(), numPlasticParams_);

    for (int i = 0; i < numPlasticParams_; i++) {
      plasticModel_->compute_dAInv_da(cd->plasticParam.data(), i, cd->dAInv_dai[i].data());
      for (int j = 0; j < numPlasticParams_; j++) {
        plasticModel_->compute_d2AInv_da2(cd->plasticParam.data(), i, j, cd->d2AInv(i, j).data());
      }
    }
  }

  for (int i = 0; i < numElasticParams_; i++) {
    cd->materialParam[i] = materialParam[i];
  }

  // Per-quadrature-point kinematics.
  for (int q = 0; q < numQuadPts; q++) {
    kernel_.computeFref(x, q, cd->Fref[q].data());
    cd->Fe[q] = cd->Fref[q] * cd->FpInv;
    computeSVD(cd->Fe[q], cd->U[q], cd->V[q], cd->S[q]);
    computeCurrent_dFdx(kernel_.rest_dFdx(q), cd->FpInv, cd->dFdx[q]);
    cd->Bm[q] = cd->detFp * cd->FpInv.transpose() * kernel_.restBm(q);
  }
}

template<class Kernel>
double DeformationGradientElementModel<Kernel>::computeEnergy(const DeformationModelCacheData *cacheDataBase) const
{
  using CD = DeformationGradientElementModelCacheData<Kernel>;
  const CD *cd = static_cast<const CD *>(cacheDataBase);

  const double *mp = cd->numElasticParams ? cd->materialParam.data() : nullptr;
  double energy = 0.0;
  for (int q = 0; q < numQuadPts; q++) {
    energy += elasticModel_->compute_psi(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data()) *
      kernel_.weightDetJ(q) * cd->detFp;
  }
  return energy;
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::compute_dE_dx(const DeformationModelCacheData *cacheDataBase, double *grad) const
{
  using CD = DeformationGradientElementModelCacheData<Kernel>;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = cd->numElasticParams ? cd->materialParam.data() : nullptr;

  Eigen::Map<Eigen::Matrix<double, localDofs, 1>> gradMap(grad);
  gradMap.setZero();

  for (int q = 0; q < numQuadPts; q++) {
    ES::M3d P;
    elasticModel_->compute_P(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), P.data());
    const M3xN localForce = P * cd->Bm[q];
    gradMap += Eigen::Map<const Eigen::Matrix<double, localDofs, 1>>(localForce.data());
  }
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::compute_d2E_dx2(const DeformationModelCacheData *cacheDataBase, double *hess) const
{
  using CD = DeformationGradientElementModelCacheData<Kernel>;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = cd->numElasticParams ? cd->materialParam.data() : nullptr;

  using HessMat = Eigen::Matrix<double, localDofs, localDofs>;
  HessMat hessMat = HessMat::Zero();

  for (int q = 0; q < numQuadPts; q++) {
    ES::M9d dPdF;
    elasticModel_->compute_dPdF(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), dPdF.data());
    dPdF *= kernel_.weightDetJ(q) * cd->detFp;
    hessMat += cd->dFdx[q].transpose() * dPdF * cd->dFdx[q];
  }

  Eigen::Map<HessMat> hessMap(hess);
  hessMap = hessMat;
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::enableSPD(int) {}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::computeSVD(
  const ES::M3d &Fe, ES::M3d &U, ES::M3d &V, ES::V3d &S)
{
  Eigen::JacobiSVD<ES::M3d, Eigen::NoQRPreconditioner> svd(Fe, Eigen::ComputeFullU | Eigen::ComputeFullV);
  U = svd.matrixU();
  V = svd.matrixV();
  S = svd.singularValues();

  if (U.determinant() < 0.0) {
    U.col(2) *= -1.0;
    S(2) *= -1.0;
  }
  if (V.determinant() < 0.0) {
    V.col(2) *= -1.0;
    S(2) *= -1.0;
  }
}

template<class Kernel>
typename DeformationModel::LocalMaxStepResult
DeformationGradientElementModel<Kernel>::computeLocalMaxStepSize(
  const double *x_local, const double *dx_local) const
{
  LocalMaxStepResult result;
  for (int q = 0; q < numQuadPts; q++) {
    double F0[9], deltaF[9];
    computeF(x_local, q, F0);
    computeF(dx_local, q, deltaF);

    const auto poly = buildDeterminantCubicFromAffineMatrixPath(F0, deltaF, kCubicRelativeDetEps);
    const ConservativeFeasibleAlphaResult alphaResult =
      findConservativeFeasibleAlpha(poly, kCubicRelativeDetEps);
    if (alphaResult.alpha < result.alpha) {
      result.alpha = alphaResult.alpha;
      result.illegalInitialState = alphaResult.illegalInitialState;
      result.phi0 = alphaResult.phi0;
      result.eps = kCubicRelativeDetEps;
      result.locationId = q;
    }
    if (result.alpha <= kMaterialMaxStepMinClamp) {
      break;
    }
  }
  return result;
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::computeF(
  const double *x, int materialLocationID, double F[9]) const
{
  kernel_.computeFref(x, materialLocationID, F);
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::computeFe(
  const DeformationModelCacheData *cacheData, int materialLocationID, double F[9]) const
{
  using CD = DeformationGradientElementModelCacheData<Kernel>;
  const CD *cd = static_cast<const CD *>(cacheData);
  Eigen::Map<ES::M3d> FMap(F);
  FMap = cd->Fe[materialLocationID];
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::computeP(
  const DeformationModelCacheData *cacheData, int materialLocationID, double POut[9]) const
{
  using CD = DeformationGradientElementModelCacheData<Kernel>;
  const CD *cd = static_cast<const CD *>(cacheData);
  const double *mp = cd->numElasticParams ? cd->materialParam.data() : nullptr;

  ES::M3d P;
  elasticModel_->compute_P(mp, cd->Fe[materialLocationID].data(),
    cd->U[materialLocationID].data(), cd->V[materialLocationID].data(),
    cd->S[materialLocationID].data(), P.data());
  Eigen::Map<ES::M3d> PMap(POut);
  PMap = P;
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::computedPdF(
  const DeformationModelCacheData *cacheData, int materialLocationID, double dPdFOut[81]) const
{
  using CD = DeformationGradientElementModelCacheData<Kernel>;
  const CD *cd = static_cast<const CD *>(cacheData);
  const double *mp = cd->numElasticParams ? cd->materialParam.data() : nullptr;

  ES::M9d dPdF;
  elasticModel_->compute_dPdF(mp, cd->Fe[materialLocationID].data(),
    cd->U[materialLocationID].data(), cd->V[materialLocationID].data(),
    cd->S[materialLocationID].data(), dPdF.data());
  Eigen::Map<ES::M9d> dPdFMap(dPdFOut);
  dPdFMap = dPdF;
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::computedFdx(
  const DeformationModelCacheData *cacheData, int materialLocationID, double *dFdxOut) const
{
  using CD = DeformationGradientElementModelCacheData<Kernel>;
  const CD *cd = static_cast<const CD *>(cacheData);
  Eigen::Map<M9xNDOF> dFdxMap(dFdxOut);
  dFdxMap = cd->dFdx[materialLocationID];
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::computeForceFromP(
  const DeformationModelCacheData *cacheDataBase, int materialLocationID,
  const double P[9], double f[localDofs]) const
{
  using CD = DeformationGradientElementModelCacheData<Kernel>;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const Eigen::Map<const ES::M3d> PMap(P);
  const M3xN localForce = PMap * cd->Bm[materialLocationID];
  Eigen::Map<Eigen::Matrix<double, localDofs, 1>> fMap(f);
  fMap = Eigen::Map<const Eigen::Matrix<double, localDofs, 1>>(localForce.data());
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::vonMisesStress(
  const DeformationModelCacheData *cacheDataBase, int &nPt, double *stresses) const
{
  using CD = DeformationGradientElementModelCacheData<Kernel>;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = cd->numElasticParams ? cd->materialParam.data() : nullptr;
  nPt = numQuadPts;

  for (int q = 0; q < numQuadPts; q++) {
    ES::M3d P;
    elasticModel_->compute_P(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), P.data());
    const double detF = cd->Fe[q].determinant();
    ES::M3d cauchyStress = P * cd->Fe[q].transpose() / detF;

    const double t1 = std::pow(cauchyStress(0, 0) - cauchyStress(1, 1), 2.0);
    const double t2 = std::pow(cauchyStress(1, 1) - cauchyStress(2, 2), 2.0);
    const double t3 = std::pow(cauchyStress(2, 2) - cauchyStress(0, 0), 2.0);
    const double t4 = 6.0 * (std::pow(cauchyStress(1, 2), 2.0) +
                             std::pow(cauchyStress(2, 0), 2.0) +
                             std::pow(cauchyStress(0, 1), 2.0));
    stresses[q] = std::sqrt((t1 + t2 + t3 + t4) * 0.5);
  }
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::maxStrain(
  const DeformationModelCacheData *cacheDataBase, int &nPt, double *stresses) const
{
  using CD = DeformationGradientElementModelCacheData<Kernel>;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  nPt = numQuadPts;

  for (int q = 0; q < numQuadPts; q++) {
    ES::M3d E = 0.5 * (cd->Fe[q].transpose() * cd->Fe[q] - ES::M3d::Identity());
    Eigen::SelfAdjointEigenSolver<ES::M3d> eigSolver(E);
    stresses[q] = eigSolver.eigenvalues().maxCoeff();
  }
}

// ============================================================
// Plastic parameter derivatives
// ============================================================

template<class Kernel>
void DeformationGradientElementModel<Kernel>::compute_dE_da(
  const DeformationModelCacheData *cacheDataBase, double *grad) const
{
  using CD = DeformationGradientElementModelCacheData<Kernel>;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = cd->numElasticParams ? cd->materialParam.data() : nullptr;

  if (numPlasticParams_ == 0) return;

  ES::VXd gradVec = ES::VXd::Zero(numPlasticParams_);
  for (int q = 0; q < numQuadPts; q++) {
    double psi = elasticModel_->compute_psi(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data());

    ES::M3d P;
    elasticModel_->compute_P(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), P.data());

    for (int i = 0; i < numPlasticParams_; i++) {
      double dVda = compute_dV_dai(kernel_.weightDetJ(q), cd->ddetA_da[i]);
      double dpsi_da = compute_dpsi_dai(cd->Fref[q], cd->dAInv_dai[i], P);
      gradVec[i] += dVda * psi + kernel_.weightDetJ(q) * cd->detFp * dpsi_da;
    }
  }

  for (int i = 0; i < numPlasticParams_; i++) {
    grad[i] = gradVec[i];
  }
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::compute_d2E_da2(
  const DeformationModelCacheData *cacheDataBase, double *hess) const
{
  using CD = DeformationGradientElementModelCacheData<Kernel>;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = cd->numElasticParams ? cd->materialParam.data() : nullptr;

  if (numPlasticParams_ == 0) return;

  ES::MXd hessMat = ES::MXd::Zero(numPlasticParams_, numPlasticParams_);
  for (int q = 0; q < numQuadPts; q++) {
    const double vol = kernel_.weightDetJ(q) * cd->detFp;
    double psi = elasticModel_->compute_psi(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data());

    ES::M3d P;
    elasticModel_->compute_P(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), P.data());

    ES::M9d dPdF;
    elasticModel_->compute_dPdF(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), dPdF.data());

    for (int i = 0; i < numPlasticParams_; i++) {
      double dVda_i = compute_dV_dai(kernel_.weightDetJ(q), cd->ddetA_da[i]);
      double dpsi_da_i = compute_dpsi_dai(cd->Fref[q], cd->dAInv_dai[i], P);

      for (int j = 0; j < numPlasticParams_; j++) {
        double dVda_j = compute_dV_dai(kernel_.weightDetJ(q), cd->ddetA_da[j]);
        double d2V = compute_d2V_daidaj(kernel_.weightDetJ(q), cd->d2detA_da2(i, j));
        double dpsi_da_j = compute_dpsi_dai(cd->Fref[q], cd->dAInv_dai[j], P);
        double d2psi = compute_d2psi_dai_daj(cd->Fref[q],
          cd->dAInv_dai[i], cd->dAInv_dai[j], cd->d2AInv(i, j), P, dPdF);
        hessMat(i, j) += d2V * psi + dVda_i * dpsi_da_j +
                         dVda_j * dpsi_da_i + vol * d2psi;
      }
    }
  }

  Eigen::Map<ES::MXd>(hess, numPlasticParams_, numPlasticParams_) = hessMat;
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::compute_d2E_dxda(
  const DeformationModelCacheData *cacheDataBase, double *hess) const
{
  using CD = DeformationGradientElementModelCacheData<Kernel>;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = cd->numElasticParams ? cd->materialParam.data() : nullptr;

  if (numPlasticParams_ == 0) return;

  ES::MXd mixed = ES::MXd::Zero(localDofs, numPlasticParams_);
  for (int q = 0; q < numQuadPts; q++) {
    ES::M3d P;
    elasticModel_->compute_P(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), P.data());

    Eigen::Matrix<double, localDofs, 1> dpsi_dx = cd->dFdx[q].transpose() * Eigen::Map<const ES::V9d>(P.data());

    ES::M9d dPdF;
    elasticModel_->compute_dPdF(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), dPdF.data());

    const double vol = kernel_.weightDetJ(q) * cd->detFp;
    for (int i = 0; i < numPlasticParams_; i++) {
      const double dVda = compute_dV_dai(kernel_.weightDetJ(q), cd->ddetA_da[i]);

      ES::M3d dFda;
      compute_dFe_dai(cd->Fref[q], cd->dAInv_dai[i], dFda);

      ES::M3d dPda;
      compute_dP_dai(dPdF, dFda, dPda);

      Eigen::Matrix<double, localDofs, 1> temp = cd->dFdx[q].transpose() * Eigen::Map<const ES::V9d>(dPda.data());

      M9xNDOF d2F_duda;
      compute_d2Fe_dx_dai(cd->dAInv_dai[i], kernel_.rest_dFdx(q), d2F_duda);

      mixed.col(i) += dVda * dpsi_dx + vol *
        (temp + d2F_duda.transpose() * Eigen::Map<const ES::V9d>(P.data()));
    }
  }

  Eigen::Map<ES::MXd>(hess, localDofs, numPlasticParams_) = mixed;
}

// ============================================================
// Elastic parameter derivatives
// ============================================================

template<class Kernel>
void DeformationGradientElementModel<Kernel>::compute_dE_db(
  const DeformationModelCacheData *cacheDataBase, double *grad) const
{
  using CD = DeformationGradientElementModelCacheData<Kernel>;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = cd->numElasticParams ? cd->materialParam.data() : nullptr;

  if (numElasticParams_ == 0) return;

  ES::VXd gradVec = ES::VXd::Zero(numElasticParams_);
  for (int q = 0; q < numQuadPts; q++) {
    const double vol = kernel_.weightDetJ(q) * cd->detFp;
    for (int i = 0; i < numElasticParams_; i++) {
      gradVec[i] += vol * elasticModel_->compute_dpsi_dparam(mp, i,
        cd->Fe[q].data(), cd->U[q].data(), cd->V[q].data(), cd->S[q].data());
    }
  }

  for (int i = 0; i < numElasticParams_; i++) {
    grad[i] = gradVec[i];
  }
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::compute_d2E_db2(
  const DeformationModelCacheData *cacheDataBase, double *hess) const
{
  using CD = DeformationGradientElementModelCacheData<Kernel>;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = cd->numElasticParams ? cd->materialParam.data() : nullptr;

  if (numElasticParams_ == 0) return;

  ES::MXd hessMat = ES::MXd::Zero(numElasticParams_, numElasticParams_);
  for (int q = 0; q < numQuadPts; q++) {
    const double vol = kernel_.weightDetJ(q) * cd->detFp;
    for (int i = 0; i < numElasticParams_; i++) {
      for (int j = 0; j < numElasticParams_; j++) {
        hessMat(i, j) += vol * elasticModel_->compute_d2psi_dparam2(mp, i, j,
          cd->Fe[q].data(), cd->U[q].data(), cd->V[q].data(), cd->S[q].data());
      }
    }
  }

  Eigen::Map<ES::MXd>(hess, numElasticParams_, numElasticParams_) = hessMat;
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::compute_d2E_dxdb(
  const DeformationModelCacheData *cacheDataBase, double *hess) const
{
  using CD = DeformationGradientElementModelCacheData<Kernel>;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = cd->numElasticParams ? cd->materialParam.data() : nullptr;

  if (numElasticParams_ == 0) return;

  ES::MXd mixed = ES::MXd::Zero(localDofs, numElasticParams_);
  for (int q = 0; q < numQuadPts; q++) {
    const double vol = kernel_.weightDetJ(q) * cd->detFp;
    for (int i = 0; i < numElasticParams_; i++) {
      ES::M3d dPdb;
      elasticModel_->compute_dP_dparam(mp, i, cd->Fe[q].data(),
        cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), dPdb.data());
      mixed.col(i) += vol * (cd->dFdx[q].transpose() * Eigen::Map<const ES::V9d>(dPdb.data()));
    }
  }

  for (int col = 0; col < numElasticParams_; col++) {
    for (int row = 0; row < localDofs; row++) {
      hess[col * localDofs + row] = mixed(row, col);
    }
  }
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::compute_d2E_dadb(
  const DeformationModelCacheData *cacheDataBase, double *hess) const
{
  using CD = DeformationGradientElementModelCacheData<Kernel>;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = cd->numElasticParams ? cd->materialParam.data() : nullptr;

  if (numElasticParams_ == 0 || numPlasticParams_ == 0) return;

  Eigen::Map<ES::MXd> mixed(hess, numPlasticParams_, numElasticParams_);
  mixed.setZero();

  for (int q = 0; q < numQuadPts; q++) {
    const double vol = kernel_.weightDetJ(q) * cd->detFp;

    std::vector<ES::M3d> dPdb(numElasticParams_, ES::M3d::Zero());
    for (int i = 0; i < numElasticParams_; i++) {
      elasticModel_->compute_dP_dparam(mp, i, cd->Fe[q].data(),
        cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), dPdb[i].data());
    }

    std::vector<ES::M3d> dFda(numPlasticParams_, ES::M3d::Zero());
    for (int i = 0; i < numPlasticParams_; i++) {
      compute_dFe_dai(cd->Fref[q], cd->dAInv_dai[i], dFda[i]);
      double dVda = compute_dV_dai(kernel_.weightDetJ(q), cd->ddetA_da[i]);
      for (int j = 0; j < numElasticParams_; j++) {
        double dpsi_db = elasticModel_->compute_dpsi_dparam(mp, j,
          cd->Fe[q].data(), cd->U[q].data(), cd->V[q].data(), cd->S[q].data());
        mixed(i, j) += dVda * dpsi_db;
      }
    }

    for (int i = 0; i < numPlasticParams_; i++) {
      for (int j = 0; j < numElasticParams_; j++) {
        mixed(i, j) += vol * Eigen::Map<const ES::V9d>(dPdb[j].data())
          .dot(Eigen::Map<const ES::V9d>(dFda[i].data()));
      }
    }
  }
}

// ============================================================
// Private helper methods
// ============================================================

template<class Kernel>
void DeformationGradientElementModel<Kernel>::computeCurrent_dFdx(
  const M9xNDOF &rest_dFdx, const ES::M3d &FpInv, M9xNDOF &dFdx) const
{
  for (int col = 0; col < localDofs; col++) {
    const Eigen::Map<const ES::M3d> dFref(rest_dFdx.col(col).data());
    const ES::M3d dFe = dFref * FpInv;
    dFdx.col(col) = Eigen::Map<const ES::V9d>(dFe.data());
  }
}

template<class Kernel>
double DeformationGradientElementModel<Kernel>::compute_dV_dai(
  double weightDetJ, double ddetA_dai) const
{
  return weightDetJ * ddetA_dai;
}

template<class Kernel>
double DeformationGradientElementModel<Kernel>::compute_d2V_daidaj(
  double weightDetJ, double d2detA_daidaj) const
{
  return weightDetJ * d2detA_daidaj;
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::compute_dFe_dai(
  const ES::M3d &Fref, const ES::M3d &dAInvdai, ES::M3d &dFdai) const
{
  dFdai = Fref * dAInvdai;
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::compute_d2Fe_dai_daj(
  const ES::M3d &Fref, const ES::M3d &dAInvdaidaj, ES::M3d &d2Fdaidaj) const
{
  d2Fdaidaj = Fref * dAInvdaidaj;
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::compute_dP_dai(
  const ES::M9d &dPdF, const ES::M3d &dFdai, ES::M3d &dPdai) const
{
  Eigen::Map<ES::V9d>(dPdai.data()) = dPdF * Eigen::Map<const ES::V9d>(dFdai.data());
}

template<class Kernel>
double DeformationGradientElementModel<Kernel>::compute_dpsi_dai(
  const ES::M3d &Fref, const ES::M3d &dAInv_dai, const ES::M3d &P) const
{
  ES::M3d dFe_dai;
  compute_dFe_dai(Fref, dAInv_dai, dFe_dai);
  return P.cwiseProduct(dFe_dai).sum();
}

template<class Kernel>
double DeformationGradientElementModel<Kernel>::compute_d2psi_dai_daj(
  const ES::M3d &Fref, const ES::M3d &dAInv_dai, const ES::M3d &dAInv_daj,
  const ES::M3d &d2AInv_dai_daj, const ES::M3d &P, const ES::M9d &dPdF) const
{
  ES::M3d dFe_dai, dFe_daj;
  compute_dFe_dai(Fref, dAInv_dai, dFe_dai);
  compute_dFe_dai(Fref, dAInv_daj, dFe_daj);

  ES::M3d dP_daj;
  compute_dP_dai(dPdF, dFe_daj, dP_daj);

  ES::M3d d2Fe_daidaj;
  compute_d2Fe_dai_daj(Fref, d2AInv_dai_daj, d2Fe_daidaj);

  return dP_daj.cwiseProduct(dFe_dai).sum() + P.cwiseProduct(d2Fe_daidaj).sum();
}

template<class Kernel>
void DeformationGradientElementModel<Kernel>::compute_d2Fe_dx_dai(
  const ES::M3d &dAInvdai, const M9xNDOF &rest_dFdx, M9xNDOF &d2Fdudai) const
{
  for (int col = 0; col < localDofs; col++) {
    const Eigen::Map<const ES::M3d> dFref(rest_dFdx.col(col).data());
    const ES::M3d d2Fe = dFref * dAInvdai;
    d2Fdudai.col(col) = Eigen::Map<const ES::V9d>(d2Fe.data());
  }
}

}  // namespace SolidDeformationModel
}  // namespace pgo
