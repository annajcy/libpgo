#include "deformationGradientElementModel.h"

#include "../../materialMaxStepPolynomialUtils.h"
#include "../parameters/constantParameterField.h"

#include <stdexcept>

namespace ES = pgo::EigenSupport;

namespace pgo
{
namespace SolidDeformationModel
{

// ============================================================
// Constructors
// ============================================================

DeformationGradientElementModel::DeformationGradientElementModel(
  int ele, DeformationGradientKernel &&kernel,
  const ElasticBlock &elasticBlock, const PlasticBlock &plasticBlock):
  DeformationModel(elasticBlock.model, plasticBlock.model),
  numNodes_(kernel.numNodes()),
  numQuadPts_(kernel.numQuadraturePoints()),
  localDofs_(kernel.localDofs()),
  kernel_(std::move(kernel)),
  ele_(ele),
  elasticBlock_(elasticBlock),
  plasticBlock_(plasticBlock)
{
  elasticModel_ = dynamic_cast<const ElasticModel3DDeformationGradient *>(elasticBlock.model);
  plasticModel_ = dynamic_cast<const PlasticModel3DDeformationGradient *>(plasticBlock.model);

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

// ============================================================
// allocateCacheData / elasticParamsPtr
// ============================================================

std::unique_ptr<DeformationModelCacheData>
DeformationGradientElementModel::allocateCacheData() const
{
  return std::make_unique<DeformationGradientElementModelCacheData>(
    numNodes_, numQuadPts_, numPlasticParams_, numElasticParams_);
}

const double *DeformationGradientElementModel::elasticParamsPtr(
  const DeformationModelCacheData *cacheData) const
{
  using CD = DeformationGradientElementModelCacheData;
  const CD *cd = static_cast<const CD *>(cacheData);
  return cd->numElasticParams ? cd->elasticParamsValue.data() : nullptr;
}

// ============================================================
// prepareData
// ============================================================

void DeformationGradientElementModel::prepareData(
  const double *x, DeformationModelCacheData *cacheDataBase) const
{
  using CD = DeformationGradientElementModelCacheData;
  CD *cd = static_cast<CD *>(cacheDataBase);

  for (int vi = 0; vi < numNodes_; vi++) {
    cd->x.col(vi) = ES::V3d(x[vi * 3 + 0], x[vi * 3 + 1], x[vi * 3 + 2]);
  }

  for (int q = 0; q < numQuadPts_; q++) {
    if (plasticBlock_.parameters && numPlasticParams_ > 0) {
      plasticBlock_.parameters->computeValue(ele_, q, cd->plasticParamsValue.data());
      if (auto *opt = dynamic_cast<const OptimizableField *>(plasticBlock_.parameters))
        opt->computeDerivative(ele_, q, cd->plasticParamsDeriv.data());
    }

    const double *plasticParams = cd->plasticParamsValue.data();
    plasticModel_->computeA(plasticParams, cd->Fp[q].data());
    plasticModel_->computeAInv(plasticParams, cd->FpInv[q].data());
    cd->detFp[q] = plasticModel_->compute_detA(plasticParams);

    if (numPlasticParams_ > 0) {
      plasticModel_->compute_ddetA_da(plasticParams, cd->ddetA_da[q].data(), numPlasticParams_);
      plasticModel_->compute_d2detA_da2(plasticParams, cd->d2detA_da2[q].data(), numPlasticParams_);

      for (int i = 0; i < numPlasticParams_; i++) {
        plasticModel_->compute_dAInv_da(plasticParams, i, cd->dAInv_dai[q][i].data());
        for (int j = 0; j < numPlasticParams_; j++) {
          plasticModel_->compute_d2AInv_da2(plasticParams, i, j,
            cd->d2AInv(q, i, j).data());
        }
      }
    }

    if (elasticBlock_.parameters && numElasticParams_ > 0) {
      elasticBlock_.parameters->computeValue(ele_, q, cd->elasticParamsValue.data());
      if (auto *opt = dynamic_cast<const OptimizableField *>(elasticBlock_.parameters))
        opt->computeDerivative(ele_, q, cd->elasticParamsDeriv.data());
    }

    kernel_.computeFref(x, q, cd->Fref[q].data());
    cd->Fe[q] = cd->Fref[q] * cd->FpInv[q];
    computeSVD(cd->Fe[q], cd->U[q], cd->V[q], cd->S[q]);
    computeCurrent_dFdx(kernel_.rest_dFdx(q), cd->FpInv[q], cd->dFdx[q]);
    cd->Bm[q] = cd->detFp[q] * cd->FpInv[q].transpose() * kernel_.restBm(q);
  }
}

// ============================================================
// Energy and derivatives w.r.t. displacement
// ============================================================

double DeformationGradientElementModel::computeEnergy(
  const DeformationModelCacheData *cacheDataBase) const
{
  using CD = DeformationGradientElementModelCacheData;
  const CD *cd = static_cast<const CD *>(cacheDataBase);

  const double *mp = elasticParamsPtr(cacheDataBase);
  double energy = 0.0;
  for (int q = 0; q < numQuadPts_; q++) {
    energy += elasticModel_->compute_psi(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data()) *
      kernel_.weightDetJ(q) * cd->detFp[q];
  }
  return energy;
}

void DeformationGradientElementModel::compute_dE_dx(
  const DeformationModelCacheData *cacheDataBase, double *grad) const
{
  using CD = DeformationGradientElementModelCacheData;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = elasticParamsPtr(cacheDataBase);

  Eigen::Map<Eigen::VectorXd> gradMap(grad, localDofs_);
  gradMap.setZero();

  for (int q = 0; q < numQuadPts_; q++) {
    ES::M3d P;
    elasticModel_->compute_P(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), P.data());
    const M3xN localForce = P * cd->Bm[q];
    gradMap += Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>>(
      localForce.data(), localDofs_);
  }
}

void DeformationGradientElementModel::compute_d2E_dx2(
  const DeformationModelCacheData *cacheDataBase, double *hess) const
{
  using CD = DeformationGradientElementModelCacheData;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = elasticParamsPtr(cacheDataBase);

  Eigen::MatrixXd hessMat = Eigen::MatrixXd::Zero(localDofs_, localDofs_);

  for (int q = 0; q < numQuadPts_; q++) {
    ES::M9d dPdF;
    elasticModel_->compute_dPdF(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), dPdF.data());
    dPdF *= kernel_.weightDetJ(q) * cd->detFp[q];
    hessMat += cd->dFdx[q].transpose() * dPdF * cd->dFdx[q];
  }

  Eigen::Map<Eigen::MatrixXd> hessMap(hess, localDofs_, localDofs_);
  hessMap = hessMat;
}

void DeformationGradientElementModel::enableSPD(int) {}

// ============================================================
// computeSVD
// ============================================================

void DeformationGradientElementModel::computeSVD(
  const ES::M3d &Fe, ES::M3d &U, ES::M3d &V, ES::V3d &S)
{
  Eigen::JacobiSVD<ES::M3d, Eigen::NoQRPreconditioner> svd(
    Fe, Eigen::ComputeFullU | Eigen::ComputeFullV);
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

// ============================================================
// computeLocalMaxStepSize
// ============================================================

DeformationModel::LocalMaxStepResult
DeformationGradientElementModel::computeLocalMaxStepSize(
  const double *x_local, const double *dx_local) const
{
  LocalMaxStepResult result;
  for (int q = 0; q < numQuadPts_; q++) {
    double F0[9], deltaF[9];
    computeF(x_local, q, F0);
    computeF(dx_local, q, deltaF);

    const auto poly = buildDeterminantCubicFromAffineMatrixPath(
      F0, deltaF, kCubicRelativeDetEps);
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

// ============================================================
// computeF / computeFe / computeP / computedPdF / computedFdx / computeForceFromP
// ============================================================

void DeformationGradientElementModel::computeF(
  const double *x, int materialLocationID, double F[9]) const
{
  kernel_.computeFref(x, materialLocationID, F);
}

void DeformationGradientElementModel::computeFe(
  const DeformationModelCacheData *cacheData, int materialLocationID, double F[9]) const
{
  using CD = DeformationGradientElementModelCacheData;
  const CD *cd = static_cast<const CD *>(cacheData);
  Eigen::Map<ES::M3d> FMap(F);
  FMap = cd->Fe[materialLocationID];
}

void DeformationGradientElementModel::computeP(
  const DeformationModelCacheData *cacheData, int materialLocationID, double POut[9]) const
{
  using CD = DeformationGradientElementModelCacheData;
  const CD *cd = static_cast<const CD *>(cacheData);
  const double *mp = elasticParamsPtr(cacheData);

  ES::M3d P;
  elasticModel_->compute_P(mp, cd->Fe[materialLocationID].data(),
    cd->U[materialLocationID].data(), cd->V[materialLocationID].data(),
    cd->S[materialLocationID].data(), P.data());
  Eigen::Map<ES::M3d> PMap(POut);
  PMap = P;
}

void DeformationGradientElementModel::computedPdF(
  const DeformationModelCacheData *cacheData, int materialLocationID, double dPdFOut[81]) const
{
  using CD = DeformationGradientElementModelCacheData;
  const CD *cd = static_cast<const CD *>(cacheData);
  const double *mp = elasticParamsPtr(cacheData);

  ES::M9d dPdF;
  elasticModel_->compute_dPdF(mp, cd->Fe[materialLocationID].data(),
    cd->U[materialLocationID].data(), cd->V[materialLocationID].data(),
    cd->S[materialLocationID].data(), dPdF.data());
  Eigen::Map<ES::M9d> dPdFMap(dPdFOut);
  dPdFMap = dPdF;
}

void DeformationGradientElementModel::computedFdx(
  const DeformationModelCacheData *cacheData, int materialLocationID, double *dFdxOut) const
{
  using CD = DeformationGradientElementModelCacheData;
  const CD *cd = static_cast<const CD *>(cacheData);
  Eigen::Map<M9xNDOF> dFdxMap(dFdxOut, 9, localDofs_);
  dFdxMap = cd->dFdx[materialLocationID];
}

void DeformationGradientElementModel::computeForceFromP(
  const DeformationModelCacheData *cacheDataBase, int materialLocationID,
  const double P[9], double f[]) const
{
  using CD = DeformationGradientElementModelCacheData;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const Eigen::Map<const ES::M3d> PMap(P);
  const M3xN localForce = PMap * cd->Bm[materialLocationID];
  Eigen::Map<Eigen::VectorXd> fMap(f, localDofs_);
  fMap = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>>(
    localForce.data(), localDofs_);
}

// ============================================================
// vonMisesStress / maxStrain
// ============================================================

void DeformationGradientElementModel::vonMisesStress(
  const DeformationModelCacheData *cacheDataBase, int &nPt, double *stresses) const
{
  using CD = DeformationGradientElementModelCacheData;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = elasticParamsPtr(cacheDataBase);
  nPt = numQuadPts_;

  for (int q = 0; q < numQuadPts_; q++) {
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

void DeformationGradientElementModel::maxStrain(
  const DeformationModelCacheData *cacheDataBase, int &nPt, double *stresses) const
{
  using CD = DeformationGradientElementModelCacheData;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  nPt = numQuadPts_;

  for (int q = 0; q < numQuadPts_; q++) {
    ES::M3d E = 0.5 * (cd->Fe[q].transpose() * cd->Fe[q] - ES::M3d::Identity());
    Eigen::SelfAdjointEigenSolver<ES::M3d> eigSolver(E);
    stresses[q] = eigSolver.eigenvalues().maxCoeff();
  }
}

// ============================================================
// Plastic parameter derivatives (with chain rule)
// ============================================================

void DeformationGradientElementModel::compute_dE_da(
  const DeformationModelCacheData *cacheDataBase, double *grad) const
{
  using CD = DeformationGradientElementModelCacheData;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = elasticParamsPtr(cacheDataBase);

  if (numPlasticParams_ == 0) return;

  ES::VXd gradVec = ES::VXd::Zero(numPlasticParams_);
  for (int q = 0; q < numQuadPts_; q++) {
    double psi = elasticModel_->compute_psi(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data());

    ES::M3d P;
    elasticModel_->compute_P(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), P.data());

    for (int i = 0; i < numPlasticParams_; i++) {
      double dVda = compute_dV_dai(kernel_.weightDetJ(q), cd->ddetA_da[q][i]);
      double dpsi_da = compute_dpsi_dai(cd->Fref[q], cd->dAInv_dai[q][i], P);
      gradVec[i] += dVda * psi + kernel_.weightDetJ(q) * cd->detFp[q] * dpsi_da;
    }
  }

  gradVec = cd->plasticParamsDeriv.transpose() * gradVec;

  for (int i = 0; i < numPlasticParams_; i++) {
    grad[i] = gradVec[i];
  }
}

void DeformationGradientElementModel::compute_d2E_da2(
  const DeformationModelCacheData *cacheDataBase, double *hess) const
{
  using CD = DeformationGradientElementModelCacheData;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = elasticParamsPtr(cacheDataBase);

  if (numPlasticParams_ == 0) return;

  ES::MXd hessMat = ES::MXd::Zero(numPlasticParams_, numPlasticParams_);
  for (int q = 0; q < numQuadPts_; q++) {
    const double vol = kernel_.weightDetJ(q) * cd->detFp[q];
    double psi = elasticModel_->compute_psi(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data());

    ES::M3d P;
    elasticModel_->compute_P(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), P.data());

    ES::M9d dPdF;
    elasticModel_->compute_dPdF(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), dPdF.data());

    for (int i = 0; i < numPlasticParams_; i++) {
      double dVda_i = compute_dV_dai(kernel_.weightDetJ(q), cd->ddetA_da[q][i]);
      double dpsi_da_i = compute_dpsi_dai(cd->Fref[q], cd->dAInv_dai[q][i], P);

      for (int j = 0; j < numPlasticParams_; j++) {
        double dVda_j = compute_dV_dai(kernel_.weightDetJ(q), cd->ddetA_da[q][j]);
        double d2V = compute_d2V_daidaj(kernel_.weightDetJ(q), cd->d2detA_da2[q](i, j));
        double dpsi_da_j = compute_dpsi_dai(cd->Fref[q], cd->dAInv_dai[q][j], P);
        double d2psi = compute_d2psi_dai_daj(cd->Fref[q],
          cd->dAInv_dai[q][i], cd->dAInv_dai[q][j],
          cd->d2AInv(q, i, j), P, dPdF);
        hessMat(i, j) += d2V * psi + dVda_i * dpsi_da_j +
                         dVda_j * dpsi_da_i + vol * d2psi;
      }
    }
  }

  const auto &dVal = cd->plasticParamsDeriv;
  hessMat = dVal.transpose() * hessMat * dVal;

  Eigen::Map<ES::MXd>(hess, numPlasticParams_, numPlasticParams_) = hessMat;
}

void DeformationGradientElementModel::compute_d2E_dxda(
  const DeformationModelCacheData *cacheDataBase, double *hess) const
{
  using CD = DeformationGradientElementModelCacheData;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = elasticParamsPtr(cacheDataBase);

  if (numPlasticParams_ == 0) return;

  ES::MXd mixed = ES::MXd::Zero(localDofs_, numPlasticParams_);
  for (int q = 0; q < numQuadPts_; q++) {
    ES::M3d P;
    elasticModel_->compute_P(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), P.data());

    Eigen::VectorXd dpsi_dx = cd->dFdx[q].transpose() *
      Eigen::Map<const ES::V9d>(P.data());

    ES::M9d dPdF;
    elasticModel_->compute_dPdF(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), dPdF.data());

    const double vol = kernel_.weightDetJ(q) * cd->detFp[q];
    for (int i = 0; i < numPlasticParams_; i++) {
      const double dVda = compute_dV_dai(kernel_.weightDetJ(q), cd->ddetA_da[q][i]);

      ES::M3d dFda;
      compute_dFe_dai(cd->Fref[q], cd->dAInv_dai[q][i], dFda);

      ES::M3d dPda;
      compute_dP_dai(dPdF, dFda, dPda);

      Eigen::VectorXd temp = cd->dFdx[q].transpose() *
        Eigen::Map<const ES::V9d>(dPda.data());

      M9xNDOF d2F_duda(9, localDofs_);
      compute_d2Fe_dx_dai(cd->dAInv_dai[q][i], kernel_.rest_dFdx(q), d2F_duda);

      mixed.col(i) += dVda * dpsi_dx + vol *
        (temp + d2F_duda.transpose() * Eigen::Map<const ES::V9d>(P.data()));
    }
  }

  mixed = mixed * cd->plasticParamsDeriv;

  Eigen::Map<ES::MXd>(hess, localDofs_, numPlasticParams_) = mixed;
}

// ============================================================
// Elastic parameter derivatives (with chain rule)
// ============================================================

void DeformationGradientElementModel::compute_dE_db(
  const DeformationModelCacheData *cacheDataBase, double *grad) const
{
  using CD = DeformationGradientElementModelCacheData;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = elasticParamsPtr(cacheDataBase);

  if (numElasticParams_ == 0) return;

  ES::VXd gradVec = ES::VXd::Zero(numElasticParams_);
  for (int q = 0; q < numQuadPts_; q++) {
    const double vol = kernel_.weightDetJ(q) * cd->detFp[q];
    for (int i = 0; i < numElasticParams_; i++) {
      gradVec[i] += vol * elasticModel_->compute_dpsi_dparam(mp, i,
        cd->Fe[q].data(), cd->U[q].data(), cd->V[q].data(), cd->S[q].data());
    }
  }

  gradVec = cd->elasticParamsDeriv.transpose() * gradVec;

  for (int i = 0; i < numElasticParams_; i++) {
    grad[i] = gradVec[i];
  }
}

void DeformationGradientElementModel::compute_d2E_db2(
  const DeformationModelCacheData *cacheDataBase, double *hess) const
{
  using CD = DeformationGradientElementModelCacheData;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = elasticParamsPtr(cacheDataBase);

  if (numElasticParams_ == 0) return;

  ES::MXd hessMat = ES::MXd::Zero(numElasticParams_, numElasticParams_);
  for (int q = 0; q < numQuadPts_; q++) {
    const double vol = kernel_.weightDetJ(q) * cd->detFp[q];
    for (int i = 0; i < numElasticParams_; i++) {
      for (int j = 0; j < numElasticParams_; j++) {
        hessMat(i, j) += vol * elasticModel_->compute_d2psi_dparam2(mp, i, j,
          cd->Fe[q].data(), cd->U[q].data(), cd->V[q].data(), cd->S[q].data());
      }
    }
  }

  const auto &dVal = cd->elasticParamsDeriv;
  hessMat = dVal.transpose() * hessMat * dVal;

  Eigen::Map<ES::MXd>(hess, numElasticParams_, numElasticParams_) = hessMat;
}

void DeformationGradientElementModel::compute_d2E_dxdb(
  const DeformationModelCacheData *cacheDataBase, double *hess) const
{
  using CD = DeformationGradientElementModelCacheData;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = elasticParamsPtr(cacheDataBase);

  if (numElasticParams_ == 0) return;

  ES::MXd mixed = ES::MXd::Zero(localDofs_, numElasticParams_);
  for (int q = 0; q < numQuadPts_; q++) {
    const double vol = kernel_.weightDetJ(q) * cd->detFp[q];
    for (int i = 0; i < numElasticParams_; i++) {
      ES::M3d dPdb;
      elasticModel_->compute_dP_dparam(mp, i, cd->Fe[q].data(),
        cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), dPdb.data());
      mixed.col(i) += vol * (cd->dFdx[q].transpose() *
        Eigen::Map<const ES::V9d>(dPdb.data()));
    }
  }

  mixed = mixed * cd->elasticParamsDeriv;

  for (int col = 0; col < numElasticParams_; col++) {
    for (int row = 0; row < localDofs_; row++) {
      hess[col * localDofs_ + row] = mixed(row, col);
    }
  }
}

void DeformationGradientElementModel::compute_d2E_dadb(
  const DeformationModelCacheData *cacheDataBase, double *hess) const
{
  using CD = DeformationGradientElementModelCacheData;
  const CD *cd = static_cast<const CD *>(cacheDataBase);
  const double *mp = elasticParamsPtr(cacheDataBase);

  if (numElasticParams_ == 0 || numPlasticParams_ == 0) return;

  Eigen::Map<ES::MXd> mixed(hess, numPlasticParams_, numElasticParams_);
  mixed.setZero();

  for (int q = 0; q < numQuadPts_; q++) {
    const double vol = kernel_.weightDetJ(q) * cd->detFp[q];

    std::vector<ES::M3d> dPdb(numElasticParams_, ES::M3d::Zero());
    for (int i = 0; i < numElasticParams_; i++) {
      elasticModel_->compute_dP_dparam(mp, i, cd->Fe[q].data(),
        cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), dPdb[i].data());
    }

    std::vector<ES::M3d> dFda(numPlasticParams_, ES::M3d::Zero());
    for (int i = 0; i < numPlasticParams_; i++) {
      compute_dFe_dai(cd->Fref[q], cd->dAInv_dai[q][i], dFda[i]);
      double dVda = compute_dV_dai(kernel_.weightDetJ(q), cd->ddetA_da[q][i]);
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

  const auto &dValP = cd->plasticParamsDeriv;
  const auto &dValE = cd->elasticParamsDeriv;
  mixed = dValP.transpose() * mixed * dValE;
}

// ============================================================
// Private helper methods
// ============================================================

void DeformationGradientElementModel::computeCurrent_dFdx(
  const M9xNDOF &rest_dFdx, const ES::M3d &FpInv, M9xNDOF &dFdx) const
{
  for (int col = 0; col < localDofs_; col++) {
    const Eigen::Map<const ES::M3d> dFref(rest_dFdx.col(col).data());
    const ES::M3d dFe = dFref * FpInv;
    dFdx.col(col) = Eigen::Map<const ES::V9d>(dFe.data());
  }
}

double DeformationGradientElementModel::compute_dV_dai(
  double weightDetJ, double ddetA_dai) const
{
  return weightDetJ * ddetA_dai;
}

double DeformationGradientElementModel::compute_d2V_daidaj(
  double weightDetJ, double d2detA_daidaj) const
{
  return weightDetJ * d2detA_daidaj;
}

void DeformationGradientElementModel::compute_dFe_dai(
  const ES::M3d &Fref, const ES::M3d &dAInvdai, ES::M3d &dFdai) const
{
  dFdai = Fref * dAInvdai;
}

void DeformationGradientElementModel::compute_d2Fe_dai_daj(
  const ES::M3d &Fref, const ES::M3d &dAInvdaidaj, ES::M3d &d2Fdaidaj) const
{
  d2Fdaidaj = Fref * dAInvdaidaj;
}

void DeformationGradientElementModel::compute_dP_dai(
  const ES::M9d &dPdF, const ES::M3d &dFdai, ES::M3d &dPdai) const
{
  Eigen::Map<ES::V9d>(dPdai.data()) = dPdF *
    Eigen::Map<const ES::V9d>(dFdai.data());
}

double DeformationGradientElementModel::compute_dpsi_dai(
  const ES::M3d &Fref, const ES::M3d &dAInv_dai, const ES::M3d &P) const
{
  ES::M3d dFe_dai;
  compute_dFe_dai(Fref, dAInv_dai, dFe_dai);
  return P.cwiseProduct(dFe_dai).sum();
}

double DeformationGradientElementModel::compute_d2psi_dai_daj(
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

void DeformationGradientElementModel::compute_d2Fe_dx_dai(
  const ES::M3d &dAInvdai, const M9xNDOF &rest_dFdx, M9xNDOF &d2Fdudai) const
{
  for (int col = 0; col < localDofs_; col++) {
    const Eigen::Map<const ES::M3d> dFref(rest_dFdx.col(col).data());
    const ES::M3d d2Fe = dFref * dAInvdai;
    d2Fdudai.col(col) = Eigen::Map<const ES::V9d>(d2Fe.data());
  }
}

}  // namespace SolidDeformationModel
}  // namespace pgo
