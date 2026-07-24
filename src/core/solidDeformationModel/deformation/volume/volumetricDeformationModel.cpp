#include "volumetricDeformationModel.h"

#include "deformation/materialMaxStepPolynomialUtils.h"
#include "pgoLogging.h"

#include <stdexcept>
#include <utility>

namespace ES = pgo::EigenSupport;

namespace pgo
{
namespace SolidDeformationModel
{
namespace
{
const VolumetricDeformationModelCacheData *checkedCacheData(
  const VolumetricDeformationModel &model,
  const DeformationModelCacheData *cacheData)
{
  PGO_ALOG(cacheData != nullptr);
  PGO_ALOG(cacheData == nullptr || cacheData->isPrepared());
  PGO_ALOG(cacheData == nullptr || model.isCacheDataCompatible(*cacheData));
  return static_cast<const VolumetricDeformationModelCacheData *>(cacheData);
}

std::pair<int, int> materialLocationRange(int materialLocation, int count)
{
  if (materialLocation < 0)
    return { 0, count };
  if (materialLocation >= count)
    throw std::out_of_range("Volumetric material location is out of range.");
  return { materialLocation, materialLocation + 1 };
}
}  // namespace

// ============================================================
// Constructors
// ============================================================

VolumetricDeformationModel::VolumetricDeformationModel(
  VolumetricElementMapping &&mapping,
  std::unique_ptr<ElasticModel3DDeformationGradient> elasticModel,
  std::unique_ptr<PlasticModel3DDeformationGradient> plasticModel):
  DeformationModel(),
  numNodes_(mapping.numNodes()),
  numQuadPts_(mapping.numQuadraturePoints()),
  localDofs_(mapping.localDofs()),
  elementMapping_(std::move(mapping)),
  elasticModel_(std::move(elasticModel)),
  plasticModel_(std::move(plasticModel))
{
  if (elasticModel_ == nullptr) {
    throw std::invalid_argument(
      "VolumetricDeformationModel requires ElasticModel3DDeformationGradient.");
  }
  if (plasticModel_ == nullptr) {
    throw std::invalid_argument(
      "VolumetricDeformationModel requires PlasticModel3DDeformationGradient.");
  }

  numPlasticParams_ = plasticModel_->getNumParameters();
  numElasticParams_ = elasticModel_->getNumParameters();
}

void VolumetricDeformationModel::defaultPlasticParams(double *params) const
{
  plasticModel_->defaultParams(params);
}

// ============================================================
// allocateCacheData / elasticParamsPtr
// ============================================================

std::unique_ptr<DeformationModelCacheData>
VolumetricDeformationModel::allocateCacheData() const
{
  return std::make_unique<VolumetricDeformationModelCacheData>(
    numNodes_, numQuadPts_, numPlasticParams_, numElasticParams_);
}

bool VolumetricDeformationModel::isCacheDataCompatible(
  const DeformationModelCacheData &cacheData) const
{
  const auto *cd = dynamic_cast<const VolumetricDeformationModelCacheData *>(&cacheData);
  return cd != nullptr &&
    cd->numNodes == numNodes_ &&
    cd->numQuadPts == numQuadPts_ &&
    cd->numPlasticParams == numPlasticParams_ &&
    cd->numElasticParams == numElasticParams_;
}

const double *VolumetricDeformationModel::elasticParamsPtr(
  const DeformationModelCacheData *cacheData, int q) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD *cd = static_cast<const CD *>(cacheData);
  return cd->numElasticParams ? cd->elasticParamsValue[q].data() : nullptr;
}

// ============================================================
// prepareData
// ============================================================

void VolumetricDeformationModel::prepareData(
  const double *x, const double *elasticParams, const double *plasticParams,
  DeformationModelCacheData *cacheDataBase) const
{
  using CD = VolumetricDeformationModelCacheData;
  PGO_ALOG(cacheDataBase != nullptr);
  PGO_ALOG(cacheDataBase == nullptr || isCacheDataCompatible(*cacheDataBase));
  CD *cd = static_cast<CD *>(cacheDataBase);

  for (int vi = 0; vi < numNodes_; vi++) {
    cd->x.col(vi) = ES::V3d(x[vi * 3 + 0], x[vi * 3 + 1], x[vi * 3 + 2]);
  }

  for (int q = 0; q < numQuadPts_; q++) {
    if (numPlasticParams_ > 0) {
      if (plasticParams) {
        cd->plasticParamsValue[q] =
          Eigen::Map<const ES::VXd>(plasticParams + static_cast<std::ptrdiff_t>(q) * numPlasticParams_, numPlasticParams_);
      }
      else {
        cd->plasticParamsValue[q].setZero();
      }
    }

    const double *plasticParamsAtQ = numPlasticParams_ > 0 ? cd->plasticParamsValue[q].data() : nullptr;
    plasticModel_->computeA(plasticParamsAtQ, cd->Fp[q].data());
    plasticModel_->computeAInv(plasticParamsAtQ, cd->FpInv[q].data());
    cd->detFp[q] = plasticModel_->compute_detA(plasticParamsAtQ);

    if (numPlasticParams_ > 0) {
      plasticModel_->compute_ddetA_da(plasticParamsAtQ, cd->ddetA_da[q].data(), numPlasticParams_);
      plasticModel_->compute_d2detA_da2(plasticParamsAtQ, cd->d2detA_da2[q].data(), numPlasticParams_);

      for (int i = 0; i < numPlasticParams_; i++) {
        plasticModel_->compute_dAInv_da(plasticParamsAtQ, i, cd->dAInv_dai[q][i].data());
        for (int j = 0; j < numPlasticParams_; j++) {
          plasticModel_->compute_d2AInv_da2(plasticParamsAtQ, i, j,
            cd->d2AInv(q, i, j).data());
        }
      }
    }

    if (numElasticParams_ > 0) {
      if (elasticParams) {
        cd->elasticParamsValue[q] =
          Eigen::Map<const ES::VXd>(elasticParams + static_cast<std::ptrdiff_t>(q) * numElasticParams_, numElasticParams_);
      }
      else {
        cd->elasticParamsValue[q].setZero();
      }
    }

    elementMapping_.computeFref(x, q, cd->Fref[q].data());
    cd->Fe[q] = cd->Fref[q] * cd->FpInv[q];
    computeSVD(cd->Fe[q], cd->U[q], cd->V[q], cd->S[q]);
    computeCurrent_dFdx(elementMapping_.rest_dFdx(q), cd->FpInv[q], cd->dFdx[q]);
    cd->Bm[q] = cd->detFp[q] * cd->FpInv[q].transpose() * elementMapping_.restBm(q);
  }
  cacheDataBase->markPrepared();
}

// ============================================================
// Energy and derivatives w.r.t. displacement
// ============================================================

double VolumetricDeformationModel::computeEnergy(
  const DeformationModelCacheData *cacheDataBase) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD *cd = checkedCacheData(*this, cacheDataBase);

  double energy = 0.0;
  for (int q = 0; q < numQuadPts_; q++) {
    const double *mp = elasticParamsPtr(cacheDataBase, q);
    energy += elasticModel_->compute_psi(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data()) *
      elementMapping_.weightDetJ(q) * cd->detFp[q];
  }
  return energy;
}

void VolumetricDeformationModel::compute_dE_dx(
  const DeformationModelCacheData *cacheDataBase, double *grad) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD *cd = checkedCacheData(*this, cacheDataBase);

  Eigen::Map<Eigen::VectorXd> gradMap(grad, localDofs_);
  gradMap.setZero();

  for (int q = 0; q < numQuadPts_; q++) {
    const double *mp = elasticParamsPtr(cacheDataBase, q);
    ES::M3d P;
    elasticModel_->compute_P(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), P.data());
    const M3xN localForce = P * cd->Bm[q];
    gradMap += Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>>(
      localForce.data(), localDofs_);
  }
}

void VolumetricDeformationModel::compute_d2E_dx2(
  const DeformationModelCacheData *cacheDataBase, double *hess) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD *cd = checkedCacheData(*this, cacheDataBase);

  Eigen::Map<Eigen::MatrixXd> hessMap(hess, localDofs_, localDofs_);
  hessMap.setZero();

  for (int q = 0; q < numQuadPts_; q++) {
    const double *mp = elasticParamsPtr(cacheDataBase, q);
    ES::M9d dPdF;
    elasticModel_->compute_dPdF(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), dPdF.data());
    dPdF *= elementMapping_.weightDetJ(q) * cd->detFp[q];
    hessMap.noalias() += cd->dFdx[q].transpose() * dPdF * cd->dFdx[q];
  }
}

void VolumetricDeformationModel::enableSPD(int enable)
{
  elasticModel_->enableSPD(enable);
}

// ============================================================
// computeSVD
// ============================================================

void VolumetricDeformationModel::computeSVD(
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
VolumetricDeformationModel::computeLocalMaxStepSize(
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

void VolumetricDeformationModel::computeF(
  const double *x, int materialLocationID, double F[9]) const
{
  elementMapping_.computeFref(x, materialLocationID, F);
}

void VolumetricDeformationModel::computeFe(
  const DeformationModelCacheData *cacheData, int materialLocationID, double F[9]) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD *cd = checkedCacheData(*this, cacheData);
  Eigen::Map<ES::M3d> FMap(F);
  FMap = cd->Fe[materialLocationID];
}

void VolumetricDeformationModel::computeP(
  const DeformationModelCacheData *cacheData, int materialLocationID, double POut[9]) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD *cd = checkedCacheData(*this, cacheData);
  const double *mp = elasticParamsPtr(cacheData, materialLocationID);

  ES::M3d P;
  elasticModel_->compute_P(mp, cd->Fe[materialLocationID].data(),
    cd->U[materialLocationID].data(), cd->V[materialLocationID].data(),
    cd->S[materialLocationID].data(), P.data());
  Eigen::Map<ES::M3d> PMap(POut);
  PMap = P;
}

void VolumetricDeformationModel::computedPdF(
  const DeformationModelCacheData *cacheData, int materialLocationID, double dPdFOut[81]) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD *cd = checkedCacheData(*this, cacheData);
  const double *mp = elasticParamsPtr(cacheData, materialLocationID);

  ES::M9d dPdF;
  elasticModel_->compute_dPdF(mp, cd->Fe[materialLocationID].data(),
    cd->U[materialLocationID].data(), cd->V[materialLocationID].data(),
    cd->S[materialLocationID].data(), dPdF.data());
  Eigen::Map<ES::M9d> dPdFMap(dPdFOut);
  dPdFMap = dPdF;
}

void VolumetricDeformationModel::computedFdx(
  const DeformationModelCacheData *cacheData, int materialLocationID, double *dFdxOut) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD *cd = checkedCacheData(*this, cacheData);
  Eigen::Map<M9xNDOF> dFdxMap(dFdxOut, 9, localDofs_);
  dFdxMap = cd->dFdx[materialLocationID];
}

void VolumetricDeformationModel::computeForceFromP(
  const DeformationModelCacheData *cacheDataBase, int materialLocationID,
  const double P[9], double f[]) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD *cd = checkedCacheData(*this, cacheDataBase);
  const Eigen::Map<const ES::M3d> PMap(P);
  const M3xN localForce = PMap * cd->Bm[materialLocationID];
  Eigen::Map<Eigen::VectorXd> fMap(f, localDofs_);
  fMap = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>>(
    localForce.data(), localDofs_);
}

// ============================================================
// vonMisesStress / maxStrain
// ============================================================

void VolumetricDeformationModel::vonMisesStress(
  const DeformationModelCacheData *cacheDataBase, int &nPt, double *stresses) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD *cd = checkedCacheData(*this, cacheDataBase);
  nPt = numQuadPts_;

  for (int q = 0; q < numQuadPts_; q++) {
    const double *mp = elasticParamsPtr(cacheDataBase, q);
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

void VolumetricDeformationModel::maxStrain(
  const DeformationModelCacheData *cacheDataBase, int &nPt, double *stresses) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD *cd = checkedCacheData(*this, cacheDataBase);
  nPt = numQuadPts_;

  for (int q = 0; q < numQuadPts_; q++) {
    ES::M3d E = 0.5 * (cd->Fe[q].transpose() * cd->Fe[q] - ES::M3d::Identity());
    Eigen::SelfAdjointEigenSolver<ES::M3d> eigSolver(E);
    stresses[q] = eigSolver.eigenvalues().maxCoeff();
  }
}

// ============================================================
// Plastic material-parameter derivatives.
// ============================================================

void VolumetricDeformationModel::compute_dE_da(
  const DeformationModelCacheData *cacheDataBase, double *grad,
  int materialLocation) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD *cd = checkedCacheData(*this, cacheDataBase);

  if (numPlasticParams_ == 0) return;

  Eigen::Map<ES::VXd> gradMap(grad, numPlasticParams_);
  gradMap.setZero();
  const auto [qBegin, qEnd] =
    materialLocationRange(materialLocation, numQuadPts_);
  for (int q = qBegin; q < qEnd; q++) {
    const double *mp = elasticParamsPtr(cacheDataBase, q);
    double psi = elasticModel_->compute_psi(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data());

    ES::M3d P;
    elasticModel_->compute_P(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), P.data());

    for (int i = 0; i < numPlasticParams_; i++) {
      double dVda = compute_dV_dai(elementMapping_.weightDetJ(q), cd->ddetA_da[q][i]);
      double dpsi_da = compute_dpsi_dai(cd->Fref[q], cd->dAInv_dai[q][i], P);
      gradMap[i] += dVda * psi + elementMapping_.weightDetJ(q) * cd->detFp[q] * dpsi_da;
    }
  }
}

void VolumetricDeformationModel::compute_d2E_da2(
  const DeformationModelCacheData *cacheDataBase, double *hess,
  int materialLocation) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD *cd = checkedCacheData(*this, cacheDataBase);

  if (numPlasticParams_ == 0) return;

  Eigen::Map<ES::MXd> hessMap(hess, numPlasticParams_, numPlasticParams_);
  hessMap.setZero();
  const auto [qBegin, qEnd] =
    materialLocationRange(materialLocation, numQuadPts_);
  for (int q = qBegin; q < qEnd; q++) {
    const double *mp = elasticParamsPtr(cacheDataBase, q);
    const double vol = elementMapping_.weightDetJ(q) * cd->detFp[q];
    double psi = elasticModel_->compute_psi(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data());

    ES::M3d P;
    elasticModel_->compute_P(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), P.data());

    ES::M9d dPdF;
    elasticModel_->compute_dPdF(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), dPdF.data());

    for (int i = 0; i < numPlasticParams_; i++) {
      double dVda_i = compute_dV_dai(elementMapping_.weightDetJ(q), cd->ddetA_da[q][i]);
      double dpsi_da_i = compute_dpsi_dai(cd->Fref[q], cd->dAInv_dai[q][i], P);

      for (int j = 0; j < numPlasticParams_; j++) {
        double dVda_j = compute_dV_dai(elementMapping_.weightDetJ(q), cd->ddetA_da[q][j]);
        double d2V = compute_d2V_daidaj(elementMapping_.weightDetJ(q), cd->d2detA_da2[q](i, j));
        double dpsi_da_j = compute_dpsi_dai(cd->Fref[q], cd->dAInv_dai[q][j], P);
        double d2psi = compute_d2psi_dai_daj(cd->Fref[q],
          cd->dAInv_dai[q][i], cd->dAInv_dai[q][j],
          cd->d2AInv(q, i, j), P, dPdF);
        hessMap(i, j) += d2V * psi + dVda_i * dpsi_da_j +
                         dVda_j * dpsi_da_i + vol * d2psi;
      }
    }
  }
}

void VolumetricDeformationModel::compute_d2E_dxda(
  const DeformationModelCacheData *cacheDataBase, double *hess,
  int materialLocation) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD *cd = checkedCacheData(*this, cacheDataBase);

  if (numPlasticParams_ == 0) return;

  Eigen::Map<ES::MXd> mixed(hess, localDofs_, numPlasticParams_);
  mixed.setZero();
  const auto [qBegin, qEnd] =
    materialLocationRange(materialLocation, numQuadPts_);
  for (int q = qBegin; q < qEnd; q++) {
    const double *mp = elasticParamsPtr(cacheDataBase, q);
    ES::M3d P;
    elasticModel_->compute_P(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), P.data());

    cd->dpsiDxScratch.noalias() = cd->dFdx[q].transpose() *
      Eigen::Map<const ES::V9d>(P.data());

    ES::M9d dPdF;
    elasticModel_->compute_dPdF(mp, cd->Fe[q].data(),
      cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), dPdF.data());

    const double vol = elementMapping_.weightDetJ(q) * cd->detFp[q];
    for (int i = 0; i < numPlasticParams_; i++) {
      const double dVda = compute_dV_dai(elementMapping_.weightDetJ(q), cd->ddetA_da[q][i]);

      ES::M3d dFda;
      compute_dFe_dai(cd->Fref[q], cd->dAInv_dai[q][i], dFda);

      ES::M3d dPda;
      compute_dP_dai(dPdF, dFda, dPda);

      cd->localDofScratch.noalias() = cd->dFdx[q].transpose() *
        Eigen::Map<const ES::V9d>(dPda.data());

      compute_d2Fe_dx_dai(cd->dAInv_dai[q][i], elementMapping_.rest_dFdx(q), cd->d2FdxdaScratch);

      mixed.col(i) += dVda * cd->dpsiDxScratch + vol *
        (cd->localDofScratch + cd->d2FdxdaScratch.transpose() * Eigen::Map<const ES::V9d>(P.data()));
    }
  }
}

// ============================================================
// Elastic material-parameter derivatives.
// ============================================================

void VolumetricDeformationModel::compute_dE_db(
  const DeformationModelCacheData *cacheDataBase, double *grad,
  int materialLocation) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD *cd = checkedCacheData(*this, cacheDataBase);

  if (numElasticParams_ == 0) return;

  Eigen::Map<ES::VXd> gradMap(grad, numElasticParams_);
  gradMap.setZero();
  const auto [qBegin, qEnd] =
    materialLocationRange(materialLocation, numQuadPts_);
  for (int q = qBegin; q < qEnd; q++) {
    const double *mp = elasticParamsPtr(cacheDataBase, q);
    const double vol = elementMapping_.weightDetJ(q) * cd->detFp[q];
    for (int i = 0; i < numElasticParams_; i++) {
      gradMap[i] += vol * elasticModel_->compute_dpsi_dparam(mp, i,
        cd->Fe[q].data(), cd->U[q].data(), cd->V[q].data(), cd->S[q].data());
    }
  }
}

void VolumetricDeformationModel::compute_d2E_db2(
  const DeformationModelCacheData *cacheDataBase, double *hess,
  int materialLocation) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD *cd = checkedCacheData(*this, cacheDataBase);

  if (numElasticParams_ == 0) return;

  Eigen::Map<ES::MXd> hessMap(hess, numElasticParams_, numElasticParams_);
  hessMap.setZero();
  const auto [qBegin, qEnd] =
    materialLocationRange(materialLocation, numQuadPts_);
  for (int q = qBegin; q < qEnd; q++) {
    const double *mp = elasticParamsPtr(cacheDataBase, q);
    const double vol = elementMapping_.weightDetJ(q) * cd->detFp[q];
    for (int i = 0; i < numElasticParams_; i++) {
      for (int j = 0; j < numElasticParams_; j++) {
        hessMap(i, j) += vol * elasticModel_->compute_d2psi_dparam2(mp, i, j,
          cd->Fe[q].data(), cd->U[q].data(), cd->V[q].data(), cd->S[q].data());
      }
    }
  }
}

void VolumetricDeformationModel::compute_d2E_dxdb(
  const DeformationModelCacheData *cacheDataBase, double *hess,
  int materialLocation) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD *cd = checkedCacheData(*this, cacheDataBase);

  if (numElasticParams_ == 0) return;

  Eigen::Map<ES::MXd> mixed(hess, localDofs_, numElasticParams_);
  mixed.setZero();
  const auto [qBegin, qEnd] =
    materialLocationRange(materialLocation, numQuadPts_);
  for (int q = qBegin; q < qEnd; q++) {
    const double *mp = elasticParamsPtr(cacheDataBase, q);
    const double vol = elementMapping_.weightDetJ(q) * cd->detFp[q];
    for (int i = 0; i < numElasticParams_; i++) {
      ES::M3d dPdb;
      elasticModel_->compute_dP_dparam(mp, i, cd->Fe[q].data(),
        cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), dPdb.data());
      mixed.col(i) += vol * (cd->dFdx[q].transpose() *
        Eigen::Map<const ES::V9d>(dPdb.data()));
    }
  }
}

void VolumetricDeformationModel::compute_d2E_dadb(
  const DeformationModelCacheData *cacheDataBase, double *hess,
  int materialLocation) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD *cd = checkedCacheData(*this, cacheDataBase);

  if (numElasticParams_ == 0 || numPlasticParams_ == 0) return;

  Eigen::Map<ES::MXd> mixed(hess, numPlasticParams_, numElasticParams_);
  mixed.setZero();

  const auto [qBegin, qEnd] =
    materialLocationRange(materialLocation, numQuadPts_);
  for (int q = qBegin; q < qEnd; q++) {
    const double *mp = elasticParamsPtr(cacheDataBase, q);
    const double vol = elementMapping_.weightDetJ(q) * cd->detFp[q];

    for (int i = 0; i < numElasticParams_; i++) {
      elasticModel_->compute_dP_dparam(mp, i, cd->Fe[q].data(),
        cd->U[q].data(), cd->V[q].data(), cd->S[q].data(), cd->dPdbScratch[i].data());
    }

    for (int i = 0; i < numPlasticParams_; i++) {
      compute_dFe_dai(cd->Fref[q], cd->dAInv_dai[q][i], cd->dFdaScratch[i]);
      double dVda = compute_dV_dai(elementMapping_.weightDetJ(q), cd->ddetA_da[q][i]);
      for (int j = 0; j < numElasticParams_; j++) {
        double dpsi_db = elasticModel_->compute_dpsi_dparam(mp, j,
          cd->Fe[q].data(), cd->U[q].data(), cd->V[q].data(), cd->S[q].data());
        mixed(i, j) += dVda * dpsi_db;
      }
    }

    for (int i = 0; i < numPlasticParams_; i++) {
      for (int j = 0; j < numElasticParams_; j++) {
        mixed(i, j) += vol * Eigen::Map<const ES::V9d>(cd->dPdbScratch[j].data())
          .dot(Eigen::Map<const ES::V9d>(cd->dFdaScratch[i].data()));
      }
    }
  }

}

// ============================================================
// Private helper methods
// ============================================================

void VolumetricDeformationModel::computeCurrent_dFdx(
  const M9xNDOF &rest_dFdx, const ES::M3d &FpInv, M9xNDOF &dFdx) const
{
  for (int col = 0; col < localDofs_; col++) {
    const Eigen::Map<const ES::M3d> dFref(rest_dFdx.col(col).data());
    const ES::M3d dFe = dFref * FpInv;
    dFdx.col(col) = Eigen::Map<const ES::V9d>(dFe.data());
  }
}

double VolumetricDeformationModel::compute_dV_dai(
  double weightDetJ, double ddetA_dai) const
{
  return weightDetJ * ddetA_dai;
}

double VolumetricDeformationModel::compute_d2V_daidaj(
  double weightDetJ, double d2detA_daidaj) const
{
  return weightDetJ * d2detA_daidaj;
}

void VolumetricDeformationModel::compute_dFe_dai(
  const ES::M3d &Fref, const ES::M3d &dAInvdai, ES::M3d &dFdai) const
{
  dFdai = Fref * dAInvdai;
}

void VolumetricDeformationModel::compute_d2Fe_dai_daj(
  const ES::M3d &Fref, const ES::M3d &dAInvdaidaj, ES::M3d &d2Fdaidaj) const
{
  d2Fdaidaj = Fref * dAInvdaidaj;
}

void VolumetricDeformationModel::compute_dP_dai(
  const ES::M9d &dPdF, const ES::M3d &dFdai, ES::M3d &dPdai) const
{
  Eigen::Map<ES::V9d>(dPdai.data()) = dPdF *
    Eigen::Map<const ES::V9d>(dFdai.data());
}

double VolumetricDeformationModel::compute_dpsi_dai(
  const ES::M3d &Fref, const ES::M3d &dAInv_dai, const ES::M3d &P) const
{
  ES::M3d dFe_dai;
  compute_dFe_dai(Fref, dAInv_dai, dFe_dai);
  return P.cwiseProduct(dFe_dai).sum();
}

double VolumetricDeformationModel::compute_d2psi_dai_daj(
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

void VolumetricDeformationModel::compute_d2Fe_dx_dai(
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
