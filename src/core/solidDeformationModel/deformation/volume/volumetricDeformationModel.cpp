#include "volumetricDeformationModel.h"
#include "volumetricDeformationModelEvaluator.h"

#include "deformation/materialMaxStepPolynomialUtils.h"
#include <stdexcept>
#include <span>
#include <utility>

namespace ES = pgo::EigenSupport;

namespace pgo
{
namespace SolidDeformationModel
{
namespace
{
std::pair<int, int> materialLocationRange(int materialLocation, int count)
{
  if (materialLocation < 0)
    return { 0, count };
  if (materialLocation >= count)
    throw std::out_of_range("Volumetric material location is out of range.");
  return { materialLocation, materialLocation + 1 };
}
}  // namespace

std::unique_ptr<DeformationModelEvaluator> VolumetricDeformationModel::createEvaluator() const
{
  return std::make_unique<VolumetricDeformationModelEvaluator>(*this);
}

// ============================================================
// Constructors
// ============================================================

VolumetricDeformationModel::VolumetricDeformationModel(
  VolumetricElementMapping &&mapping,
  std::unique_ptr<ElasticModel3DDeformationGradient> elasticModel,
  std::unique_ptr<PlasticModel3DDeformationGradient> plasticModel,
  DeformationModelConstructionOptions options):
  DeformationModel(),
  numNodes_(mapping.numNodes()),
  numQuadPts_(mapping.numQuadraturePoints()),
  localDofs_(mapping.localDofs()),
  elementMapping_(std::move(mapping)),
  elasticModel_(std::move(elasticModel)),
  plasticModel_(std::move(plasticModel)),
  projectHessianPSD_(options.projectHessianPSD)
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

void VolumetricDeformationModel::defaultPlasticParams(std::span<double> params) const
{
  plasticModel_->defaultParams(params);
}

// ============================================================
// elastic parameter access
// ============================================================

std::span<const double> VolumetricDeformationModel::elasticParams(
  const VolumetricDeformationModelCacheData &cacheData, int q) const
{
  if (cacheData.numElasticParams == 0)
    return {};
  return std::span<const double>(cacheData.elasticParamsValue[q].data(),
    static_cast<std::size_t>(cacheData.numElasticParams));
}

// ============================================================
// prepareData
// ============================================================

void VolumetricDeformationModel::prepareData(
  std::span<const double> x, std::span<const double> elasticParams, std::span<const double> plasticParams,
  VolumetricDeformationModelCacheData &cacheDataBase) const
{
  if (x.size() != static_cast<std::size_t>(localDofs_))
    throw std::invalid_argument(
      "Volumetric deformation local-position buffer has the wrong size.");
  if (elasticParams.size() != static_cast<std::size_t>(numQuadPts_ * numElasticParams_))
    throw std::invalid_argument(
      "Elastic parameters are required by this volumetric deformation model.");
  if (plasticParams.size() != static_cast<std::size_t>(numQuadPts_ * numPlasticParams_))
    throw std::invalid_argument(
      "Plastic parameters are required by this volumetric deformation model.");

  VolumetricDeformationModelCacheData &cd = cacheDataBase;

  for (int vi = 0; vi < numNodes_; vi++) {
    cd.x.col(vi) = ES::V3d(x[vi * 3 + 0], x[vi * 3 + 1], x[vi * 3 + 2]);
  }

  for (int q = 0; q < numQuadPts_; q++) {
    if (numPlasticParams_ > 0) {
      cd.plasticParamsValue[q] =
        Eigen::Map<const ES::VXd>(
          plasticParams.data() + static_cast<std::ptrdiff_t>(q) * numPlasticParams_,
          numPlasticParams_);
    }

    const std::span<const double> plasticParamsAtQ = numPlasticParams_ > 0 ?
      std::span<const double>(cd.plasticParamsValue[q].data(), numPlasticParams_) :
      std::span<const double>{};
    cd.Fp[q] = plasticModel_->computeA(plasticParamsAtQ);
    cd.FpInv[q] = plasticModel_->computeAInv(plasticParamsAtQ);
    cd.detFp[q] = plasticModel_->compute_detA(plasticParamsAtQ);

    if (numPlasticParams_ > 0) {
      plasticModel_->compute_ddetA_da(plasticParamsAtQ, cd.ddetA_da[q]);
      plasticModel_->compute_d2detA_da2(plasticParamsAtQ, cd.d2detA_da2[q]);

      for (int i = 0; i < numPlasticParams_; i++) {
        cd.dAInv_dai[q][i] = plasticModel_->compute_dAInv_da(plasticParamsAtQ, i);
        for (int j = 0; j < numPlasticParams_; j++) {
          cd.d2AInv(q, i, j) =
            plasticModel_->compute_d2AInv_da2(plasticParamsAtQ, i, j);
        }
      }
    }

    if (numElasticParams_ > 0) {
      cd.elasticParamsValue[q] =
        Eigen::Map<const ES::VXd>(
          elasticParams.data() + static_cast<std::ptrdiff_t>(q) * numElasticParams_,
          numElasticParams_);
    }

    cd.Fref[q] = elementMapping_.compute_F_ref(x, q);
    cd.spectralState[q] = computeSpectralState(cd.Fref[q] * cd.FpInv[q]);
    computeCurrent_dF_dx(elementMapping_.rest_dF_dx(q), cd.FpInv[q], cd.dFdx[q]);
    cd.Bm[q] = cd.detFp[q] * cd.FpInv[q].transpose() * elementMapping_.restBm(q);
  }
}

// ============================================================
// Energy and derivatives w.r.t. displacement
// ============================================================

double VolumetricDeformationModel::compute_E(
  const VolumetricDeformationModelCacheData &cacheDataBase) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD &cd = cacheDataBase;

  double energy = 0.0;
  for (int q = 0; q < numQuadPts_; q++) {
    const std::span<const double> mp = elasticParams(cacheDataBase, q);
    energy += elasticModel_->compute_psi(mp, cd.spectralState[q]) *
      elementMapping_.weightDetJ(q) * cd.detFp[q];
  }
  return energy;
}

void VolumetricDeformationModel::compute_dE_dx(
  const VolumetricDeformationModelCacheData &cacheDataBase, ES::RefVecXd grad) const
{
  if (grad.size() != localDofs_)
    throw std::invalid_argument("Volumetric deformation gradient has unexpected size.");
  using CD = VolumetricDeformationModelCacheData;
  const CD &cd = cacheDataBase;

  Eigen::Map<Eigen::VectorXd> gradMap(grad.data(), localDofs_);
  gradMap.setZero();

  for (int q = 0; q < numQuadPts_; q++) {
    const std::span<const double> mp = elasticParams(cacheDataBase, q);
    const ES::M3d P = elasticModel_->compute_P(mp, cd.spectralState[q]);
    const M3xN localForce = P * cd.Bm[q];
    gradMap += Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>>(
      localForce.data(), localDofs_);
  }
}

void VolumetricDeformationModel::compute_d2E_dx2(
  const VolumetricDeformationModelCacheData &cacheDataBase, ES::RefMatXd hess) const
{
  if (hess.rows() != localDofs_ || hess.cols() != localDofs_)
    throw std::invalid_argument("Volumetric deformation Hessian has unexpected shape.");
  using CD = VolumetricDeformationModelCacheData;
  const CD &cd = cacheDataBase;

  Eigen::Map<Eigen::MatrixXd> hessMap(hess.data(), localDofs_, localDofs_);
  hessMap.setZero();

  for (int q = 0; q < numQuadPts_; q++) {
    const std::span<const double> mp = elasticParams(cacheDataBase, q);
    ES::M9d dPdF;
    if (projectHessianPSD_) {
      dPdF = elasticModel_->compute_dPdF_psd(mp, cd.spectralState[q]);
    }
    else {
      dPdF = elasticModel_->compute_dPdF(mp, cd.spectralState[q]);
    }
    dPdF *= elementMapping_.weightDetJ(q) * cd.detFp[q];
    hessMap.noalias() += cd.dFdx[q].transpose() * dPdF * cd.dFdx[q];
  }
}

// ============================================================
// computeSpectralState
// ============================================================

SpectralState VolumetricDeformationModel::computeSpectralState(
  const ES::M3d &Fe)
{
  Eigen::JacobiSVD<ES::M3d, Eigen::NoQRPreconditioner> svd(
    Fe, Eigen::ComputeFullU | Eigen::ComputeFullV);
  SpectralState state;
  state.F = Fe;
  state.U = svd.matrixU();
  state.V = svd.matrixV();
  state.stretches = svd.singularValues();

  if (state.U.determinant() < 0.0) {
    state.U.col(2) *= -1.0;
    state.stretches(2) *= -1.0;
  }
  if (state.V.determinant() < 0.0) {
    state.V.col(2) *= -1.0;
    state.stretches(2) *= -1.0;
  }
  return state;
}

// ============================================================
// computeLocalMaxStepSize
// ============================================================

DeformationModel::LocalMaxStepResult
VolumetricDeformationModel::computeLocalMaxStepSize(
  std::span<const double> x_local, std::span<const double> dx_local) const
{
  if (x_local.size() != static_cast<std::size_t>(localDofs_) ||
    dx_local.size() != static_cast<std::size_t>(localDofs_))
    throw std::invalid_argument(
      "Volumetric local max-step inputs have the wrong size.");
  LocalMaxStepResult result;
  for (int q = 0; q < numQuadPts_; q++) {
    const ES::M3d F0 = compute_F(x_local, q);
    const ES::M3d deltaF = compute_F(dx_local, q);

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
// compute_F / compute_Fe / compute_P / compute_dP_dF / compute_dF_dx / computeForceFromP
// ============================================================

ES::M3d VolumetricDeformationModel::compute_F(
  std::span<const double> x, int materialLocationID) const
{
  if (x.size() != static_cast<std::size_t>(localDofs_))
    throw std::invalid_argument(
      "Volumetric deformation local-position buffer has the wrong size.");
  return elementMapping_.compute_F_ref(x, materialLocationID);
}

ES::M3d VolumetricDeformationModel::compute_Fe(
  const VolumetricDeformationModelCacheData &cacheData, int materialLocationID) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD &cd = cacheData;
  return cd.spectralState[materialLocationID].F;
}

ES::M3d VolumetricDeformationModel::compute_P(
  const VolumetricDeformationModelCacheData &cacheData, int materialLocationID) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD &cd = cacheData;
  const std::span<const double> mp = elasticParams(cacheData, materialLocationID);

  return elasticModel_->compute_P(mp, cd.spectralState[materialLocationID]);
}

ES::M9d VolumetricDeformationModel::compute_dP_dF(
  const VolumetricDeformationModelCacheData &cacheData, int materialLocationID) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD &cd = cacheData;
  const std::span<const double> mp = elasticParams(cacheData, materialLocationID);

  return elasticModel_->compute_dPdF(mp, cd.spectralState[materialLocationID]);
}

void VolumetricDeformationModel::compute_dF_dx(
  const VolumetricDeformationModelCacheData &cacheData, int materialLocationID, ES::RefMatXd dFdxOut) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD &cd = cacheData;
  if (dFdxOut.rows() != 9 || dFdxOut.cols() != localDofs_)
    throw std::invalid_argument("Volumetric dFdx output has unexpected size.");
  dFdxOut = cd.dFdx[materialLocationID];
}

void VolumetricDeformationModel::computeForceFromP(
  const VolumetricDeformationModelCacheData &cacheDataBase, int materialLocationID,
  const ES::M3d &P, ES::RefVecXd f) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD &cd = cacheDataBase;
  const M3xN localForce = P * cd.Bm[materialLocationID];
  if (f.size() != localDofs_)
    throw std::invalid_argument("Volumetric force output has unexpected size.");
  f = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>>(
    localForce.data(), localDofs_);
}

// ============================================================
// Von Mises stress / maximum strain
// ============================================================

int VolumetricDeformationModel::computeVonMisesStress(
  const VolumetricDeformationModelCacheData &cacheDataBase,
  std::span<double> stresses, int capacity) const
{
  if (capacity < numQuadPts_)
    throw std::length_error(
      "Volumetric von Mises stress output capacity is too small.");
  if (stresses.size() < static_cast<std::size_t>(numQuadPts_))
    throw std::invalid_argument(
      "Volumetric von Mises stress output must not be null.");

  using CD = VolumetricDeformationModelCacheData;
  const CD &cd = cacheDataBase;

  for (int q = 0; q < numQuadPts_; q++) {
    const std::span<const double> mp = elasticParams(cacheDataBase, q);
    const ES::M3d P = elasticModel_->compute_P(mp, cd.spectralState[q]);
    const ES::M3d &Fe = cd.spectralState[q].F;
    const double detF = Fe.determinant();
    ES::M3d cauchyStress = P * Fe.transpose() / detF;

    const double t1 = std::pow(cauchyStress(0, 0) - cauchyStress(1, 1), 2.0);
    const double t2 = std::pow(cauchyStress(1, 1) - cauchyStress(2, 2), 2.0);
    const double t3 = std::pow(cauchyStress(2, 2) - cauchyStress(0, 0), 2.0);
    const double t4 = 6.0 * (std::pow(cauchyStress(1, 2), 2.0) + std::pow(cauchyStress(2, 0), 2.0) + std::pow(cauchyStress(0, 1), 2.0));
    stresses[q] = std::sqrt((t1 + t2 + t3 + t4) * 0.5);
  }

  return numQuadPts_;
}

int VolumetricDeformationModel::computeMaxStrain(
  const VolumetricDeformationModelCacheData &cacheDataBase,
  std::span<double> strains, int capacity) const
{
  if (capacity < numQuadPts_)
    throw std::length_error(
      "Volumetric maximum strain output capacity is too small.");
  if (strains.size() < static_cast<std::size_t>(numQuadPts_))
    throw std::invalid_argument(
      "Volumetric maximum strain output must not be null.");

  using CD = VolumetricDeformationModelCacheData;
  const CD &cd = cacheDataBase;

  for (int q = 0; q < numQuadPts_; q++) {
    const ES::M3d &Fe = cd.spectralState[q].F;
    ES::M3d E = 0.5 * (Fe.transpose() * Fe - ES::M3d::Identity());
    Eigen::SelfAdjointEigenSolver<ES::M3d> eigSolver(E);
    strains[q] = eigSolver.eigenvalues().maxCoeff();
  }

  return numQuadPts_;
}

// ============================================================
// Plastic material-parameter derivatives.
// ============================================================

void VolumetricDeformationModel::compute_dE_dp(
  const VolumetricDeformationModelCacheData &cacheDataBase, ES::RefVecXd grad,
  int materialLocation) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD &cd = cacheDataBase;

  if (numPlasticParams_ == 0)
    return;

  if (grad.size() != static_cast<std::size_t>(numPlasticParams_))
    throw std::invalid_argument("Volumetric plastic gradient has unexpected size.");
  Eigen::Map<ES::VXd> gradMap(grad.data(), numPlasticParams_);
  gradMap.setZero();
  const auto [qBegin, qEnd] =
    materialLocationRange(materialLocation, numQuadPts_);
  for (int q = qBegin; q < qEnd; q++) {
    const std::span<const double> mp = elasticParams(cacheDataBase, q);
    double psi = elasticModel_->compute_psi(mp, cd.spectralState[q]);

    const ES::M3d P = elasticModel_->compute_P(mp, cd.spectralState[q]);

    for (int i = 0; i < numPlasticParams_; i++) {
      double dVda = compute_dV_dai(elementMapping_.weightDetJ(q), cd.ddetA_da[q][i]);
      double dpsi_da = compute_dpsi_dai(cd.Fref[q], cd.dAInv_dai[q][i], P);
      gradMap[i] += dVda * psi + elementMapping_.weightDetJ(q) * cd.detFp[q] * dpsi_da;
    }
  }
}

void VolumetricDeformationModel::compute_d2E_dp2(
  const VolumetricDeformationModelCacheData &cacheDataBase, ES::RefMatXd hess,
  int materialLocation) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD &cd = cacheDataBase;

  if (numPlasticParams_ == 0)
    return;

  if (hess.size() != static_cast<std::size_t>(numPlasticParams_ * numPlasticParams_))
    throw std::invalid_argument("Volumetric plastic Hessian has unexpected size.");
  Eigen::Map<ES::MXd> hessMap(hess.data(), numPlasticParams_, numPlasticParams_);
  hessMap.setZero();
  const auto [qBegin, qEnd] =
    materialLocationRange(materialLocation, numQuadPts_);
  for (int q = qBegin; q < qEnd; q++) {
    const std::span<const double> mp = elasticParams(cacheDataBase, q);
    const double vol = elementMapping_.weightDetJ(q) * cd.detFp[q];
    double psi = elasticModel_->compute_psi(mp, cd.spectralState[q]);

    const ES::M3d P = elasticModel_->compute_P(mp, cd.spectralState[q]);

    const ES::M9d dPdF = elasticModel_->compute_dPdF(mp, cd.spectralState[q]);

    for (int i = 0; i < numPlasticParams_; i++) {
      double dVda_i = compute_dV_dai(elementMapping_.weightDetJ(q), cd.ddetA_da[q][i]);
      double dpsi_da_i = compute_dpsi_dai(cd.Fref[q], cd.dAInv_dai[q][i], P);

      for (int j = 0; j < numPlasticParams_; j++) {
        double dVda_j = compute_dV_dai(elementMapping_.weightDetJ(q), cd.ddetA_da[q][j]);
        double d2V = compute_d2V_daidaj(elementMapping_.weightDetJ(q), cd.d2detA_da2[q](i, j));
        double dpsi_da_j = compute_dpsi_dai(cd.Fref[q], cd.dAInv_dai[q][j], P);
        double d2psi = compute_d2psi_dai_daj(cd.Fref[q],
          cd.dAInv_dai[q][i], cd.dAInv_dai[q][j],
          cd.d2AInv(q, i, j), P, dPdF);
        hessMap(i, j) += d2V * psi + dVda_i * dpsi_da_j +
          dVda_j * dpsi_da_i + vol * d2psi;
      }
    }
  }
}

void VolumetricDeformationModel::compute_d2E_dudp(
  const VolumetricDeformationModelCacheData &cacheDataBase, ES::RefMatXd hess,
  int materialLocation) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD &cd = cacheDataBase;

  if (numPlasticParams_ == 0)
    return;

  if (hess.size() != static_cast<std::size_t>(localDofs_ * numPlasticParams_))
    throw std::invalid_argument("Volumetric displacement-plastic Hessian has unexpected size.");
  Eigen::Map<ES::MXd> mixed(hess.data(), localDofs_, numPlasticParams_);
  mixed.setZero();
  const auto [qBegin, qEnd] =
    materialLocationRange(materialLocation, numQuadPts_);
  for (int q = qBegin; q < qEnd; q++) {
    const std::span<const double> mp = elasticParams(cacheDataBase, q);
    const ES::M3d P = elasticModel_->compute_P(mp, cd.spectralState[q]);

    cd.dpsiDxScratch.noalias() = cd.dFdx[q].transpose() *
      Eigen::Map<const ES::V9d>(P.data());

    const ES::M9d dPdF = elasticModel_->compute_dPdF(mp, cd.spectralState[q]);

    const double vol = elementMapping_.weightDetJ(q) * cd.detFp[q];
    for (int i = 0; i < numPlasticParams_; i++) {
      const double dVda = compute_dV_dai(elementMapping_.weightDetJ(q), cd.ddetA_da[q][i]);

      const ES::M3d dFda = compute_dFe_dai(
        cd.Fref[q], cd.dAInv_dai[q][i]);
      const ES::M3d dPda = compute_dP_dai(dPdF, dFda);

      cd.localDofScratch.noalias() = cd.dFdx[q].transpose() *
        Eigen::Map<const ES::V9d>(dPda.data());

      compute_d2Fe_dx_dai(cd.dAInv_dai[q][i], elementMapping_.rest_dF_dx(q), cd.d2FdxdaScratch);

      mixed.col(i) += dVda * cd.dpsiDxScratch + vol * (cd.localDofScratch + cd.d2FdxdaScratch.transpose() * Eigen::Map<const ES::V9d>(P.data()));
    }
  }
}

// ============================================================
// Elastic material-parameter derivatives.
// ============================================================

void VolumetricDeformationModel::compute_dE_de(
  const VolumetricDeformationModelCacheData &cacheDataBase, ES::RefVecXd grad,
  int materialLocation) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD &cd = cacheDataBase;

  if (numElasticParams_ == 0)
    return;

  if (grad.size() != static_cast<std::size_t>(numElasticParams_))
    throw std::invalid_argument("Volumetric elastic gradient has unexpected size.");
  Eigen::Map<ES::VXd> gradMap(grad.data(), numElasticParams_);
  gradMap.setZero();
  const auto [qBegin, qEnd] =
    materialLocationRange(materialLocation, numQuadPts_);
  for (int q = qBegin; q < qEnd; q++) {
    const std::span<const double> mp = elasticParams(cacheDataBase, q);
    const double vol = elementMapping_.weightDetJ(q) * cd.detFp[q];
    for (int i = 0; i < numElasticParams_; i++) {
      gradMap[i] += vol * elasticModel_->compute_dpsi_dparam(mp, i, cd.spectralState[q]);
    }
  }
}

void VolumetricDeformationModel::compute_d2E_de2(
  const VolumetricDeformationModelCacheData &cacheDataBase, ES::RefMatXd hess,
  int materialLocation) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD &cd = cacheDataBase;

  if (numElasticParams_ == 0)
    return;

  if (hess.size() != static_cast<std::size_t>(numElasticParams_ * numElasticParams_))
    throw std::invalid_argument("Volumetric elastic Hessian has unexpected size.");
  Eigen::Map<ES::MXd> hessMap(hess.data(), numElasticParams_, numElasticParams_);
  hessMap.setZero();
  const auto [qBegin, qEnd] =
    materialLocationRange(materialLocation, numQuadPts_);
  for (int q = qBegin; q < qEnd; q++) {
    const std::span<const double> mp = elasticParams(cacheDataBase, q);
    const double vol = elementMapping_.weightDetJ(q) * cd.detFp[q];
    for (int i = 0; i < numElasticParams_; i++) {
      for (int j = 0; j < numElasticParams_; j++) {
        hessMap(i, j) += vol * elasticModel_->compute_d2psi_dparam2(mp, i, j, cd.spectralState[q]);
      }
    }
  }
}

void VolumetricDeformationModel::compute_d2E_dude(
  const VolumetricDeformationModelCacheData &cacheDataBase, ES::RefMatXd hess,
  int materialLocation) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD &cd = cacheDataBase;

  if (numElasticParams_ == 0)
    return;

  if (hess.size() != static_cast<std::size_t>(localDofs_ * numElasticParams_))
    throw std::invalid_argument("Volumetric displacement-elastic Hessian has unexpected size.");
  Eigen::Map<ES::MXd> mixed(hess.data(), localDofs_, numElasticParams_);
  mixed.setZero();
  const auto [qBegin, qEnd] =
    materialLocationRange(materialLocation, numQuadPts_);
  for (int q = qBegin; q < qEnd; q++) {
    const std::span<const double> mp = elasticParams(cacheDataBase, q);
    const double vol = elementMapping_.weightDetJ(q) * cd.detFp[q];
    for (int i = 0; i < numElasticParams_; i++) {
      ES::M3d dPdb;
      dPdb = elasticModel_->compute_dP_dparam(mp, i, cd.spectralState[q]);
      mixed.col(i) += vol * (cd.dFdx[q].transpose() * Eigen::Map<const ES::V9d>(dPdb.data()));
    }
  }
}

void VolumetricDeformationModel::compute_d2E_dpde(
  const VolumetricDeformationModelCacheData &cacheDataBase, ES::RefMatXd hess,
  int materialLocation) const
{
  using CD = VolumetricDeformationModelCacheData;
  const CD &cd = cacheDataBase;

  if (numElasticParams_ == 0 || numPlasticParams_ == 0)
    return;

  if (hess.size() != static_cast<std::size_t>(numPlasticParams_ * numElasticParams_))
    throw std::invalid_argument("Volumetric plastic-elastic Hessian has unexpected size.");
  Eigen::Map<ES::MXd> mixed(hess.data(), numPlasticParams_, numElasticParams_);
  mixed.setZero();

  const auto [qBegin, qEnd] =
    materialLocationRange(materialLocation, numQuadPts_);
  for (int q = qBegin; q < qEnd; q++) {
    const std::span<const double> mp = elasticParams(cacheDataBase, q);
    const double vol = elementMapping_.weightDetJ(q) * cd.detFp[q];

    for (int i = 0; i < numElasticParams_; i++) {
      cd.dPdbScratch[i] = elasticModel_->compute_dP_dparam(mp, i, cd.spectralState[q]);
    }

    for (int i = 0; i < numPlasticParams_; i++) {
      cd.dFdaScratch[i] = compute_dFe_dai(
        cd.Fref[q], cd.dAInv_dai[q][i]);
      double dVda = compute_dV_dai(elementMapping_.weightDetJ(q), cd.ddetA_da[q][i]);
      for (int j = 0; j < numElasticParams_; j++) {
        double dpsi_db = elasticModel_->compute_dpsi_dparam(mp, j,
          cd.spectralState[q]);
        mixed(i, j) += dVda * dpsi_db;
      }
    }

    for (int i = 0; i < numPlasticParams_; i++) {
      for (int j = 0; j < numElasticParams_; j++) {
        mixed(i, j) += vol * Eigen::Map<const ES::V9d>(cd.dPdbScratch[j].data()).dot(Eigen::Map<const ES::V9d>(cd.dFdaScratch[i].data()));
      }
    }
  }
}

// ============================================================
// Private helper methods
// ============================================================

void VolumetricDeformationModel::computeCurrent_dF_dx(
  const M9xNDOF &rest_dF_dx, const ES::M3d &FpInv, M9xNDOF &dFdx) const
{
  for (int col = 0; col < localDofs_; col++) {
    const Eigen::Map<const ES::M3d> dFref(rest_dF_dx.col(col).data());
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

ES::M3d VolumetricDeformationModel::compute_dFe_dai(
  const ES::M3d &Fref, const ES::M3d &dAInvdai) const
{
  return Fref * dAInvdai;
}

ES::M3d VolumetricDeformationModel::compute_d2Fe_dai_daj(
  const ES::M3d &Fref, const ES::M3d &dAInvdaidaj) const
{
  return Fref * dAInvdaidaj;
}

ES::M3d VolumetricDeformationModel::compute_dP_dai(
  const ES::M9d &dPdF, const ES::M3d &dFdai) const
{
  ES::M3d dPdai;
  Eigen::Map<ES::V9d>(dPdai.data()) = dPdF *
    Eigen::Map<const ES::V9d>(dFdai.data());
  return dPdai;
}

double VolumetricDeformationModel::compute_dpsi_dai(
  const ES::M3d &Fref, const ES::M3d &dAInv_dai, const ES::M3d &P) const
{
  const ES::M3d dFe_dai = compute_dFe_dai(Fref, dAInv_dai);
  return P.cwiseProduct(dFe_dai).sum();
}

double VolumetricDeformationModel::compute_d2psi_dai_daj(
  const ES::M3d &Fref, const ES::M3d &dAInv_dai, const ES::M3d &dAInv_daj,
  const ES::M3d &d2AInv_dai_daj, const ES::M3d &P, const ES::M9d &dPdF) const
{
  const ES::M3d dFe_dai = compute_dFe_dai(Fref, dAInv_dai);
  const ES::M3d dFe_daj = compute_dFe_dai(Fref, dAInv_daj);

  const ES::M3d dP_daj = compute_dP_dai(dPdF, dFe_daj);

  const ES::M3d d2Fe_daidaj = compute_d2Fe_dai_daj(
    Fref, d2AInv_dai_daj);

  return dP_daj.cwiseProduct(dFe_dai).sum() + P.cwiseProduct(d2Fe_daidaj).sum();
}

void VolumetricDeformationModel::compute_d2Fe_dx_dai(
  const ES::M3d &dAInvdai, const M9xNDOF &rest_dF_dx, M9xNDOF &d2Fdudai) const
{
  for (int col = 0; col < localDofs_; col++) {
    const Eigen::Map<const ES::M3d> dFref(rest_dF_dx.col(col).data());
    const ES::M3d d2Fe = dFref * dAInvdai;
    d2Fdudai.col(col) = Eigen::Map<const ES::V9d>(d2Fe.data());
  }
}

}  // namespace SolidDeformationModel
}  // namespace pgo
