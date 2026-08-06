#include "volumetricDeformationElement.h"

#include "formulations/quadrature/quadrature.h"
#include "formulations/shapeFunction/shapeFunction.h"
#include <cmath>
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

// ============================================================
// Constructors
// ============================================================

VolumetricDeformationElement::RestGeometry::RestGeometry(
  std::span<const double> restPositions,
  const ShapeFunction &shapeFunction, const Quadrature &quadrature):
  numNodes(shapeFunction.numNodes()),
  numQuadraturePoints(quadrature.numPoints()),
  localDofs(shapeFunction.localDofs())
{
  if (restPositions.size() != static_cast<std::size_t>(3 * numNodes))
    throw std::invalid_argument(
      "Volumetric rest-position buffer has the wrong size.");

  M3xN restCoefficients(3, numNodes);
  for (int node = 0; node < numNodes; ++node) {
    restCoefficients.col(node) = ES::V3d(
      restPositions[node * 3], restPositions[node * 3 + 1],
      restPositions[node * 3 + 2]);
  }

  dN_dxi.resize(numQuadraturePoints, M3xN(3, numNodes));
  restDmInv.resize(numQuadraturePoints);
  rest_dF_dx.resize(
    numQuadraturePoints, M9xNDOF(9, localDofs));
  weightDetJ.resize(numQuadraturePoints);
  restBm.resize(numQuadraturePoints, M3xN(3, numNodes));

  M3xN derivativeBuffer(3, numNodes);
  for (int q = 0; q < numQuadraturePoints; ++q) {
    const ES::V3d xi = quadrature.point(q);
    shapeFunction.compute_dN_dxi(
      xi[0], xi[1], xi[2], derivativeBuffer);
    dN_dxi[q] = derivativeBuffer;

    const ES::M3d restJacobian =
      restCoefficients * dN_dxi[q].transpose();
    restDmInv[q] = restJacobian.fullPivLu().inverse();
    const M3xN dN_dX = restDmInv[q].transpose() * dN_dxi[q];
    weightDetJ[q] =
      std::abs(restJacobian.determinant()) * quadrature.weight(q);
    restBm[q] = weightDetJ[q] * dN_dX;

    rest_dF_dx[q].setZero();
    for (int node = 0; node < numNodes; ++node) {
      for (int coordinate = 0; coordinate < 3; ++coordinate) {
        const int dof = node * 3 + coordinate;
        for (int derivative = 0; derivative < 3; ++derivative) {
          rest_dF_dx[q](derivative * 3 + coordinate, dof) =
            dN_dX(derivative, node);
        }
      }
    }
  }
}

ES::M3d VolumetricDeformationElement::RestGeometry::computeDeformationGradient(
  std::span<const double> localPositions, int q) const
{
  if (q < 0 || q >= numQuadraturePoints)
    throw std::out_of_range("Volumetric quadrature index is out of range.");
  if (localPositions.size() != static_cast<std::size_t>(localDofs))
    throw std::invalid_argument(
      "Volumetric local-position buffer has the wrong size.");

  M3xN coefficients(3, numNodes);
  for (int node = 0; node < numNodes; ++node) {
    coefficients.col(node) = ES::V3d(
      localPositions[node * 3], localPositions[node * 3 + 1],
      localPositions[node * 3 + 2]);
  }
  return coefficients * dN_dxi[q].transpose() * restDmInv[q];
}

VolumetricDeformationElement::VolumetricDeformationElement(
  std::span<const double> restPositions,
  const ShapeFunction &shapeFunction, const Quadrature &quadrature,
  std::unique_ptr<ElasticModel3DDeformationGradient> elasticModel,
  std::unique_ptr<PlasticModel3DDeformationGradient> plasticModel,
  DeformationElementConstructionOptions options):
  DeformationElement(),
  geometry_(restPositions, shapeFunction, quadrature),
  numNodes_(geometry_.numNodes),
  numQuadPts_(geometry_.numQuadraturePoints),
  localDofs_(geometry_.localDofs),
  elasticModel_(std::move(elasticModel)),
  plasticModel_(std::move(plasticModel)),
  projectHessianPSD_(options.projectHessianPSD)
{
  if (elasticModel_ == nullptr) {
    throw std::invalid_argument(
      "VolumetricDeformationElement requires ElasticModel3DDeformationGradient.");
  }
  if (plasticModel_ == nullptr) {
    throw std::invalid_argument(
      "VolumetricDeformationElement requires PlasticModel3DDeformationGradient.");
  }

  numPlasticParams_ = plasticModel_->getNumParameters();
  numElasticParams_ = elasticModel_->getNumParameters();
  cache_ = VolumetricDeformationElementCache(
    numNodes_, numQuadPts_, numPlasticParams_, numElasticParams_);
}

double VolumetricDeformationElement::computeEnergy(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  return computeEnergy(cache_);
}

void VolumetricDeformationElement::computeDisplacementGradient(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, ES::RefVecXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  computeDisplacementGradient(cache_, output);
}

void VolumetricDeformationElement::computeDisplacementHessian(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, ES::RefMatXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  computeDisplacementHessian(cache_, output);
}

double VolumetricDeformationElement::computeEnergyGradient(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams,
  ES::RefVecXd displacementGradient) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  const double energy = computeEnergy(cache_);
  computeDisplacementGradient(cache_, displacementGradient);
  return energy;
}

double VolumetricDeformationElement::computeEnergyGradientHessian(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams,
  ES::RefVecXd displacementGradient, ES::RefMatXd displacementHessian) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  const double energy = computeEnergy(cache_);
  computeDisplacementGradient(cache_, displacementGradient);
  computeDisplacementHessian(cache_, displacementHessian);
  return energy;
}

void VolumetricDeformationElement::computeElasticGradient(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, ES::RefVecXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  output.setZero();
  auto local = cache_.materialGradientScratch.head(numElasticParams_);
  for (int q = 0; q < numQuadPts_; ++q) {
    computeElasticGradient(cache_, local, q);
    output += local;
  }
}

void VolumetricDeformationElement::computePlasticGradient(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, ES::RefVecXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  output.setZero();
  auto local = cache_.materialGradientScratch.head(numPlasticParams_);
  for (int q = 0; q < numQuadPts_; ++q) {
    local.setZero();
    computePlasticGradient(cache_, local, q);
    output += local;
  }
}

void VolumetricDeformationElement::computeElasticVJP(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams,
  std::span<const double> displacementAdjoint, ES::RefVecXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  auto mixed = cache_.materialMixedScratch.leftCols(numElasticParams_);
  mixed.setZero();
  auto local = cache_.materialMixedLocationScratch.leftCols(numElasticParams_);
  for (int q = 0; q < numQuadPts_; ++q) {
    computeDisplacementElasticHessian(cache_, local, q);
    mixed += local;
  }
  output.noalias() = mixed.transpose() *
    Eigen::Map<const ES::VXd>(displacementAdjoint.data(), localDofs_);
}

void VolumetricDeformationElement::computePlasticVJP(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams,
  std::span<const double> displacementAdjoint, ES::RefVecXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  auto mixed = cache_.materialMixedScratch.leftCols(numPlasticParams_);
  mixed.setZero();
  auto local = cache_.materialMixedLocationScratch.leftCols(numPlasticParams_);
  for (int q = 0; q < numQuadPts_; ++q) {
    computeDisplacementPlasticHessian(cache_, local, q);
    mixed += local;
  }
  output.noalias() = mixed.transpose() *
    Eigen::Map<const ES::VXd>(displacementAdjoint.data(), localDofs_);
}

int VolumetricDeformationElement::computeVonMisesStress(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, std::span<double> stresses) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  return computeVonMisesStress(
    cache_, stresses, static_cast<int>(stresses.size()));
}

int VolumetricDeformationElement::computeMaxStrain(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, std::span<double> strains) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  return computeMaxStrain(cache_, strains, static_cast<int>(strains.size()));
}

ES::M3d VolumetricDeformationElement::computeFirstPiola(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, int materialLocationID) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  return compute_P(cache_, materialLocationID);
}

ES::M9d VolumetricDeformationElement::computeFirstPiolaDerivative(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, int materialLocationID) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  return compute_dP_dF(cache_, materialLocationID);
}

void VolumetricDeformationElement::computeDeformationGradientDerivative(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, int materialLocationID,
  ES::RefMatXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  compute_dF_dx(cache_, materialLocationID, output);
}

void VolumetricDeformationElement::computeForceFromFirstPiola(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams, int materialLocationID,
  const ES::M3d &P, ES::RefVecXd output) const
{
  prepareData(x, elasticParams, plasticParams, cache_);
  computeForceFromP(cache_, materialLocationID, P, output);
}

// ============================================================
// elastic parameter access
// ============================================================

std::span<const double> VolumetricDeformationElement::elasticParams(
  const VolumetricDeformationElementCache &cacheData) const
{
  if (cacheData.numElasticParams == 0)
    return {};
  return std::span<const double>(cacheData.elasticParamsValue.data(),
    static_cast<std::size_t>(cacheData.numElasticParams));
}

// ============================================================
// prepareData
// ============================================================

void VolumetricDeformationElement::prepareData(
  std::span<const double> x, std::span<const double> elasticParams, std::span<const double> plasticParams,
  VolumetricDeformationElementCache &cacheDataBase) const
{
  if (x.size() != static_cast<std::size_t>(localDofs_))
    throw std::invalid_argument(
      "Volumetric deformation local-position buffer has the wrong size.");
  if (elasticParams.size() != static_cast<std::size_t>(numElasticParams_))
    throw std::invalid_argument(
      "Elastic parameters are required by this volumetric deformation model.");
  if (plasticParams.size() != static_cast<std::size_t>(numPlasticParams_))
    throw std::invalid_argument(
      "Plastic parameters are required by this volumetric deformation model.");

  VolumetricDeformationElementCache &cd = cacheDataBase;

  for (int vi = 0; vi < numNodes_; vi++) {
    cd.x.col(vi) = ES::V3d(x[vi * 3 + 0], x[vi * 3 + 1], x[vi * 3 + 2]);
  }

  if (numPlasticParams_ > 0)
    cd.plasticParamsValue = Eigen::Map<const ES::VXd>(plasticParams.data(), numPlasticParams_);
  if (numElasticParams_ > 0)
    cd.elasticParamsValue = Eigen::Map<const ES::VXd>(elasticParams.data(), numElasticParams_);

  const std::span<const double> plasticParamsForElement = numPlasticParams_ > 0 ?
    std::span<const double>(cd.plasticParamsValue.data(), numPlasticParams_) :
    std::span<const double>{};

  for (int q = 0; q < numQuadPts_; q++) {
    cd.Fp[q] = plasticModel_->computeA(plasticParamsForElement);
    cd.FpInv[q] = plasticModel_->computeAInv(plasticParamsForElement);
    cd.detFp[q] = plasticModel_->compute_detA(plasticParamsForElement);

    if (numPlasticParams_ > 0) {
      plasticModel_->compute_ddetA_da(plasticParamsForElement, cd.ddetA_da[q]);
      for (int i = 0; i < numPlasticParams_; i++)
        cd.dAInv_dai[q][i] = plasticModel_->compute_dAInv_da(plasticParamsForElement, i);
    }

    cd.Fref[q] = geometry_.computeDeformationGradient(x, q);
    cd.spectralState[q] = computeSpectralState(cd.Fref[q] * cd.FpInv[q]);
    computeCurrent_dF_dx(geometry_.rest_dF_dx[q], cd.FpInv[q], cd.dFdx[q]);
    cd.Bm[q] = cd.detFp[q] * cd.FpInv[q].transpose() * geometry_.restBm[q];
  }
}

// ============================================================
// Energy and derivatives w.r.t. displacement
// ============================================================

double VolumetricDeformationElement::computeEnergy(
  const VolumetricDeformationElementCache &cacheDataBase) const
{
  using CD = VolumetricDeformationElementCache;
  const CD &cd = cacheDataBase;

  double energy = 0.0;
  for (int q = 0; q < numQuadPts_; q++) {
    const std::span<const double> mp = elasticParams(cacheDataBase);
    energy += elasticModel_->compute_psi(mp, cd.spectralState[q]) *
      geometry_.weightDetJ[q] * cd.detFp[q];
  }
  return energy;
}

void VolumetricDeformationElement::computeDisplacementGradient(
  const VolumetricDeformationElementCache &cacheDataBase, ES::RefVecXd grad) const
{
  if (grad.size() != localDofs_)
    throw std::invalid_argument("Volumetric deformation gradient has unexpected size.");
  using CD = VolumetricDeformationElementCache;
  const CD &cd = cacheDataBase;

  Eigen::Map<Eigen::VectorXd> gradMap(grad.data(), localDofs_);
  gradMap.setZero();

  for (int q = 0; q < numQuadPts_; q++) {
    const std::span<const double> mp = elasticParams(cacheDataBase);
    const ES::M3d P = elasticModel_->compute_P(mp, cd.spectralState[q]);
    const M3xN localForce = P * cd.Bm[q];
    gradMap += Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>>(
      localForce.data(), localDofs_);
  }
}

void VolumetricDeformationElement::computeDisplacementHessian(
  const VolumetricDeformationElementCache &cacheDataBase, ES::RefMatXd hess) const
{
  if (hess.rows() != localDofs_ || hess.cols() != localDofs_)
    throw std::invalid_argument("Volumetric deformation Hessian has unexpected shape.");
  using CD = VolumetricDeformationElementCache;
  const CD &cd = cacheDataBase;

  Eigen::Map<Eigen::MatrixXd> hessMap(hess.data(), localDofs_, localDofs_);
  hessMap.setZero();

  for (int q = 0; q < numQuadPts_; q++) {
    const std::span<const double> mp = elasticParams(cacheDataBase);
    ES::M9d dPdF;
    if (projectHessianPSD_) {
      dPdF = elasticModel_->compute_dPdF_psd(mp, cd.spectralState[q]);
    }
    else {
      dPdF = elasticModel_->compute_dPdF(mp, cd.spectralState[q]);
    }
    dPdF *= geometry_.weightDetJ[q] * cd.detFp[q];
    hessMap.noalias() += cd.dFdx[q].transpose() * dPdF * cd.dFdx[q];
  }
}

// ============================================================
// computeSpectralState
// ============================================================

SpectralState VolumetricDeformationElement::computeSpectralState(
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
// compute_F / compute_Fe / compute_P / compute_dP_dF / compute_dF_dx / computeForceFromP
// ============================================================

ES::M3d VolumetricDeformationElement::compute_F(
  std::span<const double> x, int materialLocationID) const
{
  if (x.size() != static_cast<std::size_t>(localDofs_))
    throw std::invalid_argument(
      "Volumetric deformation local-position buffer has the wrong size.");
  return geometry_.computeDeformationGradient(x, materialLocationID);
}

ES::M3d VolumetricDeformationElement::compute_Fe(
  const VolumetricDeformationElementCache &cacheData, int materialLocationID) const
{
  using CD = VolumetricDeformationElementCache;
  const CD &cd = cacheData;
  return cd.spectralState[materialLocationID].F;
}

ES::M3d VolumetricDeformationElement::compute_P(
  const VolumetricDeformationElementCache &cacheData, int materialLocationID) const
{
  using CD = VolumetricDeformationElementCache;
  const CD &cd = cacheData;
  const std::span<const double> mp = elasticParams(cacheData);

  return elasticModel_->compute_P(mp, cd.spectralState[materialLocationID]);
}

ES::M9d VolumetricDeformationElement::compute_dP_dF(
  const VolumetricDeformationElementCache &cacheData, int materialLocationID) const
{
  using CD = VolumetricDeformationElementCache;
  const CD &cd = cacheData;
  const std::span<const double> mp = elasticParams(cacheData);

  return elasticModel_->compute_dPdF(mp, cd.spectralState[materialLocationID]);
}

void VolumetricDeformationElement::compute_dF_dx(
  const VolumetricDeformationElementCache &cacheData, int materialLocationID, ES::RefMatXd dFdxOut) const
{
  using CD = VolumetricDeformationElementCache;
  const CD &cd = cacheData;
  if (dFdxOut.rows() != 9 || dFdxOut.cols() != localDofs_)
    throw std::invalid_argument("Volumetric dFdx output has unexpected size.");
  dFdxOut = cd.dFdx[materialLocationID];
}

void VolumetricDeformationElement::computeForceFromP(
  const VolumetricDeformationElementCache &cacheDataBase, int materialLocationID,
  const ES::M3d &P, ES::RefVecXd f) const
{
  using CD = VolumetricDeformationElementCache;
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

int VolumetricDeformationElement::computeVonMisesStress(
  const VolumetricDeformationElementCache &cacheDataBase,
  std::span<double> stresses, int capacity) const
{
  if (capacity < numQuadPts_)
    throw std::length_error(
      "Volumetric von Mises stress output capacity is too small.");
  if (stresses.size() < static_cast<std::size_t>(numQuadPts_))
    throw std::invalid_argument(
      "Volumetric von Mises stress output must not be null.");

  using CD = VolumetricDeformationElementCache;
  const CD &cd = cacheDataBase;

  for (int q = 0; q < numQuadPts_; q++) {
    const std::span<const double> mp = elasticParams(cacheDataBase);
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

int VolumetricDeformationElement::computeMaxStrain(
  const VolumetricDeformationElementCache &cacheDataBase,
  std::span<double> strains, int capacity) const
{
  if (capacity < numQuadPts_)
    throw std::length_error(
      "Volumetric maximum strain output capacity is too small.");
  if (strains.size() < static_cast<std::size_t>(numQuadPts_))
    throw std::invalid_argument(
      "Volumetric maximum strain output must not be null.");

  using CD = VolumetricDeformationElementCache;
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

void VolumetricDeformationElement::computePlasticGradient(
  const VolumetricDeformationElementCache &cacheDataBase, ES::RefVecXd grad,
  int materialLocation) const
{
  using CD = VolumetricDeformationElementCache;
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
    const std::span<const double> mp = elasticParams(cacheDataBase);
    double psi = elasticModel_->compute_psi(mp, cd.spectralState[q]);

    const ES::M3d P = elasticModel_->compute_P(mp, cd.spectralState[q]);

    for (int i = 0; i < numPlasticParams_; i++) {
      double dVda = compute_dV_dai(geometry_.weightDetJ[q], cd.ddetA_da[q][i]);
      double dpsi_da = compute_dpsi_dai(cd.Fref[q], cd.dAInv_dai[q][i], P);
      gradMap[i] += dVda * psi + geometry_.weightDetJ[q] * cd.detFp[q] * dpsi_da;
    }
  }
}

void VolumetricDeformationElement::computeDisplacementPlasticHessian(
  const VolumetricDeformationElementCache &cacheDataBase, ES::RefMatXd hess,
  int materialLocation) const
{
  using CD = VolumetricDeformationElementCache;
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
    const std::span<const double> mp = elasticParams(cacheDataBase);
    const ES::M3d P = elasticModel_->compute_P(mp, cd.spectralState[q]);

    cd.dpsiDxScratch.noalias() = cd.dFdx[q].transpose() *
      Eigen::Map<const ES::V9d>(P.data());

    const ES::M9d dPdF = elasticModel_->compute_dPdF(mp, cd.spectralState[q]);

    const double vol = geometry_.weightDetJ[q] * cd.detFp[q];
    for (int i = 0; i < numPlasticParams_; i++) {
      const double dVda = compute_dV_dai(geometry_.weightDetJ[q], cd.ddetA_da[q][i]);

      const ES::M3d dFda = compute_dFe_dai(
        cd.Fref[q], cd.dAInv_dai[q][i]);
      const ES::M3d dPda = compute_dP_dai(dPdF, dFda);

      cd.localDofScratch.noalias() = cd.dFdx[q].transpose() *
        Eigen::Map<const ES::V9d>(dPda.data());

      compute_d2Fe_dx_dai(cd.dAInv_dai[q][i], geometry_.rest_dF_dx[q], cd.d2FdxdaScratch);

      mixed.col(i) += dVda * cd.dpsiDxScratch + vol * (cd.localDofScratch + cd.d2FdxdaScratch.transpose() * Eigen::Map<const ES::V9d>(P.data()));
    }
  }
}

// ============================================================
// Elastic material-parameter derivatives.
// ============================================================

void VolumetricDeformationElement::computeElasticGradient(
  const VolumetricDeformationElementCache &cacheDataBase, ES::RefVecXd grad,
  int materialLocation) const
{
  using CD = VolumetricDeformationElementCache;
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
    const std::span<const double> mp = elasticParams(cacheDataBase);
    const double vol = geometry_.weightDetJ[q] * cd.detFp[q];
    elasticModel_->compute_dpsi_dparams(
      mp, cd.spectralState[q], cd.dpsiDparamScratch);
    gradMap.noalias() += vol * cd.dpsiDparamScratch;
  }
}

void VolumetricDeformationElement::computeDisplacementElasticHessian(
  const VolumetricDeformationElementCache &cacheDataBase, ES::RefMatXd hess,
  int materialLocation) const
{
  using CD = VolumetricDeformationElementCache;
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
    const std::span<const double> mp = elasticParams(cacheDataBase);
    const double vol = geometry_.weightDetJ[q] * cd.detFp[q];
    elasticModel_->compute_dP_dparams(
      mp, cd.spectralState[q], cd.dPdbScratch);
    mixed.noalias() +=
      vol * cd.dFdx[q].transpose() * cd.dPdbScratch;
  }
}

// ============================================================
// Private helper methods
// ============================================================

void VolumetricDeformationElement::computeCurrent_dF_dx(
  const M9xNDOF &rest_dF_dx, const ES::M3d &FpInv, M9xNDOF &dFdx) const
{
  for (int col = 0; col < localDofs_; col++) {
    const Eigen::Map<const ES::M3d> dFref(rest_dF_dx.col(col).data());
    const ES::M3d dFe = dFref * FpInv;
    dFdx.col(col) = Eigen::Map<const ES::V9d>(dFe.data());
  }
}

double VolumetricDeformationElement::compute_dV_dai(
  double weightDetJ, double ddetA_dai) const
{
  return weightDetJ * ddetA_dai;
}

ES::M3d VolumetricDeformationElement::compute_dFe_dai(
  const ES::M3d &Fref, const ES::M3d &dAInvdai) const
{
  return Fref * dAInvdai;
}

ES::M3d VolumetricDeformationElement::compute_dP_dai(
  const ES::M9d &dPdF, const ES::M3d &dFdai) const
{
  ES::M3d dPdai;
  Eigen::Map<ES::V9d>(dPdai.data()) = dPdF *
    Eigen::Map<const ES::V9d>(dFdai.data());
  return dPdai;
}

double VolumetricDeformationElement::compute_dpsi_dai(
  const ES::M3d &Fref, const ES::M3d &dAInv_dai, const ES::M3d &P) const
{
  const ES::M3d dFe_dai = compute_dFe_dai(Fref, dAInv_dai);
  return P.cwiseProduct(dFe_dai).sum();
}

void VolumetricDeformationElement::compute_d2Fe_dx_dai(
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
