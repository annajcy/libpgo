#include "shellDeformationModel.h"

#include "pgoLogging.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{
namespace
{
double parameterStep(double value)
{
  return 1e-6 * std::max(1.0, std::abs(value));
}
}  // namespace

ShellDeformationModelCacheData::ShellDeformationModelCacheData(
  int numPlasticParams_, int numElasticParams_):
  numPlasticParams(numPlasticParams_),
  numElasticParams(numElasticParams_),
  plasticParamsValue(numPlasticParams_),
  elasticParamsValue(numElasticParams_)
{
  for (auto &xi : x)
    xi.setZero();
  plasticParamsValue.setZero();
  elasticParamsValue.setZero();
}

ShellDeformationModel::ShellDeformationModel(
  std::unique_ptr<ShellKinematics> kinematics,
  std::unique_ptr<ElasticModel2DFundamentalForms> elasticModel,
  std::unique_ptr<PlasticModel2DFundamentalForms> plasticModel)
  : DeformationModel()
  , kinematics_(std::move(kinematics))
  , elastic2D_(std::move(elasticModel))
  , plastic2D_(std::move(plasticModel))
{
  if (!kinematics_) {
    throw std::logic_error("ShellDeformationModel requires non-null shell kinematics");
  }
  if (!elastic2D_) {
    throw std::logic_error("ShellDeformationModel requires non-null ElasticModel2DFundamentalForms");
  }
  if (!plastic2D_) {
    throw std::logic_error("ShellDeformationModel requires non-null PlasticModel2DFundamentalForms");
  }

  plastic2D_->set_abar(kinematics_->restI());
  plastic2D_->set_bbar(kinematics_->restII());
  plastic2D_->setArea(kinematics_->restArea());

  numPlasticParams_ = plastic2D_->getNumParameters();
  numElasticParams_ = elastic2D_->getNumParameters();
}

std::unique_ptr<DeformationModelCacheData> ShellDeformationModel::allocateCacheData() const
{
  return std::make_unique<CacheData>(numPlasticParams_, numElasticParams_);
}

void ShellDeformationModel::prepareData(
  const double *x, const double *elasticParams, const double *plasticParams,
  DeformationModelCacheData *cacheDataBase) const
{
  CacheData *cacheData = this->cacheData(cacheDataBase);
  cacheData->x[0] = ES::V3d(x[0], x[1], x[2]);
  cacheData->x[1] = ES::V3d(x[3], x[4], x[5]);
  cacheData->x[2] = ES::V3d(x[6], x[7], x[8]);
  cacheData->x[3] = ES::V3d(x[9], x[10], x[11]);
  cacheData->x[4] = ES::V3d(x[12], x[13], x[14]);
  cacheData->x[5] = ES::V3d(x[15], x[16], x[17]);

  if (numPlasticParams_ > 0) {
    if (plasticParams) {
      cacheData->plasticParamsValue =
        Eigen::Map<const ES::VXd>(plasticParams, numPlasticParams_);
    }
    else {
      plastic2D_->defaultParams(cacheData->plasticParamsValue.data());
    }
  }

  if (numElasticParams_ > 0) {
    cacheData->elasticParamsValue.setZero();
    if (elasticParams) {
      cacheData->elasticParamsValue =
        Eigen::Map<const ES::VXd>(elasticParams, numElasticParams_);
    }
  }

  const double *plasticParamPtr = numPlasticParams_ > 0 ? cacheData->plasticParamsValue.data() : nullptr;
  plastic2D_->compute_abar(plasticParamPtr, cacheData->abar.data());
  plastic2D_->compute_bbar(plasticParamPtr, cacheData->bbar.data());
  cacheData->area = plastic2D_->computeArea(plasticParamPtr);

  cacheData->a = kinematics_->compute_a_and_derivatives(cacheData->x.data(), nullptr, nullptr);
  cacheData->b = kinematics_->compute_b_and_derivatives(cacheData->x.data(), nullptr, nullptr);
}

double ShellDeformationModel::computeEnergy(const DeformationModelCacheData *cacheDataBase) const
{
  const CacheData *cacheData = this->cacheData(cacheDataBase);
  const double *plasticParamPtr = numPlasticParams_ > 0 ? cacheData->plasticParamsValue.data() : nullptr;
  const double *elasticParamPtr = numElasticParams_ > 0 ? cacheData->elasticParamsValue.data() : nullptr;
  return computeEnergyWithParams(*cacheData, plasticParamPtr, elasticParamPtr);
}

void ShellDeformationModel::compute_dE_dx(const DeformationModelCacheData *cacheDataBase,
  double *grad) const
{
  const CacheData *cacheData = this->cacheData(cacheDataBase);

  ES::M4x9d dadx;
  ES::M4x18d dbdx;

  kinematics_->compute_a_and_derivatives(cacheData->x.data(), &dadx, nullptr);
  kinematics_->compute_b_and_derivatives(cacheData->x.data(), &dbdx, nullptr);

  ES::M2d dEda, dEdb;
  const double *elasticParamPtr = numElasticParams_ > 0 ? cacheData->elasticParamsValue.data() : nullptr;
  elastic2D_->compute_dpsi_da(
    elasticParamPtr, cacheData->a.data(),
    cacheData->abar.data(), dEda.data());
  elastic2D_->compute_dpsi_db(
    elasticParamPtr, cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), dEdb.data());

  (ES::Mp<ES::V18d>(grad)).setZero();
  (ES::Mp<ES::V9d>(grad)) =
    dadx.transpose() * ES::Mp<const ES::V4d>(dEda.data()) * cacheData->area;
  (ES::Mp<ES::V18d>(grad)) +=
    dbdx.transpose() * ES::Mp<const ES::V4d>(dEdb.data()) * cacheData->area;
}

void ShellDeformationModel::compute_d2E_dx2(const DeformationModelCacheData *cacheDataBase,
  double *hess) const
{
  const CacheData *cacheData = this->cacheData(cacheDataBase);

  ES::M4x9d dadx;
  ES::M4x18d dbdx;

  ES::M9d d2adx2[4];
  ES::M18d d2bdx2[4];

  kinematics_->compute_a_and_derivatives(cacheData->x.data(), &dadx, d2adx2);
  kinematics_->compute_b_and_derivatives(cacheData->x.data(), &dbdx, d2bdx2);

  ES::M2d dEda, dEdb;
  const double *elasticParamPtr = numElasticParams_ > 0 ? cacheData->elasticParamsValue.data() : nullptr;
  elastic2D_->compute_dpsi_da(
    elasticParamPtr, cacheData->a.data(),
    cacheData->abar.data(), dEda.data());
  elastic2D_->compute_dpsi_db(
    elasticParamPtr, cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), dEdb.data());

  ES::M4d d2Eda2, d2Edb2;
  elastic2D_->compute_d2psi_da2(
    elasticParamPtr, cacheData->a.data(),
    cacheData->abar.data(), d2Eda2.data());
  elastic2D_->compute_d2psi_db2(
    elasticParamPtr, cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), d2Edb2.data());

  ES::Mp<ES::M18d> hessMap(hess);
  hessMap.setZero();

  hessMap.block<9, 9>(0, 0) +=
    dadx.transpose() * d2Eda2 * dadx * cacheData->area;
  for (int j = 0; j < 4; j++) {
    hessMap.block<9, 9>(0, 0) +=
      dEda.data()[j] * d2adx2[j] * cacheData->area;
  }

  hessMap += dbdx.transpose() * d2Edb2 * dbdx * cacheData->area;
  for (int j = 0; j < 4; j++) {
    hessMap += dEdb.data()[j] * d2bdx2[j] * cacheData->area;
  }

  if (enableSPD_) {
    Eigen::SelfAdjointEigenSolver<ES::M18d> eigenSolver(
      hessMap, Eigen::ComputeEigenvectors);
    ES::V18d eigenvalues = eigenSolver.eigenvalues();
    ES::M18d eigenvectors = eigenSolver.eigenvectors();

    eigenvalues = eigenvalues.cwiseMax(0);
    ES::M18d hessSPD =
      eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose();
    hessMap = hessSPD;
  }
}

void ShellDeformationModel::compute_d2E_dxda(const DeformationModelCacheData *cacheDataBase,
  double *hess) const
{
  const CacheData *cacheData = this->cacheData(cacheDataBase);
  if (numPlasticParams_ == 0)
    return;

  const double *elasticParamPtr = numElasticParams_ > 0 ? cacheData->elasticParamsValue.data() : nullptr;
  const double *plasticParamPtr = cacheData->plasticParamsValue.data();
  ES::V4d dpsi_da, dpsi_db;
  elastic2D_->compute_dpsi_da(
    elasticParamPtr, cacheData->a.data(),
    cacheData->abar.data(), dpsi_da.data());
  elastic2D_->compute_dpsi_db(
    elasticParamPtr, cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), dpsi_db.data());

  ES::M4d d2psi_da_dabar, d2psi_db_dabar, d2psi_db_dbbar;
  elastic2D_->compute_d2psi_dadabar(
    elasticParamPtr, cacheData->a.data(),
    cacheData->abar.data(), d2psi_da_dabar.data());
  elastic2D_->compute_d2psi_db_dabar(
    elasticParamPtr, cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), d2psi_db_dabar.data());
  elastic2D_->compute_d2psi_db_dbbar(
    elasticParamPtr, cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), d2psi_db_dbbar.data());

  ES::M4x9d dadx;
  ES::M4x18d dbdx;
  kinematics_->compute_a_and_derivatives(cacheData->x.data(), &dadx, nullptr);
  kinematics_->compute_b_and_derivatives(cacheData->x.data(), &dbdx, nullptr);

  ES::MXd dabar_dF(4, numPlasticParams_);
  ES::MXd dbbar_dF(4, numPlasticParams_);
  ES::VXd darea_dF(numPlasticParams_);
  dabar_dF.setZero();
  dbbar_dF.setZero();
  darea_dF.setZero();
  plastic2D_->compute_dabar_dparam(
    plasticParamPtr, dabar_dF.data());
  plastic2D_->compute_dbbar_dparam(
    plasticParamPtr, dbbar_dF.data());
  plastic2D_->compute_darea_dparam(
    plasticParamPtr, darea_dF.data());

  ES::V9d dpsi_a_dx = dpsi_da.transpose() * dadx;
  ES::V18d dpsi_b_dx = dpsi_db.transpose() * dbdx;

  int np = numPlasticParams_;
  ES::MXd mixed = ES::MXd::Zero(18, np);

  mixed.block(0, 0, 9, np) +=
    dadx.transpose() * d2psi_da_dabar * dabar_dF * cacheData->area;
  mixed.block(0, 0, 9, np) +=
    dpsi_a_dx * darea_dF.transpose();

  mixed.block(0, 0, 18, np) +=
    dbdx.transpose() * d2psi_db_dabar * dabar_dF * cacheData->area;
  mixed.block(0, 0, 18, np) +=
    dbdx.transpose() * d2psi_db_dbbar * dbbar_dF * cacheData->area;
  mixed.block(0, 0, 18, np) +=
    dpsi_b_dx * darea_dF.transpose();

  ES::Mp<ES::MXd>(hess, 18, np) = mixed;
}

void ShellDeformationModel::compute_d2E_dxdb(const DeformationModelCacheData *cacheDataBase,
  double *hess) const
{
  const CacheData *cacheData = this->cacheData(cacheDataBase);
  if (numElasticParams_ == 0)
    return;

  ES::MXd d2psi_da_dparam(4, numElasticParams_);
  ES::MXd d2psi_db_dparam(4, numElasticParams_);
  d2psi_da_dparam.setZero();
  d2psi_db_dparam.setZero();
  const double *elasticParamPtr = cacheData->elasticParamsValue.data();
  elastic2D_->compute_d2psi_da_dparam(
    elasticParamPtr, cacheData->a.data(),
    cacheData->abar.data(), d2psi_da_dparam.data());
  elastic2D_->compute_d2psi_db_dparam(
    elasticParamPtr, cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), d2psi_db_dparam.data());

  ES::M4x9d dadx;
  ES::M4x18d dbdx;
  kinematics_->compute_a_and_derivatives(cacheData->x.data(), &dadx, nullptr);
  kinematics_->compute_b_and_derivatives(cacheData->x.data(), &dbdx, nullptr);

  int np = numElasticParams_;
  ES::MXd mixed = ES::MXd::Zero(18, np);

  mixed.block(0, 0, 9, np) +=
    dadx.transpose() * d2psi_da_dparam * cacheData->area;
  mixed.block(0, 0, 18, np) +=
    dbdx.transpose() * d2psi_db_dparam * cacheData->area;

  ES::Mp<ES::MXd>(hess, 18, np) = mixed;
}

void ShellDeformationModel::compute_dE_da(const DeformationModelCacheData *cacheDataBase,
  double *grad) const
{
  const CacheData *cacheData = this->cacheData(cacheDataBase);
  if (numPlasticParams_ == 0)
    return;

  const double *elasticParamPtr = numElasticParams_ > 0 ? cacheData->elasticParamsValue.data() : nullptr;
  computePlasticGradientWithParams(*cacheData, cacheData->plasticParamsValue.data(), elasticParamPtr, grad);
}

void ShellDeformationModel::compute_d2E_da2(const DeformationModelCacheData *cacheDataBase,
  double *hess) const
{
  const CacheData *cacheData = this->cacheData(cacheDataBase);
  if (numPlasticParams_ == 0)
    return;

  Eigen::Map<ES::MXd> hessMap(hess, numPlasticParams_, numPlasticParams_);
  hessMap.setZero();
  const double *elasticParamPtr = numElasticParams_ > 0 ? cacheData->elasticParamsValue.data() : nullptr;
  ES::VXd params = cacheData->plasticParamsValue;
  ES::VXd gradPlus(numPlasticParams_);
  ES::VXd gradMinus(numPlasticParams_);
  for (int col = 0; col < numPlasticParams_; col++) {
    const double h = parameterStep(params[col]);
    ES::VXd plus = params;
    ES::VXd minus = params;
    plus[col] += h;
    minus[col] -= h;
    computePlasticGradientWithParams(*cacheData, plus.data(), elasticParamPtr, gradPlus.data());
    computePlasticGradientWithParams(*cacheData, minus.data(), elasticParamPtr, gradMinus.data());
    hessMap.col(col) = (gradPlus - gradMinus) / (2.0 * h);
  }
  hessMap = (0.5 * (hessMap + hessMap.transpose())).eval();
}

void ShellDeformationModel::compute_dE_db(const DeformationModelCacheData *cacheDataBase,
  double *grad) const
{
  const CacheData *cacheData = this->cacheData(cacheDataBase);
  if (numElasticParams_ == 0)
    return;

  const double *plasticParamPtr = numPlasticParams_ > 0 ? cacheData->plasticParamsValue.data() : nullptr;
  computeElasticGradientWithParams(*cacheData, plasticParamPtr, cacheData->elasticParamsValue.data(), grad);
}

void ShellDeformationModel::compute_d2E_db2(const DeformationModelCacheData *cacheDataBase,
  double *hess) const
{
  const CacheData *cacheData = this->cacheData(cacheDataBase);
  if (numElasticParams_ == 0)
    return;

  Eigen::Map<ES::MXd> hessMap(hess, numElasticParams_, numElasticParams_);
  hessMap.setZero();
  const double *plasticParamPtr = numPlasticParams_ > 0 ? cacheData->plasticParamsValue.data() : nullptr;
  ES::VXd params = cacheData->elasticParamsValue;
  ES::VXd gradPlus(numElasticParams_);
  ES::VXd gradMinus(numElasticParams_);
  for (int col = 0; col < numElasticParams_; col++) {
    const double h = parameterStep(params[col]);
    ES::VXd plus = params;
    ES::VXd minus = params;
    plus[col] += h;
    minus[col] -= h;
    computeElasticGradientWithParams(*cacheData, plasticParamPtr, plus.data(), gradPlus.data());
    computeElasticGradientWithParams(*cacheData, plasticParamPtr, minus.data(), gradMinus.data());
    hessMap.col(col) = (gradPlus - gradMinus) / (2.0 * h);
  }
  hessMap = (0.5 * (hessMap + hessMap.transpose())).eval();
}

void ShellDeformationModel::compute_d2E_dadb(const DeformationModelCacheData *cacheDataBase,
  double *hess) const
{
  const CacheData *cacheData = this->cacheData(cacheDataBase);
  if (numPlasticParams_ == 0 || numElasticParams_ == 0)
    return;

  Eigen::Map<ES::MXd> hessMap(hess, numPlasticParams_, numElasticParams_);
  hessMap.setZero();
  ES::VXd elasticParams = cacheData->elasticParamsValue;
  ES::VXd gradPlus(numPlasticParams_);
  ES::VXd gradMinus(numPlasticParams_);
  for (int col = 0; col < numElasticParams_; col++) {
    const double h = parameterStep(elasticParams[col]);
    ES::VXd plus = elasticParams;
    ES::VXd minus = elasticParams;
    plus[col] += h;
    minus[col] -= h;
    computePlasticGradientWithParams(*cacheData, cacheData->plasticParamsValue.data(), plus.data(), gradPlus.data());
    computePlasticGradientWithParams(*cacheData, cacheData->plasticParamsValue.data(), minus.data(), gradMinus.data());
    hessMap.col(col) = (gradPlus - gradMinus) / (2.0 * h);
  }
}

void ShellDeformationModel::enableSPD(int enable)
{
  enableSPD_ = enable;
  elastic2D_->enableSPD(enable);
}

void ShellDeformationModel::defaultPlasticParams(double *params) const
{
  plastic2D_->defaultParams(params);
}

bool ShellDeformationModel::isPlasticIdentityTransform() const
{
  return plastic2D_->isIdentityTransform();
}

int ShellDeformationModel::getNumVertices() const
{
  return kinematics_->getNumNodes();
}

int ShellDeformationModel::getNumDOFs() const
{
  return kinematics_->getLocalDofs();
}

DeformationModel::LocalMaxStepResult ShellDeformationModel::computeLocalMaxStepSize(
  const double *x_local, const double *dx_local) const
{
  (void)x_local;
  (void)dx_local;
  return LocalMaxStepResult{};
}

const ShellDeformationModel::CacheData *ShellDeformationModel::cacheData(
  const DeformationModelCacheData *cacheDataBase) const
{
  PGO_ALOG(dynamic_cast<const CacheData *>(cacheDataBase) != nullptr);
  return static_cast<const CacheData *>(cacheDataBase);
}

ShellDeformationModel::CacheData *ShellDeformationModel::cacheData(
  DeformationModelCacheData *cacheDataBase) const
{
  PGO_ALOG(dynamic_cast<CacheData *>(cacheDataBase) != nullptr);
  return static_cast<CacheData *>(cacheDataBase);
}

double ShellDeformationModel::computeEnergyWithParams(
  const CacheData &cacheData, const double *plasticParams, const double *elasticParams) const
{
  ES::M2d abar;
  ES::M2d bbar;
  plastic2D_->compute_abar(plasticParams, abar.data());
  plastic2D_->compute_bbar(plasticParams, bbar.data());
  const double area = plastic2D_->computeArea(plasticParams);

  const double E1 = elastic2D_->compute_psi_a(
    elasticParams, cacheData.a.data(), abar.data());
  const double E2 = elastic2D_->compute_psi_b(
    elasticParams, cacheData.b.data(), abar.data(), bbar.data());

  return (E1 + E2) * area;
}

void ShellDeformationModel::computePlasticGradientWithParams(
  const CacheData &cacheData, const double *plasticParams, const double *elasticParams, double *grad) const
{
  Eigen::Map<ES::VXd> gradMap(grad, numPlasticParams_);
  gradMap.setZero();
  if (numPlasticParams_ == 0)
    return;

  const Eigen::Map<const ES::VXd> params(plasticParams, numPlasticParams_);
  for (int i = 0; i < numPlasticParams_; i++) {
    const double h = parameterStep(params[i]);
    ES::VXd plus = params;
    ES::VXd minus = params;
    plus[i] += h;
    minus[i] -= h;
    const double ePlus = computeEnergyWithParams(cacheData, plus.data(), elasticParams);
    const double eMinus = computeEnergyWithParams(cacheData, minus.data(), elasticParams);
    gradMap[i] = (ePlus - eMinus) / (2.0 * h);
  }
}

void ShellDeformationModel::computeElasticGradientWithParams(
  const CacheData &cacheData, const double *plasticParams, const double *elasticParams, double *grad) const
{
  Eigen::Map<ES::VXd> gradMap(grad, numElasticParams_);
  gradMap.setZero();
  if (numElasticParams_ == 0)
    return;

  const Eigen::Map<const ES::VXd> params(elasticParams, numElasticParams_);
  for (int i = 0; i < numElasticParams_; i++) {
    const double h = parameterStep(params[i]);
    ES::VXd plus = params;
    ES::VXd minus = params;
    plus[i] += h;
    minus[i] -= h;
    const double ePlus = computeEnergyWithParams(cacheData, plasticParams, plus.data());
    const double eMinus = computeEnergyWithParams(cacheData, plasticParams, minus.data());
    gradMap[i] = (ePlus - eMinus) / (2.0 * h);
  }
}

}  // namespace SolidDeformationModel
}  // namespace pgo
