#include "shellDeformationModel.h"

#include "pgoLogging.h"

#include <algorithm>
#include <stdexcept>

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{
namespace
{
void validateMaterialLocation(int materialLocation)
{
  if (materialLocation > 0)
    throw std::out_of_range("Shell material location is out of range.");
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

  const int maxParams = std::max(numPlasticParams, numElasticParams);
  elasticDpsiDparamScratch.resize(numElasticParams);
  plasticDAreaDparamScratch.resize(numPlasticParams);
  plasticDAbarDparamScratch.resize(4, numPlasticParams);
  plasticDBbarDparamScratch.resize(4, numPlasticParams);
  elasticDpsiDaDparamScratch.resize(4, numElasticParams);
  elasticDpsiDbDparamScratch.resize(4, numElasticParams);
  elasticDpsiDabarDparamScratch.resize(4, numElasticParams);
  elasticDpsiDbbarDparamScratch.resize(4, numElasticParams);
  mixedDerivativeScratch.resize(18, maxParams);
}

ShellDeformationModel::ShellDeformationModel(
  std::unique_ptr<ShellElementMapping> mapping,
  std::unique_ptr<ElasticModel2DFundamentalForms> elasticModel,
  std::unique_ptr<PlasticModel2DFundamentalForms> plasticModel)
  : DeformationModel()
  , elementMapping_(std::move(mapping))
  , elastic2D_(std::move(elasticModel))
  , plastic2D_(std::move(plasticModel))
{
  if (!elementMapping_) {
    throw std::logic_error("ShellDeformationModel requires non-null shell mapping");
  }
  if (!elastic2D_) {
    throw std::logic_error("ShellDeformationModel requires non-null ElasticModel2DFundamentalForms");
  }
  if (!plastic2D_) {
    throw std::logic_error("ShellDeformationModel requires non-null PlasticModel2DFundamentalForms");
  }

  plastic2D_->set_abar(elementMapping_->restI());
  plastic2D_->set_bbar(elementMapping_->restII());
  plastic2D_->setArea(elementMapping_->restArea());

  numPlasticParams_ = plastic2D_->getNumParameters();
  numElasticParams_ = elastic2D_->getNumParameters();
}

std::unique_ptr<DeformationModelCacheData> ShellDeformationModel::allocateCacheData() const
{
  return std::make_unique<CacheData>(numPlasticParams_, numElasticParams_);
}

bool ShellDeformationModel::isCacheDataCompatible(
  const DeformationModelCacheData &cacheData) const
{
  const auto *cd = dynamic_cast<const CacheData *>(&cacheData);
  return cd != nullptr &&
    cd->numPlasticParams == numPlasticParams_ &&
    cd->numElasticParams == numElasticParams_;
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

  cacheData->a = elementMapping_->compute_a_and_derivatives(cacheData->x.data(), nullptr, nullptr);
  cacheData->b = elementMapping_->compute_b_and_derivatives(cacheData->x.data(), nullptr, nullptr);
  cacheDataBase->markPrepared();
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

  elementMapping_->compute_a_and_derivatives(cacheData->x.data(), &dadx, nullptr);
  elementMapping_->compute_b_and_derivatives(cacheData->x.data(), &dbdx, nullptr);

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

  elementMapping_->compute_a_and_derivatives(cacheData->x.data(), &dadx, d2adx2);
  elementMapping_->compute_b_and_derivatives(cacheData->x.data(), &dbdx, d2bdx2);

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
  double *hess, int materialLocation) const
{
  validateMaterialLocation(materialLocation);
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
  elementMapping_->compute_a_and_derivatives(cacheData->x.data(), &dadx, nullptr);
  elementMapping_->compute_b_and_derivatives(cacheData->x.data(), &dbdx, nullptr);

  ES::MXd &dabar_dF = cacheData->plasticDAbarDparamScratch;
  ES::MXd &dbbar_dF = cacheData->plasticDBbarDparamScratch;
  ES::VXd &darea_dF = cacheData->plasticDAreaDparamScratch;
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
  ES::MXd &mixed = cacheData->mixedDerivativeScratch;
  mixed.block(0, 0, 18, np).setZero();

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

  ES::Mp<ES::MXd>(hess, 18, np) = mixed.block(0, 0, 18, np);
}

void ShellDeformationModel::compute_d2E_dxdb(const DeformationModelCacheData *cacheDataBase,
  double *hess, int materialLocation) const
{
  validateMaterialLocation(materialLocation);
  const CacheData *cacheData = this->cacheData(cacheDataBase);
  if (numElasticParams_ == 0)
    return;

  ES::MXd &d2psi_da_dparam = cacheData->elasticDpsiDaDparamScratch;
  ES::MXd &d2psi_db_dparam = cacheData->elasticDpsiDbDparamScratch;
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
  elementMapping_->compute_a_and_derivatives(cacheData->x.data(), &dadx, nullptr);
  elementMapping_->compute_b_and_derivatives(cacheData->x.data(), &dbdx, nullptr);

  int np = numElasticParams_;
  ES::MXd &mixed = cacheData->mixedDerivativeScratch;
  mixed.block(0, 0, 18, np).setZero();

  mixed.block(0, 0, 9, np) +=
    dadx.transpose() * d2psi_da_dparam * cacheData->area;
  mixed.block(0, 0, 18, np) +=
    dbdx.transpose() * d2psi_db_dparam * cacheData->area;

  ES::Mp<ES::MXd>(hess, 18, np) = mixed.block(0, 0, 18, np);
}

void ShellDeformationModel::compute_dE_da(const DeformationModelCacheData *cacheDataBase,
  double *grad, int materialLocation) const
{
  validateMaterialLocation(materialLocation);
  const CacheData *cacheData = this->cacheData(cacheDataBase);
  if (numPlasticParams_ == 0)
    return;

  const double *elasticParamPtr = numElasticParams_ > 0 ? cacheData->elasticParamsValue.data() : nullptr;
  Eigen::Map<ES::VXd> gradMap(grad, numPlasticParams_);
  gradMap.setZero();

  const double *plasticParamPtr = cacheData->plasticParamsValue.data();
  const double psi = computeEnergyWithParams(*cacheData, plasticParamPtr, elasticParamPtr) / cacheData->area;

  ES::V4d dpsiDabar;
  ES::V4d dpsiDbbar;
  elastic2D_->compute_dpsi_dabar(
    elasticParamPtr, cacheData->a.data(), cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), dpsiDabar.data());
  elastic2D_->compute_dpsi_dbbar(
    elasticParamPtr, cacheData->a.data(), cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), dpsiDbbar.data());

  plastic2D_->compute_dabar_dparam(plasticParamPtr, cacheData->plasticDAbarDparamScratch.data());
  plastic2D_->compute_dbbar_dparam(plasticParamPtr, cacheData->plasticDBbarDparamScratch.data());
  plastic2D_->compute_darea_dparam(plasticParamPtr, cacheData->plasticDAreaDparamScratch.data());

  const Eigen::Map<const ES::MXd> dabarDp(
    cacheData->plasticDAbarDparamScratch.data(), 4, numPlasticParams_);
  const Eigen::Map<const ES::MXd> dbbarDp(
    cacheData->plasticDBbarDparamScratch.data(), 4, numPlasticParams_);
  for (int i = 0; i < numPlasticParams_; i++) {
    gradMap[i] = cacheData->plasticDAreaDparamScratch[i] * psi +
      cacheData->area * (dpsiDabar.dot(dabarDp.col(i)) + dpsiDbbar.dot(dbbarDp.col(i)));
  }
}

void ShellDeformationModel::compute_d2E_da2(const DeformationModelCacheData *cacheDataBase,
  double *hess, int materialLocation) const
{
  validateMaterialLocation(materialLocation);
  const CacheData *cacheData = this->cacheData(cacheDataBase);
  if (numPlasticParams_ == 0)
    return;

  Eigen::Map<ES::MXd> hessMap(hess, numPlasticParams_, numPlasticParams_);
  hessMap.setZero();
  const double *elasticParamPtr = numElasticParams_ > 0 ? cacheData->elasticParamsValue.data() : nullptr;
  const double *plasticParamPtr = cacheData->plasticParamsValue.data();
  const double psi = computeEnergyWithParams(*cacheData, plasticParamPtr, elasticParamPtr) / cacheData->area;

  ES::V4d dpsiDabar;
  ES::V4d dpsiDbbar;
  ES::M4d d2psiDabar2;
  ES::M4d d2psiDabarDbbar;
  ES::M4d d2psiDbbar2;
  elastic2D_->compute_dpsi_dabar(
    elasticParamPtr, cacheData->a.data(), cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), dpsiDabar.data());
  elastic2D_->compute_dpsi_dbbar(
    elasticParamPtr, cacheData->a.data(), cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), dpsiDbbar.data());
  elastic2D_->compute_d2psi_dabar2(
    elasticParamPtr, cacheData->a.data(), cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), d2psiDabar2.data());
  elastic2D_->compute_d2psi_dabar_dbbar(
    elasticParamPtr, cacheData->a.data(), cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), d2psiDabarDbbar.data());
  elastic2D_->compute_d2psi_dbbar2(
    elasticParamPtr, cacheData->a.data(), cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), d2psiDbbar2.data());

  plastic2D_->compute_dabar_dparam(plasticParamPtr, cacheData->plasticDAbarDparamScratch.data());
  plastic2D_->compute_dbbar_dparam(plasticParamPtr, cacheData->plasticDBbarDparamScratch.data());
  plastic2D_->compute_darea_dparam(plasticParamPtr, cacheData->plasticDAreaDparamScratch.data());

  const Eigen::Map<const ES::MXd> dabarDp(
    cacheData->plasticDAbarDparamScratch.data(), 4, numPlasticParams_);
  const Eigen::Map<const ES::MXd> dbbarDp(
    cacheData->plasticDBbarDparamScratch.data(), 4, numPlasticParams_);

  for (int i = 0; i < numPlasticParams_; i++) {
    const double dpsiDpI = dpsiDabar.dot(dabarDp.col(i)) + dpsiDbbar.dot(dbbarDp.col(i));
    for (int j = 0; j < numPlasticParams_; j++) {
      const double dpsiDpJ = dpsiDabar.dot(dabarDp.col(j)) + dpsiDbbar.dot(dbbarDp.col(j));

      ES::M2d d2abar;
      ES::M2d d2bbar;
      plastic2D_->compute_d2abar_dparam2(plasticParamPtr, i, j, d2abar.data());
      plastic2D_->compute_d2dbbar_dparam2(plasticParamPtr, i, j, d2bbar.data());
      const double d2area = plastic2D_->compute_d2area_dparam2(plasticParamPtr, i, j);

      const ES::V4d d2abarVec = ES::Mp<const ES::V4d>(d2abar.data());
      const ES::V4d d2bbarVec = ES::Mp<const ES::V4d>(d2bbar.data());
      hessMap(i, j) =
        d2area * psi +
        cacheData->plasticDAreaDparamScratch[i] * dpsiDpJ +
        cacheData->plasticDAreaDparamScratch[j] * dpsiDpI +
        cacheData->area * (
          dabarDp.col(i).dot(d2psiDabar2 * dabarDp.col(j)) +
          dabarDp.col(i).dot(d2psiDabarDbbar * dbbarDp.col(j)) +
          dbbarDp.col(i).dot(d2psiDabarDbbar.transpose() * dabarDp.col(j)) +
          dbbarDp.col(i).dot(d2psiDbbar2 * dbbarDp.col(j)) +
          dpsiDabar.dot(d2abarVec) +
          dpsiDbbar.dot(d2bbarVec));
    }
  }
}

void ShellDeformationModel::compute_dE_db(const DeformationModelCacheData *cacheDataBase,
  double *grad, int materialLocation) const
{
  validateMaterialLocation(materialLocation);
  const CacheData *cacheData = this->cacheData(cacheDataBase);
  if (numElasticParams_ == 0)
    return;

  const double *plasticParamPtr = numPlasticParams_ > 0 ? cacheData->plasticParamsValue.data() : nullptr;
  (void)plasticParamPtr;
  Eigen::Map<ES::VXd> gradMap(grad, numElasticParams_);
  gradMap.setZero();
  elastic2D_->compute_dpsi_dparam(
    cacheData->elasticParamsValue.data(), cacheData->a.data(), cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), gradMap.data());
  gradMap *= cacheData->area;
}

void ShellDeformationModel::compute_d2E_db2(const DeformationModelCacheData *cacheDataBase,
  double *hess, int materialLocation) const
{
  validateMaterialLocation(materialLocation);
  const CacheData *cacheData = this->cacheData(cacheDataBase);
  if (numElasticParams_ == 0)
    return;

  Eigen::Map<ES::MXd> hessMap(hess, numElasticParams_, numElasticParams_);
  hessMap.setZero();
  const double *plasticParamPtr = numPlasticParams_ > 0 ? cacheData->plasticParamsValue.data() : nullptr;
  (void)plasticParamPtr;
  elastic2D_->compute_d2psi_dparam2(
    cacheData->elasticParamsValue.data(), cacheData->a.data(), cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), hess);
  hessMap *= cacheData->area;
}

void ShellDeformationModel::compute_d2E_dadb(const DeformationModelCacheData *cacheDataBase,
  double *hess, int materialLocation) const
{
  validateMaterialLocation(materialLocation);
  const CacheData *cacheData = this->cacheData(cacheDataBase);
  if (numPlasticParams_ == 0 || numElasticParams_ == 0)
    return;

  Eigen::Map<ES::MXd> hessMap(hess, numPlasticParams_, numElasticParams_);
  hessMap.setZero();
  const double *elasticParamPtr = cacheData->elasticParamsValue.data();
  const double *plasticParamPtr = cacheData->plasticParamsValue.data();

  ES::VXd &dpsiDparam = cacheData->elasticDpsiDparamScratch;
  dpsiDparam.setZero();
  elastic2D_->compute_dpsi_dparam(
    elasticParamPtr, cacheData->a.data(), cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), dpsiDparam.data());

  elastic2D_->compute_d2psi_dabar_dparam(
    elasticParamPtr, cacheData->a.data(), cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), cacheData->elasticDpsiDabarDparamScratch.data());
  elastic2D_->compute_d2psi_dbbar_dparam(
    elasticParamPtr, cacheData->a.data(), cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), cacheData->elasticDpsiDbbarDparamScratch.data());
  plastic2D_->compute_dabar_dparam(plasticParamPtr, cacheData->plasticDAbarDparamScratch.data());
  plastic2D_->compute_dbbar_dparam(plasticParamPtr, cacheData->plasticDBbarDparamScratch.data());
  plastic2D_->compute_darea_dparam(plasticParamPtr, cacheData->plasticDAreaDparamScratch.data());

  const Eigen::Map<const ES::MXd> dpsiDabarDparam(
    cacheData->elasticDpsiDabarDparamScratch.data(), 4, numElasticParams_);
  const Eigen::Map<const ES::MXd> dpsiDbbarDparam(
    cacheData->elasticDpsiDbbarDparamScratch.data(), 4, numElasticParams_);
  const Eigen::Map<const ES::MXd> dabarDp(
    cacheData->plasticDAbarDparamScratch.data(), 4, numPlasticParams_);
  const Eigen::Map<const ES::MXd> dbbarDp(
    cacheData->plasticDBbarDparamScratch.data(), 4, numPlasticParams_);

  for (int i = 0; i < numPlasticParams_; i++) {
    for (int j = 0; j < numElasticParams_; j++) {
      hessMap(i, j) = cacheData->plasticDAreaDparamScratch[i] * dpsiDparam[j] +
        cacheData->area * (
          dpsiDabarDparam.col(j).dot(dabarDp.col(i)) +
          dpsiDbbarDparam.col(j).dot(dbbarDp.col(i)));
    }
  }
}

void ShellDeformationModel::vonMisesStress(
  const DeformationModelCacheData *cacheDataBase, int &nPt, double *stresses) const
{
  nPt = 0;
  if (numElasticParams_ == 0)
    return;

  const CacheData *cd = cacheData(cacheDataBase);
  double value = 0.0;
  bool ok = elastic2D_->computeVonMisesStress(
    cd->elasticParamsValue.data(),
    cd->a.data(), cd->b.data(),
    cd->abar.data(), cd->bbar.data(),
    value);

  if (!ok)
    return;

  nPt = 1;
  stresses[0] = value;
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

int ShellDeformationModel::getNumVertices() const
{
  return elementMapping_->getNumNodes();
}

int ShellDeformationModel::getNumDOFs() const
{
  return elementMapping_->getLocalDofs();
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
  PGO_ALOG(cacheDataBase != nullptr);
  PGO_ALOG(dynamic_cast<const CacheData *>(cacheDataBase) != nullptr);
  PGO_ALOG(cacheDataBase == nullptr || isCacheDataCompatible(*cacheDataBase));
  PGO_ALOG(cacheDataBase == nullptr || cacheDataBase->isPrepared());
  return static_cast<const CacheData *>(cacheDataBase);
}

ShellDeformationModel::CacheData *ShellDeformationModel::cacheData(
  DeformationModelCacheData *cacheDataBase) const
{
  PGO_ALOG(cacheDataBase != nullptr);
  PGO_ALOG(dynamic_cast<CacheData *>(cacheDataBase) != nullptr);
  PGO_ALOG(cacheDataBase == nullptr || isCacheDataCompatible(*cacheDataBase));
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

}  // namespace SolidDeformationModel
}  // namespace pgo
