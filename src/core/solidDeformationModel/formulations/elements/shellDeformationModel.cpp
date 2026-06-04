#include "shellDeformationModel.h"
#include "../parameters/parameterField.h"

#include "pgoLogging.h"

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

ShellDeformationModel::ShellDeformationModel(
  int ele, std::unique_ptr<ShellKernel> kernel,
  std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel,
  const ParameterField *elasticParams, const ParameterField *plasticParams)
  : DeformationModel(std::move(elasticModel), std::move(plasticModel), elasticParams, plasticParams)
  , kernel_(std::move(kernel))
  , ele_(ele)
{
  elasticModel_ = dynamic_cast<ElasticModel2DFundamentalForms *>(getElasticModel());
  if (!elasticModel_) {
    throw std::logic_error(
      "ShellDeformationModel requires ElasticModel2DFundamentalForms");
  }
  plasticModel_ = dynamic_cast<PlasticModel2DFundamentalForms *>(getPlasticModel());
  if (!plasticModel_) {
    throw std::logic_error(
      "ShellDeformationModel requires PlasticModel2DFundamentalForms");
  }

  plasticModel_->set_abar(kernel_->restI());
  plasticModel_->set_bbar(kernel_->restII());
  plasticModel_->setArea(kernel_->restArea());
}

std::unique_ptr<DeformationModelCacheData> ShellDeformationModel::allocateCacheData() const
{
  return std::make_unique<CacheData>();
}

void ShellDeformationModel::prepareData(
  const double *x,
  DeformationModelCacheData *cacheDataBase) const
{
  PGO_ALOG(dynamic_cast<CacheData *>(cacheDataBase) != nullptr);
  CacheData *cacheData = static_cast<CacheData *>(cacheDataBase);
  cacheData->x[0] = ES::V3d(x[0], x[1], x[2]);
  cacheData->x[1] = ES::V3d(x[3], x[4], x[5]);
  cacheData->x[2] = ES::V3d(x[6], x[7], x[8]);
  cacheData->x[3] = ES::V3d(x[9], x[10], x[11]);
  cacheData->x[4] = ES::V3d(x[12], x[13], x[14]);
  cacheData->x[5] = ES::V3d(x[15], x[16], x[17]);

  const int numPlasticParams = getPlasticModel()->getNumParameters();
  const int numElasticParams = getElasticModel()->getNumParameters();

  if (plasticParams() && numPlasticParams > 0) {
    plasticParams()->computeValue(ele_, 0, cacheData->plasticParams.data());
    plasticModel_->compute_abar(cacheData->plasticParams.data(), cacheData->abar.data());
    plasticModel_->compute_bbar(cacheData->plasticParams.data(), cacheData->bbar.data());
    cacheData->area = plasticModel_->computeArea(cacheData->plasticParams.data());
  }

  if (elasticParams() && numElasticParams > 0) {
    elasticParams()->computeValue(ele_, 0, cacheData->elasticParams.data());
  }

  cacheData->a = kernel_->compute_a_and_derivatives(cacheData->x, nullptr, nullptr);
  cacheData->b = kernel_->compute_b_and_derivatives(cacheData->x, nullptr, nullptr);
}

double ShellDeformationModel::computeEnergy(const DeformationModelCacheData *cacheDataBase) const
{
  PGO_ALOG(dynamic_cast<const CacheData *>(cacheDataBase) != nullptr);
  const CacheData *cacheData = static_cast<const CacheData *>(cacheDataBase);

  double E1 = elasticModel_->compute_psi_a(
    cacheData->elasticParams.data(), cacheData->a.data(),
    cacheData->abar.data());
  double E2 = elasticModel_->compute_psi_b(
    cacheData->elasticParams.data(), cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data());

  return (E1 + E2) * cacheData->area;
}

void ShellDeformationModel::compute_dE_dx(const DeformationModelCacheData *cacheDataBase,
  double *grad) const
{
  PGO_ALOG(dynamic_cast<const CacheData *>(cacheDataBase) != nullptr);
  const CacheData *cacheData = static_cast<const CacheData *>(cacheDataBase);

  ES::M4x9d dadx;
  ES::M4x18d dbdx;

  kernel_->compute_a_and_derivatives(cacheData->x, &dadx, nullptr);
  kernel_->compute_b_and_derivatives(cacheData->x, &dbdx, nullptr);

  ES::M2d dEda, dEdb;
  elasticModel_->compute_dpsi_da(
    cacheData->elasticParams.data(), cacheData->a.data(),
    cacheData->abar.data(), dEda.data());
  elasticModel_->compute_dpsi_db(
    cacheData->elasticParams.data(), cacheData->b.data(),
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
  PGO_ALOG(dynamic_cast<const CacheData *>(cacheDataBase) != nullptr);
  const CacheData *cacheData = static_cast<const CacheData *>(cacheDataBase);

  ES::M4x9d dadx;
  ES::M4x18d dbdx;

  ES::M9d d2adx2[4];
  ES::M18d d2bdx2[4];

  kernel_->compute_a_and_derivatives(cacheData->x, &dadx, d2adx2);
  kernel_->compute_b_and_derivatives(cacheData->x, &dbdx, d2bdx2);

  ES::M2d dEda, dEdb;
  elasticModel_->compute_dpsi_da(
    cacheData->elasticParams.data(), cacheData->a.data(),
    cacheData->abar.data(), dEda.data());
  elasticModel_->compute_dpsi_db(
    cacheData->elasticParams.data(), cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), dEdb.data());

  ES::M4d d2Eda2, d2Edb2;
  elasticModel_->compute_d2psi_da2(
    cacheData->elasticParams.data(), cacheData->a.data(),
    cacheData->abar.data(), d2Eda2.data());
  elasticModel_->compute_d2psi_db2(
    cacheData->elasticParams.data(), cacheData->b.data(),
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
  PGO_ALOG(dynamic_cast<const CacheData *>(cacheDataBase) != nullptr);
  const CacheData *cacheData = static_cast<const CacheData *>(cacheDataBase);
  ES::V4d dpsi_da, dpsi_db;
  elasticModel_->compute_dpsi_da(
    cacheData->elasticParams.data(), cacheData->a.data(),
    cacheData->abar.data(), dpsi_da.data());
  elasticModel_->compute_dpsi_db(
    cacheData->elasticParams.data(), cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), dpsi_db.data());

  ES::M4d d2psi_da_dabar, d2psi_db_dabar, d2psi_db_dbbar;
  elasticModel_->compute_d2psi_dadabar(
    cacheData->elasticParams.data(), cacheData->a.data(),
    cacheData->abar.data(), d2psi_da_dabar.data());
  elasticModel_->compute_d2psi_db_dabar(
    cacheData->elasticParams.data(), cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), d2psi_db_dabar.data());
  elasticModel_->compute_d2psi_db_dbbar(
    cacheData->elasticParams.data(), cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), d2psi_db_dbbar.data());

  ES::M4x9d dadx;
  ES::M4x18d dbdx;
  kernel_->compute_a_and_derivatives(cacheData->x, &dadx, nullptr);
  kernel_->compute_b_and_derivatives(cacheData->x, &dbdx, nullptr);

  ES::M4x18d dabar_dF;
  ES::M4x18d dbbar_dF;
  ES::V18d darea_dF;
  plasticModel_->compute_dabar_dparam(
    cacheData->plasticParams.data(), dabar_dF.data());
  plasticModel_->compute_dbbar_dparam(
    cacheData->plasticParams.data(), dbbar_dF.data());
  plasticModel_->compute_darea_dparam(
    cacheData->plasticParams.data(), darea_dF.data());

  ES::V9d dpsi_a_dx = dpsi_da.transpose() * dadx;
  ES::V18d dpsi_b_dx = dpsi_db.transpose() * dbdx;

  int np = getPlasticModel()->getNumParameters();
  ES::Mp<ES::MXd> hessMap(hess, 18, np);
  hessMap.setZero();

  hessMap.block(0, 0, 9, np) +=
    dadx.transpose() * d2psi_da_dabar * dabar_dF.leftCols(np) * cacheData->area;
  hessMap.block(0, 0, 9, np) +=
    dpsi_a_dx * darea_dF.head(np).transpose();

  hessMap.block(0, 0, 18, np) +=
    dbdx.transpose() * d2psi_db_dabar * dabar_dF.leftCols(np) * cacheData->area;
  hessMap.block(0, 0, 18, np) +=
    dbdx.transpose() * d2psi_db_dbbar * dbbar_dF.leftCols(np) * cacheData->area;
  hessMap.block(0, 0, 18, np) +=
    dpsi_b_dx * darea_dF.head(np).transpose();
}

void ShellDeformationModel::compute_d2E_dxdb(const DeformationModelCacheData *cacheDataBase,
  double *hess) const
{
  PGO_ALOG(dynamic_cast<const CacheData *>(cacheDataBase) != nullptr);
  const CacheData *cacheData = static_cast<const CacheData *>(cacheDataBase);

  ES::M4x18d d2psi_da_dparam, d2psi_db_dparam;
  d2psi_da_dparam.setZero();
  d2psi_db_dparam.setZero();
  elasticModel_->compute_d2psi_da_dparam(
    cacheData->elasticParams.data(), cacheData->a.data(),
    cacheData->abar.data(), d2psi_da_dparam.data());
  elasticModel_->compute_d2psi_db_dparam(
    cacheData->elasticParams.data(), cacheData->b.data(),
    cacheData->abar.data(), cacheData->bbar.data(), d2psi_db_dparam.data());

  ES::M4x9d dadx;
  ES::M4x18d dbdx;
  kernel_->compute_a_and_derivatives(cacheData->x, &dadx, nullptr);
  kernel_->compute_b_and_derivatives(cacheData->x, &dbdx, nullptr);

  int np = getElasticModel()->getNumParameters();
  ES::Mp<ES::MXd> hessMap(hess, 18, np);
  hessMap.setZero();

  hessMap.block(0, 0, 9, np) +=
    dadx.transpose() * d2psi_da_dparam.leftCols(np) * cacheData->area;
  hessMap.block(0, 0, 18, np) +=
    dbdx.transpose() * d2psi_db_dparam.leftCols(np) * cacheData->area;
}

void ShellDeformationModel::enableSPD(int enable)
{
  enableSPD_ = enable;
  DeformationModel::enableSPD(enable);
}

int ShellDeformationModel::getNumVertices() const
{
  return kernel_->getNumNodes();
}

int ShellDeformationModel::getNumDOFs() const
{
  return kernel_->getLocalDofs();
}

DeformationModel::LocalMaxStepResult ShellDeformationModel::computeLocalMaxStepSize(
  const double *x_local, const double *dx_local) const
{
  (void)x_local;
  (void)dx_local;
  return LocalMaxStepResult{};
}

}  // namespace SolidDeformationModel
}  // namespace pgo
