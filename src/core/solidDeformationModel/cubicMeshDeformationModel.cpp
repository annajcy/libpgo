#include "cubicMeshDeformationModel.h"
#include "elasticModel3DDeformationGradient.h"
#include "materialMaxStepPolynomialUtils.h"
#include "plasticModel3DDeformationGradient.h"

#include "formulations/basis/hexTrilinearBasis.h"
#include "formulations/quadrature/gaussLegendreHexQuadrature.h"
#include "formulations/kernels/deformationGradientKernel.h"
#include "formulations/elements/deformationGradientElementModel.h"

#include "EigenSupport.h"

#include <memory>
#include <stdexcept>

namespace ES = pgo::EigenSupport;

namespace pgo
{
namespace SolidDeformationModel
{

using HexTrilinearKernel = DeformationGradientKernel<HexTrilinearBasis, GaussLegendreHexQuadrature2>;
using HexTrilinearModel = DeformationGradientElementModel<HexTrilinearKernel>;

class CubicMeshDeformationModelInternal
{
public:
  std::unique_ptr<HexTrilinearModel> model;
};

}  // namespace SolidDeformationModel
}  // namespace pgo

using namespace pgo::SolidDeformationModel;

CubicMeshDeformationModel::CubicMeshDeformationModel(
  const double restPositions[24], ElasticModel *elasticModel, PlasticModel *plasticModel):
  DeformationModel(elasticModel, plasticModel)
{
  ind = new CubicMeshDeformationModelInternal;
  ind->model = std::make_unique<HexTrilinearModel>(restPositions, elasticModel, plasticModel);

  numPlasticParams_ = plasticModel ? plasticModel->getNumParameters() : 0;
  numElasticParams_ = elasticModel ? elasticModel->getNumParameters() : 0;
}

CubicMeshDeformationModel::~CubicMeshDeformationModel()
{
  delete ind;
}

void CubicMeshDeformationModel::enableSPD(int enable)
{
  ind->model->enableSPD(enable);
}

DeformationModelCacheData *CubicMeshDeformationModel::allocateCacheData() const
{
  return ind->model->allocateCacheData();
}

void CubicMeshDeformationModel::freeCacheData(DeformationModelCacheData *data) const
{
  ind->model->freeCacheData(data);
}

void CubicMeshDeformationModel::prepareData(const double *x, const double *param,
  const double *materialParam, CacheData *cacheData) const
{
  ind->model->prepareData(x, param, materialParam, cacheData);
}

double CubicMeshDeformationModel::computeEnergy(const CacheData *cacheData) const
{
  return ind->model->computeEnergy(cacheData);
}

void CubicMeshDeformationModel::compute_dE_dx(const CacheData *cacheData, double *grad) const
{
  ind->model->compute_dE_dx(cacheData, grad);
}

void CubicMeshDeformationModel::compute_d2E_dx2(const CacheData *cacheData, double *hess) const
{
  ind->model->compute_d2E_dx2(cacheData, hess);
}

void CubicMeshDeformationModel::compute_dE_da(const CacheData *cacheData, double *grad) const
{
  ind->model->compute_dE_da(cacheData, grad);
}

void CubicMeshDeformationModel::compute_d2E_da2(const CacheData *cacheData, double *hess) const
{
  ind->model->compute_d2E_da2(cacheData, hess);
}

void CubicMeshDeformationModel::compute_d2E_dxda(const CacheData *cacheData, double *hess) const
{
  ind->model->compute_d2E_dxda(cacheData, hess);
}

void CubicMeshDeformationModel::compute_dE_db(const CacheData *cacheData, double *grad) const
{
  ind->model->compute_dE_db(cacheData, grad);
}

void CubicMeshDeformationModel::compute_d2E_db2(const CacheData *cacheData, double *hess) const
{
  ind->model->compute_d2E_db2(cacheData, hess);
}

void CubicMeshDeformationModel::compute_d2E_dxdb(const CacheData *cacheData, double *hess) const
{
  ind->model->compute_d2E_dxdb(cacheData, hess);
}

void CubicMeshDeformationModel::compute_d2E_dadb(const CacheData *cacheData, double *hess) const
{
  ind->model->compute_d2E_dadb(cacheData, hess);
}

void CubicMeshDeformationModel::vonMisesStress(const CacheData *cacheData, int &nPt, double *stresses) const
{
  ind->model->vonMisesStress(cacheData, nPt, stresses);
}

void CubicMeshDeformationModel::maxStrain(const CacheData *cacheData, int &nPt, double *stresses) const
{
  ind->model->maxStrain(cacheData, nPt, stresses);
}

DeformationModel::LocalMaxStepResult CubicMeshDeformationModel::computeLocalMaxStepSize(
  const double *x_local, const double *dx_local) const
{
  return ind->model->computeLocalMaxStepSize(x_local, dx_local);
}

void CubicMeshDeformationModel::computeF(const double *x, int materialLocationID, double F[9]) const
{
  ind->model->computeF(x, materialLocationID, F);
}

void CubicMeshDeformationModel::computeFe(const CacheData *cacheData, int materialLocationID, double F[9]) const
{
  ind->model->computeFe(cacheData, materialLocationID, F);
}

void CubicMeshDeformationModel::computeP(const CacheData *cacheData, int materialLocationID, double P[9]) const
{
  ind->model->computeP(cacheData, materialLocationID, P);
}

void CubicMeshDeformationModel::computedPdF(const CacheData *cacheData, int materialLocationID, double dPdF[81]) const
{
  ind->model->computedPdF(cacheData, materialLocationID, dPdF);
}

void CubicMeshDeformationModel::computedFdx(const CacheData *cacheData, int materialLocationID, double *dFdx) const
{
  ind->model->computedFdx(cacheData, materialLocationID, dFdx);
}

void CubicMeshDeformationModel::computeForceFromP(const CacheData *cacheData, int materialLocationID, const double P[9], double f[24]) const
{
  ind->model->computeForceFromP(cacheData, materialLocationID, P, f);
}

double CubicMeshDeformationModel::getWeightDetJ(int materialLocationID) const
{
  return ind->model->kernel().weightDetJ(materialLocationID);
}
