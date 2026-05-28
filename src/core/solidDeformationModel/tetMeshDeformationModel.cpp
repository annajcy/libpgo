#include "tetMeshDeformationModel.h"
#include "elasticModel3DDeformationGradient.h"
#include "materialMaxStepPolynomialUtils.h"
#include "plasticModel3DDeformationGradient.h"

#include "formulations/basis/tetP1Basis.h"
#include "formulations/quadrature/tetP1DefaultQuadrature.h"
#include "formulations/kernels/deformationGradientKernel.h"
#include "formulations/elements/deformationGradientElementModel.h"

#include "geometryQuery.h"
#include "EigenSupport.h"

#include <memory>

namespace ES = pgo::EigenSupport;

namespace pgo
{
namespace SolidDeformationModel
{

using TetP1Kernel = DeformationGradientKernel<TetP1Basis, TetP1DefaultQuadrature>;
using TetP1Model = DeformationGradientElementModel<TetP1Kernel>;

class TetMeshDeformationModelInternal
{
public:
  std::unique_ptr<TetP1Model> model;
};

}  // namespace SolidDeformationModel
}  // namespace pgo

using namespace pgo::SolidDeformationModel;

void TetMeshDeformationModel::enableSPD(int enable)
{
  ind->model->enableSPD(enable);
}

TetMeshDeformationModel::TetMeshDeformationModel(
  const double X0[3], const double X1[3], const double X2[3], const double X3[3],
  ElasticModel *elasticModel, PlasticModel *plasticModel):
  DeformationModel(elasticModel, plasticModel)
{
  double restPositions[12] = {
    X0[0], X0[1], X0[2],
    X1[0], X1[1], X1[2],
    X2[0], X2[1], X2[2],
    X3[0], X3[1], X3[2],
  };

  ind = new TetMeshDeformationModelInternal;
  ind->model = std::make_unique<TetP1Model>(restPositions, elasticModel, plasticModel);
}

TetMeshDeformationModel::~TetMeshDeformationModel()
{
  delete ind;
}

DeformationModelCacheData *TetMeshDeformationModel::allocateCacheData() const
{
  return ind->model->allocateCacheData();
}

void TetMeshDeformationModel::freeCacheData(DeformationModelCacheData *d) const
{
  ind->model->freeCacheData(d);
}

void TetMeshDeformationModel::prepareData(const double *x, const double *param,
  const double *materialParam, CacheData *cacheData) const
{
  ind->model->prepareData(x, param, materialParam, cacheData);
}

double TetMeshDeformationModel::computeEnergy(const CacheData *cacheData) const
{
  return ind->model->computeEnergy(cacheData);
}

void TetMeshDeformationModel::compute_dE_dx(const CacheData *cacheData, double *grad) const
{
  ind->model->compute_dE_dx(cacheData, grad);
}

void TetMeshDeformationModel::compute_d2E_dx2(const CacheData *cacheData, double *hess) const
{
  ind->model->compute_d2E_dx2(cacheData, hess);
}

void TetMeshDeformationModel::compute_dE_da(const CacheData *cacheData, double *grad) const
{
  ind->model->compute_dE_da(cacheData, grad);
}

void TetMeshDeformationModel::compute_d2E_da2(const CacheData *cacheData, double *hess) const
{
  ind->model->compute_d2E_da2(cacheData, hess);
}

void TetMeshDeformationModel::compute_d2E_dxda(const CacheData *cacheData, double *hess) const
{
  ind->model->compute_d2E_dxda(cacheData, hess);
}

void TetMeshDeformationModel::compute_dE_db(const CacheData *cacheData, double *grad) const
{
  ind->model->compute_dE_db(cacheData, grad);
}

void TetMeshDeformationModel::compute_d2E_db2(const CacheData *cacheData, double *hess) const
{
  ind->model->compute_d2E_db2(cacheData, hess);
}

void TetMeshDeformationModel::compute_d2E_dxdb(const CacheData *cacheData, double *hess) const
{
  ind->model->compute_d2E_dxdb(cacheData, hess);
}

void TetMeshDeformationModel::compute_d2E_dadb(const CacheData *cacheData, double *hess) const
{
  ind->model->compute_d2E_dadb(cacheData, hess);
}

void TetMeshDeformationModel::vonMisesStress(const CacheData *cacheData, int &nPt, double *stresses) const
{
  ind->model->vonMisesStress(cacheData, nPt, stresses);
}

void TetMeshDeformationModel::maxStrain(const CacheData *cacheData, int &nPt, double *stresses) const
{
  ind->model->maxStrain(cacheData, nPt, stresses);
}

DeformationModel::LocalMaxStepResult TetMeshDeformationModel::computeLocalMaxStepSize(
  const double *x_local, const double *dx_local) const
{
  return ind->model->computeLocalMaxStepSize(x_local, dx_local);
}

void TetMeshDeformationModel::computeF(const double *x, int materialLocationIDs, double F[9]) const
{
  ind->model->computeF(x, materialLocationIDs, F);
}

void TetMeshDeformationModel::computeP(const CacheData *cacheDataBase, int materialLocationIDs, double POut[9]) const
{
  ind->model->computeP(cacheDataBase, materialLocationIDs, POut);
}

void TetMeshDeformationModel::computedPdF(const CacheData *cacheDataBase, int materialLocationIDs, double dPdFOut[81]) const
{
  ind->model->computedPdF(cacheDataBase, materialLocationIDs, dPdFOut);
}

void TetMeshDeformationModel::computedFdx(const CacheData *cacheDataBase, int materialLocationIDs, double *dFdxOut) const
{
  ind->model->computedFdx(cacheDataBase, materialLocationIDs, dFdxOut);
}

void TetMeshDeformationModel::computeForceFromP(const CacheData *cacheDataBase, const double P[9], double f[12]) const
{
  ind->model->computeForceFromP(cacheDataBase, 0, P, f);
}

// Static utility methods — keep legacy implementation for backward compatibility.

void TetMeshDeformationModel::computeDs(const double xIn[12], double DsOut[9])
{
  ES::M3d Ds;
  Ds.col(0) = ES::V3d(xIn[3], xIn[4], xIn[5]) - ES::V3d(xIn[0], xIn[1], xIn[2]);
  Ds.col(1) = ES::V3d(xIn[6], xIn[7], xIn[8]) - ES::V3d(xIn[0], xIn[1], xIn[2]);
  Ds.col(2) = ES::V3d(xIn[9], xIn[10], xIn[11]) - ES::V3d(xIn[0], xIn[1], xIn[2]);
  (Eigen::Map<ES::M3d>(DsOut)) = Ds;
}

void TetMeshDeformationModel::computeDm(const double XIn[12], double DmOut[9])
{
  ES::M3d Dm;
  Dm.col(0) = ES::V3d(XIn[3], XIn[4], XIn[5]) - ES::V3d(XIn[0], XIn[1], XIn[2]);
  Dm.col(1) = ES::V3d(XIn[6], XIn[7], XIn[8]) - ES::V3d(XIn[0], XIn[1], XIn[2]);
  Dm.col(2) = ES::V3d(XIn[9], XIn[10], XIn[11]) - ES::V3d(XIn[0], XIn[1], XIn[2]);
  (Eigen::Map<ES::M3d>(DmOut)) = Dm;
}

double TetMeshDeformationModel::computeVolume(const double X0[3], const double X1[3],
  const double X2[3], const double X3[3])
{
  return Mesh::getTetVolume(asVec3d(X0), asVec3d(X1), asVec3d(X2), asVec3d(X3));
}

void TetMeshDeformationModel::compute_dF_dx(const double DmInvIn[9], double dFdxOut[9 * 12])
{
  ES::M9x12d dFdx = ES::M9x12d::Zero();
  ES::M3d DmInv = Eigen::Map<const ES::M3d>(DmInvIn);

  double v0 = -(DmInv(0, 0) + DmInv(1, 0) + DmInv(2, 0));
  double v1 = -(DmInv(0, 1) + DmInv(1, 1) + DmInv(2, 1));
  double v2 = -(DmInv(0, 2) + DmInv(1, 2) + DmInv(2, 2));

  dFdx(0, 0) = v0;  dFdx(3, 0) = v1;  dFdx(6, 0) = v2;
  dFdx(1, 1) = v0;  dFdx(4, 1) = v1;  dFdx(7, 1) = v2;
  dFdx(2, 2) = v0;  dFdx(5, 2) = v1;  dFdx(8, 2) = v2;

  dFdx(0, 3) = DmInv(0, 0);  dFdx(3, 3) = DmInv(0, 1);  dFdx(6, 3) = DmInv(0, 2);
  dFdx(1, 4) = DmInv(0, 0);  dFdx(4, 4) = DmInv(0, 1);  dFdx(7, 4) = DmInv(0, 2);
  dFdx(2, 5) = DmInv(0, 0);  dFdx(5, 5) = DmInv(0, 1);  dFdx(8, 5) = DmInv(0, 2);

  dFdx(0, 6) = DmInv(1, 0);  dFdx(3, 6) = DmInv(1, 1);  dFdx(6, 6) = DmInv(1, 2);
  dFdx(1, 7) = DmInv(1, 0);  dFdx(4, 7) = DmInv(1, 1);  dFdx(7, 7) = DmInv(1, 2);
  dFdx(2, 8) = DmInv(1, 0);  dFdx(5, 8) = DmInv(1, 1);  dFdx(8, 8) = DmInv(1, 2);

  dFdx(0, 9) = DmInv(2, 0);  dFdx(3, 9) = DmInv(2, 1);  dFdx(6, 9) = DmInv(2, 2);
  dFdx(1, 10) = DmInv(2, 0); dFdx(4, 10) = DmInv(2, 1); dFdx(7, 10) = DmInv(2, 2);
  dFdx(2, 11) = DmInv(2, 0); dFdx(5, 11) = DmInv(2, 1); dFdx(8, 11) = DmInv(2, 2);

  (Eigen::Map<ES::M9x12d>(dFdxOut)) = dFdx;
}
