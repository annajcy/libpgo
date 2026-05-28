#include "gtest/gtest.h"

#include "tetMeshDeformationModel.h"
#include "cubicMeshDeformationModel.h"
#include "elasticModelStableNeoHookeanMaterial.h"
#include "plasticModel3DConstant.h"
#include "plasticModel3D3DOF.h"

#include "formulations/basis/tetP1Basis.h"
#include "formulations/basis/hexTrilinearBasis.h"
#include "formulations/quadrature/tetP1DefaultQuadrature.h"
#include "formulations/quadrature/gaussLegendreHexQuadrature.h"
#include "formulations/kernels/deformationGradientKernel.h"
#include "formulations/elements/deformationGradientElementModel.h"

#include "EigenSupport.h"

#include <memory>

namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

using TetKernel = DeformationGradientKernel<TetP1Basis, TetP1DefaultQuadrature>;
using TetNewModel = DeformationGradientElementModel<TetKernel>;

using HexKernel = DeformationGradientKernel<HexTrilinearBasis, GaussLegendreHexQuadrature2>;
using HexNewModel = DeformationGradientElementModel<HexKernel>;

namespace
{
const double restTet[12] = {
  0.0, 0.0, 0.0,
  2.0, 0.0, 0.0,
  0.0, 3.0, 0.0,
  0.0, 0.0, 4.0,
};

const double restHex[24] = {
  0.0, 0.0, 0.0,  1.5, 0.0, 0.0,  1.5, 2.0, 0.0,  0.0, 2.0, 0.0,
  0.0, 0.0, 3.0,  1.5, 0.0, 3.0,  1.5, 2.0, 3.0,  0.0, 2.0, 3.0,
};

void makeIdentity3x3(double m[9])
{
  for (int i = 0; i < 9; i++) m[i] = (i % 4 == 0) ? 1.0 : 0.0;
}
}  // namespace

// ============================================================
// Tet: new element model matches old
// ============================================================

TEST(DeformationGradientElementModelGTest, TetNewModelMatchesOld)
{
  ElasticModelStableNeoHookeanMaterial elasticModel(1200.0, 1800.0);
  double identity[9];
  makeIdentity3x3(identity);
  PlasticModel3DConstant plasticModel(identity);

  TetMeshDeformationModel oldModel(
    restTet, restTet + 3, restTet + 6, restTet + 9,
    &elasticModel, &plasticModel);
  TetNewModel newModel(restTet, &elasticModel, &plasticModel);

  ES::V12d xVec;
  for (int i = 0; i < 12; i++) {
    xVec[i] = restTet[i] + 0.1 * std::sin(0.7 * static_cast<double>(i));
  }

  const double materialParam[12] = {};
  const double plasticParam[1] = {};

  DeformationModelCacheData *oldCD = oldModel.allocateCacheData();
  oldModel.prepareData(xVec.data(), plasticParam, materialParam, oldCD);

  DeformationModelCacheData *newCD = newModel.allocateCacheData();
  newModel.prepareData(xVec.data(), plasticParam, materialParam, newCD);

  double eOld = oldModel.computeEnergy(oldCD);
  double eNew = newModel.computeEnergy(newCD);
  EXPECT_NEAR(eNew, eOld, 1e-12);

  ES::V12d gOld, gNew;
  oldModel.compute_dE_dx(oldCD, gOld.data());
  newModel.compute_dE_dx(newCD, gNew.data());
  EXPECT_LT((gNew - gOld).cwiseAbs().maxCoeff(), 1e-12);

  ES::M12d hOld, hNew;
  oldModel.compute_d2E_dx2(oldCD, hOld.data());
  newModel.compute_d2E_dx2(newCD, hNew.data());
  EXPECT_LT((hNew - hOld).cwiseAbs().maxCoeff(), 1e-12);

  oldModel.freeCacheData(oldCD);
  newModel.freeCacheData(newCD);
}

// ============================================================
// Hex: new element model matches old
// ============================================================

TEST(DeformationGradientElementModelGTest, HexNewModelMatchesOld)
{
  ElasticModelStableNeoHookeanMaterial elasticModel(1200.0, 1800.0);
  double identity[9];
  makeIdentity3x3(identity);
  PlasticModel3DConstant plasticModel(identity);

  CubicMeshDeformationModel oldModel(restHex, &elasticModel, &plasticModel);
  HexNewModel newModel(restHex, &elasticModel, &plasticModel);

  ES::V24d xVec;
  for (int i = 0; i < 24; i++) {
    xVec[i] = restHex[i] + 0.05 * std::sin(0.7 * static_cast<double>(i));
  }

  const double materialParam[12] = {};
  const double plasticParam[1] = {};

  DeformationModelCacheData *oldCD = oldModel.allocateCacheData();
  oldModel.prepareData(xVec.data(), plasticParam, materialParam, oldCD);

  DeformationModelCacheData *newCD = newModel.allocateCacheData();
  newModel.prepareData(xVec.data(), plasticParam, materialParam, newCD);

  double eOld = oldModel.computeEnergy(oldCD);
  double eNew = newModel.computeEnergy(newCD);
  EXPECT_NEAR(eNew, eOld, 1e-12);

  ES::V24d gOld, gNew;
  oldModel.compute_dE_dx(oldCD, gOld.data());
  newModel.compute_dE_dx(newCD, gNew.data());
  EXPECT_LT((gNew - gOld).cwiseAbs().maxCoeff(), 1e-12);

  ES::M24d hOld, hNew;
  oldModel.compute_d2E_dx2(oldCD, hOld.data());
  newModel.compute_d2E_dx2(newCD, hNew.data());
  EXPECT_LT((hNew - hOld).cwiseAbs().maxCoeff(), 1e-12);

  oldModel.freeCacheData(oldCD);
  newModel.freeCacheData(newCD);
}

// ============================================================
// Tet: plastic parameter derivative regression
// ============================================================

TEST(DeformationGradientElementModelGTest, TetPlasticDerivativesMatchOld)
{
  ElasticModelStableNeoHookeanMaterial elasticModel(1200.0, 1800.0);
  double identity[9];
  makeIdentity3x3(identity);
  PlasticModel3D3DOF plasticModel(identity);

  TetMeshDeformationModel oldModel(
    restTet, restTet + 3, restTet + 6, restTet + 9,
    &elasticModel, &plasticModel);
  TetNewModel newModel(restTet, &elasticModel, &plasticModel);

  ES::V12d xVec;
  for (int i = 0; i < 12; i++) {
    xVec[i] = restTet[i] + 0.05 * std::sin(0.7 * static_cast<double>(i));
  }

  const double materialParam[12] = {};
  double plasticParam[3] = { 0.1, -0.05, 0.2 };

  DeformationModelCacheData *oldCD = oldModel.allocateCacheData();
  oldModel.prepareData(xVec.data(), plasticParam, materialParam, oldCD);

  DeformationModelCacheData *newCD = newModel.allocateCacheData();
  newModel.prepareData(xVec.data(), plasticParam, materialParam, newCD);

  ES::V3d gOld_a = ES::V3d::Zero();
  ES::V3d gNew_a = ES::V3d::Zero();
  oldModel.compute_dE_da(oldCD, gOld_a.data());
  newModel.compute_dE_da(newCD, gNew_a.data());
  EXPECT_LT((gNew_a - gOld_a).cwiseAbs().maxCoeff(), 1e-12);

  ES::M3d hOld_a = ES::M3d::Zero();
  ES::M3d hNew_a = ES::M3d::Zero();
  oldModel.compute_d2E_da2(oldCD, hOld_a.data());
  newModel.compute_d2E_da2(newCD, hNew_a.data());
  EXPECT_LT((hNew_a - hOld_a).cwiseAbs().maxCoeff(), 1e-12);

  const int np = plasticModel.getNumParameters();
  ES::VXd mixedOld = ES::VXd::Zero(12 * np);
  ES::VXd mixedNew = ES::VXd::Zero(12 * np);
  oldModel.compute_d2E_dxda(oldCD, mixedOld.data());
  newModel.compute_d2E_dxda(newCD, mixedNew.data());
  EXPECT_LT((mixedNew - mixedOld).cwiseAbs().maxCoeff(), 1e-12);

  oldModel.freeCacheData(oldCD);
  newModel.freeCacheData(newCD);
}

// ============================================================
// Hex: material max-step
// ============================================================

TEST(DeformationGradientElementModelGTest, HexNewModelMaxStepMatchesOld)
{
  ElasticModelStableNeoHookeanMaterial elasticModel(1200.0, 1800.0);
  double identity[9];
  makeIdentity3x3(identity);
  PlasticModel3DConstant plasticModel(identity);

  CubicMeshDeformationModel oldModel(restHex, &elasticModel, &plasticModel);
  HexNewModel newModel(restHex, &elasticModel, &plasticModel);

  ES::V24d xVec;
  for (int i = 0; i < 24; i++) {
    xVec[i] = restHex[i];
  }
  ES::V24d dx = ES::V24d::Zero();
  dx[0] = 0.1;

  auto oldResult = oldModel.computeLocalMaxStepSize(xVec.data(), dx.data());
  auto newResult = newModel.computeLocalMaxStepSize(xVec.data(), dx.data());

  // Results should be identical.
  EXPECT_DOUBLE_EQ(oldResult.alpha, newResult.alpha);
  EXPECT_EQ(oldResult.illegalInitialState, newResult.illegalInitialState);
}
