#include "gtest/gtest.h"

#include "koiterDeformationModel.h"
#include "elasticModel2DFundamentalFormsSTVK.h"
#include "plasticModel2DFundamentalFormsUniformStretch.h"

#include "formulations/elements/koiterShellElementModel.h"

#include <cmath>

namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

namespace
{

// Interior triangle: all 6 nodes present, slightly curved out of plane.
const double interiorRestX[18] = {
  0.0, 0.0, 0.0,
  2.0, 0.0, 0.5,
  1.0, 1.5, -0.3,
  -0.5, -0.2, 1.0,  // node 3
  2.5, -0.1, 0.8,   // node 4
  1.2, 2.0, 1.2,    // node 5
};

// Same geometry but node 4 marked as missing via -10496 sentinel.
const double boundaryRestX[18] = {
  0.0, 0.0, 0.0,
  2.0, 0.0, 0.5,
  1.0, 1.5, -0.3,
  -0.5, -0.2, 1.0,
  -10496.0, -10496.0, -10496.0,  // node 4 missing
  1.2, 2.0, 1.2,
};

void perturbedDisplacement(double *x, const double *rest, int n, double scale)
{
  for (int i = 0; i < n; i++) {
    x[i] = rest[i] + scale * 0.05 * std::sin(0.7 * static_cast<double>(i) + 0.3);
  }
}

void setupElasticParams(double *params)
{
  params[0] = 1e5;   // E
  params[1] = 0.3;   // nu
  params[2] = 1e4;   // E_bend
  params[3] = 0.3;   // nu_bend
  params[4] = 0.01;  // h
}

}  // namespace

// ============================================================
// Parity test — interior triangle (all nodes present)
// ============================================================

TEST(KoiterShellElementModelParityTest, InteriorTriangleMatchesOracle)
{
  ElasticModel2DFundamentalFormsSTVK elasticModel;
  PlasticModel2DFundamentalFormsUniformStretch plasticModel;

  const bool hasVtx[6] = { true, true, true, true, true, true };

  KoiterDeformationModel oracle(
    interiorRestX, interiorRestX + 3, interiorRestX + 6,
    interiorRestX + 9, interiorRestX + 12, interiorRestX + 15,
    &elasticModel, &plasticModel);

  KoiterShellElementModel newModel(
    interiorRestX, hasVtx, &elasticModel, &plasticModel);

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX, 18, 0.1);

  double matParam[5] = {};
  setupElasticParams(matParam);
  double plasticParam[1] = {};

  auto *cdOld = oracle.allocateCacheData();
  oracle.prepareData(x, plasticParam, matParam, cdOld);

  auto *cdNew = newModel.allocateCacheData();
  newModel.prepareData(x, plasticParam, matParam, cdNew);

  // Energy
  double eOld = oracle.computeEnergy(cdOld);
  double eNew = newModel.computeEnergy(cdNew);
  EXPECT_NEAR(eOld, eNew, 1e-10);

  // Gradient
  ES::V18d gOld, gNew;
  oracle.compute_dE_dx(cdOld, gOld.data());
  newModel.compute_dE_dx(cdNew, gNew.data());
  for (int i = 0; i < 18; i++)
    EXPECT_NEAR(gOld[i], gNew[i], 1e-10) << "gradient mismatch at index " << i;

  // Hessian
  ES::M18d hOld, hNew;
  oracle.compute_d2E_dx2(cdOld, hOld.data());
  newModel.compute_d2E_dx2(cdNew, hNew.data());
  for (int i = 0; i < 18; i++)
    for (int j = 0; j < 18; j++)
      EXPECT_NEAR(hOld(i, j), hNew(i, j), 1e-10)
        << "hessian mismatch at (" << i << ", " << j << ")";

  // SPD enable
  oracle.enableSPD(1);
  newModel.enableSPD(1);
  oracle.compute_d2E_dx2(cdOld, hOld.data());
  newModel.compute_d2E_dx2(cdNew, hNew.data());
  for (int i = 0; i < 18; i++)
    for (int j = 0; j < 18; j++)
      EXPECT_NEAR(hOld(i, j), hNew(i, j), 1e-10)
        << "SPD hessian mismatch at (" << i << ", " << j << ")";

  // d2E/dxda
  int np = plasticModel.getNumParameters();
  ES::MXd hxdaOld(18, np), hxdaNew(18, np);
  oracle.compute_d2E_dxda(cdOld, hxdaOld.data());
  newModel.compute_d2E_dxda(cdNew, hxdaNew.data());
  for (int i = 0; i < 18; i++)
    for (int j = 0; j < np; j++)
      EXPECT_NEAR(hxdaOld(i, j), hxdaNew(i, j), 1e-10)
        << "d2E/dxda mismatch at (" << i << ", " << j << ")";

  // d2E/dxdb
  int ne = elasticModel.getNumParameters();
  ES::MXd hxdbOld(18, ne), hxdbNew(18, ne);
  oracle.compute_d2E_dxdb(cdOld, hxdbOld.data());
  newModel.compute_d2E_dxdb(cdNew, hxdbNew.data());
  for (int i = 0; i < 18; i++)
    for (int j = 0; j < ne; j++)
      EXPECT_NEAR(hxdbOld(i, j), hxdbNew(i, j), 1e-10)
        << "d2E/dxdb mismatch at (" << i << ", " << j << ")";

  oracle.freeCacheData(cdOld);
  newModel.freeCacheData(cdNew);
}

TEST(KoiterShellElementModelParityTest, InteriorZeroDisplacementMatchesOracle)
{
  ElasticModel2DFundamentalFormsSTVK elasticModel;
  PlasticModel2DFundamentalFormsUniformStretch plasticModel;

  const bool hasVtx[6] = { true, true, true, true, true, true };

  KoiterDeformationModel oracle(
    interiorRestX, interiorRestX + 3, interiorRestX + 6,
    interiorRestX + 9, interiorRestX + 12, interiorRestX + 15,
    &elasticModel, &plasticModel);

  KoiterShellElementModel newModel(
    interiorRestX, hasVtx, &elasticModel, &plasticModel);

  double matParam[5] = {};
  setupElasticParams(matParam);
  double plasticParam[1] = {};

  auto *cdOld = oracle.allocateCacheData();
  oracle.prepareData(interiorRestX, plasticParam, matParam, cdOld);
  auto *cdNew = newModel.allocateCacheData();
  newModel.prepareData(interiorRestX, plasticParam, matParam, cdNew);

  double eOld = oracle.computeEnergy(cdOld);
  double eNew = newModel.computeEnergy(cdNew);
  EXPECT_NEAR(eOld, eNew, 1e-10);

  ES::V18d gOld, gNew;
  oracle.compute_dE_dx(cdOld, gOld.data());
  newModel.compute_dE_dx(cdNew, gNew.data());
  for (int i = 0; i < 18; i++)
    EXPECT_NEAR(gOld[i], gNew[i], 1e-10) << "gradient mismatch at index " << i;

  oracle.freeCacheData(cdOld);
  newModel.freeCacheData(cdNew);
}

// ============================================================
// Boundary triangle — node 4 missing
// ============================================================

TEST(KoiterShellElementModelParityTest, BoundaryMissingNode4MatchesOracle)
{
  ElasticModel2DFundamentalFormsSTVK elasticModel;
  PlasticModel2DFundamentalFormsUniformStretch plasticModel;

  // Oracle uses -10496 sentinel in position data to detect missing node.
  KoiterDeformationModel oracle(
    boundaryRestX, boundaryRestX + 3, boundaryRestX + 6,
    boundaryRestX + 9, boundaryRestX + 12, boundaryRestX + 15,
    &elasticModel, &plasticModel);

  // New model uses explicit hasVtx mask; position data for node 4 is unused.
  const bool hasVtx[6] = { true, true, true, true, false, true };
  KoiterShellElementModel newModel(
    interiorRestX, hasVtx, &elasticModel, &plasticModel);

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX, 18, 0.1);

  double matParam[5] = {};
  setupElasticParams(matParam);
  double plasticParam[1] = {};

  auto *cdOld = oracle.allocateCacheData();
  oracle.prepareData(x, plasticParam, matParam, cdOld);
  auto *cdNew = newModel.allocateCacheData();
  newModel.prepareData(x, plasticParam, matParam, cdNew);

  double eOld = oracle.computeEnergy(cdOld);
  double eNew = newModel.computeEnergy(cdNew);
  EXPECT_NEAR(eOld, eNew, 1e-10);

  ES::V18d gOld, gNew;
  oracle.compute_dE_dx(cdOld, gOld.data());
  newModel.compute_dE_dx(cdNew, gNew.data());
  for (int i = 0; i < 18; i++)
    EXPECT_NEAR(gOld[i], gNew[i], 1e-10) << "gradient mismatch at index " << i;

  ES::M18d hOld, hNew;
  oracle.compute_d2E_dx2(cdOld, hOld.data());
  newModel.compute_d2E_dx2(cdNew, hNew.data());
  for (int i = 0; i < 18; i++)
    for (int j = 0; j < 18; j++)
      EXPECT_NEAR(hOld(i, j), hNew(i, j), 1e-10)
        << "hessian mismatch at (" << i << ", " << j << ")";

  oracle.freeCacheData(cdOld);
  newModel.freeCacheData(cdNew);
}

// ============================================================
// FD sanity check
// ============================================================

TEST(KoiterShellElementModelFDTest, GradientMatchesFiniteDifference)
{
  ElasticModel2DFundamentalFormsSTVK elasticModel;
  PlasticModel2DFundamentalFormsUniformStretch plasticModel;

  const bool hasVtx[6] = { true, true, true, true, true, true };
  KoiterShellElementModel model(
    interiorRestX, hasVtx, &elasticModel, &plasticModel);

  double matParam[5] = {};
  setupElasticParams(matParam);
  double plasticParam[1] = {};

  auto *cd = model.allocateCacheData();

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX, 18, 0.1);
  model.prepareData(x, plasticParam, matParam, cd);
  double e0 = model.computeEnergy(cd);

  ES::V18d g;
  model.compute_dE_dx(cd, g.data());

  const double eps = 1e-6;
  for (int i = 0; i < 18; i++) {
    double xPlus[18];
    std::copy(x, x + 18, xPlus);
    xPlus[i] += eps;

    auto *cdP = model.allocateCacheData();
    model.prepareData(xPlus, plasticParam, matParam, cdP);
    double ePlus = model.computeEnergy(cdP);
    model.freeCacheData(cdP);

    double fdGrad = (ePlus - e0) / eps;
    EXPECT_NEAR(fdGrad, g[i], 1e-5) << "FD gradient mismatch at index " << i;
  }

  model.freeCacheData(cd);
}
