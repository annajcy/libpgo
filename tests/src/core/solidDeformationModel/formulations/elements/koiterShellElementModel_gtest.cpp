#include "gtest/gtest.h"

#include "elasticModel2DFundamentalFormsSTVK.h"
#include "plasticModel2DFundamentalFormsUniformStretch.h"

#include "formulations/elements/koiterShellElementModel.h"

#include <cmath>
#include <algorithm>

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

void perturbedDisplacement(double *x, const double *rest, int n, double scale)
{
  for (int i = 0; i < n; i++) {
    x[i] = rest[i] + scale * 0.05 * std::sin(0.7 * static_cast<double>(i) + 0.3);
  }
}

}  // namespace

// ============================================================
// Interior triangle: energy finite at rest and gradient FD check
// ============================================================

TEST(KoiterShellElementModelTest, InteriorEnergyFiniteAtRest)
{
  ElasticModel2DFundamentalFormsSTVK elasticModel;
  PlasticModel2DFundamentalFormsUniformStretch plasticModel;
  const bool hasVtx[6] = { true, true, true, true, true, true };

  KoiterShellElementModel model(interiorRestX, hasVtx, &elasticModel, &plasticModel);

  auto cd = model.allocateCacheData();
  model.prepareData(interiorRestX, cd.get());

  double energy = model.computeEnergy(cd.get());
  EXPECT_TRUE(std::isfinite(energy));

  ES::V18d grad;
  model.compute_dE_dx(cd.get(), grad.data());
  for (int i = 0; i < 18; i++)
    EXPECT_TRUE(std::isfinite(grad[i]));
}

// ============================================================
// Boundary triangle (node 4 missing): energy finite at rest
// ============================================================

TEST(KoiterShellElementModelTest, BoundaryMissingNode4EnergyFinite)
{
  ElasticModel2DFundamentalFormsSTVK elasticModel;
  PlasticModel2DFundamentalFormsUniformStretch plasticModel;
  const bool hasVtx[6] = { true, true, true, true, false, true };

  KoiterShellElementModel model(interiorRestX, hasVtx, &elasticModel, &plasticModel);

  auto cd = model.allocateCacheData();
  model.prepareData(interiorRestX, cd.get());

  double energy = model.computeEnergy(cd.get());
  EXPECT_TRUE(std::isfinite(energy));

  ES::V18d grad;
  model.compute_dE_dx(cd.get(), grad.data());
  for (int i = 0; i < 18; i++)
    EXPECT_TRUE(std::isfinite(grad[i]));
}

// ============================================================
// FD sanity check — gradient matches finite difference
// ============================================================

TEST(KoiterShellElementModelFDTest, GradientMatchesFiniteDifference)
{
  ElasticModel2DFundamentalFormsSTVK elasticModel;
  PlasticModel2DFundamentalFormsUniformStretch plasticModel;
  const bool hasVtx[6] = { true, true, true, true, true, true };

  KoiterShellElementModel model(interiorRestX, hasVtx, &elasticModel, &plasticModel);

  auto cd = model.allocateCacheData();

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX, 18, 0.1);
  model.prepareData(x, cd.get());
  double e0 = model.computeEnergy(cd.get());

  ES::V18d g;
  model.compute_dE_dx(cd.get(), g.data());

  const double eps = 1e-6;
  for (int i = 0; i < 18; i++) {
    double xPlus[18];
    std::copy(x, x + 18, xPlus);
    xPlus[i] += eps;

    auto cdP = model.allocateCacheData();
    model.prepareData(xPlus, cdP.get());
    double ePlus = model.computeEnergy(cdP.get());

    double fdGrad = (ePlus - e0) / eps;
    EXPECT_NEAR(fdGrad, g[i], 1e-5) << "FD gradient mismatch at index " << i;
  }
}

// ============================================================
// SPD enable produces symmetric PSD hessian
// ============================================================

TEST(KoiterShellElementModelTest, SPDEnableProducesSymmetricPSD)
{
  ElasticModel2DFundamentalFormsSTVK elasticModel;
  PlasticModel2DFundamentalFormsUniformStretch plasticModel;
  const bool hasVtx[6] = { true, true, true, true, true, true };

  KoiterShellElementModel model(interiorRestX, hasVtx, &elasticModel, &plasticModel);

  auto cd = model.allocateCacheData();

  double x[18] = {};
  perturbedDisplacement(x, interiorRestX, 18, 0.1);
  model.prepareData(x, cd.get());

  model.enableSPD(1);
  ES::M18d hess;
  model.compute_d2E_dx2(cd.get(), hess.data());

  // Check symmetry.
  for (int i = 0; i < 18; i++)
    for (int j = 0; j < i; j++)
      EXPECT_NEAR(hess(i, j), hess(j, i), 1e-10)
        << "SPD hessian asymmetry at (" << i << ", " << j << ")";
}
