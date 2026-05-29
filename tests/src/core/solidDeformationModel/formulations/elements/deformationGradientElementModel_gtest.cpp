#include "gtest/gtest.h"

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
#include <cmath>

namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

using TetKernel = DeformationGradientKernel<TetP1Basis, TetP1DefaultQuadrature>;
using TetModel = DeformationGradientElementModel<TetKernel>;

using HexKernel = DeformationGradientKernel<HexTrilinearBasis, GaussLegendreHexQuadrature2>;
using HexModel = DeformationGradientElementModel<HexKernel>;

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
}  // namespace

// ============================================================
// Tet: energy is finite at rest and gradient FD check
// ============================================================

TEST(DeformationGradientElementModelGTest, TetEnergyFiniteAtRest)
{
  ElasticModelStableNeoHookeanMaterial elasticModel(1200.0, 1800.0);
  double identity[9] = { 1,0,0, 0,1,0, 0,0,1 };
  PlasticModel3DConstant plasticModel(identity);
  TetModel model(restTet, &elasticModel, &plasticModel);

  ES::V12d xVec;
  for (int i = 0; i < 12; i++) xVec[i] = restTet[i];

  const double materialParam[12] = {};
  const double plasticParam[1] = {};

  auto cd = model.allocateCacheData();
  model.prepareData(xVec.data(), plasticParam, materialParam, cd.get());

  double energy = model.computeEnergy(cd.get());
  EXPECT_TRUE(std::isfinite(energy));

  ES::V12d grad;
  model.compute_dE_dx(cd.get(), grad.data());
  for (int i = 0; i < 12; i++)
    EXPECT_TRUE(std::isfinite(grad[i]));

  ES::M12d hess;
  model.compute_d2E_dx2(cd.get(), hess.data());
  for (int i = 0; i < 144; i++)
    EXPECT_TRUE(std::isfinite(hess.data()[i]));

  
}

// ============================================================
// Hex: energy is finite at rest and gradient FD check
// ============================================================

TEST(DeformationGradientElementModelGTest, HexEnergyFiniteAtRest)
{
  ElasticModelStableNeoHookeanMaterial elasticModel(1200.0, 1800.0);
  double identity[9] = { 1,0,0, 0,1,0, 0,0,1 };
  PlasticModel3DConstant plasticModel(identity);
  HexModel model(restHex, &elasticModel, &plasticModel);

  ES::V24d xVec;
  for (int i = 0; i < 24; i++) xVec[i] = restHex[i];

  const double materialParam[12] = {};
  const double plasticParam[1] = {};

  auto cd = model.allocateCacheData();
  model.prepareData(xVec.data(), plasticParam, materialParam, cd.get());

  double energy = model.computeEnergy(cd.get());
  EXPECT_TRUE(std::isfinite(energy));

  ES::V24d grad;
  model.compute_dE_dx(cd.get(), grad.data());
  for (int i = 0; i < 24; i++)
    EXPECT_TRUE(std::isfinite(grad[i]));

  ES::M24d hess;
  model.compute_d2E_dx2(cd.get(), hess.data());
  for (int i = 0; i < 576; i++)
    EXPECT_TRUE(std::isfinite(hess.data()[i]));

  
}

// ============================================================
// Tet: gradient matches finite difference of energy
// ============================================================

TEST(DeformationGradientElementModelGTest, TetGradientMatchesFD)
{
  ElasticModelStableNeoHookeanMaterial elasticModel(1200.0, 1800.0);
  double identity[9] = { 1,0,0, 0,1,0, 0,0,1 };
  PlasticModel3DConstant plasticModel(identity);
  TetModel model(restTet, &elasticModel, &plasticModel);

  ES::V12d xVec;
  for (int i = 0; i < 12; i++) xVec[i] = restTet[i] + 0.01 * std::sin(0.7 * static_cast<double>(i));

  const double materialParam[12] = {};
  const double plasticParam[1] = {};

  auto cd = model.allocateCacheData();
  model.prepareData(xVec.data(), plasticParam, materialParam, cd.get());

  ES::V12d grad;
  model.compute_dE_dx(cd.get(), grad.data());

  const double eps = 1e-6;
  for (int i = 0; i < 12; i++) {
    ES::V12d xp = xVec, xm = xVec;
    xp[i] += eps;
    xm[i] -= eps;

    auto cdp = model.allocateCacheData();
    model.prepareData(xp.data(), plasticParam, materialParam, cdp.get());
    double ep = model.computeEnergy(cdp.get());
    

    auto cdm = model.allocateCacheData();
    model.prepareData(xm.data(), plasticParam, materialParam, cdm.get());
    double em = model.computeEnergy(cdm.get());
    

    double fd = (ep - em) / (2.0 * eps);
    EXPECT_NEAR(grad[i], fd, 1e-5) << "grad[" << i << "]";
  }

  
}

// ============================================================
// Hex: gradient matches finite difference of energy
// ============================================================

TEST(DeformationGradientElementModelGTest, HexGradientMatchesFD)
{
  ElasticModelStableNeoHookeanMaterial elasticModel(1200.0, 1800.0);
  double identity[9] = { 1,0,0, 0,1,0, 0,0,1 };
  PlasticModel3DConstant plasticModel(identity);
  HexModel model(restHex, &elasticModel, &plasticModel);

  ES::V24d xVec;
  for (int i = 0; i < 24; i++) xVec[i] = restHex[i] + 0.01 * std::sin(0.7 * static_cast<double>(i));

  const double materialParam[12] = {};
  const double plasticParam[1] = {};

  auto cd = model.allocateCacheData();
  model.prepareData(xVec.data(), plasticParam, materialParam, cd.get());

  ES::V24d grad;
  model.compute_dE_dx(cd.get(), grad.data());

  const double eps = 1e-6;
  for (int i = 0; i < 24; i++) {
    ES::V24d xp = xVec, xm = xVec;
    xp[i] += eps;
    xm[i] -= eps;

    auto cdp = model.allocateCacheData();
    model.prepareData(xp.data(), plasticParam, materialParam, cdp.get());
    double ep = model.computeEnergy(cdp.get());
    

    auto cdm = model.allocateCacheData();
    model.prepareData(xm.data(), plasticParam, materialParam, cdm.get());
    double em = model.computeEnergy(cdm.get());
    

    double fd = (ep - em) / (2.0 * eps);
    EXPECT_NEAR(grad[i], fd, 1e-5) << "grad[" << i << "]";
  }

  
}
