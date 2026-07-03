#include "gtest/gtest.h"

#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/plastic/plasticModel3DConstant.h"
#include "material/plastic/plasticModel3D3DOF.h"

#include "formulations/shapeFunction/tetLinearShapeFunction.h"
#include "formulations/shapeFunction/cubicLinearShapeFunction.h"
#include "formulations/quadrature/tetLinearDefaultQuadrature.h"
#include "formulations/quadrature/gaussLegendreHexQuadrature.h"
#include "deformation/volume/volumetricElementMapping.h"
#include "deformation/volume/volumetricDeformationModel.h"
#include "EigenSupport.h"

#include <memory>
#include <cmath>

namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

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

TEST(VolumetricDeformationModelGTest, TetEnergyFiniteAtRest)
{
  auto elasticModel = std::make_unique<ElasticModelStableNeoHookeanMaterial>(1200.0, 1800.0);
  double identity[9] = { 1,0,0, 0,1,0, 0,0,1 };
  auto plasticModel = std::make_unique<PlasticModel3DConstant>(identity);
  VolumetricElementMapping mapping(restTet, TetLinearShapeFunction{}, TetLinearDefaultQuadrature{});
  VolumetricDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  ES::V12d xVec;
  for (int i = 0; i < 12; i++) xVec[i] = restTet[i];


  auto cd = model.allocateCacheData();
  EXPECT_FALSE(cd->isPrepared());
  model.prepareData(xVec.data(), cd.get());
  EXPECT_TRUE(cd->isPrepared());

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

TEST(VolumetricDeformationModelGTest, HexEnergyFiniteAtRest)
{
  auto elasticModel = std::make_unique<ElasticModelStableNeoHookeanMaterial>(1200.0, 1800.0);
  double identity[9] = { 1,0,0, 0,1,0, 0,0,1 };
  auto plasticModel = std::make_unique<PlasticModel3DConstant>(identity);
  VolumetricElementMapping mapping(restHex, CubicLinearShapeFunction{}, GaussLegendreHexQuadrature2{});
  VolumetricDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  ES::V24d xVec;
  for (int i = 0; i < 24; i++) xVec[i] = restHex[i];


  auto cd = model.allocateCacheData();
  model.prepareData(xVec.data(), cd.get());

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

TEST(VolumetricDeformationModelGTest, TetGradientMatchesFD)
{
  auto elasticModel = std::make_unique<ElasticModelStableNeoHookeanMaterial>(1200.0, 1800.0);
  double identity[9] = { 1,0,0, 0,1,0, 0,0,1 };
  auto plasticModel = std::make_unique<PlasticModel3DConstant>(identity);
  VolumetricElementMapping mapping(restTet, TetLinearShapeFunction{}, TetLinearDefaultQuadrature{});
  VolumetricDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  ES::V12d xVec;
  for (int i = 0; i < 12; i++) xVec[i] = restTet[i] + 0.01 * std::sin(0.7 * static_cast<double>(i));


  auto cd = model.allocateCacheData();
  model.prepareData(xVec.data(), cd.get());

  ES::V12d grad;
  model.compute_dE_dx(cd.get(), grad.data());

  const double eps = 1e-6;
  for (int i = 0; i < 12; i++) {
    ES::V12d xp = xVec, xm = xVec;
    xp[i] += eps;
    xm[i] -= eps;

    auto cdp = model.allocateCacheData();
    model.prepareData(xp.data(), cdp.get());
    double ep = model.computeEnergy(cdp.get());
    

    auto cdm = model.allocateCacheData();
    model.prepareData(xm.data(), cdm.get());
    double em = model.computeEnergy(cdm.get());
    

    double fd = (ep - em) / (2.0 * eps);
    EXPECT_NEAR(grad[i], fd, 1e-5) << "grad[" << i << "]";
  }

  
}

// ============================================================
// Hex: gradient matches finite difference of energy
// ============================================================

TEST(VolumetricDeformationModelGTest, HexGradientMatchesFD)
{
  auto elasticModel = std::make_unique<ElasticModelStableNeoHookeanMaterial>(1200.0, 1800.0);
  double identity[9] = { 1,0,0, 0,1,0, 0,0,1 };
  auto plasticModel = std::make_unique<PlasticModel3DConstant>(identity);
  VolumetricElementMapping mapping(restHex, CubicLinearShapeFunction{}, GaussLegendreHexQuadrature2{});
  VolumetricDeformationModel model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  ES::V24d xVec;
  for (int i = 0; i < 24; i++) xVec[i] = restHex[i] + 0.01 * std::sin(0.7 * static_cast<double>(i));


  auto cd = model.allocateCacheData();
  model.prepareData(xVec.data(), cd.get());

  ES::V24d grad;
  model.compute_dE_dx(cd.get(), grad.data());

  const double eps = 1e-6;
  for (int i = 0; i < 24; i++) {
    ES::V24d xp = xVec, xm = xVec;
    xp[i] += eps;
    xm[i] -= eps;

    auto cdp = model.allocateCacheData();
    model.prepareData(xp.data(), cdp.get());
    double ep = model.computeEnergy(cdp.get());
    

    auto cdm = model.allocateCacheData();
    model.prepareData(xm.data(), cdm.get());
    double em = model.computeEnergy(cdm.get());
    

    double fd = (ep - em) / (2.0 * eps);
    EXPECT_NEAR(grad[i], fd, 1e-5) << "grad[" << i << "]";
  }

  
}
