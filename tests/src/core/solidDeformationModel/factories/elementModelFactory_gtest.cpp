#include <gtest/gtest.h>

#include "deformationModelFactory.h"

#include "deformationModelEnergy.h"
#include "deformationModelAssembler.h"
#include "simulationMesh.h"
#include "tetMesh.h"
#include "cubicMesh.h"
#include "factories/elementModelFactory.h"
#include "factories/elasticModelFactory.h"
#include "factories/plasticModelFactory.h"
#include "pgoLogging.h"

#include <memory>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

constexpr const char *kTorusVegPath = LIBPGO_TEST_TORUS_VEG;
constexpr const char *kCubicBoxVegPath = LIBPGO_TEST_CUBIC_BOX_VEG;
}  // namespace

// Helper: build energy via the legacy non-template path.
static DeformationModelBundle buildLegacyEnergy(
  const SimulationMesh &mesh,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic)
{
  DeformationModelOptions opts;
  opts.enforceSPD = true;
  return detail::makeDeformationModelBundle(mesh, elastic, plastic, opts);
}

// Compare the legacy energy path with the formulation-aware path for tet.
TEST(ElementModelFactoryGTest, TetEnergyParityWithLegacyPath)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto simMesh = loadTetMesh(&tetMesh);
  ASSERT_NE(simMesh, nullptr);

  const auto elastic = DeformationModelElasticMaterial::STABLE_NEO;
  const auto plastic = DeformationModelPlasticMaterial::VOLUMETRIC_DOF6;

  DeformationModelOptions opts;
  opts.enforceSPD = true;
  auto legacy = buildLegacyEnergy(*simMesh, elastic, plastic);
  auto formulated = detail::makeDeformationModelBundle<TetP1>(*simMesh, elastic, plastic, opts);

  ASSERT_NE(legacy.energy, nullptr);
  ASSERT_NE(formulated.energy, nullptr);
  EXPECT_EQ(legacy.energy->getNumDOFs(), formulated.energy->getNumDOFs());

  // Both should have the same rest position.
  ASSERT_EQ(legacy.restPosition.size(), formulated.restPosition.size());
  for (Eigen::Index i = 0; i < legacy.restPosition.size(); i++)
    EXPECT_DOUBLE_EQ(legacy.restPosition[i], formulated.restPosition[i]);

  // Energy at zero displacement.
  ES::VXd u0 = ES::VXd::Zero(legacy.energy->getNumDOFs());
  const double fLegacy = legacy.energy->func(u0);
  const double fFormulated = formulated.energy->func(u0);
  EXPECT_TRUE(std::isfinite(fLegacy));
  EXPECT_TRUE(std::isfinite(fFormulated));
  EXPECT_NEAR(fLegacy, fFormulated, 1e-12);

  // Gradient at zero displacement.
  ES::VXd gLegacy = ES::VXd::Zero(legacy.energy->getNumDOFs());
  ES::VXd gFormulated = ES::VXd::Zero(formulated.energy->getNumDOFs());
  legacy.energy->gradient(u0, gLegacy);
  formulated.energy->gradient(u0, gFormulated);
  for (Eigen::Index i = 0; i < gLegacy.size(); i++)
    EXPECT_NEAR(gLegacy[i], gFormulated[i], 1e-10);

  // Hessian at zero displacement.
  ES::SpMatD hLegacy, hFormulated;
  legacy.energy->createHessian(hLegacy);
  formulated.energy->createHessian(hFormulated);
  legacy.energy->hessian(u0, hLegacy);
  formulated.energy->hessian(u0, hFormulated);
  EXPECT_EQ(hLegacy.nonZeros(), hFormulated.nonZeros());
  for (Eigen::Index i = 0; i < hLegacy.nonZeros(); i++)
    EXPECT_NEAR(hLegacy.valuePtr()[i], hFormulated.valuePtr()[i], 1e-8);

  // Energy at small perturbation.
  ES::VXd up = u0;
  up[0] += 0.01;
  const double fpLegacy = legacy.energy->func(up);
  const double fpFormulated = formulated.energy->func(up);
  EXPECT_NEAR(fpLegacy, fpFormulated, 1e-10);
}

// Compare the legacy energy path with the formulation-aware path for cubic.
TEST(ElementModelFactoryGTest, CubicEnergyParityWithLegacyPath)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  auto simMesh = loadCubicMesh(&cubicMesh);
  ASSERT_NE(simMesh, nullptr);

  const auto elastic = DeformationModelElasticMaterial::STABLE_NEO;
  const auto plastic = DeformationModelPlasticMaterial::VOLUMETRIC_DOF6;

  DeformationModelOptions opts;
  opts.enforceSPD = true;
  auto legacy = buildLegacyEnergy(*simMesh, elastic, plastic);
  auto formulated = detail::makeDeformationModelBundle<HexTrilinear>(*simMesh, elastic, plastic, opts);

  ASSERT_NE(legacy.energy, nullptr);
  ASSERT_NE(formulated.energy, nullptr);
  EXPECT_EQ(legacy.energy->getNumDOFs(), formulated.energy->getNumDOFs());

  ASSERT_EQ(legacy.restPosition.size(), formulated.restPosition.size());
  for (Eigen::Index i = 0; i < legacy.restPosition.size(); i++)
    EXPECT_DOUBLE_EQ(legacy.restPosition[i], formulated.restPosition[i]);

  ES::VXd u0 = ES::VXd::Zero(legacy.energy->getNumDOFs());
  const double fLegacy = legacy.energy->func(u0);
  const double fFormulated = formulated.energy->func(u0);
  EXPECT_TRUE(std::isfinite(fLegacy));
  EXPECT_TRUE(std::isfinite(fFormulated));
  EXPECT_NEAR(fLegacy, fFormulated, 1e-12);

  ES::VXd gLegacy = ES::VXd::Zero(legacy.energy->getNumDOFs());
  ES::VXd gFormulated = ES::VXd::Zero(formulated.energy->getNumDOFs());
  legacy.energy->gradient(u0, gLegacy);
  formulated.energy->gradient(u0, gFormulated);
  for (Eigen::Index i = 0; i < gLegacy.size(); i++)
    EXPECT_NEAR(gLegacy[i], gFormulated[i], 1e-10);

  ES::SpMatD hLegacy, hFormulated;
  legacy.energy->createHessian(hLegacy);
  formulated.energy->createHessian(hFormulated);
  legacy.energy->hessian(u0, hLegacy);
  formulated.energy->hessian(u0, hFormulated);
  EXPECT_EQ(hLegacy.nonZeros(), hFormulated.nonZeros());
  for (Eigen::Index i = 0; i < hLegacy.nonZeros(); i++)
    EXPECT_NEAR(hLegacy.valuePtr()[i], hFormulated.valuePtr()[i], 1e-8);

  ES::VXd up = u0;
  up[0] += 0.01;
  const double fpLegacy = legacy.energy->func(up);
  const double fpFormulated = formulated.energy->func(up);
  EXPECT_NEAR(fpLegacy, fpFormulated, 1e-10);
}
