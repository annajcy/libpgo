#include <gtest/gtest.h>

#include "deformationModelFactory.h"

#include "deformationModelEnergy.h"
#include "deformationModelAssembler.h"
#include "deformationModelManager.h"  // DeformationModelElasticMaterial / DeformationModelPlasticMaterial
#include "simulationMesh.h"
#include "tetMesh.h"
#include "cubicMesh.h"
#include "triMeshGeo.h"
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

// ============================================================
// Shell element model factory tests
// ============================================================

// Verify that ElementModelFactory::create<ShellKoiter> returns the new
// KoiterShellElementModel type.
TEST(ElementModelFactoryGTest, CreateShellKoiterReturnsNewElementModelType)
{
  pgo::Logging::init();

  pgo::Mesh::TriMeshGeo surfaceMesh;
  ASSERT_TRUE(surfaceMesh.load(LIBPGO_TEST_SHELL_OBJ));
  SimulationMeshENuhMaterial shellMaterial(1000.0, 0.45, 1e-3);
  auto simMesh = loadShellMesh(surfaceMesh, &shellMaterial);
  ASSERT_NE(simMesh, nullptr);
  ASSERT_GT(simMesh->getNumElements(), 0);

  auto elasticResult = ElasticModelFactory::create(
    *simMesh, 0, DeformationModelElasticMaterial::KOITER_STVK, nullptr);
  std::unique_ptr<ElasticModel> elasticOwner(elasticResult.elementMaterial);
  auto plasticResult = PlasticModelFactory::create(
    *simMesh, 0, DeformationModelPlasticMaterial::SHELL_FF_DOF1, nullptr);
  std::unique_ptr<PlasticModel> plasticOwner(plasticResult.model);

  auto *fem = ElementModelFactory::create<ShellKoiter>(
    *simMesh, 0, elasticResult.elementMaterial, plasticResult.model,
    DeformationModelElasticMaterial::KOITER_STVK);
  std::unique_ptr<DeformationModel> femOwner(fem);

  ASSERT_NE(fem, nullptr);
  EXPECT_EQ(fem->getNumVertices(), 6);
  EXPECT_EQ(fem->getNumDOFs(), 18);

  auto *typed = dynamic_cast<KoiterShellElementModel *>(fem);
  EXPECT_NE(typed, nullptr) << "expected KoiterShellElementModel from factory";
}

// Factory throws for non-Koiter elastic materials on shell elements.
TEST(ElementModelFactoryGTest, CreateShellKoiterRejectsNonShellElasticMaterial)
{
  pgo::Logging::init();

  pgo::Mesh::TriMeshGeo surfaceMesh;
  ASSERT_TRUE(surfaceMesh.load(LIBPGO_TEST_SHELL_OBJ));
  SimulationMeshENuhMaterial shellMaterial(1000.0, 0.45, 1e-3);
  auto simMesh = loadShellMesh(surfaceMesh, &shellMaterial);
  ASSERT_NE(simMesh, nullptr);

  auto elasticResult = ElasticModelFactory::create(
    *simMesh, 0, DeformationModelElasticMaterial::STABLE_NEO, nullptr);
  std::unique_ptr<ElasticModel> elasticOwner(elasticResult.elementMaterial);
  auto plasticResult = PlasticModelFactory::create(
    *simMesh, 0, DeformationModelPlasticMaterial::SHELL_FF_DOF1, nullptr);
  std::unique_ptr<PlasticModel> plasticOwner(plasticResult.model);

  EXPECT_THROW(
    ElementModelFactory::create<ShellKoiter>(
      *simMesh, 0, elasticResult.elementMaterial, plasticResult.model,
      DeformationModelElasticMaterial::STABLE_NEO),
    std::logic_error);
}

// ============================================================
// Task 5p/5q: Verify production factories use formulation-aware element models
// ============================================================

// Tet: DeformationModelManager initImpl creates DeformationGradientElementModel.
TEST(ElementModelFactoryGTest, TetManagerCreatesFormulationAwareModel)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto simMesh = loadTetMesh(&tetMesh);
  ASSERT_NE(simMesh, nullptr);

  DeformationModelManager manager(*simMesh,
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    DeformationModelElasticMaterial::STABLE_NEO, 1);

  for (int ele = 0; ele < simMesh->getNumElements(); ele++) {
    const auto *fem = manager.getDeformationModel(ele);
    ASSERT_NE(fem, nullptr);
    EXPECT_EQ(fem->getNumVertices(), 4);
    EXPECT_EQ(fem->getNumDOFs(), 12);
  }
}

// Cubic: DeformationModelManager initImpl creates DeformationGradientElementModel.
TEST(ElementModelFactoryGTest, CubicManagerCreatesFormulationAwareModel)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  auto simMesh = loadCubicMesh(&cubicMesh);
  ASSERT_NE(simMesh, nullptr);

  DeformationModelManager manager(*simMesh,
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    DeformationModelElasticMaterial::STABLE_NEO, 1);

  for (int ele = 0; ele < simMesh->getNumElements(); ele++) {
    const auto *fem = manager.getDeformationModel(ele);
    ASSERT_NE(fem, nullptr);
    EXPECT_EQ(fem->getNumVertices(), 8);
    EXPECT_EQ(fem->getNumDOFs(), 24);
  }
}

// Verify public topology-specific factory produces valid energy end-to-end.
TEST(ElementModelFactoryGTest, MakeTetDeformationModelEndToEnd)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto simMesh = loadTetMesh(&tetMesh);
  ASSERT_NE(simMesh, nullptr);

  DeformationModelOptions opts;
  opts.enforceSPD = true;
  auto bundle = makeTetDeformationModel(*simMesh, TetP1{},
    DeformationModelElasticMaterial::STABLE_NEO,
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, opts);

  ASSERT_NE(bundle.energy, nullptr);
  const int ndof = bundle.energy->getNumDOFs();
  EXPECT_EQ(ndof, simMesh->getNumVertices() * 3);

  ES::VXd u0 = ES::VXd::Zero(ndof);
  const double f = bundle.energy->func(u0);
  EXPECT_TRUE(std::isfinite(f));
  ES::VXd g(ndof);
  bundle.energy->gradient(u0, g);
  for (int i = 0; i < ndof; i++)
    EXPECT_TRUE(std::isfinite(g[i]));
}

// Verify public cubic topology-specific factory produces valid energy end-to-end.
TEST(ElementModelFactoryGTest, MakeCubicDeformationModelEndToEnd)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  auto simMesh = loadCubicMesh(&cubicMesh);
  ASSERT_NE(simMesh, nullptr);

  DeformationModelOptions opts;
  opts.enforceSPD = true;
  auto bundle = makeCubicDeformationModel(*simMesh, HexTrilinear{},
    DeformationModelElasticMaterial::STABLE_NEO,
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, opts);

  ASSERT_NE(bundle.energy, nullptr);
  const int ndof = bundle.energy->getNumDOFs();
  EXPECT_EQ(ndof, simMesh->getNumVertices() * 3);

  ES::VXd u0 = ES::VXd::Zero(ndof);
  const double f = bundle.energy->func(u0);
  EXPECT_TRUE(std::isfinite(f));
  ES::VXd g(ndof);
  bundle.energy->gradient(u0, g);
  for (int i = 0; i < ndof; i++)
    EXPECT_TRUE(std::isfinite(g[i]));
}
