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

  auto fem = ElementModelFactory::create<ShellKoiter>(
    *simMesh, 0, elasticResult.elementMaterial, plasticResult.model,
    DeformationModelElasticMaterial::KOITER_STVK);

  ASSERT_NE(fem, nullptr);
  EXPECT_EQ(fem->getNumVertices(), 6);
  EXPECT_EQ(fem->getNumDOFs(), 18);

  auto *typed = dynamic_cast<KoiterShellElementModel *>(fem.get());
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
