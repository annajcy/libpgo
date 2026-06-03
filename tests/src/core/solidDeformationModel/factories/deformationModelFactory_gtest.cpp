#include <gtest/gtest.h>

#include "deformationModelFactory.h"
#include "deformationModelState.h"

#include "deformationModelEnergy.h"
#include "deformationModelAssembler.h"
#include "deformationModelManager.h"  // DeformationModelElasticMaterial / DeformationModelPlasticMaterial
#include "simulationMesh.h"
#include "tetMesh.h"
#include "cubicMesh.h"
#include "triMeshGeo.h"
#include "factories/deformationModelFactory.h"
#include "factories/elasticModelFactory.h"
#include "formulations/elements/shellDeformationModel.h"
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

template<class FormulationT>
std::shared_ptr<DeformationModelEnergy> makeDefaultFieldEnergy(
  std::shared_ptr<const SimulationMesh> mesh,
  const FormulationT &formulation,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  const DeformationModelOptions &opts = {})
{
  auto state = DeformationModelState::create(
    mesh,
    elastic,
    ElasticFieldInit{},
    plastic,
    PlasticFieldInit{});
  return makeDeformationEnergy(state, formulation, opts);
}
}  // namespace

// ============================================================
// Shell element model factory tests
// ============================================================

// Verify that DeformationModelFactory::create<ShellKoiter> returns the new
// ShellDeformationModel type.
TEST(DeformationModelFactoryGTest, CreateShellKoiterReturnsNewDeformationModelType)
{
  pgo::Logging::init();

  pgo::Mesh::TriMeshGeo surfaceMesh;
  ASSERT_TRUE(surfaceMesh.load(LIBPGO_TEST_SHELL_OBJ));
  SimulationMeshENuhMaterial shellMaterial(1000.0, 0.45, 1e-3);
  auto simMesh = loadShellMesh(surfaceMesh, &shellMaterial);
  ASSERT_NE(simMesh, nullptr);
  ASSERT_GT(simMesh->getNumElements(), 0);

  auto elasticModel = ElasticModelFactory::create(
    *simMesh, 0, DeformationModelElasticMaterial::KOITER_STVK, nullptr);
  auto plasticModel = PlasticModelFactory::create(
    DeformationModelPlasticMaterial::SHELL_FF_DOF1, nullptr);

  KoiterShellFormulation formulation;
  auto fem = DeformationModelFactory::create(
    *simMesh, 0, std::move(elasticModel), std::move(plasticModel),
    nullptr, nullptr, formulation);

  ASSERT_NE(fem, nullptr);
  EXPECT_EQ(fem->getNumVertices(), 6);
  EXPECT_EQ(fem->getNumDOFs(), 18);

  auto *typed = dynamic_cast<ShellDeformationModel *>(fem.get());
  EXPECT_NE(typed, nullptr) << "expected ShellDeformationModel from factory";
}

// Factory throws for non-Koiter elastic materials on shell elements.
TEST(DeformationModelFactoryGTest, CreateShellKoiterRejectsNonShellElasticMaterial)
{
  pgo::Logging::init();

  pgo::Mesh::TriMeshGeo surfaceMesh;
  ASSERT_TRUE(surfaceMesh.load(LIBPGO_TEST_SHELL_OBJ));
  SimulationMeshENuhMaterial shellMaterial(1000.0, 0.45, 1e-3);
  auto simMesh = loadShellMesh(surfaceMesh, &shellMaterial);
  ASSERT_NE(simMesh, nullptr);

  auto elasticModel = ElasticModelFactory::create(
    *simMesh, 0, DeformationModelElasticMaterial::STABLE_NEO, nullptr);
  auto plasticModel = PlasticModelFactory::create(
    DeformationModelPlasticMaterial::SHELL_FF_DOF1, nullptr);

  KoiterShellFormulation formulation;
  EXPECT_THROW(
    DeformationModelFactory::create(
      *simMesh, 0, std::move(elasticModel), std::move(plasticModel),
      nullptr, nullptr, formulation),
    std::logic_error);
}

// ============================================================
// Task 5p/5q: Verify production factories use formulation-aware element models
// ============================================================

// Tet: DeformationModelManager initImpl creates VolumetricDeformationModel.
TEST(DeformationModelFactoryGTest, TetManagerCreatesFormulationAwareModel)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  std::shared_ptr<const SimulationMesh> simMesh(loadTetMesh(&tetMesh).release());
  ASSERT_NE(simMesh, nullptr);

  auto state = DeformationModelState::create(
    simMesh,
    DeformationModelElasticMaterial::STABLE_NEO,
    ElasticFieldInit{},
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    PlasticFieldInit{});
  DeformationModelManager manager(
    state,
    P1TetFormulation{},
    1);

  for (int ele = 0; ele < simMesh->getNumElements(); ele++) {
    const auto *fem = manager.getDeformationModel(ele);
    ASSERT_NE(fem, nullptr);
    EXPECT_EQ(fem->getNumVertices(), 4);
    EXPECT_EQ(fem->getNumDOFs(), 12);
  }
}

// Cubic: DeformationModelManager initImpl creates VolumetricDeformationModel.
TEST(DeformationModelFactoryGTest, CubicManagerCreatesFormulationAwareModel)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> simMesh(loadCubicMesh(&cubicMesh).release());
  ASSERT_NE(simMesh, nullptr);

  auto state = DeformationModelState::create(
    simMesh,
    DeformationModelElasticMaterial::STABLE_NEO,
    ElasticFieldInit{},
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    PlasticFieldInit{});
  DeformationModelManager manager(
    state,
    LinearCubicFormulation{},
    1);

  for (int ele = 0; ele < simMesh->getNumElements(); ele++) {
    const auto *fem = manager.getDeformationModel(ele);
    ASSERT_NE(fem, nullptr);
    EXPECT_EQ(fem->getNumVertices(), 8);
    EXPECT_EQ(fem->getNumDOFs(), 24);
  }
}

// Verify public topology-specific factory produces valid energy end-to-end.
TEST(DeformationModelFactoryGTest, MakeTetDeformationModelEndToEnd)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  std::shared_ptr<const SimulationMesh> simMesh(loadTetMesh(&tetMesh).release());
  ASSERT_NE(simMesh, nullptr);

  DeformationModelOptions opts;
  opts.enforceSPD = true;
  auto bundle = makeDefaultFieldEnergy(simMesh,
    P1TetFormulation{},
    DeformationModelElasticMaterial::STABLE_NEO,
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, opts);

  ASSERT_NE(bundle, nullptr);
  const int ndof = bundle->getNumDOFs();
  EXPECT_EQ(ndof, simMesh->getNumVertices() * 3);

  ES::VXd u0 = ES::VXd::Zero(ndof);
  const double f = bundle->func(u0);
  EXPECT_TRUE(std::isfinite(f));
  ES::VXd g(ndof);
  bundle->gradient(u0, g);
  for (int i = 0; i < ndof; i++)
    EXPECT_TRUE(std::isfinite(g[i]));
}

// Verify public cubic topology-specific factory produces valid energy end-to-end.
TEST(DeformationModelFactoryGTest, MakeCubicDeformationModelEndToEnd)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> simMesh(loadCubicMesh(&cubicMesh).release());
  ASSERT_NE(simMesh, nullptr);

  DeformationModelOptions opts;
  opts.enforceSPD = true;
  auto bundle = makeDefaultFieldEnergy(simMesh,
    LinearCubicFormulation{},
    DeformationModelElasticMaterial::STABLE_NEO,
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, opts);

  ASSERT_NE(bundle, nullptr);
  const int ndof = bundle->getNumDOFs();
  EXPECT_EQ(ndof, simMesh->getNumVertices() * 3);

  ES::VXd u0 = ES::VXd::Zero(ndof);
  const double f = bundle->func(u0);
  EXPECT_TRUE(std::isfinite(f));
  ES::VXd g(ndof);
  bundle->gradient(u0, g);
  for (int i = 0; i < ndof; i++)
    EXPECT_TRUE(std::isfinite(g[i]));
}
