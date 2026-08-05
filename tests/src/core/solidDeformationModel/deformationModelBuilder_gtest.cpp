#include <gtest/gtest.h>
#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/elastic/elasticModel3DMooneyRivlin.h"
#include "material/elastic/elasticModelCombinedMaterial.h"
#include "material/elastic/elasticModel2DFundamentalFormsSTVK.h"
#include "material/plastic/plasticModel3D3DOF.h"
#include "material/plastic/plasticModel3D6DOF.h"
#include "material/plastic/plasticModel3DConstant.h"
#include "material/plastic/plasticModel2DFundamentalFormsUniformStretch.h"

#include "energy/deformationEnergyOperator.h"
#include "deformation/deformationModelAssembler.h"
#include "formulations/formulation/formulations.h"

#include "energy/deformationEnergyOperator.h"
#include "simulation/simulationMesh.h"
#include "tetMesh.h"
#include "cubicMesh.h"
#include "triMeshGeo.h"
#include "pgoLogging.h"
#include "volumetricMeshMooneyRivlinMaterial.h"
#include "materialTestUtils.h"

#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>

namespace
{
namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

constexpr const char *kTorusVegPath = LIBPGO_TEST_TORUS_VEG;
constexpr const char *kCubicBoxVegPath = LIBPGO_TEST_CUBIC_BOX_VEG;
constexpr const char *kShellObjPath = LIBPGO_TEST_SHELL_OBJ;

template<class FormulationT>
std::shared_ptr<DeformationPotentialEnergy> makeDefaultFieldEnergy(
  std::shared_ptr<const SimulationImportResult> asset,
  const FormulationT &formulation,
  std::shared_ptr<const ElasticModelDefinition> elastic,
  std::shared_ptr<const PlasticModelDefinition> plastic)
{
  return TestUtils::makeTestEnergy(
    std::move(asset), formulation, std::move(elastic),
    std::move(plastic));
}
}  // namespace

TEST(DeformationModelBuilderGTest, RejectsNullConfigsBeforeDefaultInitialization)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto asset = TestUtils::shareAsset(loadTetMesh(tetMesh));
  ASSERT_NE(asset, nullptr);

  std::shared_ptr<const ElasticModelDefinition> noElastic;
  std::shared_ptr<const PlasticModelDefinition> noPlastic;
  auto plastic = std::make_shared<VolumetricPlasticity6Definition>();
  auto elastic = std::make_shared<StableNeoDefinition>();

  EXPECT_THROW(
    TestUtils::makeMaterialBinding(asset, noElastic, plastic),
    std::invalid_argument);
  EXPECT_THROW(
    TestUtils::makeMaterialBinding(asset, elastic, noPlastic),
    std::invalid_argument);
}

// Baseline: tet deformation energy at zero displacement has near-zero energy
// and finite gradient. State x is displacement from rest, NOT absolute position.
TEST(DeformationModelBuilderGTest, TetZeroDisplacementBaseline)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto asset = TestUtils::shareAsset(loadTetMesh(tetMesh));
  ASSERT_NE(asset, nullptr);
  auto energy = makeDefaultFieldEnergy(
    asset, TetLinearFormulation{}, std::make_shared<StableNeoDefinition>(), std::make_shared<VolumetricPlasticity6Definition>());

  ASSERT_NE(energy, nullptr);
  EXPECT_GT(energy->getNumDOFs(), 0);

  ES::VXd u0 = ES::VXd::Zero(energy->getNumDOFs());
  const double f0 = energy->func(u0);
  EXPECT_TRUE(std::isfinite(f0));
  EXPECT_NEAR(f0, 0.0, 1e-10);

  ES::VXd grad0 = ES::VXd::Zero(energy->getNumDOFs());
  energy->gradient(u0, grad0);
  for (Eigen::Index i = 0; i < grad0.size(); i++)
    EXPECT_TRUE(std::isfinite(grad0[i])) << "Non-finite gradient entry at " << i;

  ES::SpMatD h0;
  energy->hessian(u0, h0);
  for (Eigen::Index i = 0; i < h0.nonZeros(); i++)
    EXPECT_TRUE(std::isfinite(h0.valuePtr()[i])) << "Non-finite Hessian entry at " << i;
}

// The structured-input overload must preserve the caller-provided immutable
// material frame field instead of silently replacing it with global axes.
TEST(DeformationModelBuilderGTest, StructuredInputsCarryCustomMaterialFrames)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto asset = TestUtils::shareAsset(loadTetMesh(tetMesh));
  ASSERT_NE(asset, nullptr);

  const double angle = 0.61;
  MaterialFrame frame;
  frame << std::cos(angle), -std::sin(angle), 0.0,
    std::sin(angle), std::cos(angle), 0.0,
    0.0, 0.0, 1.0;
  auto materialFrames =
    std::make_shared<const ConstantMaterialFrameField>(
      asset->mesh()->getNumElements(), frame);

  auto parameters = TestUtils::makeDefaultMaterialState(
    *asset,
    *std::make_shared<StableNeoDefinition>(),
    *std::make_shared<VolumetricPlasticity3Definition>());

  auto energy = TestUtils::makeTestEnergy(
    asset,
    TetLinearFormulation{},
    std::make_shared<StableNeoDefinition>(),
    std::make_shared<VolumetricPlasticity3Definition>(),
    std::move(parameters),
    materialFrames);
  ASSERT_NE(energy, nullptr);

  const auto &manager =
    energy->assembler().getDeformationModelManager();
  EXPECT_EQ(manager.materialFrameFieldPtr().get(), materialFrames.get());
  EXPECT_TRUE(
    manager.materialToReferenceFrame(0).isApprox(frame, 1e-12));
}

// Baseline: cubic deformation energy at zero displacement.
// State convention: func(x) computes energy at restPosition + x.
TEST(DeformationModelBuilderGTest, CubicZeroDisplacementBaseline)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  auto asset = TestUtils::shareAsset(loadCubicMesh(cubicMesh));
  ASSERT_NE(asset, nullptr);
  auto energy = makeDefaultFieldEnergy(
    asset, CubicLinearFormulation{}, std::make_shared<StableNeoDefinition>(), std::make_shared<VolumetricPlasticity6Definition>());

  ASSERT_NE(energy, nullptr);
  EXPECT_GT(energy->getNumDOFs(), 0);

  ES::VXd u0 = ES::VXd::Zero(energy->getNumDOFs());
  const double f0 = energy->func(u0);
  EXPECT_TRUE(std::isfinite(f0));
  EXPECT_NEAR(f0, 0.0, 1e-10);

  ES::VXd grad0 = ES::VXd::Zero(energy->getNumDOFs());
  energy->gradient(u0, grad0);
  for (Eigen::Index i = 0; i < grad0.size(); i++)
    EXPECT_TRUE(std::isfinite(grad0[i])) << "Non-finite gradient entry at " << i;

  ES::SpMatD h0;
  energy->hessian(u0, h0);
  for (Eigen::Index i = 0; i < h0.nonZeros(); i++)
    EXPECT_TRUE(std::isfinite(h0.valuePtr()[i])) << "Non-finite Hessian entry at " << i;
}

TEST(DeformationModelBuilderGTest, MooneyRivlinDefinitionBuildsTetEnergy)
{
  pgo::Logging::init();

  const double vertices[] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  const int elements[] = {0, 1, 2, 3};
  pgo::VolumetricMeshes::VolumetricMesh::MooneyRivlinMaterial material(
    "mr_test", 1000.0, 0.5, 0.3, 0.1);
  const pgo::VolumetricMeshes::VolumetricMesh::Material *materials[] = {&material};
  pgo::VolumetricMeshes::VolumetricMesh::Set set("all", std::set<int>{0});
  pgo::VolumetricMeshes::VolumetricMesh::Region region(0, 0);
  pgo::VolumetricMeshes::TetMesh tetMesh(
    4, vertices, 1, elements, 1, materials, 1, &set, 1, &region);

  auto asset = TestUtils::shareAsset(loadTetMesh(tetMesh));
  ASSERT_NE(asset, nullptr);
  auto energy = makeDefaultFieldEnergy(
    asset, TetLinearFormulation{}, std::make_shared<MooneyRivlinDefinition>(),
    std::make_shared<VolumetricPlasticity0Definition>());
  ASSERT_NE(energy, nullptr);

  ES::VXd u0 = ES::VXd::Zero(energy->getNumDOFs());
  EXPECT_NEAR(energy->func(u0), 0.0, 1e-12);
  ES::VXd gradient = ES::VXd::Zero(energy->getNumDOFs());
  energy->gradient(u0, gradient);
  for (Eigen::Index i = 0; i < gradient.size(); ++i)
    EXPECT_TRUE(std::isfinite(gradient[i]));
}

// MakeTetDeformationModel with SimulationMesh reference validates TET topology.
TEST(DeformationModelBuilderGTest, TetSimulationMeshBuilderValidatesTopology)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto asset = TestUtils::shareAsset(loadTetMesh(tetMesh));
  ASSERT_NE(asset, nullptr);

  auto energy = makeDefaultFieldEnergy(
    asset, TetLinearFormulation{}, std::make_shared<StableNeoDefinition>(), std::make_shared<VolumetricPlasticity6Definition>());
  ASSERT_NE(energy, nullptr);
  EXPECT_GT(energy->getNumDOFs(), 0);
}

// MakeCubicDeformationModel with SimulationMesh reference validates CUBIC topology.
TEST(DeformationModelBuilderGTest, CubicSimulationMeshBuilderValidatesTopology)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  auto asset = TestUtils::shareAsset(loadCubicMesh(cubicMesh));
  ASSERT_NE(asset, nullptr);

  auto energy = makeDefaultFieldEnergy(
    asset, CubicLinearFormulation{}, std::make_shared<StableNeoDefinition>(), std::make_shared<VolumetricPlasticity6Definition>());
  ASSERT_NE(energy, nullptr);
  EXPECT_GT(energy->getNumDOFs(), 0);
}

// MakeShellDeformationModel with SimulationMesh reference validates SHELL topology
// and uses the existing Koiter shell path.
TEST(DeformationModelBuilderGTest, ShellSimulationMeshBuilderValidatesTopology)
{
  pgo::Logging::init();

  pgo::Mesh::TriMeshGeo surfaceMesh;
  ASSERT_TRUE(surfaceMesh.load(kShellObjPath));
  auto asset = TestUtils::shareAsset(
    loadShellMesh(surfaceMesh),
    TestUtils::uniformImportedMaterialCatalog(
      surfaceMesh.numTriangles(), {"E", "nu", "h", "J"},
      {1000.0, 0.45, 1e-3, 10000.0}, "shell"));
  ASSERT_NE(asset, nullptr);

  auto energy = makeDefaultFieldEnergy(
    asset, KoiterShellFormulation{}, std::make_shared<KoiterStVKDefinition>(), std::make_shared<ShellPlasticity1Definition>());

  ASSERT_NE(energy, nullptr);
  EXPECT_GT(energy->getNumDOFs(), 0);
  EXPECT_EQ(energy->getNumDOFs(), asset->mesh()->getNumVertices() * 3);

  ES::VXd u0 = ES::VXd::Zero(energy->getNumDOFs());
  EXPECT_TRUE(std::isfinite(energy->func(u0)));
}

// Wrong topology/SimulationMesh type fails at runtime.
TEST(DeformationModelBuilderGTest, TetBuilderRejectsCubicSimulationMesh)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  auto asset = TestUtils::shareAsset(loadCubicMesh(cubicMesh));
  ASSERT_NE(asset, nullptr);

  EXPECT_THROW(
    makeDefaultFieldEnergy(asset, TetLinearFormulation{}, std::make_shared<StableNeoDefinition>(), std::make_shared<VolumetricPlasticity6Definition>()),
    std::invalid_argument);
}

// One SimulationMesh owner can be used to construct two independent deformation
// energies. Both must remain evaluable while the owner is alive.
TEST(DeformationModelBuilderGTest, OneMeshOwnerTwoTetEnergies)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto asset = TestUtils::shareAsset(loadTetMesh(tetMesh));
  ASSERT_NE(asset, nullptr);

  auto b1 = makeDefaultFieldEnergy(
    asset, TetLinearFormulation{}, std::make_shared<StableNeoDefinition>(), std::make_shared<VolumetricPlasticity6Definition>());
  auto b2 = makeDefaultFieldEnergy(
    asset, TetLinearFormulation{}, std::make_shared<StableNeoDefinition>(), std::make_shared<VolumetricPlasticity6Definition>());

  ASSERT_NE(b1, nullptr);
  ASSERT_NE(b2, nullptr);
  EXPECT_EQ(b1->getNumDOFs(), b2->getNumDOFs());

  ES::VXd u1 = ES::VXd::Zero(b1->getNumDOFs());
  ES::VXd u2 = ES::VXd::Zero(b2->getNumDOFs());

  const double f1 = b1->func(u1);
  const double f2 = b2->func(u2);
  EXPECT_TRUE(std::isfinite(f1));
  EXPECT_TRUE(std::isfinite(f2));
  EXPECT_NEAR(f1, f2, 1e-12);

  ES::VXd g1 = ES::VXd::Zero(b1->getNumDOFs());
  ES::VXd g2 = ES::VXd::Zero(b2->getNumDOFs());
  b1->gradient(u1, g1);
  b2->gradient(u2, g2);
  for (Eigen::Index i = 0; i < g1.size(); i++) {
    EXPECT_TRUE(std::isfinite(g1[i]));
    EXPECT_TRUE(std::isfinite(g2[i]));
  }

  // Perturb only the first energy's state; second energy must be unaffected.
  u1[0] += 0.01;
  const double f1p = b1->func(u1);
  const double f2p = b2->func(u2);
  EXPECT_TRUE(std::isfinite(f1p));
  EXPECT_NEAR(f2p, f2, 1e-12);
}

// One SimulationMesh owner can be used to construct two independent cubic
// deformation energies.
TEST(DeformationModelBuilderGTest, OneMeshOwnerTwoCubicEnergies)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  auto asset = TestUtils::shareAsset(loadCubicMesh(cubicMesh));
  ASSERT_NE(asset, nullptr);

  auto b1 = makeDefaultFieldEnergy(
    asset, CubicLinearFormulation{}, std::make_shared<StableNeoDefinition>(), std::make_shared<VolumetricPlasticity6Definition>());
  auto b2 = makeDefaultFieldEnergy(
    asset, CubicLinearFormulation{}, std::make_shared<StableNeoDefinition>(), std::make_shared<VolumetricPlasticity6Definition>());

  ASSERT_NE(b1, nullptr);
  ASSERT_NE(b2, nullptr);
  EXPECT_EQ(b1->getNumDOFs(), b2->getNumDOFs());

  ES::VXd u1 = ES::VXd::Zero(b1->getNumDOFs());
  ES::VXd u2 = ES::VXd::Zero(b2->getNumDOFs());

  const double f1 = b1->func(u1);
  const double f2 = b2->func(u2);
  EXPECT_TRUE(std::isfinite(f1));
  EXPECT_TRUE(std::isfinite(f2));
  EXPECT_NEAR(f1, f2, 1e-12);

  // Hessian at zero displacement: both must produce same sparsity pattern.
  ES::SpMatD h1, h2;
  b1->hessianAlloc(h1);
  b2->hessianAlloc(h2);
  b1->hessianInPlace(u1, h1);
  b2->hessianInPlace(u2, h2);
  EXPECT_EQ(h1.nonZeros(), h2.nonZeros());
}
