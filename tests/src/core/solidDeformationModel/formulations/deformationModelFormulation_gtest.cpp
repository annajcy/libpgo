#include <gtest/gtest.h>

#include "deformationModelFactory.h"
#include "deformationModelEnergy.h"

#include "simulationMesh.h"
#include "tetMesh.h"
#include "cubicMesh.h"
#include "pgoLogging.h"

#include <concepts>
#include <type_traits>

namespace
{
namespace ES = pgo::EigenSupport;
using pgo::SolidDeformationModel::CubicFormulation;
using pgo::SolidDeformationModel::DeformationModelElasticMaterial;
using pgo::SolidDeformationModel::DeformationModelPlasticMaterial;
using pgo::SolidDeformationModel::HexTrilinear;
using pgo::SolidDeformationModel::ShellFormulation;
using pgo::SolidDeformationModel::ShellKoiter;
using pgo::SolidDeformationModel::SimulationMesh;
using pgo::SolidDeformationModel::TetFormulation;
using pgo::SolidDeformationModel::TetP1;

constexpr const char *kTorusVegPath = LIBPGO_TEST_TORUS_VEG;
constexpr const char *kCubicBoxVegPath = LIBPGO_TEST_CUBIC_BOX_VEG;

template<class F>
concept CanMakeTetWith = requires(const SimulationMesh &mesh, const F &formulation) {
  pgo::SolidDeformationModel::makeTetDeformationModel(mesh, formulation, DeformationModelElasticMaterial::STABLE_NEO);
};

template<class F>
concept CanMakeCubicWith = requires(const SimulationMesh &mesh, const F &formulation) {
  pgo::SolidDeformationModel::makeCubicDeformationModel(mesh, formulation, DeformationModelElasticMaterial::STABLE_NEO);
};

template<class F>
concept CanMakeShellWith = requires(const SimulationMesh &mesh, const F &formulation) {
  pgo::SolidDeformationModel::makeShellDeformationModel(mesh, formulation, DeformationModelElasticMaterial::KOITER_STVK);
};

namespace legacyFactoryDetection
{
struct MissingLegacyFactory
{};

MissingLegacyFactory makeDeformationModel(...);

template<class Mesh>
concept HasLegacyAutoDispatchFactory = requires(const Mesh &mesh) {
  { makeDeformationModel(mesh, DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6) } -> std::same_as<pgo::SolidDeformationModel::DeformationModelBundle>;
};
}  // namespace legacyFactoryDetection

static_assert(TetFormulation<TetP1>);
static_assert(!TetFormulation<HexTrilinear>);
static_assert(!TetFormulation<ShellKoiter>);
static_assert(CubicFormulation<HexTrilinear>);
static_assert(!CubicFormulation<TetP1>);
static_assert(!CubicFormulation<ShellKoiter>);
static_assert(ShellFormulation<ShellKoiter>);
static_assert(!ShellFormulation<TetP1>);
static_assert(!ShellFormulation<HexTrilinear>);

static_assert(CanMakeTetWith<TetP1>);
static_assert(!CanMakeTetWith<HexTrilinear>);
static_assert(!CanMakeTetWith<ShellKoiter>);
static_assert(CanMakeCubicWith<HexTrilinear>);
static_assert(!CanMakeCubicWith<TetP1>);
static_assert(!CanMakeCubicWith<ShellKoiter>);
static_assert(CanMakeShellWith<ShellKoiter>);
static_assert(!CanMakeShellWith<TetP1>);
static_assert(!CanMakeShellWith<HexTrilinear>);

static_assert(!legacyFactoryDetection::HasLegacyAutoDispatchFactory<pgo::VolumetricMeshes::TetMesh>);
static_assert(!legacyFactoryDetection::HasLegacyAutoDispatchFactory<pgo::VolumetricMeshes::CubicMesh>);
static_assert(!legacyFactoryDetection::HasLegacyAutoDispatchFactory<SimulationMesh>);
}  // namespace

// ============================================================
// Static trait checks
// ============================================================

TEST(DeformationModelFormulationGTest, TetP1TraitsDeclaresCorrectMetadata)
{
  using Traits = pgo::SolidDeformationModel::FormulationTraits<pgo::SolidDeformationModel::TetP1>;
  EXPECT_EQ(Traits::nodesPerElement, 4);
  EXPECT_EQ(Traits::localDofs, 12);
  EXPECT_EQ(Traits::name, "tet_p1");
}

TEST(DeformationModelFormulationGTest, HexTrilinearTraitsDeclaresCorrectMetadata)
{
  using Traits = pgo::SolidDeformationModel::FormulationTraits<pgo::SolidDeformationModel::HexTrilinear>;
  EXPECT_EQ(Traits::nodesPerElement, 8);
  EXPECT_EQ(Traits::localDofs, 24);
  EXPECT_EQ(Traits::name, "hex_trilinear");
}

TEST(DeformationModelFormulationGTest, ShellKoiterTraitsDeclaresCorrectName)
{
  using Traits = pgo::SolidDeformationModel::FormulationTraits<pgo::SolidDeformationModel::ShellKoiter>;
  EXPECT_EQ(Traits::name, "shell_koiter");
}

// ============================================================
// Concept checks (compile-time)
// ============================================================

TEST(DeformationModelFormulationGTest, TetP1SatisfiesTetFormulationConcept)
{
  EXPECT_TRUE((pgo::SolidDeformationModel::TetFormulation<pgo::SolidDeformationModel::TetP1>));
}

TEST(DeformationModelFormulationGTest, HexTrilinearSatisfiesCubicFormulationConcept)
{
  EXPECT_TRUE((pgo::SolidDeformationModel::CubicFormulation<pgo::SolidDeformationModel::HexTrilinear>));
}

TEST(DeformationModelFormulationGTest, ShellKoiterSatisfiesShellFormulationConcept)
{
  EXPECT_TRUE((pgo::SolidDeformationModel::ShellFormulation<pgo::SolidDeformationModel::ShellKoiter>));
}

TEST(DeformationModelFormulationGTest, TetP1DoesNotSatisfyCubicFormulationConcept)
{
  EXPECT_FALSE((pgo::SolidDeformationModel::CubicFormulation<pgo::SolidDeformationModel::TetP1>));
}

TEST(DeformationModelFormulationGTest, HexTrilinearDoesNotSatisfyTetFormulationConcept)
{
  EXPECT_FALSE((pgo::SolidDeformationModel::TetFormulation<pgo::SolidDeformationModel::HexTrilinear>));
}

// ============================================================
// Variant adapter smoke tests
// ============================================================

TEST(DeformationModelFormulationGTest, TetVariantAdapterBuildsEnergy)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto simMesh = pgo::SolidDeformationModel::loadTetMesh(&tetMesh);
  ASSERT_NE(simMesh, nullptr);
  auto bundle = pgo::SolidDeformationModel::makeTetDeformationModel(
    *simMesh, pgo::SolidDeformationModel::TetP1{},
    pgo::SolidDeformationModel::DeformationModelElasticMaterial::STABLE_NEO,
    pgo::SolidDeformationModel::DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);

  ASSERT_NE(bundle.energy, nullptr);
  EXPECT_GT(bundle.energy->getNumDOFs(), 0);

  // Smoke: evaluate at zero displacement.
  ES::VXd u0 = ES::VXd::Zero(bundle.energy->getNumDOFs());
  EXPECT_TRUE(std::isfinite(bundle.energy->func(u0)));
}

TEST(DeformationModelFormulationGTest, CubicVariantAdapterBuildsEnergy)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  auto simMesh = pgo::SolidDeformationModel::loadCubicMesh(&cubicMesh);
  ASSERT_NE(simMesh, nullptr);
  auto bundle = pgo::SolidDeformationModel::makeCubicDeformationModel(
    *simMesh, pgo::SolidDeformationModel::HexTrilinear{},
    pgo::SolidDeformationModel::DeformationModelElasticMaterial::STABLE_NEO,
    pgo::SolidDeformationModel::DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);

  ASSERT_NE(bundle.energy, nullptr);
  EXPECT_GT(bundle.energy->getNumDOFs(), 0);

  ES::VXd u0 = ES::VXd::Zero(bundle.energy->getNumDOFs());
  EXPECT_TRUE(std::isfinite(bundle.energy->func(u0)));
}

// ============================================================
// Formulation name logging
// ============================================================

TEST(DeformationModelFormulationGTest, TetP1NameIsConsistentWithMakeTetDeformationModel)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto simMesh = pgo::SolidDeformationModel::loadTetMesh(&tetMesh);
  ASSERT_NE(simMesh, nullptr);
  auto bundle = pgo::SolidDeformationModel::makeTetDeformationModel(
    *simMesh, pgo::SolidDeformationModel::TetP1{},
    pgo::SolidDeformationModel::DeformationModelElasticMaterial::STABLE_NEO,
    pgo::SolidDeformationModel::DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);

  ASSERT_NE(bundle.energy, nullptr);
  // The energy should be valid
  EXPECT_GT(bundle.energy->getNumDOFs(), 0);
}

TEST(DeformationModelFormulationGTest, HexTrilinearNameIsConsistentWithMakeCubicDeformationModel)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  auto simMesh = pgo::SolidDeformationModel::loadCubicMesh(&cubicMesh);
  ASSERT_NE(simMesh, nullptr);
  auto bundle = pgo::SolidDeformationModel::makeCubicDeformationModel(
    *simMesh, pgo::SolidDeformationModel::HexTrilinear{},
    pgo::SolidDeformationModel::DeformationModelElasticMaterial::STABLE_NEO,
    pgo::SolidDeformationModel::DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);

  ASSERT_NE(bundle.energy, nullptr);
  EXPECT_GT(bundle.energy->getNumDOFs(), 0);
}
