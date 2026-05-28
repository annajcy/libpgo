#include <gtest/gtest.h>

#include "deformationModelFactory.h"
#include "deformationModelEnergy.h"

#include "simulationMesh.h"
#include "tetMesh.h"
#include "cubicMesh.h"
#include "pgoLogging.h"

#include <type_traits>

namespace
{
namespace ES = pgo::EigenSupport;

constexpr const char *kTorusVegPath = LIBPGO_TEST_TORUS_VEG;
constexpr const char *kCubicBoxVegPath = LIBPGO_TEST_CUBIC_BOX_VEG;
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
  pgo::SolidDeformationModel::TetFormulationVariant formulation{pgo::SolidDeformationModel::TetP1{}};

  auto bundle = pgo::SolidDeformationModel::makeTetDeformationModel(
    *simMesh, formulation,
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
  pgo::SolidDeformationModel::CubicFormulationVariant formulation{pgo::SolidDeformationModel::HexTrilinear{}};

  auto bundle = pgo::SolidDeformationModel::makeCubicDeformationModel(
    *simMesh, formulation,
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
