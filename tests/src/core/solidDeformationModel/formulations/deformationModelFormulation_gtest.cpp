#include <gtest/gtest.h>

#include "deformationModelFactory.h"
#include "deformationModelState.h"
#include "deformationModelEnergy.h"

#include "simulationMesh.h"
#include "tetMesh.h"
#include "cubicMesh.h"
#include "pgoLogging.h"

namespace
{
namespace ES = pgo::EigenSupport;
using pgo::SolidDeformationModel::DeformationModelElasticMaterial;
using pgo::SolidDeformationModel::DeformationModelPlasticMaterial;
using pgo::SolidDeformationModel::KoiterShellFormulation;
using pgo::SolidDeformationModel::LinearCubicFormulation;
using pgo::SolidDeformationModel::P1TetFormulation;
using pgo::SolidDeformationModel::SimulationMesh;
using pgo::SolidDeformationModel::DeformationModelState;
using pgo::SolidDeformationModel::ElasticFieldInit;
using pgo::SolidDeformationModel::PlasticFieldInit;

constexpr const char *kTorusVegPath = LIBPGO_TEST_TORUS_VEG;
constexpr const char *kCubicBoxVegPath = LIBPGO_TEST_CUBIC_BOX_VEG;

template<class FormulationT>
std::shared_ptr<pgo::SolidDeformationModel::DeformationModelEnergy> makeDefaultFieldEnergy(
  std::shared_ptr<const SimulationMesh> mesh,
  const FormulationT &formulation,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic)
{
  auto state = DeformationModelState::create(
    mesh,
    elastic,
    ElasticFieldInit{},
    plastic,
    PlasticFieldInit{});
  return pgo::SolidDeformationModel::makeDeformationEnergy(
    state,
    formulation);
}
}  // namespace

// ============================================================
// Formulation metadata checks
// ============================================================

TEST(DeformationModelFormulationGTest, P1TetFormulationProvidesCorrectMetadata)
{
  P1TetFormulation f;
  EXPECT_EQ(f.getName(), "tet_p1");
  EXPECT_EQ(f.getNodesPerElement(), 4);
  EXPECT_EQ(f.getLocalDofs(), 12);
}

TEST(DeformationModelFormulationGTest, LinearCubicFormulationProvidesCorrectMetadata)
{
  LinearCubicFormulation f;
  EXPECT_EQ(f.getName(), "hex_trilinear");
  EXPECT_EQ(f.getNodesPerElement(), 8);
  EXPECT_EQ(f.getLocalDofs(), 24);
}

TEST(DeformationModelFormulationGTest, KoiterShellFormulationProvidesCorrectMetadata)
{
  KoiterShellFormulation f;
  EXPECT_EQ(f.getName(), "shell_koiter");
  EXPECT_EQ(f.getNodesPerElement(), 6);
  EXPECT_EQ(f.getLocalDofs(), 18);
}

// ============================================================
// Smoke tests — build energy and evaluate
// ============================================================

TEST(DeformationModelFormulationGTest, TetFormulationBuildsEnergy)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  std::shared_ptr<const SimulationMesh> simMesh(pgo::SolidDeformationModel::loadTetMesh(&tetMesh).release());
  ASSERT_NE(simMesh, nullptr);
  auto bundle = makeDefaultFieldEnergy(
    simMesh,
    P1TetFormulation{},
    DeformationModelElasticMaterial::STABLE_NEO,
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);

  ASSERT_NE(bundle, nullptr);
  EXPECT_GT(bundle->getNumDOFs(), 0);

  ES::VXd u0 = ES::VXd::Zero(bundle->getNumDOFs());
  EXPECT_TRUE(std::isfinite(bundle->func(u0)));
}

TEST(DeformationModelFormulationGTest, CubicFormulationBuildsEnergy)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> simMesh(pgo::SolidDeformationModel::loadCubicMesh(&cubicMesh).release());
  ASSERT_NE(simMesh, nullptr);
  auto bundle = makeDefaultFieldEnergy(
    simMesh,
    LinearCubicFormulation{},
    DeformationModelElasticMaterial::STABLE_NEO,
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);

  ASSERT_NE(bundle, nullptr);
  EXPECT_GT(bundle->getNumDOFs(), 0);

  ES::VXd u0 = ES::VXd::Zero(bundle->getNumDOFs());
  EXPECT_TRUE(std::isfinite(bundle->func(u0)));
}
