#include <gtest/gtest.h>

#include "deformationModelFactory.h"

#include "deformationModelEnergy.h"
#include "simulationMesh.h"
#include "tetMesh.h"
#include "cubicMesh.h"
#include "pgoLogging.h"

#include <cmath>

namespace
{
namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

constexpr const char *kTorusVegPath = LIBPGO_TEST_TORUS_VEG;
constexpr const char *kCubicBoxVegPath = LIBPGO_TEST_CUBIC_BOX_VEG;
}  // namespace

TEST(DeformationModelFactoryGTest, MakeSimulationMeshReturnsOwner)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto mesh = makeSimulationMesh(tetMesh);
  ASSERT_NE(mesh, nullptr);
  EXPECT_GT(mesh->getNumVertices(), 0);
  EXPECT_GT(mesh->getNumElements(), 0);
}

// Baseline: tet deformation energy at zero displacement has near-zero energy
// and finite gradient. State x is displacement from rest, NOT absolute position.
TEST(DeformationModelFactoryGTest, TetZeroDisplacementBaseline)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto simMesh = loadTetMesh(&tetMesh);
  ASSERT_NE(simMesh, nullptr);
  DeformationModelBundle bundle = makeTetDeformationModel(
    *simMesh, TetP1{}, DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);

  ASSERT_NE(bundle.energy, nullptr);
  EXPECT_GT(bundle.energy->getNumDOFs(), 0);

  ES::VXd u0 = ES::VXd::Zero(bundle.energy->getNumDOFs());
  const double f0 = bundle.energy->func(u0);
  EXPECT_TRUE(std::isfinite(f0));
  EXPECT_NEAR(f0, 0.0, 1e-10);

  ES::VXd grad0 = ES::VXd::Zero(bundle.energy->getNumDOFs());
  bundle.energy->gradient(u0, grad0);
  for (Eigen::Index i = 0; i < grad0.size(); i++)
    EXPECT_TRUE(std::isfinite(grad0[i])) << "Non-finite gradient entry at " << i;

  ES::SpMatD h0;
  bundle.energy->createHessian(h0);
  bundle.energy->hessian(u0, h0);
  for (Eigen::Index i = 0; i < h0.nonZeros(); i++)
    EXPECT_TRUE(std::isfinite(h0.valuePtr()[i])) << "Non-finite Hessian entry at " << i;
}

// Baseline: cubic deformation energy at zero displacement.
// State convention: func(x) computes energy at restPosition + x.
TEST(DeformationModelFactoryGTest, CubicZeroDisplacementBaseline)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  auto simMesh = loadCubicMesh(&cubicMesh);
  ASSERT_NE(simMesh, nullptr);
  DeformationModelBundle bundle = makeCubicDeformationModel(
    *simMesh, HexTrilinear{}, DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);

  ASSERT_NE(bundle.energy, nullptr);
  EXPECT_GT(bundle.energy->getNumDOFs(), 0);

  ES::VXd u0 = ES::VXd::Zero(bundle.energy->getNumDOFs());
  const double f0 = bundle.energy->func(u0);
  EXPECT_TRUE(std::isfinite(f0));
  EXPECT_NEAR(f0, 0.0, 1e-10);

  ES::VXd grad0 = ES::VXd::Zero(bundle.energy->getNumDOFs());
  bundle.energy->gradient(u0, grad0);
  for (Eigen::Index i = 0; i < grad0.size(); i++)
    EXPECT_TRUE(std::isfinite(grad0[i])) << "Non-finite gradient entry at " << i;

  ES::SpMatD h0;
  bundle.energy->createHessian(h0);
  bundle.energy->hessian(u0, h0);
  for (Eigen::Index i = 0; i < h0.nonZeros(); i++)
    EXPECT_TRUE(std::isfinite(h0.valuePtr()[i])) << "Non-finite Hessian entry at " << i;
}

// MakeTetDeformationModel with SimulationMesh reference validates TET topology.
TEST(DeformationModelFactoryGTest, TetSimulationMeshFactoryValidatesTopology)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto simMesh = loadTetMesh(&tetMesh);
  ASSERT_NE(simMesh, nullptr);

  DeformationModelBundle bundle = makeTetDeformationModel(
    *simMesh, TetP1{}, DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);
  ASSERT_NE(bundle.energy, nullptr);
  EXPECT_GT(bundle.energy->getNumDOFs(), 0);
}

// MakeCubicDeformationModel with SimulationMesh reference validates CUBIC topology.
TEST(DeformationModelFactoryGTest, CubicSimulationMeshFactoryValidatesTopology)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  auto simMesh = loadCubicMesh(&cubicMesh);
  ASSERT_NE(simMesh, nullptr);

  DeformationModelBundle bundle = makeCubicDeformationModel(
    *simMesh, HexTrilinear{}, DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);
  ASSERT_NE(bundle.energy, nullptr);
  EXPECT_GT(bundle.energy->getNumDOFs(), 0);
}

// Wrong topology/SimulationMesh type fails at runtime.
TEST(DeformationModelFactoryGTest, TetFactoryRejectsCubicSimulationMesh)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  auto simMesh = loadCubicMesh(&cubicMesh);
  ASSERT_NE(simMesh, nullptr);

  EXPECT_THROW(
    makeTetDeformationModel(*simMesh, TetP1{}, DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6),
    std::invalid_argument);
}
