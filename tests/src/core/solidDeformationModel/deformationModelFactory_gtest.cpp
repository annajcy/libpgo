#include <gtest/gtest.h>

#include "deformationModelFactory.h"
#include "deformationModelState.h"

#include "deformationModelEnergy.h"
#include "simulationMesh.h"
#include "tetMesh.h"
#include "cubicMesh.h"
#include "triMeshGeo.h"
#include "pgoLogging.h"

#include <cmath>

namespace
{
namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

constexpr const char *kTorusVegPath = LIBPGO_TEST_TORUS_VEG;
constexpr const char *kCubicBoxVegPath = LIBPGO_TEST_CUBIC_BOX_VEG;
constexpr const char *kShellObjPath = LIBPGO_TEST_SHELL_OBJ;

template<class FormulationT>
std::shared_ptr<DeformationModelEnergy> makeDefaultFieldEnergy(
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
  return makeDeformationEnergy(state, formulation);
}
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
  std::shared_ptr<const SimulationMesh> simMesh(loadTetMesh(&tetMesh).release());
  ASSERT_NE(simMesh, nullptr);
  auto energy = makeDefaultFieldEnergy(
    simMesh, P1TetFormulation{}, DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);

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

// Baseline: cubic deformation energy at zero displacement.
// State convention: func(x) computes energy at restPosition + x.
TEST(DeformationModelFactoryGTest, CubicZeroDisplacementBaseline)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> simMesh(loadCubicMesh(&cubicMesh).release());
  ASSERT_NE(simMesh, nullptr);
  auto energy = makeDefaultFieldEnergy(
    simMesh, LinearCubicFormulation{}, DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);

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

// MakeTetDeformationModel with SimulationMesh reference validates TET topology.
TEST(DeformationModelFactoryGTest, TetSimulationMeshFactoryValidatesTopology)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  std::shared_ptr<const SimulationMesh> simMesh(loadTetMesh(&tetMesh).release());
  ASSERT_NE(simMesh, nullptr);

  auto energy = makeDefaultFieldEnergy(
    simMesh, P1TetFormulation{}, DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);
  ASSERT_NE(energy, nullptr);
  EXPECT_GT(energy->getNumDOFs(), 0);
}

// MakeCubicDeformationModel with SimulationMesh reference validates CUBIC topology.
TEST(DeformationModelFactoryGTest, CubicSimulationMeshFactoryValidatesTopology)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> simMesh(loadCubicMesh(&cubicMesh).release());
  ASSERT_NE(simMesh, nullptr);

  auto energy = makeDefaultFieldEnergy(
    simMesh, LinearCubicFormulation{}, DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);
  ASSERT_NE(energy, nullptr);
  EXPECT_GT(energy->getNumDOFs(), 0);
}

// MakeShellDeformationModel with SimulationMesh reference validates SHELL topology
// and uses the existing Koiter shell path.
TEST(DeformationModelFactoryGTest, ShellSimulationMeshFactoryValidatesTopology)
{
  pgo::Logging::init();

  pgo::Mesh::TriMeshGeo surfaceMesh;
  ASSERT_TRUE(surfaceMesh.load(kShellObjPath));
  SimulationMeshENuhMaterial shellMaterial(1000.0, 0.45, 1e-3);
  std::shared_ptr<const SimulationMesh> simMesh(loadShellMesh(surfaceMesh, &shellMaterial).release());
  ASSERT_NE(simMesh, nullptr);

  auto energy = makeDefaultFieldEnergy(
    simMesh, KoiterShellFormulation{}, DeformationModelElasticMaterial::KOITER_STVK, DeformationModelPlasticMaterial::SHELL_FF_DOF1);

  ASSERT_NE(energy, nullptr);
  EXPECT_GT(energy->getNumDOFs(), 0);
  EXPECT_EQ(energy->getNumDOFs(), simMesh->getNumVertices() * 3);

  ES::VXd u0 = ES::VXd::Zero(energy->getNumDOFs());
  EXPECT_TRUE(std::isfinite(energy->func(u0)));
}

// Wrong topology/SimulationMesh type fails at runtime.
TEST(DeformationModelFactoryGTest, TetFactoryRejectsCubicSimulationMesh)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> simMesh(loadCubicMesh(&cubicMesh).release());
  ASSERT_NE(simMesh, nullptr);

  EXPECT_THROW(
    makeDefaultFieldEnergy(simMesh, P1TetFormulation{}, DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6),
    std::invalid_argument);
}

// One SimulationMesh owner can be used to construct two independent deformation
// energies. Both must remain evaluable while the owner is alive.
TEST(DeformationModelFactoryGTest, OneMeshOwnerTwoTetEnergies)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  std::shared_ptr<const SimulationMesh> simMesh(loadTetMesh(&tetMesh).release());
  ASSERT_NE(simMesh, nullptr);

  auto b1 = makeDefaultFieldEnergy(
    simMesh, P1TetFormulation{}, DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);
  auto b2 = makeDefaultFieldEnergy(
    simMesh, P1TetFormulation{}, DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);

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
TEST(DeformationModelFactoryGTest, OneMeshOwnerTwoCubicEnergies)
{
  pgo::Logging::init();

  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> simMesh(loadCubicMesh(&cubicMesh).release());
  ASSERT_NE(simMesh, nullptr);

  auto b1 = makeDefaultFieldEnergy(
    simMesh, LinearCubicFormulation{}, DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);
  auto b2 = makeDefaultFieldEnergy(
    simMesh, LinearCubicFormulation{}, DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);

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
