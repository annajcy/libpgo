#include <gtest/gtest.h>

#include "energy/deformationEnergyBuilder.h"
#include "deformation/deformationModelState.h"
#include "energy/deformationModelEnergy.h"
#include "deformation/deformationModelAssembler.h"
#include "deformation/deformationModelManager.h"

#include "formulations/formulation.h"
#include "formulations/dof/dofLayout.h"

#include "simulation/simulationMesh.h"
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

// ============================================================
// Formulation DOF-layout / rest-state policy (vertex3 defaults)
//
// The formulation now owns the DOF layout and the global rest vector (the seam a tricubic Hermite
// formulation overrides). The default must reproduce the historical Vertex3 + vertex-position
// behavior exactly, and the manager must surface it. These pin both.
// ============================================================

template<class FormulationT>
void checkVertex3DefaultPolicy(const SimulationMesh &mesh, const FormulationT &f)
{
  auto layout = f.createDofLayout(mesh);
  ASSERT_NE(layout, nullptr);
  EXPECT_EQ(layout->numGlobalDofs(), mesh.getNumVertices() * 3);

  ES::VXd rest = f.buildGlobalRestDofs(mesh);
  ASSERT_EQ(rest.size(), static_cast<Eigen::Index>(mesh.getNumVertices()) * 3);

  // Default rest DOFs are exactly the vertex positions.
  for (int vi = 0; vi < mesh.getNumVertices(); vi++) {
    double p[3];
    mesh.getVertex(vi, p);
    EXPECT_DOUBLE_EQ(rest[vi * 3 + 0], p[0]);
    EXPECT_DOUBLE_EQ(rest[vi * 3 + 1], p[1]);
    EXPECT_DOUBLE_EQ(rest[vi * 3 + 2], p[2]);
  }

  // The invariant the assembler/energy rely on.
  EXPECT_EQ(rest.size(), static_cast<Eigen::Index>(layout->numGlobalDofs()));
}

TEST(DeformationModelFormulationGTest, TetFormulationVertex3PolicyDefaults)
{
  pgo::Logging::init();
  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  std::shared_ptr<const SimulationMesh> simMesh(pgo::SolidDeformationModel::loadTetMesh(&tetMesh).release());
  ASSERT_NE(simMesh, nullptr);
  checkVertex3DefaultPolicy(*simMesh, P1TetFormulation{});
}

TEST(DeformationModelFormulationGTest, CubicFormulationVertex3PolicyDefaults)
{
  pgo::Logging::init();
  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> simMesh(pgo::SolidDeformationModel::loadCubicMesh(&cubicMesh).release());
  ASSERT_NE(simMesh, nullptr);
  checkVertex3DefaultPolicy(*simMesh, LinearCubicFormulation{});
}

TEST(DeformationModelFormulationGTest, ManagerSurfacesFormulationRestInvariant)
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

  const auto &manager = bundle->assembler().getDeformationModelManager();
  auto layout = manager.createDofLayout();
  ASSERT_NE(layout, nullptr);
  EXPECT_EQ(layout->numGlobalDofs(), simMesh->getNumVertices() * 3);
  EXPECT_EQ(manager.buildRestPosition().size(),
    static_cast<Eigen::Index>(layout->numGlobalDofs()));
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
