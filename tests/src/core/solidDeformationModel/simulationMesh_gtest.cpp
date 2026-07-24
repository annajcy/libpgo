#include <gtest/gtest.h>

#include "simulation/simulationMesh.h"
#include "cubicMesh.h"
#include "tetMesh.h"
#include "volumetricMeshENuMaterial.h"
#include "volumetricMeshMooneyRivlinMaterial.h"
#include "volumetricMeshOrthotropicMaterial.h"

#include <memory>

namespace
{
constexpr const char *kCubicBoxVegPath = LIBPGO_TEST_CUBIC_BOX_VEG;
constexpr const char *kTorusVegPath = LIBPGO_TEST_TORUS_VEG;
}

TEST(SimulationMeshGTest, LoadsCubicMeshFromExampleFile)
{
  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<pgo::SolidDeformationModel::SimulationMesh> simMesh =
    pgo::SolidDeformationModel::loadCubicMesh(&cubicMesh);

  ASSERT_NE(simMesh, nullptr);
  EXPECT_EQ(simMesh->getElementType(), pgo::SolidDeformationModel::SimulationMeshType::CUBIC);
  EXPECT_EQ(simMesh->getNumVertices(), cubicMesh.getNumVertices());
  EXPECT_EQ(simMesh->getNumElements(), cubicMesh.getNumElements());
  EXPECT_EQ(simMesh->getNumElementVertices(), 8);

  for (int j = 0; j < 8; j++) {
    EXPECT_EQ(simMesh->getVertexIndex(0, j), cubicMesh.getVertexIndex(0, j));
  }

  double simPos[3];
  simMesh->getVertex(0, simPos);
  const pgo::Vec3d cubicPos = cubicMesh.getVertex(0);
  EXPECT_DOUBLE_EQ(simPos[0], cubicPos[0]);
  EXPECT_DOUBLE_EQ(simPos[1], cubicPos[1]);
  EXPECT_DOUBLE_EQ(simPos[2], cubicPos[2]);

  const auto *simMat = dynamic_cast<const pgo::SolidDeformationModel::SimulationMeshENuMaterial *>(
    simMesh->getElementMaterial(0, 0));
  ASSERT_NE(simMat, nullptr);

  const auto *cubicMat = pgo::VolumetricMeshes::downcastENuMaterial(cubicMesh.getElementMaterial(0));
  ASSERT_NE(cubicMat, nullptr);
  EXPECT_DOUBLE_EQ(simMat->getE(), cubicMat->getE());
  EXPECT_DOUBLE_EQ(simMat->getNu(), cubicMat->getNu());
}

// Characterization: current tet ENu load path produces
// SimulationMeshENuMaterial payloads.
TEST(SimulationMeshGTest, TetLoadProducesENuMaterialPayloads)
{
  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  std::shared_ptr<pgo::SolidDeformationModel::SimulationMesh> simMesh =
    pgo::SolidDeformationModel::loadTetMesh(&tetMesh);

  ASSERT_NE(simMesh, nullptr);
  EXPECT_EQ(simMesh->getElementType(), pgo::SolidDeformationModel::SimulationMeshType::TET);

  // Every element must carry an ENu material payload.
  for (int ei = 0; ei < simMesh->getNumElements(); ei++) {
    const auto *simMat = dynamic_cast<const pgo::SolidDeformationModel::SimulationMeshENuMaterial *>(
      simMesh->getElementMaterial(ei, 0));
    ASSERT_NE(simMat, nullptr) << "Element " << ei << " does not have ENu material";
    EXPECT_GT(simMat->getE(), 0.0);
    EXPECT_GT(simMat->getNu(), 0.0);
  }
}

// Characterization: Mooney-Rivlin material exists at the Vega .veg /
// VolumetricMesh level before deformation conversion. The current
// loadTetMesh/loadCubicMesh only convert ENu materials; Mooney-Rivlin
// payloads exist at the Vega layer but are not yet converted to
// SimulationMeshMooneyRivlinMaterial during loading.
TEST(SimulationMeshGTest, MooneyRivlinPayloadExistsAtVegaLevel)
{
  // Build a minimal tet mesh with one element and Mooney-Rivlin material.
  const double vertices[] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };
  const int elements[] = { 0, 1, 2, 3 };

  pgo::VolumetricMeshes::VolumetricMesh::MooneyRivlinMaterial mrMat(
    "mr_test", 1000.0, 0.5, 0.3, 0.1);
  const pgo::VolumetricMeshes::VolumetricMesh::Material *materials[] = { &mrMat };

  pgo::VolumetricMeshes::VolumetricMesh::Set set("all", std::set<int>{ 0 });
  pgo::VolumetricMeshes::VolumetricMesh::Region region(0, 0);

  pgo::VolumetricMeshes::TetMesh tetMesh(
    4, vertices, 1, elements, 1, materials, 1, &set, 1, &region);

  // Material exists at Vega level with correct type.
  const auto *mat = tetMesh.getElementMaterial(0);
  ASSERT_NE(mat, nullptr);
  EXPECT_EQ(mat->getType(), pgo::VolumetricMeshes::VolumetricMesh::Material::MOONEYRIVLIN);

  const auto *mrDowncast = pgo::VolumetricMeshes::downcastMooneyRivlinMaterial(
    tetMesh.getElementMaterial(0));
  ASSERT_NE(mrDowncast, nullptr);
  EXPECT_DOUBLE_EQ(mrDowncast->getmu01(), 0.5);
  EXPECT_DOUBLE_EQ(mrDowncast->getmu10(), 0.3);
  EXPECT_DOUBLE_EQ(mrDowncast->getv1(), 0.1);

  // loadTetMesh only handles ENu; it will fail on Mooney-Rivlin.
  // This documents the current limitation.
  const auto *enuDowncast = pgo::VolumetricMeshes::downcastENuMaterial(
    tetMesh.getElementMaterial(0));
  EXPECT_EQ(enuDowncast, nullptr);
}

// Characterization: Orthotropic material payload can be read from Vega
// but there is no corresponding deformation ElasticModel yet (no
// ElasticModel3DOrthotropicStVK exists).
TEST(SimulationMeshGTest, OrthotropicPayloadExistsAtVegaLevelButNoElasticModel)
{
  const double vertices[] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };
  const int elements[] = { 0, 1, 2, 3 };

  double R[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
  pgo::VolumetricMeshes::VolumetricMesh::OrthotropicMaterial orthoMat(
    "ortho_test", 1000.0, 1e6, 1e6, 1e6, 0.3, 0.3, 0.3, 0.4e6, 0.4e6, 0.4e6, R);
  const pgo::VolumetricMeshes::VolumetricMesh::Material *materials[] = { &orthoMat };

  pgo::VolumetricMeshes::VolumetricMesh::Set set("all", std::set<int>{ 0 });
  pgo::VolumetricMeshes::VolumetricMesh::Region region(0, 0);

  pgo::VolumetricMeshes::TetMesh tetMesh(
    4, vertices, 1, elements, 1, materials, 1, &set, 1, &region);

  const auto *mat = tetMesh.getElementMaterial(0);
  ASSERT_NE(mat, nullptr);
  EXPECT_EQ(mat->getType(), pgo::VolumetricMeshes::VolumetricMesh::Material::ORTHOTROPIC);

  const auto *orthoDowncast = pgo::VolumetricMeshes::downcastOrthotropicMaterial(
    tetMesh.getElementMaterial(0));
  ASSERT_NE(orthoDowncast, nullptr);
  EXPECT_DOUBLE_EQ(orthoDowncast->getE1(), 1e6);
  EXPECT_DOUBLE_EQ(orthoDowncast->getNu12(), 0.3);
  EXPECT_DOUBLE_EQ(orthoDowncast->getG12(), 0.4e6);

  // loadTetMesh only handles ENu; it will fail on Orthotropic.
  const auto *enuDowncast = pgo::VolumetricMeshes::downcastENuMaterial(
    tetMesh.getElementMaterial(0));
  EXPECT_EQ(enuDowncast, nullptr);

  // There is no ElasticModelConfig enum entry for
  // Orthotropic, and no ElasticModel3DOrthotropicStVK class exists yet.
  // This test documents that Orthotropic is payload-only at the Vega
  // level and has not yet reached the solver deformation energy path.
}

// Characterization: Hill active-fiber path requires an extra
// SimulationMeshHillMaterial slot. Orientation is supplied separately by an
// immutable MaterialFrameField.
// This test creates a SimulationMesh with ENu + Hill materials and
// verifies that both slots exist and have the expected types.
TEST(SimulationMeshGTest, HillRequiresExtraMaterialSlot)
{
  using namespace pgo::SolidDeformationModel;

  const double vertices[] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    1.0, 1.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
    1.0, 0.0, 1.0,
    1.0, 1.0, 1.0,
    0.0, 1.0, 1.0,
  };
  const int elementVertices[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
  const int elementMaterialIndices[] = { 0 };

  SimulationMeshENuMaterial baseMaterial(1200.0, 0.45);
  const SimulationMeshMaterial *materials[] = { &baseMaterial };

  auto mesh = std::make_unique<SimulationMesh>(
    8, vertices, 1, 8, elementVertices,
    elementMaterialIndices, 1, materials,
    SimulationMeshType::CUBIC);

  ASSERT_NE(mesh, nullptr);
  // Before adding Hill, there is only the base ENu material slot.
  EXPECT_EQ(mesh->getElementNumMaterials(0), 1);

  // Adding a Hill material creates a second material slot.
  SimulationMeshHillMaterial hillMaterial(2500.0, 0.35, 1.0);
  mesh->appendMaterialToAllElements(&hillMaterial);
  EXPECT_EQ(mesh->getElementNumMaterials(0), 2);

  const auto *hillSlot = dynamic_cast<const SimulationMeshHillMaterial *>(
    mesh->getElementMaterial(0, 1));
  ASSERT_NE(hillSlot, nullptr);
  EXPECT_DOUBLE_EQ(hillSlot->getEact(), 2500.0);

  // The base material is still at slot 0.
  const auto *baseSlot = dynamic_cast<const SimulationMeshENuMaterial *>(
    mesh->getElementMaterial(0, 0));
  ASSERT_NE(baseSlot, nullptr);

  // Orientation is not stored in SimulationMesh. The deformation builder
  // supplies a MaterialFrameField (GlobalAxes by default).
}
