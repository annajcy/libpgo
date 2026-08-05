#include <gtest/gtest.h>

#include "simulation/simulationMesh.h"
#include "simulation/simulationMeshVolume.h"
#include "material/runtime/materialState.h"
#include "cubicMesh.h"
#include "tetMesh.h"
#include "triMeshGeo.h"
#include "volumetricMeshENuMaterial.h"
#include "volumetricMeshMooneyRivlinMaterial.h"
#include "volumetricMeshOrthotropicMaterial.h"
#include "vegFile.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
constexpr const char *kCubicBoxVegPath = LIBPGO_TEST_CUBIC_BOX_VEG;
constexpr const char *kTorusVegPath = LIBPGO_TEST_TORUS_VEG;

}

TEST(SimulationMeshGTest, LoadsCubicMeshFromExampleFile)
{
  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  auto simMesh = pgo::SolidDeformationModel::loadCubicMesh(cubicMesh);

  ASSERT_NE(simMesh, nullptr);
  EXPECT_EQ(simMesh->getElementType(), pgo::SolidDeformationModel::SimulationMeshType::CUBIC);
  EXPECT_EQ(simMesh->getNumVertices(), cubicMesh.getNumVertices());
  EXPECT_EQ(simMesh->getNumElements(), cubicMesh.getNumElements());
  EXPECT_EQ(simMesh->getNumElementVertices(), 8);

  for (int j = 0; j < 8; j++) {
    EXPECT_EQ(simMesh->getVertexIndex(0, j), cubicMesh.getVertexIndex(0, j));
  }

  const auto &simPos = simMesh->getVertex(0);
  const pgo::Vec3d cubicPos = cubicMesh.getVertex(0);
  EXPECT_DOUBLE_EQ(simPos[0], cubicPos[0]);
  EXPECT_DOUBLE_EQ(simPos[1], cubicPos[1]);
  EXPECT_DOUBLE_EQ(simPos[2], cubicPos[2]);

}

TEST(SimulationMeshGTest, LoadsTetMeshFromExampleFile)
{
  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto simMesh = pgo::SolidDeformationModel::loadTetMesh(tetMesh);

  ASSERT_NE(simMesh, nullptr);
  EXPECT_EQ(simMesh->getElementType(), pgo::SolidDeformationModel::SimulationMeshType::TET);
  EXPECT_EQ(simMesh->getNumVertices(), tetMesh.getNumVertices());
  EXPECT_EQ(simMesh->getNumElements(), tetMesh.getNumElements());
  EXPECT_EQ(simMesh->getNumElementVertices(), 4);
}

TEST(SimulationMeshGTest, ShellLoadPreservesTriangleZeroAsNeighbor)
{
  using namespace pgo;
  using namespace pgo::SolidDeformationModel;

  std::vector<Vec3d> vertices{
    Vec3d(0.0, 0.0, 0.0),
    Vec3d(1.0, 0.0, 0.0),
    Vec3d(1.0, 1.0, 0.0),
    Vec3d(0.0, 1.0, 0.0),
  };
  std::vector<Vec3i> triangles{
    Vec3i(0, 1, 2),
    Vec3i(0, 2, 3),
  };
  Mesh::TriMeshGeo surface(
    std::move(vertices), std::move(triangles));
  auto mesh = loadShellMesh(surface);

  ASSERT_NE(mesh, nullptr);
  ASSERT_EQ(mesh->getNumElements(), 2);
  ASSERT_EQ(mesh->getNumElementVertices(), 6);

  // Triangle 0 sees triangle 1 across local edge (2, 0), whose opposite
  // vertex is 3.
  EXPECT_EQ(mesh->getVertexIndex(0, 5), 3);
  // Triangle 1 sees triangle 0 across local edge (0, 2), whose opposite
  // vertex is 1. Triangle index 0 is a valid neighbor, not a boundary.
  EXPECT_EQ(mesh->getVertexIndex(1, 3), 1);
}

TEST(SimulationMeshGTest, TriangleLoadBuildsStandaloneGeometry)
{
  using namespace pgo;
  using namespace pgo::SolidDeformationModel;

  std::vector<Vec3d> vertices{
    Vec3d(0.0, 0.0, 0.0),
    Vec3d(1.0, 0.0, 0.0),
    Vec3d(1.0, 1.0, 0.0),
    Vec3d(0.0, 1.0, 0.0),
  };
  std::vector<Vec3i> triangles{
    Vec3i(0, 1, 2),
    Vec3i(0, 2, 3),
  };
  Mesh::TriMeshGeo surface(std::move(vertices), std::move(triangles));

  auto mesh = loadTriangleMesh(surface);

  ASSERT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->getElementType(), SimulationMeshType::TRIANGLE);
  EXPECT_EQ(mesh->getNumElements(), 2);
  EXPECT_EQ(mesh->getNumElementVertices(), 3);
  EXPECT_EQ(mesh->getVertexIndex(1, 0), 0);
  EXPECT_EQ(mesh->getVertexIndex(1, 1), 2);
  EXPECT_EQ(mesh->getVertexIndex(1, 2), 3);

  const std::span<const int> indices = mesh->getVertexIndices(1);
  ASSERT_EQ(indices.size(), 3u);
  EXPECT_EQ(indices[0], 0);
  EXPECT_EQ(indices[1], 2);
  EXPECT_EQ(indices[2], 3);

  computeTriangleUV(
    *mesh, 1.0);
  ASSERT_TRUE(mesh->hasElementUV());
  EXPECT_TRUE(mesh->getElementUV(0, 0).isApprox(EigenSupport::V2d::Zero()));
  EXPECT_TRUE(mesh->getElementUV(0, 1).isApprox(EigenSupport::V2d(1.0, 0.0)));
  EXPECT_TRUE(mesh->getElementUV(0, 2).isApprox(EigenSupport::V2d(1.0, 1.0)));
}

TEST(SimulationMeshGTest, EdgeQuadLoadBuildsStandaloneGeometry)
{
  using namespace pgo;
  using namespace pgo::SolidDeformationModel;

  std::vector<Vec3d> vertices{
    Vec3d(0.0, 0.0, 0.0),
    Vec3d(1.0, 0.0, 0.0),
    Vec3d(1.0, 1.0, 0.0),
    Vec3d(0.0, 1.0, 0.0),
  };
  std::vector<Vec3i> triangles{
    Vec3i(0, 1, 2),
    Vec3i(0, 2, 3),
  };
  Mesh::TriMeshGeo surface(std::move(vertices), std::move(triangles));

  auto mesh = loadEdgeQuadMesh(surface);

  ASSERT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->getElementType(), SimulationMeshType::EDGE_QUAD);
  EXPECT_EQ(mesh->getNumElements(), 1);
  EXPECT_EQ(mesh->getNumElementVertices(), 4);
  EXPECT_EQ(mesh->getVertexIndex(0, 0), 1);
  EXPECT_EQ(mesh->getVertexIndex(0, 1), 0);
  EXPECT_EQ(mesh->getVertexIndex(0, 2), 2);
  EXPECT_EQ(mesh->getVertexIndex(0, 3), 3);
}

TEST(SimulationMeshGTest, GeometryLoadIsIndependentOfMooneyRivlinPayload)
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

  auto simMesh = pgo::SolidDeformationModel::loadTetMesh(tetMesh);
  ASSERT_NE(simMesh, nullptr);
  EXPECT_EQ(simMesh->getNumElements(), 1);
  EXPECT_EQ(simMesh->getElementType(),
    pgo::SolidDeformationModel::SimulationMeshType::TET);
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

  // Geometry conversion is material-agnostic.
  const auto *enuDowncast = pgo::VolumetricMeshes::downcastENuMaterial(
    tetMesh.getElementMaterial(0));
  EXPECT_EQ(enuDowncast, nullptr);

  auto simMesh = pgo::SolidDeformationModel::loadTetMesh(tetMesh);
  ASSERT_NE(simMesh, nullptr);
  EXPECT_EQ(simMesh->getNumElements(), 1);

  // There is no ElasticModelDefinition implementation for
  // Orthotropic, and no ElasticModel3DOrthotropicStVK class exists yet.
  // This test documents that Orthotropic is payload-only at the Vega
  // level and has not yet reached the solver deformation energy path.
}

// Fixed fields and frames are independent simulation inputs; neither is
// embedded in the geometry-only SimulationMesh.
