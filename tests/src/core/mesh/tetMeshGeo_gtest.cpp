#include <gtest/gtest.h>

#include "tetMeshGeo.h"
#include "meshData.h"

#include <vector>

namespace
{
using pgo::Mesh::TetMeshGeo;
using pgo::Mesh::MeshData;
using pgo::Vec3d;
using pgo::Vec4i;

std::vector<Vec3d> makeSimpleVertices()
{
  return {
    Vec3d(0.0, 0.0, 0.0),
    Vec3d(1.0, 0.0, 0.0),
    Vec3d(0.0, 1.0, 0.0),
    Vec3d(0.0, 0.0, 1.0),
  };
}
}  // namespace

TEST(TetMeshGeoGTest, PreservesTetFacingApiOverMeshData)
{
  const std::vector<Vec3d> vertices = makeSimpleVertices();
  const std::vector<Vec4i> tets{Vec4i(0, 1, 2, 3)};

  const TetMeshGeo tetGeo(std::vector<Vec3d>(vertices), tets);

  EXPECT_EQ(tetGeo.numVertices(), 4);
  EXPECT_EQ(tetGeo.numTets(), 1);
  EXPECT_EQ(tetGeo.tetVtxID(0, 2), 2);
  EXPECT_EQ(tetGeo.tet(0)[3], 3);
  EXPECT_EQ(tetGeo.tets()[0][1], 1);
}

TEST(TetMeshGeoGTest, TetsViewSupportsExistingReadPatterns)
{
  const std::vector<Vec3d> vertices = makeSimpleVertices();
  const std::vector<Vec4i> tets{Vec4i(0, 1, 2, 3)};

  const TetMeshGeo tetGeo(std::vector<Vec3d>(vertices), tets);

  EXPECT_EQ(tetGeo.positions()[0][0], 0.0);
  EXPECT_EQ(tetGeo.positions()[3][2], 1.0);
  EXPECT_EQ(tetGeo.tets()[0][1], 1);
}

TEST(TetMeshGeoGTest, BridgesToAndFromMeshData)
{
  const std::vector<Vec3d> vertices = makeSimpleVertices();
  const std::vector<Vec4i> tets{Vec4i(0, 1, 2, 3)};

  const TetMeshGeo tetGeo(std::vector<Vec3d>(vertices), tets);
  const MeshData<4> meshData = tetGeo.toMeshData();

  EXPECT_EQ(meshData.numVertices(), 4);
  EXPECT_EQ(meshData.numElements(), 1);
  EXPECT_EQ(meshData.elementVtxID(0, 2), 2);
  EXPECT_EQ(meshData.elementVtxID(0, 3), 3);

  const TetMeshGeo reconstructed(meshData);
  EXPECT_EQ(reconstructed.numVertices(), 4);
  EXPECT_EQ(reconstructed.numTets(), 1);
  EXPECT_EQ(reconstructed.tetVtxID(0, 2), 2);
  EXPECT_EQ(reconstructed.tet(0)[3], 3);
}

#include "tetMesh.h"
using pgo::VolumetricMeshes::TetMesh;

TEST(TetMeshGeoGTest, ConstructsTetMeshFromMeshDataZeroCopy)
{
  const std::vector<Vec3d> vertices = makeSimpleVertices();
  const std::vector<Vec4i> tets{Vec4i(0, 1, 2, 3)};

  const TetMeshGeo tetGeo(std::vector<Vec3d>(vertices), tets);
  MeshData<4> meshData = tetGeo.toMeshData();

  // Test zero-copy move constructor
  TetMesh tetMesh(std::move(meshData), 1e6, 0.33, 1200.0);
  EXPECT_EQ(tetMesh.getNumVertices(), 4);
  EXPECT_EQ(tetMesh.getNumElements(), 1);
  EXPECT_EQ(tetMesh.getVertexIndex(0, 2), 2);
  EXPECT_NEAR(tetMesh.getVertex(0, 2)[1], 1.0, 1e-7);

  // meshData should now be empty due to move
  EXPECT_EQ(meshData.numVertices(), 0);
  EXPECT_EQ(meshData.numElements(), 0);
}

TEST(TetMeshGeoGTest, ConstructsTetMeshFromMeshDataCopy)
{
  const std::vector<Vec3d> vertices = makeSimpleVertices();
  const std::vector<Vec4i> tets{Vec4i(0, 1, 2, 3)};

  const TetMeshGeo tetGeo(std::vector<Vec3d>(vertices), tets);
  const MeshData<4> meshData = tetGeo.toMeshData();

  // Test copy constructor
  TetMesh tetMesh(meshData, 1e6, 0.33, 1200.0);
  EXPECT_EQ(tetMesh.getNumVertices(), 4);
  EXPECT_EQ(tetMesh.getNumElements(), 1);

  // meshData should NOT be empty
  EXPECT_EQ(meshData.numVertices(), 4);
  EXPECT_EQ(meshData.numElements(), 1);
}