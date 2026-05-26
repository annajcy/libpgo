#include <gtest/gtest.h>

#include "cubicMeshGeo.h"
#include "meshData.h"

#include <vector>

namespace
{
using pgo::Mesh::CubicMeshGeo;
using pgo::Mesh::MeshData;
using pgo::Vec3d;
using pgo::Vec8i;

std::vector<Vec3d> makeSimpleVertices()
{
  return {
    Vec3d(0.0, 0.0, 0.0),
    Vec3d(1.0, 0.0, 0.0),
    Vec3d(0.0, 1.0, 0.0),
    Vec3d(0.0, 0.0, 1.0),
    Vec3d(1.0, 1.0, 0.0),
    Vec3d(1.0, 0.0, 1.0),
    Vec3d(0.0, 1.0, 1.0),
    Vec3d(1.0, 1.0, 1.0),
  };
}
}  // namespace

TEST(CubicMeshGeoGTest, PreservesCubicFacingApi)
{
  const std::vector<Vec3d> vertices = makeSimpleVertices();
  Vec8i cubeElement;
  cubeElement << 0, 1, 2, 3, 4, 5, 6, 7;
  const std::vector<Vec8i> cubes{cubeElement};

  const CubicMeshGeo cubicGeo(std::vector<Vec3d>(vertices), cubes);

  EXPECT_EQ(cubicGeo.numVertices(), 8);
  EXPECT_EQ(cubicGeo.numCubes(), 1);
  EXPECT_EQ(cubicGeo.cubeVtxID(0, 7), 7);
  EXPECT_EQ(cubicGeo.cube(0)[4], 4);
}

TEST(CubicMeshGeoGTest, BridgesToAndFromMeshData)
{
  const std::vector<Vec3d> vertices = makeSimpleVertices();
  Vec8i cubeElement;
  cubeElement << 0, 1, 2, 3, 4, 5, 6, 7;
  const std::vector<Vec8i> cubes{cubeElement};

  const CubicMeshGeo cubicGeo(std::vector<Vec3d>(vertices), cubes);
  const MeshData<8> meshData = cubicGeo.toMeshData();

  EXPECT_EQ(meshData.numVertices(), 8);
  EXPECT_EQ(meshData.numElements(), 1);
  EXPECT_EQ(meshData.elementVtxID(0, 4), 4);
  EXPECT_EQ(meshData.elementVtxID(0, 7), 7);

  const CubicMeshGeo reconstructed(meshData);
  EXPECT_EQ(reconstructed.numVertices(), 8);
  EXPECT_EQ(reconstructed.numCubes(), 1);
  EXPECT_EQ(reconstructed.cubeVtxID(0, 4), 4);
  EXPECT_EQ(reconstructed.cube(0)[7], 7);
}

#include "cubicMesh.h"
using pgo::VolumetricMeshes::CubicMesh;

TEST(CubicMeshGeoGTest, ConstructsCubicMeshFromMeshDataZeroCopy)
{
  const std::vector<Vec3d> vertices = makeSimpleVertices();
  Vec8i cubeElement;
  cubeElement << 0, 1, 2, 3, 4, 5, 6, 7;
  const std::vector<Vec8i> cubes{cubeElement};

  const CubicMeshGeo cubicGeo(std::vector<Vec3d>(vertices), cubes);
  MeshData<8> meshData = cubicGeo.toMeshData();

  // Test zero-copy move constructor
  CubicMesh cubicMesh(std::move(meshData), 1e6, 0.33, 1200.0);
  EXPECT_EQ(cubicMesh.getNumVertices(), 8);
  EXPECT_EQ(cubicMesh.getNumElements(), 1);
  EXPECT_EQ(cubicMesh.getVertexIndex(0, 4), 4);
  EXPECT_NEAR(cubicMesh.getVertex(0, 4)[1], 1.0, 1e-7);

  // meshData should now be empty due to move
  EXPECT_EQ(meshData.numVertices(), 0);
  EXPECT_EQ(meshData.numElements(), 0);
}

TEST(CubicMeshGeoGTest, ConstructsCubicMeshFromMeshDataCopy)
{
  const std::vector<Vec3d> vertices = makeSimpleVertices();
  Vec8i cubeElement;
  cubeElement << 0, 1, 2, 3, 4, 5, 6, 7;
  const std::vector<Vec8i> cubes{cubeElement};

  const CubicMeshGeo cubicGeo(std::vector<Vec3d>(vertices), cubes);
  const MeshData<8> meshData = cubicGeo.toMeshData();

  // Test copy constructor
  CubicMesh cubicMesh(meshData, 1e6, 0.33, 1200.0);
  EXPECT_EQ(cubicMesh.getNumVertices(), 8);
  EXPECT_EQ(cubicMesh.getNumElements(), 1);

  // meshData should NOT be empty
  EXPECT_EQ(meshData.numVertices(), 8);
  EXPECT_EQ(meshData.numElements(), 1);
}
