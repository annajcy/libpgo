#include <gtest/gtest.h>

#include "triMeshGeo.h"
#include "meshData.h"

#include <vector>

namespace
{
using pgo::Mesh::TriMeshGeo;
using pgo::Mesh::MeshData;
using pgo::Vec3d;
using pgo::Vec3i;

TriMeshGeo makeUnitSquareMesh()
{
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
  return TriMeshGeo(std::move(vertices), std::move(triangles));
}
}  // namespace

TEST(TriMeshGeoGTest, PreservesTriangleFacingApi)
{
  const auto triGeo = makeUnitSquareMesh();

  EXPECT_EQ(triGeo.numVertices(), 4);
  EXPECT_EQ(triGeo.numTriangles(), 2);
  EXPECT_EQ(triGeo.triVtxID(0, 1), 1);
  EXPECT_EQ(triGeo.tri(1)[2], 3);
  EXPECT_EQ(triGeo.triangles()[0][0], 0);
}

TEST(TriMeshGeoGTest, TrianglesViewSupportsExistingReadPatterns)
{
  const auto triGeo = makeUnitSquareMesh();

  EXPECT_EQ(triGeo.positions()[0][0], 0.0);
  EXPECT_EQ(triGeo.positions()[2][1], 1.0);
  EXPECT_EQ(triGeo.triangles()[1][1], 2);
}

TEST(TriMeshGeoGTest, BridgesToAndFromMeshData)
{
  const auto triGeo = makeUnitSquareMesh();
  const MeshData<3> meshData = triGeo.toMeshData();

  EXPECT_EQ(meshData.numVertices(), 4);
  EXPECT_EQ(meshData.numElements(), 2);
  EXPECT_EQ(meshData.elementVtxID(0, 1), 1);
  EXPECT_EQ(meshData.elementVtxID(1, 2), 3);

  const TriMeshGeo reconstructed(meshData);
  EXPECT_EQ(reconstructed.numVertices(), 4);
  EXPECT_EQ(reconstructed.numTriangles(), 2);
  EXPECT_EQ(reconstructed.triVtxID(0, 1), 1);
  EXPECT_EQ(reconstructed.tri(1)[2], 3);
}