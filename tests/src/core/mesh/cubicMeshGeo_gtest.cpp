#include <gtest/gtest.h>

#include "cubicMeshGeo.h"
#include "cellMeshGeo.h"

#include <vector>

namespace
{
using pgo::Mesh::CubicMeshGeo;
using pgo::Mesh::CellMeshGeo;
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
  Vec8i cubeCell;
  cubeCell << 0, 1, 2, 3, 4, 5, 6, 7;
  const std::vector<Vec8i> cubes{cubeCell};

  const CubicMeshGeo cubicGeo(std::vector<Vec3d>(vertices), cubes);

  EXPECT_EQ(cubicGeo.numVertices(), 8);
  EXPECT_EQ(cubicGeo.numCubes(), 1);
  EXPECT_EQ(cubicGeo.cubeVtxID(0, 7), 7);
  EXPECT_EQ(cubicGeo.cube(0)[4], 4);
}

TEST(CubicMeshGeoGTest, BridgesToAndFromCellMesh)
{
  const std::vector<Vec3d> vertices = makeSimpleVertices();
  Vec8i cubeCell;
  cubeCell << 0, 1, 2, 3, 4, 5, 6, 7;
  const std::vector<Vec8i> cubes{cubeCell};

  const CubicMeshGeo cubicGeo(std::vector<Vec3d>(vertices), cubes);
  const CellMeshGeo<8> cellMesh = cubicGeo.toCellMesh();

  EXPECT_EQ(cellMesh.numVertices(), 8);
  EXPECT_EQ(cellMesh.numCells(), 1);
  EXPECT_EQ(cellMesh.cellVtxID(0, 4), 4);
  EXPECT_EQ(cellMesh.cellVtxID(0, 7), 7);

  const CubicMeshGeo reconstructed(cellMesh);
  EXPECT_EQ(reconstructed.numVertices(), 8);
  EXPECT_EQ(reconstructed.numCubes(), 1);
  EXPECT_EQ(reconstructed.cubeVtxID(0, 4), 4);
  EXPECT_EQ(reconstructed.cube(0)[7], 7);
}

#include "cubicMesh.h"
using pgo::VolumetricMeshes::CubicMesh;

TEST(CubicMeshGeoGTest, ConstructsCubicMeshFromCellMeshGeoZeroCopy)
{
  const std::vector<Vec3d> vertices = makeSimpleVertices();
  Vec8i cubeCell;
  cubeCell << 0, 1, 2, 3, 4, 5, 6, 7;
  const std::vector<Vec8i> cubes{cubeCell};

  const CubicMeshGeo cubicGeo(std::vector<Vec3d>(vertices), cubes);
  CellMeshGeo<8> cellMesh = cubicGeo.toCellMesh();

  // Test zero-copy move constructor
  CubicMesh cubicMesh(std::move(cellMesh), 1e6, 0.33, 1200.0);
  EXPECT_EQ(cubicMesh.getNumVertices(), 8);
  EXPECT_EQ(cubicMesh.getNumElements(), 1);
  EXPECT_EQ(cubicMesh.getVertexIndex(0, 4), 4);
  EXPECT_NEAR(cubicMesh.getVertex(0, 4)[1], 1.0, 1e-7);

  // cellMesh should now be empty due to move
  EXPECT_EQ(cellMesh.numVertices(), 0);
  EXPECT_EQ(cellMesh.numCells(), 0);
}

TEST(CubicMeshGeoGTest, ConstructsCubicMeshFromCellMeshGeoCopy)
{
  const std::vector<Vec3d> vertices = makeSimpleVertices();
  Vec8i cubeCell;
  cubeCell << 0, 1, 2, 3, 4, 5, 6, 7;
  const std::vector<Vec8i> cubes{cubeCell};

  const CubicMeshGeo cubicGeo(std::vector<Vec3d>(vertices), cubes);
  const CellMeshGeo<8> cellMesh = cubicGeo.toCellMesh();

  // Test copy constructor
  CubicMesh cubicMesh(cellMesh, 1e6, 0.33, 1200.0);
  EXPECT_EQ(cubicMesh.getNumVertices(), 8);
  EXPECT_EQ(cubicMesh.getNumElements(), 1);

  // cellMesh should NOT be empty
  EXPECT_EQ(cellMesh.numVertices(), 8);
  EXPECT_EQ(cellMesh.numCells(), 1);
}
