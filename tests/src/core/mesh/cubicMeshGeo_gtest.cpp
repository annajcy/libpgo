#include <gtest/gtest.h>

#include "cubicMeshGeo.h"

#include <array>
#include <vector>

namespace
{
using pgo::Mesh::CubicMeshGeo;
using pgo::Mesh::VolumeCellType;
using pgo::Mesh::VolumeMeshGeo;
using pgo::Vec3d;

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

TEST(CubicMeshGeoGTest, ConstructsCubicGeometry)
{
  const auto vertices = makeSimpleVertices();
  const std::vector<std::array<int, 8>> cubes{std::array<int, 8>{0, 1, 2, 3, 4, 5, 6, 7}};

  const CubicMeshGeo cubicGeo(std::vector<Vec3d>(vertices), cubes);

  EXPECT_EQ(cubicGeo.numCubes(), 1);
  EXPECT_EQ(cubicGeo.cubeVtxID(0, 7), 7);
  EXPECT_EQ(cubicGeo.cube(0)[4], 4);
}

TEST(CubicMeshGeoGTest, CubesViewSupportsIndexingWithoutCopy)
{
  const auto vertices = makeSimpleVertices();
  const std::vector<std::array<int, 8>> cubes{std::array<int, 8>{0, 1, 2, 3, 4, 5, 6, 7}};

  const CubicMeshGeo cubicGeo(std::vector<Vec3d>(vertices), cubes);
  const auto cells = cubicGeo.cubes();

  EXPECT_EQ(cells.data()[0], 0);
  EXPECT_EQ(cells[0][4], 4);
}