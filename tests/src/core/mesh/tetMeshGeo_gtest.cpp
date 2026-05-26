#include <gtest/gtest.h>

#include "tetMeshGeo.h"

#include <vector>

namespace
{
using pgo::Mesh::TetMeshGeo;
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

TEST(TetMeshGeoGTest, PreservesTetFacingApiOverCellMeshGeo)
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