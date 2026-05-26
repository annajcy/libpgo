#include <gtest/gtest.h>

#include "cellMeshGeo.h"

#include <array>
#include <vector>

namespace
{
using pgo::Mesh::CellMeshGeo;
using pgo::Mesh::CellMeshType;
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

TEST(CellMeshGeoGTest, ConstructsTetGeometryAndReportsFlatCells)
{
  const auto vertices = makeSimpleVertices();
  const std::vector<std::array<int, 4>> tets{std::array<int, 4>{0, 1, 2, 3}};

  const CellMeshGeo<4> geo = CellMeshGeo<4>::fromCells(vertices, tets);

  EXPECT_EQ(geo.cellType(), CellMeshType::Tet);
  EXPECT_EQ(geo.verticesPerCell(), 4);
  EXPECT_EQ(geo.numVertices(), 8);
  EXPECT_EQ(geo.numCells(), 1);
  EXPECT_EQ(geo.cellVtxID(0, 3), 3);
}

TEST(CellMeshGeoGTest, ConstructsCubicGeometryAndReportsFlatCells)
{
  const auto vertices = makeSimpleVertices();
  const std::vector<std::array<int, 8>> cubes{std::array<int, 8>{0, 1, 2, 3, 4, 5, 6, 7}};

  const CellMeshGeo<8> geo = CellMeshGeo<8>::fromCells(vertices, cubes);

  EXPECT_EQ(geo.cellType(), CellMeshType::Cubic);
  EXPECT_EQ(geo.verticesPerCell(), 8);
  EXPECT_EQ(geo.numVertices(), 8);
  EXPECT_EQ(geo.numCells(), 1);
  EXPECT_EQ(geo.cellVtxID(0, 7), 7);
}

TEST(CellMeshGeoGTest, RejectsCellArrayWithWrongMultiple)
{
  const auto vertices = makeSimpleVertices();
  const std::vector<int> cells{0, 1, 2};

  EXPECT_THROW((CellMeshGeo<4>(vertices, cells)), std::exception);
}

TEST(CellMeshGeoGTest, RejectsCellIndexOutOfRange)
{
  const auto vertices = makeSimpleVertices();
  const std::vector<int> cells{0, 1, 2, 99};

  EXPECT_THROW((CellMeshGeo<4>(vertices, cells)), std::exception);
}

TEST(CellMeshGeoGTest, CellViewsIndexWithoutCopy)
{
  const auto vertices = makeSimpleVertices();

  const CellMeshGeo<4> geo = CellMeshGeo<4>::fromCells(vertices, {std::array<int, 4>{0, 1, 2, 3}});
  const auto cells = geo.cells();

  EXPECT_EQ(cells.data()[0], 0);
  EXPECT_EQ(cells[0][1], 1);
}
