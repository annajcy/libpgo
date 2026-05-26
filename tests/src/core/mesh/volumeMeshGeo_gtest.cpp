#include <gtest/gtest.h>

#include "volumeMeshGeo.h"

#include <array>

#include <vector>

namespace
{
using pgo::Mesh::VolumeCellType;
using pgo::Mesh::TetCellMeshGeo;
using pgo::Mesh::CubicCellMeshGeo;
using pgo::Mesh::TetCellsView;
using pgo::Mesh::CubicCellsView;
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

TEST(VolumeMeshGeoGTest, ConstructsTetGeometryAndReportsFlatCells)
{
  const auto vertices = makeSimpleVertices();
  const std::vector<std::array<int, 4>> tets{std::array<int, 4>{0, 1, 2, 3}};

  const TetCellMeshGeo geo = TetCellMeshGeo::fromCells(vertices, tets);

  EXPECT_EQ(geo.cellType(), VolumeCellType::Tet);
  EXPECT_EQ(geo.verticesPerCell(), 4);
  EXPECT_EQ(geo.numVertices(), 8);
  EXPECT_EQ(geo.numCells(), 1);
  EXPECT_EQ(geo.cellVtxID(0, 3), 3);
}

TEST(VolumeMeshGeoGTest, ConstructsCubicGeometryAndReportsFlatCells)
{
  const auto vertices = makeSimpleVertices();
  const std::vector<std::array<int, 8>> cubes{std::array<int, 8>{0, 1, 2, 3, 4, 5, 6, 7}};

  const CubicCellMeshGeo geo = CubicCellMeshGeo::fromCells(vertices, cubes);

  EXPECT_EQ(geo.cellType(), VolumeCellType::Cubic);
  EXPECT_EQ(geo.verticesPerCell(), 8);
  EXPECT_EQ(geo.numVertices(), 8);
  EXPECT_EQ(geo.numCells(), 1);
  EXPECT_EQ(geo.cellVtxID(0, 7), 7);
}

TEST(VolumeMeshGeoGTest, RejectsCellArrayWithWrongMultiple)
{
  const auto vertices = makeSimpleVertices();
  const std::vector<int> cells{0, 1, 2};

  EXPECT_THROW((TetCellMeshGeo(vertices, cells)), std::exception);
}

TEST(VolumeMeshGeoGTest, RejectsCellIndexOutOfRange)
{
  const auto vertices = makeSimpleVertices();
  const std::vector<int> cells{0, 1, 2, 99};

  EXPECT_THROW((TetCellMeshGeo(vertices, cells)), std::exception);
}

TEST(VolumeMeshGeoGTest, TetCellsViewIndexesWithoutCopy)
{
  const auto vertices = makeSimpleVertices();

  const TetCellMeshGeo geo = TetCellMeshGeo::fromCells(vertices, {std::array<int, 4>{0, 1, 2, 3}});
  const TetCellsView cells(geo.cells().data(), geo.numCells());

  EXPECT_EQ(cells.data()[0], 0);
  EXPECT_EQ(cells[0][1], 1);
}

TEST(VolumeMeshGeoGTest, CubicCellsViewIndexesWithoutCopy)
{
  const auto vertices = makeSimpleVertices();
  const std::vector<std::array<int, 8>> cubes{std::array<int, 8>{0, 1, 2, 3, 4, 5, 6, 7}};

  const CubicCellMeshGeo geo = CubicCellMeshGeo::fromCells(vertices, cubes);
  const CubicCellsView cells(geo.cells().data(), geo.numCells());

  EXPECT_EQ(cells.data()[0], 0);
  EXPECT_EQ(cells[0][4], 4);
}