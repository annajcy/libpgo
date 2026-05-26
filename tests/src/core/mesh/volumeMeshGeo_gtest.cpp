#include <gtest/gtest.h>

#include "volumeMeshGeo.h"

#include <vector>

namespace
{
using pgo::Mesh::VolumeCellType;
using pgo::Mesh::VolumeMeshGeo;
using pgo::Vec3d;
using pgo::Vec4i;

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
  const std::vector<Vec4i> tets{Vec4i(0, 1, 2, 3)};

  const VolumeMeshGeo geo = VolumeMeshGeo::fromTets(vertices, tets);

  EXPECT_EQ(geo.cellType(), VolumeCellType::Tet);
  EXPECT_EQ(geo.verticesPerCell(), 4);
  EXPECT_EQ(geo.numVertices(), 8);
  EXPECT_EQ(geo.numCells(), 1);
  EXPECT_EQ(geo.cellVtxID(0, 3), 3);
}

TEST(VolumeMeshGeoGTest, RejectsCellArrayWithWrongMultiple)
{
  const auto vertices = makeSimpleVertices();
  const std::vector<int> cells{0, 1, 2};

  EXPECT_THROW((VolumeMeshGeo(pgo::Mesh::VolumeCellType::Tet, vertices, cells)), std::exception);
}

TEST(VolumeMeshGeoGTest, RejectsCellIndexOutOfRange)
{
  const auto vertices = makeSimpleVertices();
  const std::vector<int> cells{0, 1, 2, 99};

  EXPECT_THROW((VolumeMeshGeo(pgo::Mesh::VolumeCellType::Tet, vertices, cells)), std::exception);
}

TEST(VolumeMeshGeoGTest, TetCellsViewIndexesWithoutCopy)
{
  const auto vertices = makeSimpleVertices();
  const std::vector<Vec4i> tets{Vec4i(0, 1, 2, 3)};

  const VolumeMeshGeo geo = VolumeMeshGeo::fromTets(vertices, tets);
  const auto cells = geo.cells();

  EXPECT_EQ(cells.data()[0], 0);
  EXPECT_EQ(cells[0][1], 1);
}