#include <gtest/gtest.h>

#include "meshData.h"

#include <array>
#include <vector>

namespace
{
using pgo::Mesh::MeshData;
using pgo::Mesh::MeshDataType;
using pgo::Mesh::filterMeshComponentsByFace;
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

std::vector<Vec3d> makeVertices(int count)
{
  std::vector<Vec3d> vertices;
  for (int i = 0; i < count; ++i)
    vertices.emplace_back((double)i, 0.0, 0.0);
  return vertices;
}
}  // namespace

TEST(MeshDataGTest, ConstructsTetGeometryAndReportsFlatElements)
{
  const auto vertices = makeSimpleVertices();
  const std::vector<std::array<int, 4>> tets{std::array<int, 4>{0, 1, 2, 3}};

  const MeshData<4> geo = MeshData<4>::fromElements(vertices, tets);

  EXPECT_EQ(geo.meshType(), MeshDataType::Tet);
  EXPECT_EQ(geo.verticesPerElement(), 4);
  EXPECT_EQ(geo.numVertices(), 8);
  EXPECT_EQ(geo.numElements(), 1);
  EXPECT_EQ(geo.elementVtxID(0, 3), 3);
}

TEST(MeshDataGTest, ConstructsCubicGeometryAndReportsFlatElements)
{
  const auto vertices = makeSimpleVertices();
  const std::vector<std::array<int, 8>> cubes{std::array<int, 8>{0, 1, 2, 3, 4, 5, 6, 7}};

  const MeshData<8> geo = MeshData<8>::fromElements(vertices, cubes);

  EXPECT_EQ(geo.meshType(), MeshDataType::Cubic);
  EXPECT_EQ(geo.verticesPerElement(), 8);
  EXPECT_EQ(geo.numVertices(), 8);
  EXPECT_EQ(geo.numElements(), 1);
  EXPECT_EQ(geo.elementVtxID(0, 7), 7);
}

TEST(MeshDataGTest, RejectsElementArrayWithWrongMultiple)
{
  const auto vertices = makeSimpleVertices();
  const std::vector<int> elements{0, 1, 2};

  EXPECT_THROW((MeshData<4>(vertices, elements)), std::exception);
}

TEST(MeshDataGTest, RejectsElementIndexOutOfRange)
{
  const auto vertices = makeSimpleVertices();
  const std::vector<int> elements{0, 1, 2, 99};

  EXPECT_THROW((MeshData<4>(vertices, elements)), std::exception);
}

TEST(MeshDataGTest, ElementsViewIndexWithoutCopy)
{
  const auto vertices = makeSimpleVertices();

  const MeshData<4> geo = MeshData<4>::fromElements(vertices, {std::array<int, 4>{0, 1, 2, 3}});
  const auto elements = geo.elements();

  EXPECT_EQ(elements.data()[0], 0);
  EXPECT_EQ(elements[0][1], 1);
}

TEST(MeshDataGTest, FiltersTriangleComponentsBySharedEdge)
{
  const pgo::Mesh::TriMeshData mesh(makeVertices(7), {
    0, 1, 2,
    1, 3, 2,
    0, 4, 5,
  });

  const auto filtered = filterMeshComponentsByFace(mesh, 0, 1);

  EXPECT_EQ(filtered.numElements(), 2);
  EXPECT_EQ(filtered.numVertices(), 4);
}

TEST(MeshDataGTest, FiltersTetComponentsBySharedFace)
{
  const pgo::Mesh::TetMeshData mesh(makeVertices(8), {
    0, 1, 2, 3,
    0, 2, 1, 4,
    0, 1, 5, 6,
  });

  const auto filtered = filterMeshComponentsByFace(mesh, 0, 1);

  EXPECT_EQ(filtered.numElements(), 2);
  EXPECT_EQ(filtered.numVertices(), 5);
}

TEST(MeshDataGTest, FiltersCubicComponentsBySharedFace)
{
  const pgo::Mesh::CubicMeshData mesh(makeVertices(19), {
    0, 1, 2, 3, 4, 5, 6, 7,
    1, 8, 9, 2, 5, 10, 11, 6,
    0, 1, 12, 13, 14, 15, 16, 17,
  });

  const auto filtered = filterMeshComponentsByFace(mesh, 0, 1);

  EXPECT_EQ(filtered.numElements(), 2);
  EXPECT_EQ(filtered.numVertices(), 12);
}
