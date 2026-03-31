#include "generateSurfaceMesh.h"

#include "volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <vector>

namespace pgo::Mesh::test {

TEST(CoreSceneVolumetricMeshTest, GenerateSurfaceMeshForTetProducesFourFaces) {
    pgo::VolumetricMeshes::TetMesh mesh = makeSingleTetMesh();

    std::vector<pgo::EigenSupport::V3d> vertices;
    std::vector<std::vector<int>>       faces;
    pgo::VolumetricMeshes::GenerateSurfaceMesh::computeMesh(&mesh, vertices, faces, false, false);

    EXPECT_EQ(static_cast<int>(vertices.size()), mesh.getNumVertices());
    EXPECT_EQ(faces.size(), 4u);
    for (const auto& face : faces) {
        EXPECT_EQ(face.size(), 3u);
    }
}

TEST(CoreSceneVolumetricMeshTest, GenerateSurfaceMeshForCubeProducesSixQuads) {
    pgo::VolumetricMeshes::CubicMesh mesh = makeSingleCubeMesh();

    std::vector<pgo::EigenSupport::V3d> vertices;
    std::vector<std::vector<int>>       faces;
    pgo::VolumetricMeshes::GenerateSurfaceMesh::computeMesh(&mesh, vertices, faces, false, false);

    EXPECT_EQ(static_cast<int>(vertices.size()), mesh.getNumVertices());
    EXPECT_EQ(faces.size(), 6u);
    for (const auto& face : faces) {
        EXPECT_EQ(face.size(), 4u);
    }
}

TEST(CoreSceneVolumetricMeshTest, GenerateSurfaceMeshForCubeTriangulatedProducesTwelveTriangles) {
    pgo::VolumetricMeshes::CubicMesh mesh = makeSingleCubeMesh();

    std::vector<pgo::EigenSupport::V3d> vertices;
    std::vector<std::vector<int>>       faces;
    pgo::VolumetricMeshes::GenerateSurfaceMesh::computeMesh(&mesh, vertices, faces, true, false);

    EXPECT_EQ(static_cast<int>(vertices.size()), mesh.getNumVertices());
    EXPECT_EQ(faces.size(), 12u);
    for (const auto& face : faces) {
        EXPECT_EQ(face.size(), 3u);
    }
}

}  // namespace pgo::Mesh::test
