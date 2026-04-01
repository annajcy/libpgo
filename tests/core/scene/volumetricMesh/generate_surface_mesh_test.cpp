#include "algorithms/generate_surface_mesh.h"
#include "ops/element_face_ops.h"

#include "volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <set>
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

TEST(CoreSceneVolumetricMeshTest, GenerateSurfaceMeshAllElementFacesForAdjacentCubesIncludesInteriorFaceTwice) {
    using pgo::VolumetricMeshes::CubicElement;
    using pgo::VolumetricMeshes::CubicMesh;

    const std::vector<pgo::Vec3d> vertices{
        pgo::Vec3d(0.0, 0.0, 0.0), pgo::Vec3d(1.0, 0.0, 0.0), pgo::Vec3d(1.0, 1.0, 0.0), pgo::Vec3d(0.0, 1.0, 0.0),
        pgo::Vec3d(0.0, 0.0, 1.0), pgo::Vec3d(1.0, 0.0, 1.0), pgo::Vec3d(1.0, 1.0, 1.0), pgo::Vec3d(0.0, 1.0, 1.0),
        pgo::Vec3d(2.0, 0.0, 0.0), pgo::Vec3d(2.0, 1.0, 0.0), pgo::Vec3d(2.0, 0.0, 1.0), pgo::Vec3d(2.0, 1.0, 1.0),
    };
    const std::vector<CubicElement> elements{{0, 1, 2, 3, 4, 5, 6, 7}, {1, 8, 9, 2, 5, 10, 11, 6}};
    const CubicMesh mesh(vertices, elements, 500.0, 0.25, 3.0);

    std::vector<pgo::EigenSupport::V3d> surface_vertices;
    std::vector<std::vector<int>> surface_faces;
    pgo::VolumetricMeshes::GenerateSurfaceMesh::computeMesh(&mesh, surface_vertices, surface_faces, false, false);

    std::vector<pgo::EigenSupport::V3d> all_vertices;
    std::vector<std::vector<int>> all_faces;
    pgo::VolumetricMeshes::GenerateSurfaceMesh::computeMesh(&mesh, all_vertices, all_faces, false, true);

    EXPECT_EQ(surface_faces.size(), 10u);
    EXPECT_EQ(all_faces.size(), 12u);
}

TEST(CoreSceneVolumetricMeshTest, GenerateSurfaceMeshForNegativelyOrientedTetMatchesFaceOps) {
    const std::vector<pgo::Vec3d> vertices{
        pgo::Vec3d(0.0, 0.0, 0.0), pgo::Vec3d(1.0, 0.0, 0.0), pgo::Vec3d(0.0, 1.0, 0.0), pgo::Vec3d(0.0, 0.0, 1.0),
    };
    const std::vector<pgo::Vec4i> elements{pgo::Vec4i(0, 1, 3, 2)};
    const pgo::VolumetricMeshes::TetMesh mesh(vertices, elements, 1234.0, 0.31, 7.5);

    ASSERT_LT(mesh.getTetDeterminant(0), 0.0);

    std::vector<pgo::EigenSupport::V3d> surface_vertices;
    std::vector<std::vector<int>> surface_faces;
    pgo::VolumetricMeshes::GenerateSurfaceMesh::computeMesh(&mesh, surface_vertices, surface_faces, false, false);

    std::set<std::vector<int>> actual_faces(surface_faces.begin(), surface_faces.end());
    std::set<std::vector<int>> expected_faces;
    for (const auto& face : pgo::VolumetricMeshes::ops::tet_element_faces(mesh, 0)) {
        expected_faces.insert(std::vector<int>{face[0], face[1], face[2]});
    }

    EXPECT_EQ(surface_faces.size(), 4u);
    EXPECT_EQ(actual_faces, expected_faces);
}

}  // namespace pgo::Mesh::test
