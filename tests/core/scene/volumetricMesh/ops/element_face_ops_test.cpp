#include "ops/element_face_ops.h"

#include "../volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <array>

namespace {

TEST(ElementFaceOpsTest, TetElementFacesRespectPositiveOrientation) {
    const pgo::VolumetricMeshes::TetMesh mesh = pgo::Mesh::test::makeSingleTetMesh();

    const auto faces = pgo::VolumetricMeshes::ops::tet_element_faces(mesh, 0);

    EXPECT_EQ(pgo::VolumetricMeshes::ops::face_degree(mesh), 3);
    EXPECT_EQ(faces[0], pgo::Mesh::OTriKey(1, 2, 3));
    EXPECT_EQ(faces[1], pgo::Mesh::OTriKey(2, 0, 3));
    EXPECT_EQ(faces[2], pgo::Mesh::OTriKey(3, 0, 1));
    EXPECT_EQ(faces[3], pgo::Mesh::OTriKey(1, 0, 2));
}

TEST(ElementFaceOpsTest, TetElementFacesRespectNegativeOrientation) {
    const std::vector<pgo::Vec3d> vertices{
        pgo::Vec3d(0.0, 0.0, 0.0), pgo::Vec3d(1.0, 0.0, 0.0), pgo::Vec3d(0.0, 1.0, 0.0), pgo::Vec3d(0.0, 0.0, 1.0),
    };
    const std::vector<pgo::Vec4i> elements{pgo::Vec4i(0, 1, 3, 2)};
    const pgo::VolumetricMeshes::TetMesh mesh(vertices, elements, 1234.0, 0.31, 7.5);

    ASSERT_LT(mesh.getTetDeterminant(0), 0.0);

    const auto faces = pgo::VolumetricMeshes::ops::tet_element_faces(mesh, 0);

    EXPECT_EQ(faces[0], pgo::Mesh::OTriKey(2, 3, 1));
    EXPECT_EQ(faces[1], pgo::Mesh::OTriKey(2, 0, 3));
    EXPECT_EQ(faces[2], pgo::Mesh::OTriKey(1, 0, 2));
    EXPECT_EQ(faces[3], pgo::Mesh::OTriKey(3, 0, 1));
}

TEST(ElementFaceOpsTest, CubicElementFacesMatchCurrentFaceOrdering) {
    const pgo::VolumetricMeshes::CubicMesh mesh = pgo::Mesh::test::makeSingleCubeMesh();

    const auto faces = pgo::VolumetricMeshes::ops::cubic_element_faces(mesh, 0);

    EXPECT_EQ(pgo::VolumetricMeshes::ops::face_degree(mesh), 4);
    EXPECT_EQ(faces[0], pgo::Mesh::ORectKey(0, 3, 2, 1));
    EXPECT_EQ(faces[1], pgo::Mesh::ORectKey(4, 5, 6, 7));
    EXPECT_EQ(faces[2], pgo::Mesh::ORectKey(0, 1, 5, 4));
    EXPECT_EQ(faces[3], pgo::Mesh::ORectKey(3, 7, 6, 2));
    EXPECT_EQ(faces[4], pgo::Mesh::ORectKey(1, 2, 6, 5));
    EXPECT_EQ(faces[5], pgo::Mesh::ORectKey(0, 4, 7, 3));
}

TEST(ElementFaceOpsTest, AppendRectFaceTriangulatesUsingCurrentDiagonal) {
    std::vector<std::vector<int>> faces;
    pgo::VolumetricMeshes::ops::append_face(pgo::Mesh::ORectKey(0, 3, 2, 1), true, faces);

    ASSERT_EQ(faces.size(), 2u);
    EXPECT_EQ(faces[0], (std::vector<int>{0, 3, 2}));
    EXPECT_EQ(faces[1], (std::vector<int>{2, 1, 0}));
}

}  // namespace
