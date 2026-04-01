#include "traits/mesh_traits.h"

#include "../volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <type_traits>

namespace {

using pgo::Vec3d;
using pgo::VolumetricMeshes::CubicMesh;
using pgo::VolumetricMeshes::ElementType;
using pgo::VolumetricMeshes::TetMesh;
using pgo::VolumetricMeshes::traits::CubicTag;
using pgo::VolumetricMeshes::traits::TetTag;
using pgo::VolumetricMeshes::traits::mesh_traits;

static_assert(std::is_same_v<mesh_traits<TetMesh>::tag, TetTag>);
static_assert(std::is_same_v<mesh_traits<CubicMesh>::tag, CubicTag>);
static_assert(mesh_traits<TetMesh>::kElementType == ElementType::Tet);
static_assert(mesh_traits<CubicMesh>::kElementType == ElementType::Cubic);
static_assert(mesh_traits<TetMesh>::kElementVertices == 4);
static_assert(mesh_traits<CubicMesh>::kElementVertices == 8);

TEST(MeshTraitsTest, TetTraitsDispatchElementSpecificOps) {
    const TetMesh mesh = pgo::Mesh::test::makeSingleTetMesh();

    double weights[4] = {0.0, 0.0, 0.0, 0.0};
    mesh_traits<TetMesh>::compute_barycentric_weights(mesh, 0, Vec3d(0.1, 0.2, 0.3), weights);

    EXPECT_TRUE(mesh_traits<TetMesh>::contains_vertex(mesh, 0, Vec3d(0.1, 0.2, 0.3)));
    EXPECT_DOUBLE_EQ(mesh_traits<TetMesh>::element_volume(mesh, 0), mesh.getElementVolume(0));
    pgo::Mesh::test::expectWeightsNormalized(weights, 4);
}

TEST(MeshTraitsTest, CubicTraitsDispatchElementSpecificOps) {
    const CubicMesh mesh = pgo::Mesh::test::makeSingleCubeMesh();

    double weights[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    mesh_traits<CubicMesh>::compute_barycentric_weights(mesh, 0, Vec3d(0.25, 0.5, 0.75), weights);

    EXPECT_TRUE(mesh_traits<CubicMesh>::contains_vertex(mesh, 0, Vec3d(0.25, 0.5, 0.75)));
    EXPECT_DOUBLE_EQ(mesh_traits<CubicMesh>::element_volume(mesh, 0), mesh.getElementVolume(0));
    pgo::Mesh::test::expectWeightsNormalized(weights, 8);
}

}  // namespace
