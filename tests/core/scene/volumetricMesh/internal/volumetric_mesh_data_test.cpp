#include "internal/volumetric_mesh_data.h"

#include <gtest/gtest.h>

#include <array>
#include <stdexcept>
#include <vector>

namespace pgo::Mesh::test {
namespace {

void expectVec3dEq(const pgo::Vec3d& actual, const pgo::Vec3d& expected) {
    EXPECT_DOUBLE_EQ(actual[0], expected[0]);
    EXPECT_DOUBLE_EQ(actual[1], expected[1]);
    EXPECT_DOUBLE_EQ(actual[2], expected[2]);
}

}  // namespace

TEST(VolumetricMeshDataTest, ConstructionExposesCountsAndElementConnectivity) {
    pgo::VolumetricMeshes::internal::VolumetricMeshData geometry(
        4, {pgo::Vec3d(0.0, 0.0, 0.0), pgo::Vec3d(1.0, 0.0, 0.0), pgo::Vec3d(0.0, 1.0, 0.0), pgo::Vec3d(0.0, 0.0, 1.0)},
        {0, 1, 2, 3});

    EXPECT_EQ(geometry.num_vertices(), 4);
    EXPECT_EQ(geometry.num_elements(), 1);
    EXPECT_EQ(geometry.num_element_vertices(), 4);
    EXPECT_EQ(geometry.vertex_index(0, 0), 0);
    EXPECT_EQ(geometry.vertex_index(0, 3), 3);
    expectVec3dEq(geometry.vertex(2), pgo::Vec3d(0.0, 1.0, 0.0));
}

TEST(VolumetricMeshDataTest, RenumberVerticesUpdatesStorageAndIndices) {
    pgo::VolumetricMeshes::internal::VolumetricMeshData geometry(
        4, {pgo::Vec3d(0.0, 0.0, 0.0), pgo::Vec3d(1.0, 0.0, 0.0), pgo::Vec3d(0.0, 1.0, 0.0), pgo::Vec3d(0.0, 0.0, 1.0)},
        {0, 1, 2, 3});

    const std::array<int, 4> permutation = {2, 0, 3, 1};
    geometry.renumber_vertices(permutation);

    EXPECT_EQ(geometry.vertex_index(0, 0), 2);
    EXPECT_EQ(geometry.vertex_index(0, 1), 0);
    EXPECT_EQ(geometry.vertex_index(0, 2), 3);
    EXPECT_EQ(geometry.vertex_index(0, 3), 1);
    expectVec3dEq(geometry.vertex(0), pgo::Vec3d(1.0, 0.0, 0.0));
    expectVec3dEq(geometry.vertex(1), pgo::Vec3d(0.0, 0.0, 1.0));
    expectVec3dEq(geometry.vertex(2), pgo::Vec3d(0.0, 0.0, 0.0));
    expectVec3dEq(geometry.vertex(3), pgo::Vec3d(0.0, 1.0, 0.0));
}

TEST(VolumetricMeshDataTest, InvalidConnectivityIsRejected) {
    EXPECT_THROW(
        pgo::VolumetricMeshes::internal::VolumetricMeshData(
            4,
            {pgo::Vec3d(0.0, 0.0, 0.0), pgo::Vec3d(1.0, 0.0, 0.0), pgo::Vec3d(0.0, 1.0, 0.0)},
            {0, 1, 2, 3}),
        std::out_of_range);
}

}  // namespace pgo::Mesh::test
