#include "ops/tet_mesh_ops.h"

#include "../volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <set>
#include <utility>

namespace {

using pgo::Vec3d;
using pgo::VolumetricMeshes::TetMesh;
using pgo::VolumetricMeshes::ops::closest_element;
using pgo::VolumetricMeshes::ops::compute_barycentric_weights;
using pgo::VolumetricMeshes::ops::fill_element_edges;
using pgo::VolumetricMeshes::ops::signed_tet_volume;
using pgo::VolumetricMeshes::ops::tet_determinant;
using pgo::VolumetricMeshes::ops::tet_volume;

TEST(TetMeshOpsTest, TetOpsVolumeAndDeterminantAreConsistent) {
    const TetMesh mesh = pgo::Mesh::test::makeSingleTetMesh();

    const Vec3d& a = mesh.getVertex(0, 0);
    const Vec3d& b = mesh.getVertex(0, 1);
    const Vec3d& c = mesh.getVertex(0, 2);
    const Vec3d& d = mesh.getVertex(0, 3);

    EXPECT_DOUBLE_EQ(tet_determinant(a, b, c, d), 1.0);
    EXPECT_DOUBLE_EQ(signed_tet_volume(a, b, c, d), 1.0 / 6.0);
    EXPECT_DOUBLE_EQ(tet_volume(a, b, c, d), 1.0 / 6.0);
    EXPECT_DOUBLE_EQ(pgo::VolumetricMeshes::ops::element_volume(mesh, 0), 1.0 / 6.0);
}

TEST(TetMeshOpsTest, TetOpsBarycentricWeightsForInteriorPointAreNormalized) {
    const TetMesh mesh = pgo::Mesh::test::makeSingleTetMesh();

    double weights[4] = {0.0, 0.0, 0.0, 0.0};
    compute_barycentric_weights(mesh, 0, Vec3d(0.1, 0.2, 0.3), weights);

    pgo::Mesh::test::expectWeightsNormalized(weights, 4);
}

TEST(TetMeshOpsTest, TetOpsClosestElementMatchesSingleTetExpectation) {
    const TetMesh mesh = pgo::Mesh::test::makeSingleTetMesh();
    EXPECT_EQ(closest_element(mesh, Vec3d(0.1, 0.1, 0.1)), 0);
}

TEST(TetMeshOpsTest, TetOpsFillElementEdgesReturnsSixEdges) {
    const TetMesh mesh = pgo::Mesh::test::makeSingleTetMesh();

    std::array<int, 12> edge_buffer{};
    fill_element_edges(mesh, 0, edge_buffer.data());

    std::set<std::pair<int, int>> actual_edges;
    for (int edge = 0; edge < 6; ++edge) {
        const int v0 = edge_buffer[2 * edge + 0];
        const int v1 = edge_buffer[2 * edge + 1];
        actual_edges.insert(std::minmax(v0, v1));
    }

    const std::set<std::pair<int, int>> expected_edges{
        {0, 1}, {1, 2}, {0, 2}, {0, 3}, {1, 3}, {2, 3},
    };
    EXPECT_EQ(actual_edges, expected_edges);
}

}  // namespace
