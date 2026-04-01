#include "ops/cubic_mesh_ops.h"

#include "../volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <numeric>
#include <set>
#include <utility>

namespace {

using pgo::Vec3d;
using pgo::VolumetricMeshes::CubicMesh;
using pgo::VolumetricMeshes::ops::compute_alpha_beta_gamma;
using pgo::VolumetricMeshes::ops::compute_barycentric_weights;
using pgo::VolumetricMeshes::ops::compute_element_mass_matrix;
using pgo::VolumetricMeshes::ops::fill_element_edges;

TEST(CubicMeshOpsTest, CubicOpsAlphaBetaGammaForInteriorPointAreInUnitRange) {
    const CubicMesh mesh = pgo::Mesh::test::makeSingleCubeMesh();

    double alpha = 0.0;
    double beta = 0.0;
    double gamma = 0.0;
    compute_alpha_beta_gamma(mesh, 0, Vec3d(0.25, 0.5, 0.75), &alpha, &beta, &gamma);

    EXPECT_DOUBLE_EQ(alpha, 0.25);
    EXPECT_DOUBLE_EQ(beta, 0.5);
    EXPECT_DOUBLE_EQ(gamma, 0.75);
}

TEST(CubicMeshOpsTest, CubicOpsBarycentricWeightsForInteriorPointAreNormalized) {
    const CubicMesh mesh = pgo::Mesh::test::makeSingleCubeMesh();

    double weights[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    compute_barycentric_weights(mesh, 0, Vec3d(0.25, 0.5, 0.75), weights);

    pgo::Mesh::test::expectWeightsNormalized(weights, 8);
}

TEST(CubicMeshOpsTest, CubicOpsFillElementEdgesReturnsTwelveEdges) {
    const CubicMesh mesh = pgo::Mesh::test::makeSingleCubeMesh();

    std::array<int, 24> edge_buffer{};
    fill_element_edges(mesh, 0, edge_buffer.data());

    std::set<std::pair<int, int>> actual_edges;
    for (int edge = 0; edge < 12; ++edge) {
        const int v0 = edge_buffer[2 * edge + 0];
        const int v1 = edge_buffer[2 * edge + 1];
        actual_edges.insert(std::minmax(v0, v1));
    }

    const std::set<std::pair<int, int>> expected_edges{
        {0, 1}, {1, 2}, {2, 3}, {0, 3}, {4, 5}, {5, 6}, {6, 7}, {4, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7},
    };
    EXPECT_EQ(actual_edges, expected_edges);
}

TEST(CubicMeshOpsTest, CubicOpsMassMatrixUsesCurrentCubeVolumeAndDensityConvention) {
    const CubicMesh mesh = pgo::Mesh::test::makeSingleCubeMesh(500.0, 0.25, 3.0);

    std::array<double, 64> mass_matrix{};
    compute_element_mass_matrix(mesh, 0, mass_matrix.data());

    const double sum = std::accumulate(mass_matrix.begin(), mass_matrix.end(), 0.0);
    EXPECT_NEAR(sum, mesh.getElementDensity(0) * mesh.getElementVolume(0), 1e-6);
    EXPECT_NEAR(mass_matrix[0], mesh.getElementDensity(0) * mesh.getElementVolume(0) * 0.0370370335876942, 1e-6);
}

}  // namespace
