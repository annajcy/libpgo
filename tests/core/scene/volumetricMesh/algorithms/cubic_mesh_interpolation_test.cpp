#include "algorithms/cubic_mesh_interpolation.h"

#include "../volumetric_mesh_test_fixtures.h"

#include "EigenSupport.h"

#include <gtest/gtest.h>

#include <array>

namespace {

namespace ES = pgo::EigenSupport;

TEST(CubicMeshAlgorithmsTest, CubicMeshInterpolationInterpolatesConstantFieldAtInteriorPoint) {
    const pgo::VolumetricMeshes::CubicMesh mesh = pgo::Mesh::test::makeSingleCubeMesh();

    std::array<double, 3 * 8> vertex_data{};
    for (int vertex = 0; vertex < mesh.getNumVertices(); ++vertex) {
        vertex_data[3 * vertex + 0] = 1.25;
        vertex_data[3 * vertex + 1] = -2.0;
        vertex_data[3 * vertex + 2] = 0.5;
    }

    std::array<double, 3> interpolation_locations{0.25, 0.5, 0.75};
    std::array<double, 3> dest_matrix{};

    const int num_external = pgo::VolumetricMeshes::algorithms::interpolate_cubic_mesh_data(
        mesh, vertex_data.data(), 1, 1, interpolation_locations.data(), dest_matrix.data(), -1.0);

    EXPECT_EQ(num_external, 0);
    EXPECT_DOUBLE_EQ(dest_matrix[0], 1.25);
    EXPECT_DOUBLE_EQ(dest_matrix[1], -2.0);
    EXPECT_DOUBLE_EQ(dest_matrix[2], 0.5);
}

TEST(CubicMeshAlgorithmsTest, CubicMeshInterpolationZeroesFarLocationWhenThresholdExceeded) {
    const pgo::VolumetricMeshes::CubicMesh mesh = pgo::Mesh::test::makeSingleCubeMesh();

    std::array<double, 3 * 8> vertex_data{};
    vertex_data.fill(3.0);

    std::array<double, 3> interpolation_locations{5.0, 5.0, 5.0};
    std::array<double, 3> dest_matrix{1.0, 1.0, 1.0};

    const int num_external = pgo::VolumetricMeshes::algorithms::interpolate_cubic_mesh_data(
        mesh, vertex_data.data(), 1, 1, interpolation_locations.data(), dest_matrix.data(), 0.1);

    EXPECT_EQ(num_external, 1);
    EXPECT_DOUBLE_EQ(dest_matrix[0], 0.0);
    EXPECT_DOUBLE_EQ(dest_matrix[1], 0.0);
    EXPECT_DOUBLE_EQ(dest_matrix[2], 0.0);
}

TEST(CubicMeshAlgorithmsTest, CubicMeshInterpolationCountsExteriorLocationsUsingCurrentContainmentRule) {
    const pgo::VolumetricMeshes::CubicMesh mesh = pgo::Mesh::test::makeSingleCubeMesh();

    std::array<double, 3 * 8> vertex_data{};
    vertex_data.fill(1.0);

    std::array<double, 3> interpolation_locations{-0.1, 0.25, 0.25};
    std::array<double, 3> dest_matrix{};

    const int num_external = pgo::VolumetricMeshes::algorithms::interpolate_cubic_mesh_data(
        mesh, vertex_data.data(), 1, 1, interpolation_locations.data(), dest_matrix.data(), -1.0);

    EXPECT_EQ(num_external, 1);
}

}  // namespace
