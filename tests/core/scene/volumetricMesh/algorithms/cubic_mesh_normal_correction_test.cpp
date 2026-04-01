#include "algorithms/cubic_mesh_normal_correction.h"

#include "../volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <array>

namespace {

TEST(CubicMeshAlgorithmsTest, CubicMeshNormalCorrectionReturnsZeroForZeroDisplacement) {
    const pgo::VolumetricMeshes::CubicMesh mesh = pgo::Mesh::test::makeSingleCubeMesh();

    std::array<double, 3 * 8> vertex_data{};
    std::array<double, 3> interpolation_locations{0.25, 0.5, 0.75};
    std::array<double, 3> static_normals{0.0, 0.0, 1.0};
    std::array<double, 3> normal_correction{1.0, 1.0, 1.0};

    const int num_external = pgo::VolumetricMeshes::algorithms::cubic_mesh_normal_correction(
        mesh, vertex_data.data(), 1, 1, interpolation_locations.data(), static_normals.data(), normal_correction.data(),
        -1.0);

    EXPECT_EQ(num_external, 0);
    EXPECT_DOUBLE_EQ(normal_correction[0], 0.0);
    EXPECT_DOUBLE_EQ(normal_correction[1], 0.0);
    EXPECT_DOUBLE_EQ(normal_correction[2], 0.0);
}

TEST(CubicMeshAlgorithmsTest, CubicMeshNormalCorrectionZeroesFarLocationWhenThresholdExceeded) {
    const pgo::VolumetricMeshes::CubicMesh mesh = pgo::Mesh::test::makeSingleCubeMesh();

    std::array<double, 3 * 8> vertex_data{};
    vertex_data.fill(2.0);
    std::array<double, 3> interpolation_locations{5.0, 5.0, 5.0};
    std::array<double, 3> static_normals{0.0, 1.0, 0.0};
    std::array<double, 3> normal_correction{1.0, 1.0, 1.0};

    const int num_external = pgo::VolumetricMeshes::algorithms::cubic_mesh_normal_correction(
        mesh, vertex_data.data(), 1, 1, interpolation_locations.data(), static_normals.data(), normal_correction.data(),
        0.1);

    EXPECT_EQ(num_external, 1);
    EXPECT_DOUBLE_EQ(normal_correction[0], 0.0);
    EXPECT_DOUBLE_EQ(normal_correction[1], 0.0);
    EXPECT_DOUBLE_EQ(normal_correction[2], 0.0);
}

}  // namespace
