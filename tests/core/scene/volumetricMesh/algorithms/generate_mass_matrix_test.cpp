#include "algorithms/generate_mass_matrix.h"

#include "../volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <array>

namespace {

using pgo::VolumetricMeshes::GenerateMassMatrix::computeMassMatrix;
using pgo::VolumetricMeshes::GenerateMassMatrix::computeVertexMassesByAveragingNeighboringElements;

TEST(GenerateMassMatrixTest, TemplatePathComputesTetMassMatrixWithExpectedTotalMass) {
    const auto mesh = pgo::Mesh::test::makeSingleTetMesh(1234.0, 0.31, 6.0);

    pgo::EigenSupport::SpMatD mass_matrix;
    computeMassMatrix(mesh, mass_matrix, false);

    EXPECT_EQ(mass_matrix.rows(), 4);
    EXPECT_EQ(mass_matrix.cols(), 4);
    EXPECT_NEAR(mass_matrix.sum(), 1.0, 1e-12);
}

TEST(GenerateMassMatrixTest, TemplatePathComputesCubeVertexMassesByAveragingNeighboringElements) {
    const auto mesh = pgo::Mesh::test::makeSingleCubeMesh(500.0, 0.25, 3.0);

    std::array<double, 8> masses{};
    computeVertexMassesByAveragingNeighboringElements(mesh, masses.data(), false);

    for (double mass : masses) {
        EXPECT_NEAR(mass, 3.0 / 8.0, 1e-12);
    }
}

}  // namespace
