#include "algorithms/mesh_interpolation.h"

#include "../volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <vector>

namespace {

using pgo::Vec3d;
using pgo::VolumetricMeshes::interpolation::containing_elements;
using pgo::VolumetricMeshes::interpolation::generate_weights;

TEST(MeshInterpolationTest, TemplateGenerateWeightsForTetKeepsIndicesAndNormalizedWeights) {
    const auto mesh = pgo::Mesh::test::makeSingleTetMesh();
    const std::vector<Vec3d> targets{Vec3d(0.1, 0.2, 0.3)};

    const auto interpolation = generate_weights(mesh, targets, -1.0, true, 0);

    ASSERT_EQ(interpolation.numElementVertices, 4);
    ASSERT_EQ(interpolation.indices.size(), 4u);
    ASSERT_EQ(interpolation.elements, std::vector<int>({0}));
    EXPECT_EQ(interpolation.indices, std::vector<int>({0, 1, 2, 3}));
    pgo::Mesh::test::expectWeightsNormalized(interpolation.weights.data(), 4);
}

TEST(MeshInterpolationTest, TemplateContainingElementsFallsBackToClosestElementForExteriorPoint) {
    const auto mesh = pgo::Mesh::test::makeSingleCubeMesh();
    const std::vector<Vec3d> targets{Vec3d(2.0, 2.0, 2.0)};

    const auto elements = containing_elements(mesh, targets, true, 0);

    EXPECT_EQ(elements, std::vector<int>({0}));
}

}  // namespace
