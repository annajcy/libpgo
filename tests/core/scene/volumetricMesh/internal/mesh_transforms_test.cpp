#include "internal/mesh_transforms.h"

#include "../volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <array>
#include <vector>

namespace {

using pgo::Vec3d;
using pgo::VolumetricMeshes::internal::mesh_transforms::apply_deformation;
using pgo::VolumetricMeshes::internal::mesh_transforms::apply_linear_transformation;
using pgo::VolumetricMeshes::internal::mesh_transforms::compute_gravity;

TEST(MeshTransformsTest, TemplateComputeGravityMatchesSingleTetMassDistribution) {
    const auto mesh = pgo::Mesh::test::makeSingleTetMesh(1234.0, 0.31, 6.0);
    std::array<double, 12> gravity_force{};

    compute_gravity(mesh, gravity_force.data(), 9.81, false);

    for (int vertex = 0; vertex < 4; ++vertex) {
        EXPECT_DOUBLE_EQ(gravity_force[3 * vertex + 0], 0.0);
        EXPECT_NEAR(gravity_force[3 * vertex + 1], -9.81 / 4.0, 1e-12);
        EXPECT_DOUBLE_EQ(gravity_force[3 * vertex + 2], 0.0);
    }
}

TEST(MeshTransformsTest, TemplateApplyDeformationAndLinearTransformUpdateVertices) {
    auto mesh = pgo::Mesh::test::makeSingleCubeMesh();

    std::vector<double> displacement(static_cast<size_t>(3 * mesh.getNumVertices()), 0.0);
    displacement[0] = 0.5;
    displacement[1] = -0.25;
    displacement[2] = 0.75;
    apply_deformation(mesh, displacement.data());
    EXPECT_DOUBLE_EQ(mesh.getVertex(0)[0], 0.5);
    EXPECT_DOUBLE_EQ(mesh.getVertex(0)[1], -0.25);
    EXPECT_DOUBLE_EQ(mesh.getVertex(0)[2], 0.75);

    double pos[3] = {1.0, 2.0, 3.0};
    double R[9]   = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    };
    apply_linear_transformation(mesh, pos, R);
    EXPECT_DOUBLE_EQ(mesh.getVertex(0)[0], 1.5);
    EXPECT_DOUBLE_EQ(mesh.getVertex(0)[1], 1.75);
    EXPECT_DOUBLE_EQ(mesh.getVertex(0)[2], 3.75);
}

}  // namespace
