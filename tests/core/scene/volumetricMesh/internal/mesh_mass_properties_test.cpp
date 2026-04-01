#include "internal/mesh_mass_properties.h"

#include "../volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <array>

namespace {

using pgo::Vec3d;
using pgo::VolumetricMeshes::internal::mesh_mass_properties::get_bounding_box;
using pgo::VolumetricMeshes::internal::mesh_mass_properties::get_mass;
using pgo::VolumetricMeshes::internal::mesh_mass_properties::get_mesh_geometric_parameters;
using pgo::VolumetricMeshes::internal::mesh_mass_properties::get_vertex_volumes;
using pgo::VolumetricMeshes::internal::mesh_mass_properties::get_volume;

TEST(MeshMassPropertiesTest, TemplateMassPropertiesMatchSingleTetExpectations) {
    const auto mesh = pgo::Mesh::test::makeSingleTetMesh(1234.0, 0.31, 7.5);

    std::array<double, 4> vertex_volumes{};
    get_vertex_volumes(mesh, vertex_volumes.data());

    EXPECT_DOUBLE_EQ(get_volume(mesh), 1.0 / 6.0);
    EXPECT_DOUBLE_EQ(get_mass(mesh), 7.5 / 6.0);
    for (double volume : vertex_volumes) {
        EXPECT_DOUBLE_EQ(volume, 1.0 / 24.0);
    }
}

TEST(MeshMassPropertiesTest, TemplateMassPropertiesComputeBoundingGeometryForCube) {
    const auto mesh = pgo::Mesh::test::makeSingleCubeMesh();

    Vec3d centroid(0.0, 0.0, 0.0);
    double radius = 0.0;
    get_mesh_geometric_parameters(mesh, centroid, &radius);
    const auto bbox = get_bounding_box(mesh);

    EXPECT_DOUBLE_EQ(centroid[0], 0.5);
    EXPECT_DOUBLE_EQ(centroid[1], 0.5);
    EXPECT_DOUBLE_EQ(centroid[2], 0.5);
    EXPECT_NEAR(radius, std::sqrt(3.0) * 0.5, 1e-12);
    EXPECT_DOUBLE_EQ(bbox.bmin()[0], 0.0);
    EXPECT_DOUBLE_EQ(bbox.bmax()[0], 1.0);
}

}  // namespace
