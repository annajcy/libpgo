#include "internal/mesh_queries.h"

#include "volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <vector>

namespace pgo::Mesh::test {

TEST(CoreSceneVolumetricMeshTest, MeshQueriesReturnConsistentNeighborhoodAndContainment) {
    pgo::VolumetricMeshes::TetMesh mesh = makeTwoTetMeshWithDistinctMaterials();

    std::vector<int> vertices;
    mesh.getVerticesInElements(std::vector<int>({1}), vertices);
    EXPECT_EQ(vertices, std::vector<int>({4, 5, 6, 7}));

    std::vector<int> touching_elements;
    mesh.getElementsTouchingVertices(std::vector<int>({4}), touching_elements);
    EXPECT_EQ(touching_elements, std::vector<int>({1}));

    std::vector<int> with_only_vertices;
    mesh.getElementsWithOnlyVertices(std::vector<int>({4, 5, 6, 7}), with_only_vertices);
    EXPECT_EQ(with_only_vertices, std::vector<int>({1}));

    std::vector<int> neighborhood;
    mesh.getVertexNeighborhood(std::vector<int>({4}), neighborhood);
    EXPECT_EQ(neighborhood, std::vector<int>({4, 5, 6, 7}));

    EXPECT_EQ(mesh.getContainingElement(pgo::Vec3d(3.1, 0.1, 0.1)), 1);
    EXPECT_EQ(mesh.getClosestVertex(pgo::Vec3d(3.05, 0.0, 0.0)), 4);
}

TEST(CoreSceneVolumetricMeshTest, MeshQueriesTemplateEntryPointsWorkForTetMesh) {
    pgo::VolumetricMeshes::TetMesh mesh = makeTwoTetMeshWithDistinctMaterials();

    std::vector<int> vertices;
    pgo::VolumetricMeshes::internal::mesh_queries::get_vertices_in_elements(mesh, std::vector<int>({1}), vertices);
    EXPECT_EQ(vertices, std::vector<int>({4, 5, 6, 7}));

    std::vector<int> touching_elements;
    pgo::VolumetricMeshes::internal::mesh_queries::get_elements_touching_vertices(mesh, std::vector<int>({4}),
                                                                                  touching_elements);
    EXPECT_EQ(touching_elements, std::vector<int>({1}));

    EXPECT_EQ(pgo::VolumetricMeshes::internal::mesh_queries::get_containing_element(mesh, pgo::Vec3d(3.1, 0.1, 0.1)),
              1);
}

}  // namespace pgo::Mesh::test
