#include "volumetricMesh.h"

#include "volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <map>
#include <set>

namespace pgo::Mesh::test {

TEST(CoreSceneVolumetricMeshTest, SubsetInPlacePreservesSetAndMaterialObservables) {
    using namespace pgo::VolumetricMeshes;

    TetMesh             mesh = makeTwoTetMeshWithDistinctMaterials();
    const std::set<int> subset{1};

    editing::subset_in_place(mesh, subset, 0, nullptr);

    EXPECT_EQ(mesh.getNumElements(), 1);
    EXPECT_EQ(mesh.getNumVertices(), 8);
    expectAllElementsSetCoversMesh(mesh);

    const auto* material = try_get_material<EnuMaterialData>(mesh.getElementMaterial(0));
    ASSERT_NE(material, nullptr);
    EXPECT_DOUBLE_EQ(material->E, 4321.0);
    EXPECT_DOUBLE_EQ(material->nu, 0.21);
    EXPECT_DOUBLE_EQ(material->density, 9.25);

    for (int local_vertex = 0; local_vertex < mesh.getNumElementVertices(); ++local_vertex) {
        const int vertex_index = mesh.getVertexIndex(0, local_vertex);
        EXPECT_GE(vertex_index, 4);
        EXPECT_LT(vertex_index, 8);
    }
}

TEST(CoreSceneVolumetricMeshTest, SubsetInPlaceRemovesIsolatedVerticesAndRemapsIndices) {
    using namespace pgo::VolumetricMeshes;

    TetMesh             mesh = makeTwoTetMeshWithDistinctMaterials();
    const std::set<int> subset{1};
    std::map<int, int>  old_to_new_vertex_ids;

    editing::subset_in_place(mesh, subset, 1, &old_to_new_vertex_ids);

    EXPECT_EQ(mesh.getNumElements(), 1);
    EXPECT_EQ(mesh.getNumVertices(), 4);
    expectAllElementsSetCoversMesh(mesh);
    ASSERT_EQ(old_to_new_vertex_ids.size(), 4u);

    std::set<int> remapped_ids;
    for (const auto& [old_id, new_id] : old_to_new_vertex_ids) {
        EXPECT_GE(old_id, 4);
        EXPECT_LT(old_id, 8);
        EXPECT_GE(new_id, 0);
        EXPECT_LT(new_id, 4);
        remapped_ids.insert(new_id);
    }
    EXPECT_EQ(remapped_ids, std::set<int>({0, 1, 2, 3}));

    for (int local_vertex = 0; local_vertex < mesh.getNumElementVertices(); ++local_vertex) {
        const int vertex_index = mesh.getVertexIndex(0, local_vertex);
        EXPECT_GE(vertex_index, 0);
        EXPECT_LT(vertex_index, mesh.getNumVertices());
    }
}

}  // namespace pgo::Mesh::test
