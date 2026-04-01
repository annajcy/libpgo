#include "internal/mesh_subset_editor.h"

#include "tetMesh.h"

#include "../volumetric_mesh_test_fixtures.h"

#include <gtest/gtest.h>

#include <map>
#include <set>

namespace pgo::Mesh::test {

TEST(MeshSubsetEditorTest, SubsetInPlaceKeepsAllElementsSetAndMaterialAlignment) {
    pgo::VolumetricMeshes::TetMesh mesh = makeTwoTetMeshWithDistinctMaterials();
    const std::set<int>            subset{1};

    pgo::VolumetricMeshes::internal::MeshSubsetEditor::subset_in_place(mesh, subset, false, nullptr);

    EXPECT_EQ(mesh.getNumElements(), 1);
    expectAllElementsSetCoversMesh(mesh);

    const auto* material =
        pgo::VolumetricMeshes::try_get_material<pgo::VolumetricMeshes::EnuMaterialData>(mesh.getElementMaterial(0));
    ASSERT_NE(material, nullptr);
    EXPECT_DOUBLE_EQ(material->E, 4321.0);
    EXPECT_DOUBLE_EQ(material->nu, 0.21);
    EXPECT_DOUBLE_EQ(material->density, 9.25);
}

TEST(MeshSubsetEditorTest, RemoveIsolatedVerticesProducesDenseVertexIds) {
    pgo::VolumetricMeshes::TetMesh mesh = makeTwoTetMeshWithDistinctMaterials();
    std::map<int, int>             old_to_new_vertex_ids;

    pgo::VolumetricMeshes::internal::MeshSubsetEditor::subset_in_place(mesh, std::set<int>({1}), true,
                                                                       &old_to_new_vertex_ids);

    ASSERT_EQ(old_to_new_vertex_ids.size(), 4u);
    EXPECT_EQ(mesh.getNumVertices(), 4);
    for (int local_vertex = 0; local_vertex < mesh.getNumElementVertices(); ++local_vertex) {
        EXPECT_EQ(mesh.getVertexIndex(0, local_vertex), local_vertex);
    }
}

}  // namespace pgo::Mesh::test
