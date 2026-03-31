#include "volumetricMeshEdit.h"

#include "internal/mesh_subset_editor.h"

namespace pgo::VolumetricMeshes {
namespace editing {

void subset_in_place(VolumetricMesh& mesh, const std::set<int>& subsetElements, int removeIsolatedVertices,
                     std::map<int, int>* old2NewVertexIDMap) {
    internal::MeshSubsetEditor::subset_in_place(mesh, subsetElements, removeIsolatedVertices != 0, old2NewVertexIDMap);
}

void remove_isolated_vertices(VolumetricMesh& mesh, std::map<int, int>* old2NewVertexIDMap) {
    internal::MeshSubsetEditor::remove_isolated_vertices(mesh, old2NewVertexIDMap);
}

}  // namespace editing
}  // namespace pgo::VolumetricMeshes
