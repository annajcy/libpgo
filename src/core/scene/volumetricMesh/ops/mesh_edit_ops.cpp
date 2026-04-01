#include "ops/mesh_edit_ops.h"

#include "internal/mesh_subset_editor.h"

namespace pgo::VolumetricMeshes::ops {

void subset_in_place(VolumetricMesh& mesh, const std::set<int>& subset_elements, int remove_isolated_vertices,
                     std::map<int, int>* old_to_new_vertex_id_map) {
    internal::MeshSubsetEditor::subset_in_place(mesh, subset_elements, remove_isolated_vertices,
                                                old_to_new_vertex_id_map);
}

void remove_isolated_vertices(VolumetricMesh& mesh, std::map<int, int>* old_to_new_vertex_id_map) {
    internal::MeshSubsetEditor::remove_isolated_vertices(mesh, old_to_new_vertex_id_map);
}

}  // namespace pgo::VolumetricMeshes::ops
