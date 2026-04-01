#pragma once

#include "internal/mesh_subset_editor.h"

#include <map>
#include <set>

namespace pgo::VolumetricMeshes::ops {

template <class MeshT>
void subset_in_place(MeshT& mesh, const std::set<int>& subset_elements, int remove_isolated_vertices,
                     std::map<int, int>* old_to_new_vertex_id_map) {
    internal::MeshSubsetEditor::subset_in_place(mesh, subset_elements, remove_isolated_vertices,
                                                old_to_new_vertex_id_map);
    mesh.sync_storage_from_legacy_state_for_transition();
}

template <class MeshT>
void remove_isolated_vertices(MeshT& mesh, std::map<int, int>* old_to_new_vertex_id_map) {
    internal::MeshSubsetEditor::remove_isolated_vertices(mesh, old_to_new_vertex_id_map);
    mesh.sync_storage_from_legacy_state_for_transition();
}

}  // namespace pgo::VolumetricMeshes::ops
