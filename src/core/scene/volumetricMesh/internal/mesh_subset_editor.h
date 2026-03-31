#pragma once

#include <map>
#include <set>

namespace pgo::VolumetricMeshes {
class VolumetricMesh;
}

namespace pgo::VolumetricMeshes::internal {

class MeshSubsetEditor {
public:
    static void subset_in_place(VolumetricMesh& mesh, const std::set<int>& subset_elements,
                                bool remove_isolated_vertices, std::map<int, int>* old_to_new_vertex_id_map);
    static void remove_isolated_vertices(VolumetricMesh& mesh, std::map<int, int>* old_to_new_vertex_id_map);
};

}  // namespace pgo::VolumetricMeshes::internal
