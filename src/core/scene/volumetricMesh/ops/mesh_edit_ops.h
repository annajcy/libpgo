#pragma once

#include <map>
#include <set>

namespace pgo::VolumetricMeshes {
class VolumetricMesh;

namespace ops {

void subset_in_place(VolumetricMesh& mesh, const std::set<int>& subset_elements, int remove_isolated_vertices,
                     std::map<int, int>* old_to_new_vertex_id_map);
void remove_isolated_vertices(VolumetricMesh& mesh, std::map<int, int>* old_to_new_vertex_id_map);

}  // namespace ops

}  // namespace pgo::VolumetricMeshes
