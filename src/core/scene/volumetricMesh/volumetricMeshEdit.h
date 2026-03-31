#pragma once

#include "volumetricMesh.h"

namespace pgo::VolumetricMeshes {

namespace editing {

void subset_in_place(VolumetricMesh& mesh, const std::set<int>& subsetElements, int removeIsolatedVertices = 1,
                     std::map<int, int>* old2NewVertexIDMap = nullptr);
void remove_isolated_vertices(VolumetricMesh& mesh, std::map<int, int>* old2NewVertexIDMap = nullptr);

}  // namespace editing

}  // namespace pgo::VolumetricMeshes
