#pragma once

#include "volumetricMesh.h"

#include <vector>

namespace pgo::VolumetricMeshes::internal::mesh_queries {

void get_vertices_in_elements(const VolumetricMesh& mesh, const std::vector<int>& elements,
                              std::vector<int>& vertices);
void get_elements_touching_vertices(const VolumetricMesh& mesh, const std::vector<int>& vertices,
                                    std::vector<int>& elements);
void get_elements_with_only_vertices(const VolumetricMesh& mesh, const std::vector<int>& vertices,
                                     std::vector<int>& elements);
void get_vertex_neighborhood(const VolumetricMesh& mesh, const std::vector<int>& vertices, std::vector<int>& neighborhood);
int  get_closest_vertex(const VolumetricMesh& mesh, Vec3d pos);
int  get_closest_element(const VolumetricMesh& mesh, const Vec3d& pos);
int  get_containing_element(const VolumetricMesh& mesh, Vec3d pos);

}  // namespace pgo::VolumetricMeshes::internal::mesh_queries
