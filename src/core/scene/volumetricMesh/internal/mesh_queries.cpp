#include "internal/mesh_queries.h"

#include "volumetricMesh.h"

namespace pgo::VolumetricMeshes::internal::mesh_queries {

void get_vertices_in_elements(const VolumetricMesh& mesh, const std::vector<int>& elements, std::vector<int>& vertices) {
    detail::get_vertices_in_elements_impl(mesh, elements, vertices);
}

void get_elements_touching_vertices(const VolumetricMesh& mesh, const std::vector<int>& vertices,
                                    std::vector<int>& elements) {
    detail::get_elements_touching_vertices_impl(mesh, vertices, elements);
}

void get_elements_with_only_vertices(const VolumetricMesh& mesh, const std::vector<int>& vertices,
                                     std::vector<int>& elements) {
    detail::get_elements_with_only_vertices_impl(mesh, vertices, elements);
}

void get_vertex_neighborhood(const VolumetricMesh& mesh, const std::vector<int>& vertices, std::vector<int>& neighborhood) {
    detail::get_vertex_neighborhood_impl(mesh, vertices, neighborhood);
}

int get_closest_vertex(const VolumetricMesh& mesh, Vec3d pos) {
    return detail::get_closest_vertex_impl(mesh, pos);
}

int get_closest_element(const VolumetricMesh& mesh, const Vec3d& pos) {
    return detail::get_closest_element_impl(mesh, pos);
}

int get_containing_element(const VolumetricMesh& mesh, Vec3d pos) {
    return detail::get_containing_element_impl(mesh, pos);
}

}  // namespace pgo::VolumetricMeshes::internal::mesh_queries
