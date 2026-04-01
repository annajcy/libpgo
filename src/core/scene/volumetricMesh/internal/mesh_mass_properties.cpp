#include "internal/mesh_mass_properties.h"

#include "volumetricMesh.h"

namespace pgo::VolumetricMeshes::internal::mesh_mass_properties {

double get_volume(const VolumetricMesh& mesh) {
    return detail::get_volume_impl(mesh);
}

void get_vertex_volumes(const VolumetricMesh& mesh, double* vertex_volumes) {
    detail::get_vertex_volumes_impl(mesh, vertex_volumes);
}

double get_mass(const VolumetricMesh& mesh) {
    return detail::get_mass_impl(mesh);
}

void get_inertia_parameters(const VolumetricMesh& mesh, double& mass, Vec3d& center_of_mass, Mat3d& inertia_tensor) {
    detail::get_inertia_parameters_impl(mesh, mass, center_of_mass, inertia_tensor);
}

void get_mesh_geometric_parameters(const VolumetricMesh& mesh, Vec3d& centroid, double* radius) {
    detail::get_mesh_geometric_parameters_impl(mesh, centroid, radius);
}

Mesh::BoundingBox get_bounding_box(const VolumetricMesh& mesh) {
    return detail::get_bounding_box_impl(mesh);
}

}  // namespace pgo::VolumetricMeshes::internal::mesh_mass_properties
