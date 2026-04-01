#include "internal/mesh_transforms.h"

namespace pgo::VolumetricMeshes::internal::mesh_transforms {

void compute_gravity(const VolumetricMesh& mesh, double* gravity_force, double g, bool add_force) {
    detail::compute_gravity_impl(mesh, gravity_force, g, add_force);
}

void apply_deformation(VolumetricMesh& mesh, const double* u) {
    detail::apply_deformation_impl(mesh, u);
}

void apply_linear_transformation(VolumetricMesh& mesh, double* pos, double* R) {
    detail::apply_linear_transformation_impl(mesh, pos, R);
}

void renumber_vertices(VolumetricMesh& mesh, const std::vector<int>& permutation) {
    detail::renumber_vertices_impl(mesh, permutation);
}

}  // namespace pgo::VolumetricMeshes::internal::mesh_transforms
