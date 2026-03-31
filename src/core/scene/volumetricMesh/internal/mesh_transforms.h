#pragma once

#include "volumetricMesh.h"

namespace pgo::VolumetricMeshes::internal::mesh_transforms {

void compute_gravity(const VolumetricMesh& mesh, double* gravity_force, double g, bool add_force);
void apply_deformation(VolumetricMesh& mesh, const double* u);
void apply_linear_transformation(VolumetricMesh& mesh, double* pos, double* R);
void renumber_vertices(VolumetricMesh& mesh, const std::vector<int>& permutation);

}  // namespace pgo::VolumetricMeshes::internal::mesh_transforms
