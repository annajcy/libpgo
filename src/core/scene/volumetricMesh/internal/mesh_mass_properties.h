#pragma once

#include "volumetricMesh.h"

namespace pgo::VolumetricMeshes::internal::mesh_mass_properties {

double            get_volume(const VolumetricMesh& mesh);
void              get_vertex_volumes(const VolumetricMesh& mesh, double* vertex_volumes);
double            get_mass(const VolumetricMesh& mesh);
void              get_inertia_parameters(const VolumetricMesh& mesh, double& mass, Vec3d& center_of_mass,
                                         Mat3d& inertia_tensor);
void              get_mesh_geometric_parameters(const VolumetricMesh& mesh, Vec3d& centroid, double* radius);
Mesh::BoundingBox get_bounding_box(const VolumetricMesh& mesh);

}  // namespace pgo::VolumetricMeshes::internal::mesh_mass_properties
