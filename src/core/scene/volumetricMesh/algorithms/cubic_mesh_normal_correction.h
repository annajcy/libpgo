#pragma once

namespace pgo::VolumetricMeshes {
class CubicMesh;
}

namespace pgo::VolumetricMeshes::algorithms {

int cubic_mesh_normal_correction(const CubicMesh& mesh, double* vertex_data, int num_locations, int r,
                                 double* interpolation_locations, double* static_normals,
                                 double* normal_correction, double distance_threshold);

}  // namespace pgo::VolumetricMeshes::algorithms
