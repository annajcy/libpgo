#pragma once

namespace pgo::VolumetricMeshes {
class CubicMesh;
}

namespace pgo::VolumetricMeshes::algorithms {

int interpolate_cubic_mesh_data(const CubicMesh& mesh, double* vertex_data, int num_locations, int r,
                                double* interpolation_locations, double* dest_matrix, double distance_threshold);

}  // namespace pgo::VolumetricMeshes::algorithms
