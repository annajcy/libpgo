#pragma once

#include "io/mesh_io_types.h"
#include "storage/mesh_storage.h"

#include <span>
#include <vector>

namespace pgo::VolumetricMeshes {
class VolumetricMesh;
}

namespace pgo::VolumetricMeshes::ops {

storage::MeshStorage make_mesh_storage(std::span<const Vec3d> vertices, int num_element_vertices,
                                       std::span<const int> elements, double E, double nu, double density);
storage::MeshStorage make_mesh_storage(std::span<const Vec3d> vertices, int num_element_vertices,
                                       std::span<const int> elements, std::vector<MaterialRecord> materials,
                                       std::vector<ElementSet> sets, std::vector<MaterialRegion> regions,
                                       int verbose = 0);
storage::MeshStorage make_mesh_storage(io::detail::LoadedMeshData data);
void assign_common_storage(VolumetricMesh& mesh, storage::MeshStorage storage);
void assign_common_loaded_data(VolumetricMesh& mesh, io::detail::LoadedMeshData data);

}  // namespace pgo::VolumetricMeshes::ops
