#pragma once

#include "io/mesh_io_types.h"
#include "storage/mesh_storage.h"

#include <span>
#include <vector>

namespace pgo::VolumetricMeshes::ops {

storage::MeshStorage make_mesh_storage(std::span<const Vec3d> vertices, int num_element_vertices,
                                       std::span<const int> elements, double E, double nu, double density);
storage::MeshStorage make_mesh_storage(std::span<const Vec3d> vertices, int num_element_vertices,
                                       std::span<const int> elements, std::vector<MaterialRecord> materials,
                                       std::vector<ElementSet> sets, std::vector<MaterialRegion> regions,
                                       int verbose = 0);
storage::MeshStorage make_mesh_storage(io::detail::LoadedMeshData data);

template <class MeshT>
void assign_common_storage(MeshT& mesh, storage::MeshStorage storage) {
    storage.validate_invariants();
    mesh.geometry_data() = std::move(storage.geometry());
    mesh.material_catalog() = std::move(storage.material_catalog());
    mesh.material_catalog().validate_against_num_elements(mesh.getNumElements());
}

template <class MeshT>
void assign_common_loaded_data(MeshT& mesh, io::detail::LoadedMeshData data) {
    assign_common_storage(mesh, make_mesh_storage(std::move(data)));
}

}  // namespace pgo::VolumetricMeshes::ops
