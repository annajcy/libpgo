#include "ops/mesh_construction.h"

namespace pgo::VolumetricMeshes::ops {

storage::MeshStorage make_mesh_storage(std::span<const Vec3d> vertices, int num_element_vertices,
                                       std::span<const int> elements, double E, double nu, double density) {
    storage::MeshStorage storage(
        internal::VolumetricMeshData(num_element_vertices, std::vector<Vec3d>(vertices.begin(), vertices.end()),
                                     std::vector<int>(elements.begin(), elements.end())),
        internal::MaterialCatalog(static_cast<int>(elements.size()) / num_element_vertices, E, nu, density));
    storage.validate_invariants();
    return storage;
}

storage::MeshStorage make_mesh_storage(std::span<const Vec3d> vertices, int num_element_vertices,
                                       std::span<const int> elements, std::vector<MaterialRecord> materials,
                                       std::vector<ElementSet> sets, std::vector<MaterialRegion> regions,
                                       int verbose) {
    storage::MeshStorage storage(
        internal::VolumetricMeshData(num_element_vertices, std::vector<Vec3d>(vertices.begin(), vertices.end()),
                                     std::vector<int>(elements.begin(), elements.end())),
        internal::MaterialCatalog(std::move(materials), std::move(sets), std::move(regions),
                                  static_cast<int>(elements.size()) / num_element_vertices, verbose));
    storage.validate_invariants();
    return storage;
}

storage::MeshStorage make_mesh_storage(io::detail::LoadedMeshData data) {
    data.material_catalog.validate_against_num_elements(data.geometry.num_elements());
    storage::MeshStorage storage(std::move(data.geometry), std::move(data.material_catalog));
    storage.validate_invariants();
    return storage;
}

}  // namespace pgo::VolumetricMeshes::ops
