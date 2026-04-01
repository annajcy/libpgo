#include "storage/mesh_storage.h"

#include <utility>

namespace pgo::VolumetricMeshes::storage {

MeshStorage::MeshStorage(internal::VolumetricMeshData geometry, internal::MaterialCatalog material_catalog)
    : m_geometry(std::move(geometry)),
      m_material_catalog(std::move(material_catalog)) {}

internal::VolumetricMeshData& MeshStorage::geometry() {
    return m_geometry;
}

const internal::VolumetricMeshData& MeshStorage::geometry() const {
    return m_geometry;
}

internal::MaterialCatalog& MeshStorage::material_catalog() {
    return m_material_catalog;
}

const internal::MaterialCatalog& MeshStorage::material_catalog() const {
    return m_material_catalog;
}

void MeshStorage::validate_invariants() const {
    m_geometry.validate_basic_invariants();
    m_material_catalog.validate_against_num_elements(m_geometry.num_elements());
}

}  // namespace pgo::VolumetricMeshes::storage
