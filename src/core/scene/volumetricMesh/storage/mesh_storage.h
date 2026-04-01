#pragma once

#include "internal/material_catalog.h"
#include "internal/volumetric_mesh_data.h"

namespace pgo::VolumetricMeshes::storage {

class MeshStorage {
public:
    MeshStorage() = default;
    MeshStorage(internal::VolumetricMeshData geometry, internal::MaterialCatalog material_catalog);
    MeshStorage(const MeshStorage&) = default;
    MeshStorage& operator=(const MeshStorage&) = default;
    MeshStorage(MeshStorage&&) noexcept = default;
    MeshStorage& operator=(MeshStorage&&) noexcept = default;
    ~MeshStorage() = default;

    internal::VolumetricMeshData& geometry();
    const internal::VolumetricMeshData& geometry() const;

    internal::MaterialCatalog& material_catalog();
    const internal::MaterialCatalog& material_catalog() const;

    void validate_invariants() const;

private:
    internal::VolumetricMeshData m_geometry;
    internal::MaterialCatalog m_material_catalog;
};

}  // namespace pgo::VolumetricMeshes::storage
