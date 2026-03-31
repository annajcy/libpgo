#pragma once

#include "internal/material_catalog.h"

namespace pgo::VolumetricMeshes::io::detail {

struct LoadedMeshData {
    VolumetricMesh::ElementType  element_type = VolumetricMesh::ElementType::Invalid;
    internal::VolumetricMeshData geometry;
    internal::MaterialCatalog    material_catalog;
};

}  // namespace pgo::VolumetricMeshes::io::detail
