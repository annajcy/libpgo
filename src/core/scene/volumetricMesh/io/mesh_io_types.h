#pragma once

#include "internal/material_catalog.h"
#include "internal/volumetric_mesh_data.h"
#include "types/mesh_enums.h"

namespace pgo::VolumetricMeshes::io::detail {

struct LoadedMeshData {
    ElementType                  element_type = ElementType::Invalid;
    internal::VolumetricMeshData geometry;
    internal::MaterialCatalog    material_catalog;
};

}  // namespace pgo::VolumetricMeshes::io::detail
