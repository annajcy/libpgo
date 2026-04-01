#pragma once

#include "internal/material_catalog.h"
#include "internal/volumetric_mesh_data.h"

namespace pgo::VolumetricMeshes::internal {

class MeshMutation {
public:
    template <class MeshT>
    static VolumetricMeshData& geometry(MeshT& mesh) {
        return mesh.geometry_data();
    }

    template <class MeshT>
    static MaterialCatalog& material_catalog(MeshT& mesh) {
        return mesh.material_catalog();
    }

    template <class MeshT>
    static void replace_geometry(MeshT& mesh, VolumetricMeshData geometry) {
        mesh.geometry_data() = std::move(geometry);
    }
};

}  // namespace pgo::VolumetricMeshes::internal
