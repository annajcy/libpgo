#pragma once

#include "internal/material_catalog.h"
#include "internal/volumetric_mesh_data.h"

namespace pgo::VolumetricMeshes {
class VolumetricMesh;
}

namespace pgo::VolumetricMeshes::internal {

class MeshMutation {
public:
    static VolumetricMeshData& geometry(VolumetricMesh& mesh);
    static MaterialCatalog&    material_catalog(VolumetricMesh& mesh);
    static void                replace_geometry(VolumetricMesh& mesh, VolumetricMeshData geometry);
};

}  // namespace pgo::VolumetricMeshes::internal
