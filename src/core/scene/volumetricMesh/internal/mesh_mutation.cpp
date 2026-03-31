#include "internal/mesh_mutation.h"

#include "volumetricMesh.h"

namespace pgo::VolumetricMeshes::internal {

VolumetricMeshData& MeshMutation::geometry(VolumetricMesh& mesh) {
    return mesh.geometry_data();
}

MaterialCatalog& MeshMutation::material_catalog(VolumetricMesh& mesh) {
    return mesh.material_catalog();
}

void MeshMutation::replace_geometry(VolumetricMesh& mesh, VolumetricMeshData geometry) {
    mesh.geometry_data() = std::move(geometry);
}

}  // namespace pgo::VolumetricMeshes::internal
