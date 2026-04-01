#include "ops/mesh_construction.h"

#include "internal/mesh_mutation.h"
#include "volumetricMesh.h"

namespace pgo::VolumetricMeshes::ops {

void assign_common_loaded_data(VolumetricMesh& mesh, io::detail::LoadedMeshData data) {
    data.material_catalog.validate_against_num_elements(data.geometry.num_elements());
    internal::MeshMutation::replace_geometry(mesh, std::move(data.geometry));
    internal::MeshMutation::material_catalog(mesh) = std::move(data.material_catalog);
    internal::MeshMutation::material_catalog(mesh).validate_against_num_elements(mesh.getNumElements());
}

}  // namespace pgo::VolumetricMeshes::ops
