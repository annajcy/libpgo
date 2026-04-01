#pragma once

#include "io/mesh_io_types.h"

namespace pgo::VolumetricMeshes {
class VolumetricMesh;
}

namespace pgo::VolumetricMeshes::ops {

void assign_common_loaded_data(VolumetricMesh& mesh, io::detail::LoadedMeshData data);

}  // namespace pgo::VolumetricMeshes::ops
