#pragma once

#include "volumetricMesh.h"

#include <filesystem>

namespace pgo::VolumetricMeshes::io::detail {

void write_ascii_mesh(const VolumetricMesh& mesh, const std::filesystem::path& filename);

}  // namespace pgo::VolumetricMeshes::io::detail
