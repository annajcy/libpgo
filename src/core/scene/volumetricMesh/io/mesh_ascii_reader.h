#pragma once

#include "io/mesh_io_types.h"

#include <filesystem>

namespace pgo::VolumetricMeshes::io::detail {

LoadedMeshData read_ascii_mesh(const std::filesystem::path& filename, int verbose);

}  // namespace pgo::VolumetricMeshes::io::detail
