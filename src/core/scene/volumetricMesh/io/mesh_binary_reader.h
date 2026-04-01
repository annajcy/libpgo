#pragma once

#include "io/mesh_io_types.h"

#include <filesystem>
#include <istream>
#include <span>

namespace pgo::VolumetricMeshes::io::detail {

LoadedMeshData read_binary_mesh(std::istream& input);
LoadedMeshData read_binary_mesh(std::span<const std::byte> binary_data);
LoadedMeshData read_binary_mesh(const std::filesystem::path& filename);

}  // namespace pgo::VolumetricMeshes::io::detail
