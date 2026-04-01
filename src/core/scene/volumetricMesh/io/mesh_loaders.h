#pragma once

#include "io/mesh_io_types.h"

#include <span>

namespace pgo::VolumetricMeshes::io::detail {

LoadedMeshData load_tet_data(const std::filesystem::path& filename, VolumetricMesh::FileFormatType file_format,
                             int verbose);
LoadedMeshData load_tet_data(std::span<const std::byte> binary_data);
LoadedMeshData load_cubic_data(const std::filesystem::path& filename, VolumetricMesh::FileFormatType file_format,
                               int verbose);
LoadedMeshData load_cubic_data(std::span<const std::byte> binary_data);

}  // namespace pgo::VolumetricMeshes::io::detail
