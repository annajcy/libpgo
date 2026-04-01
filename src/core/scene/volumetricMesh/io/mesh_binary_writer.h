#pragma once

#include "volumetricMesh.h"

#include <filesystem>
#include <ostream>

namespace pgo::VolumetricMeshes::io::detail {

void write_binary_mesh(const VolumetricMesh& mesh, std::ostream& out, unsigned int* bytes_written);
void write_binary_mesh(const VolumetricMesh& mesh, const std::filesystem::path& filename,
                       unsigned int* bytes_written);

}  // namespace pgo::VolumetricMeshes::io::detail
