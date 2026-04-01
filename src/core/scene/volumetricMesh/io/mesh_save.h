#pragma once

#include "volumetricMesh.h"

namespace pgo::VolumetricMeshes::io {

int save(const VolumetricMesh& mesh, const std::filesystem::path& filename);
int save_to_ascii(const VolumetricMesh& mesh, const std::filesystem::path& filename);
int save_to_binary(const VolumetricMesh& mesh, const std::filesystem::path& filename,
                   unsigned int* bytesWritten = nullptr);
int save_to_node_ele(const VolumetricMesh& mesh, const std::filesystem::path& baseFilename,
                     bool includeRegions = false);

}  // namespace pgo::VolumetricMeshes::io
