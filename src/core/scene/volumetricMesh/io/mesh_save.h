#pragma once

#include "dispatch/any_mesh_ref.h"

#include <filesystem>

namespace pgo::VolumetricMeshes {
class VolumetricMesh;
}

namespace pgo::VolumetricMeshes::io {

int save(AnyMeshRef mesh, const std::filesystem::path& filename);
int save_to_ascii(AnyMeshRef mesh, const std::filesystem::path& filename);
int save_to_binary(AnyMeshRef mesh, const std::filesystem::path& filename, unsigned int* bytesWritten = nullptr);
int save_to_node_ele(AnyMeshRef mesh, const std::filesystem::path& baseFilename, bool includeRegions = false);

int save(const VolumetricMesh& mesh, const std::filesystem::path& filename);
int save_to_ascii(const VolumetricMesh& mesh, const std::filesystem::path& filename);
int save_to_binary(const VolumetricMesh& mesh, const std::filesystem::path& filename,
                   unsigned int* bytesWritten = nullptr);
int save_to_node_ele(const VolumetricMesh& mesh, const std::filesystem::path& baseFilename,
                     bool includeRegions = false);

}  // namespace pgo::VolumetricMeshes::io
