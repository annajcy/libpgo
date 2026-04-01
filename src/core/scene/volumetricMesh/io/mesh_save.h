#pragma once

#include "dispatch/any_mesh_ref.h"

#include <filesystem>

namespace pgo::VolumetricMeshes::io {

int save(AnyMeshRef mesh, const std::filesystem::path& filename);
int save_to_ascii(AnyMeshRef mesh, const std::filesystem::path& filename);
int save_to_binary(AnyMeshRef mesh, const std::filesystem::path& filename, unsigned int* bytesWritten = nullptr);
int save_to_node_ele(AnyMeshRef mesh, const std::filesystem::path& baseFilename, bool includeRegions = false);

template <class MeshT>
int save(const MeshT& mesh, const std::filesystem::path& filename) {
    return save(make_any_mesh_ref(mesh), filename);
}

template <class MeshT>
int save_to_ascii(const MeshT& mesh, const std::filesystem::path& filename) {
    return save_to_ascii(make_any_mesh_ref(mesh), filename);
}

template <class MeshT>
int save_to_binary(const MeshT& mesh, const std::filesystem::path& filename, unsigned int* bytesWritten = nullptr) {
    return save_to_binary(make_any_mesh_ref(mesh), filename, bytesWritten);
}

template <class MeshT>
int save_to_node_ele(const MeshT& mesh, const std::filesystem::path& baseFilename, bool includeRegions = false) {
    return save_to_node_ele(make_any_mesh_ref(mesh), baseFilename, includeRegions);
}

}  // namespace pgo::VolumetricMeshes::io
