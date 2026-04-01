#pragma once

#include "volumetricMesh.h"

#include <memory>

namespace pgo::VolumetricMeshes {

class TetMesh;
class CubicMesh;

namespace io {

int save(const VolumetricMesh& mesh, const std::filesystem::path& filename);
int save_to_ascii(const VolumetricMesh& mesh, const std::filesystem::path& filename);
int save_to_binary(const VolumetricMesh& mesh, const std::filesystem::path& filename,
                   unsigned int* bytesWritten = nullptr);

std::unique_ptr<TetMesh> load_tet(const std::filesystem::path& filename,
                                  VolumetricMesh::FileFormatType fileFormat =
                                      VolumetricMesh::FileFormatType::ByExtension,
                                  int verbose = 1);
std::unique_ptr<CubicMesh> load_cubic(const std::filesystem::path& filename,
                                      VolumetricMesh::FileFormatType fileFormat =
                                          VolumetricMesh::FileFormatType::ByExtension,
                                      int verbose = 1);

VolumetricMesh::ElementType detect_element_type(
    const std::filesystem::path& filename,
    VolumetricMesh::FileFormatType fileFormat = VolumetricMesh::FileFormatType::ByExtension);
VolumetricMesh::ElementType detect_element_type(std::span<const std::byte> binaryData);
VolumetricMesh::FileFormatType detect_file_format(const std::filesystem::path& filename);

}  // namespace io

}  // namespace pgo::VolumetricMeshes
