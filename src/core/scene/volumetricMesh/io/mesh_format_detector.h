#pragma once

#include "volumetricMesh.h"

#include <istream>
#include <span>

namespace pgo::VolumetricMeshes::io {

VolumetricMesh::ElementType detect_element_type(
    const std::filesystem::path& filename,
    VolumetricMesh::FileFormatType fileFormat = VolumetricMesh::FileFormatType::ByExtension);
VolumetricMesh::ElementType detect_element_type(std::span<const std::byte> binaryData);
VolumetricMesh::FileFormatType detect_file_format(const std::filesystem::path& filename);

namespace detail {

VolumetricMesh::ElementType detect_ascii_element_type(const std::filesystem::path& filename);
VolumetricMesh::ElementType detect_binary_element_type(std::istream& input);
VolumetricMesh::FileFormatType detect_file_format_by_ext(const std::filesystem::path& filename);

}  // namespace detail

}  // namespace pgo::VolumetricMeshes::io
