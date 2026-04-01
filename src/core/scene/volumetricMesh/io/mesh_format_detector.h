#pragma once

#include "volumetricMesh.h"

#include <istream>

namespace pgo::VolumetricMeshes::io::detail {

VolumetricMesh::ElementType detect_ascii_element_type(const std::filesystem::path& filename);
VolumetricMesh::ElementType detect_binary_element_type(std::istream& input);
VolumetricMesh::FileFormatType detect_file_format_by_ext(const std::filesystem::path& filename);

}  // namespace pgo::VolumetricMeshes::io::detail
