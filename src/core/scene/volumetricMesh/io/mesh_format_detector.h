#pragma once

#include "types/mesh_enums.h"

#include <cstddef>
#include <filesystem>
#include <istream>
#include <span>

namespace pgo::VolumetricMeshes::io {

ElementType detect_element_type(const std::filesystem::path& filename,
                                FileFormatType fileFormat = FileFormatType::ByExtension);
ElementType detect_element_type(std::span<const std::byte> binaryData);
FileFormatType detect_file_format(const std::filesystem::path& filename);

namespace detail {

ElementType detect_ascii_element_type(const std::filesystem::path& filename);
ElementType detect_binary_element_type(std::istream& input);
FileFormatType detect_file_format_by_ext(const std::filesystem::path& filename);

}  // namespace detail

}  // namespace pgo::VolumetricMeshes::io
