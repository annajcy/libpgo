#pragma once

#include "materials/material_access.h"
#include "materials/material_record.h"

#include <cstdio>
#include <filesystem>
#include <istream>
#include <ostream>
#include <string>

namespace pgo::VolumetricMeshes::io::detail {

MaterialRecord parse_ascii_material(
    const std::string& material_name,
    const std::string& material_specification,
    const std::filesystem::path& filename,
    const std::string& offending_line);

MaterialRecord read_binary_material(std::istream& input);

void write_ascii_material(FILE* output, const MaterialRecord& material);

unsigned int write_binary_material(std::ostream& output, const MaterialRecord& material);

}  // namespace pgo::VolumetricMeshes::io::detail
