#pragma once

#include "volumetricMesh.h"

#include <cstdio>
#include <filesystem>
#include <istream>
#include <memory>
#include <ostream>
#include <string>

namespace pgo::VolumetricMeshes::io::detail {

std::unique_ptr<VolumetricMesh::Material> parse_ascii_material(
    const std::string& material_name,
    const std::string& material_specification,
    const std::filesystem::path& filename,
    const std::string& offending_line);

std::unique_ptr<VolumetricMesh::Material> read_binary_material(std::istream& input);

void write_ascii_material(FILE* output, const VolumetricMesh::Material& material);

unsigned int write_binary_material(std::ostream& output, const VolumetricMesh::Material& material);

}  // namespace pgo::VolumetricMeshes::io::detail
