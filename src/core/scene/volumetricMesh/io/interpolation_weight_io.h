#pragma once

#include "types/interpolation_weights.h"

#include <filesystem>

namespace pgo::VolumetricMeshes {

namespace interpolation {

int get_num_interpolation_element_vertices(const std::filesystem::path& filename);
InterpolationWeights load_weights(const std::filesystem::path& filename, int numTargetLocations,
                                  int numElementVertices);
int save_weights(const std::filesystem::path& filename, const InterpolationWeights& interpolation);
InterpolationWeights load_weights_binary(const std::filesystem::path& filename);
int save_weights_binary(const std::filesystem::path& filename, const InterpolationWeights& interpolation);

}  // namespace interpolation

}  // namespace pgo::VolumetricMeshes
