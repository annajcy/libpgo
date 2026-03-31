#pragma once

#include "volumetricMeshTypes.h"
#include "volumetricMesh.h"

namespace pgo::VolumetricMeshes {

namespace interpolation {

InterpolationWeights generate_weights(const VolumetricMesh& mesh, std::span<const Vec3d> targetLocations,
                                      double zeroThreshold = -1.0, bool useClosestElementIfOutside = true,
                                      int verbose = 0);
InterpolationWeights generate_weights(const VolumetricMesh& mesh, std::span<const Vec3d> targetLocations,
                                      std::span<const int> elements, double zeroThreshold = -1.0, int verbose = 0);
std::vector<int> containing_elements(const VolumetricMesh& mesh, std::span<const Vec3d> targetLocations,
                                     bool useClosestElementIfOutside = true, int verbose = 0);

int get_num_interpolation_element_vertices(const std::filesystem::path& filename);
InterpolationWeights load_weights(const std::filesystem::path& filename, int numTargetLocations,
                                  int numElementVertices);
int save_weights(const std::filesystem::path& filename, const InterpolationWeights& interpolation);
InterpolationWeights load_weights_binary(const std::filesystem::path& filename);
int save_weights_binary(const std::filesystem::path& filename, const InterpolationWeights& interpolation);

void apply(const double* u, double* uTarget, int numTargetLocations, int numElementVertices, const int* vertices,
           const double* weights);

}  // namespace interpolation

}  // namespace pgo::VolumetricMeshes
