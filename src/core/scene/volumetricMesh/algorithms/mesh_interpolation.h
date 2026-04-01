#pragma once

#include "types/interpolation_weights.h"
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

void apply(const double* u, double* uTarget, int numTargetLocations, int numElementVertices, const int* vertices,
           const double* weights);

}  // namespace interpolation

}  // namespace pgo::VolumetricMeshes
