#include "algorithms/mesh_interpolation.h"

#include "volumetricMesh.h"

#include "EigenSupport.h"

#include <tbb/parallel_for.h>

namespace pgo::VolumetricMeshes {
namespace interpolation {

InterpolationWeights generate_weights(const VolumetricMesh& mesh, std::span<const Vec3d> targetLocations,
                                      double zeroThreshold, bool useClosestElementIfOutside, int verbose) {
    const auto elements = detail::containing_elements_impl(mesh, targetLocations, useClosestElementIfOutside);
    return detail::generate_weights_impl(mesh, targetLocations, elements, zeroThreshold, verbose);
}

InterpolationWeights generate_weights(const VolumetricMesh& mesh, std::span<const Vec3d> targetLocations,
                                      std::span<const int> elements, double zeroThreshold, int verbose) {
    return detail::generate_weights_impl(mesh, targetLocations, elements, zeroThreshold, verbose);
}

std::vector<int> containing_elements(const VolumetricMesh& mesh, std::span<const Vec3d> targetLocations,
                                     bool useClosestElementIfOutside, int verbose) {
    (void)verbose;
    return detail::containing_elements_impl(mesh, targetLocations, useClosestElementIfOutside);
}

void apply(const double* u, double* uTarget, int numTargetLocations, int numElementVertices, const int* vertices,
           const double* weights) {
    tbb::parallel_for(0, numTargetLocations, [&](int i) {
        Vec3d defo(0, 0, 0);
        for (int j = 0; j < numElementVertices; j++) {
            int volumetricMeshVertexIndex = vertices[numElementVertices * i + j];
            defo += weights[numElementVertices * i + j] * asVec3d(u + 3 * volumetricMeshVertexIndex);
        }
        (Eigen::Map<Vec3d>(uTarget + 3 * i)) = defo;
    });
}

}  // namespace interpolation
}  // namespace pgo::VolumetricMeshes
