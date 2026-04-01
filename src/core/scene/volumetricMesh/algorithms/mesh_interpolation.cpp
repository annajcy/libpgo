#include "algorithms/mesh_interpolation.h"
#include "EigenSupport.h"

#include <tbb/parallel_for.h>

namespace pgo::VolumetricMeshes {
namespace interpolation {

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
