#include "algorithms/mesh_interpolation.h"

#include "EigenSupport.h"

#include <tbb/parallel_for.h>

#include <algorithm>
#include <cfloat>
#include <cstdio>
#include <stdexcept>

namespace pgo::VolumetricMeshes {
namespace interpolation {
namespace detail {

std::vector<int> containing_elements_impl(const VolumetricMesh& mesh, std::span<const Vec3d> targetLocations,
                                          bool useClosestElementIfOutside) {
    std::vector<int> elements(targetLocations.size(), -1);
    for (int i = 0; i < static_cast<int>(targetLocations.size()); i++) {
        int element = mesh.getContainingElement(targetLocations[static_cast<size_t>(i)]);
        if (useClosestElementIfOutside && (element < 0))
            element = mesh.getClosestElement(targetLocations[static_cast<size_t>(i)]);
        elements[static_cast<size_t>(i)] = element;
    }
    return elements;
}

InterpolationWeights generate_weights_impl(const VolumetricMesh& mesh, std::span<const Vec3d> targetLocations,
                                           std::span<const int> elements, double zeroThreshold, int verbose) {
    InterpolationWeights result;
    result.numElementVertices = mesh.getNumElementVertices();
    result.indices.resize(static_cast<size_t>(mesh.getNumElementVertices()) * targetLocations.size());
    result.weights.resize(static_cast<size_t>(mesh.getNumElementVertices()) * targetLocations.size());
    result.elements.assign(elements.begin(), elements.end());

    std::vector<double> barycentricWeights(static_cast<size_t>(mesh.getNumElementVertices()));

    for (int i = 0; i < static_cast<int>(targetLocations.size()); i++) {
        if (verbose && (i % 100 == 0)) {
            printf("%d ", i);
            fflush(nullptr);
        }

        const Vec3d& pos = targetLocations[static_cast<size_t>(i)];
        int element = elements[static_cast<size_t>(i)];
        if (element < 0)
            throw std::runtime_error("Invalid element index in generate_weights.");

        mesh.computeBarycentricWeights(element, pos, barycentricWeights.data());

        if (zeroThreshold > 0) {
            double minDistance = DBL_MAX;
            for (int ii = 0; ii < mesh.getNumElementVertices(); ii++) {
                const Vec3d& vpos = mesh.getVertex(element, ii);
                minDistance = std::min(minDistance, (vpos - pos).norm());
            }
            if (minDistance > zeroThreshold)
                std::fill(barycentricWeights.begin(), barycentricWeights.end(), 0.0);
        }

        for (int ii = 0; ii < mesh.getNumElementVertices(); ii++) {
            result.indices[static_cast<size_t>(mesh.getNumElementVertices() * i + ii)] = mesh.getVertexIndex(element, ii);
            result.weights[static_cast<size_t>(mesh.getNumElementVertices() * i + ii)] =
                barycentricWeights[static_cast<size_t>(ii)];
        }
    }

    return result;
}

}  // namespace detail

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
