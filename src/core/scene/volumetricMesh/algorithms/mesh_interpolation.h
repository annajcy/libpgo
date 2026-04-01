#pragma once

#include "concepts/mesh_concepts.h"
#include "meshLinearAlgebra.h"
#include "types/interpolation_weights.h"

#include <algorithm>
#include <cfloat>
#include <cstdio>
#include <stdexcept>

namespace pgo::VolumetricMeshes {

namespace interpolation {

namespace detail {

template <class MeshT>
std::vector<int> containing_elements_impl(const MeshT& mesh, std::span<const Vec3d> target_locations,
                                          bool use_closest_element_if_outside) {
    std::vector<int> elements(target_locations.size(), -1);
    for (int i = 0; i < static_cast<int>(target_locations.size()); i++) {
        int element = mesh.getContainingElement(target_locations[static_cast<size_t>(i)]);
        if (use_closest_element_if_outside && (element < 0)) {
            element = mesh.getClosestElement(target_locations[static_cast<size_t>(i)]);
        }
        elements[static_cast<size_t>(i)] = element;
    }
    return elements;
}

template <class MeshT>
InterpolationWeights generate_weights_impl(const MeshT& mesh, std::span<const Vec3d> target_locations,
                                           std::span<const int> elements, double zero_threshold, int verbose) {
    InterpolationWeights result;
    result.numElementVertices = mesh.getNumElementVertices();
    result.indices.resize(static_cast<size_t>(mesh.getNumElementVertices()) * target_locations.size());
    result.weights.resize(static_cast<size_t>(mesh.getNumElementVertices()) * target_locations.size());
    result.elements.assign(elements.begin(), elements.end());

    std::vector<double> barycentric_weights(static_cast<size_t>(mesh.getNumElementVertices()));

    for (int i = 0; i < static_cast<int>(target_locations.size()); i++) {
        if (verbose && (i % 100 == 0)) {
            printf("%d ", i);
            fflush(nullptr);
        }

        const Vec3d& pos = target_locations[static_cast<size_t>(i)];
        int element      = elements[static_cast<size_t>(i)];
        if (element < 0) {
            throw std::runtime_error("Invalid element index in generate_weights.");
        }

        mesh.computeBarycentricWeights(element, pos, barycentric_weights.data());

        if (zero_threshold > 0) {
            double min_distance = DBL_MAX;
            for (int ii = 0; ii < mesh.getNumElementVertices(); ii++) {
                const Vec3d& vpos = mesh.getVertex(element, ii);
                min_distance      = std::min(min_distance, (vpos - pos).norm());
            }
            if (min_distance > zero_threshold) {
                std::fill(barycentric_weights.begin(), barycentric_weights.end(), 0.0);
            }
        }

        for (int ii = 0; ii < mesh.getNumElementVertices(); ii++) {
            result.indices[static_cast<size_t>(mesh.getNumElementVertices() * i + ii)] = mesh.getVertexIndex(element, ii);
            result.weights[static_cast<size_t>(mesh.getNumElementVertices() * i + ii)] =
                barycentric_weights[static_cast<size_t>(ii)];
        }
    }

    return result;
}

}  // namespace detail

template <concepts::VolumetricMeshLike MeshT>
InterpolationWeights generate_weights(const MeshT& mesh, std::span<const Vec3d> target_locations,
                                      double zero_threshold = -1.0, bool use_closest_element_if_outside = true,
                                      int verbose = 0) {
    const auto elements = detail::containing_elements_impl(mesh, target_locations, use_closest_element_if_outside);
    return detail::generate_weights_impl(mesh, target_locations, elements, zero_threshold, verbose);
}

template <concepts::VolumetricMeshLike MeshT>
InterpolationWeights generate_weights(const MeshT& mesh, std::span<const Vec3d> target_locations,
                                      std::span<const int> elements, double zero_threshold = -1.0, int verbose = 0) {
    return detail::generate_weights_impl(mesh, target_locations, elements, zero_threshold, verbose);
}

template <concepts::VolumetricMeshLike MeshT>
std::vector<int> containing_elements(const MeshT& mesh, std::span<const Vec3d> target_locations,
                                     bool use_closest_element_if_outside = true, int verbose = 0) {
    (void)verbose;
    return detail::containing_elements_impl(mesh, target_locations, use_closest_element_if_outside);
}

void apply(const double* u, double* uTarget, int numTargetLocations, int numElementVertices, const int* vertices,
           const double* weights);

}  // namespace interpolation

}  // namespace pgo::VolumetricMeshes
