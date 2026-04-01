#pragma once

#include "ops/mesh_export_types.h"

namespace pgo::VolumetricMeshes {
namespace ops {

template <class MeshT>
Geometry geometry(const MeshT& mesh) {
    Geometry result;
    const auto vertices_span = mesh.getVertices();
    const auto elements_span = mesh.getElements();
    result.vertices = std::vector<Vec3d>(vertices_span.begin(), vertices_span.end());
    result.elements = std::vector<int>(elements_span.begin(), elements_span.end());
    result.numElementVertices = mesh.getNumElementVertices();
    return result;
}

template <class MeshT>
void geometry(const MeshT& mesh, std::vector<Vec3d>& vertices, std::vector<int>& elements) {
    const auto vertices_span = mesh.getVertices();
    const auto elements_span = mesh.getElements();
    vertices.assign(vertices_span.begin(), vertices_span.end());
    elements.assign(elements_span.begin(), elements_span.end());
}

template <class MeshT>
void vertices(const MeshT& mesh, std::vector<Vec3d>& vertices) {
    const auto vertices_span = mesh.getVertices();
    vertices.assign(vertices_span.begin(), vertices_span.end());
}

}  // namespace ops
}  // namespace pgo::VolumetricMeshes
