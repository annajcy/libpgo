#include "ops/mesh_export_ops.h"

namespace pgo::VolumetricMeshes {
namespace ops {
namespace detail {

Geometry geometry_impl(const VolumetricMesh& mesh) {
    Geometry result;
    const auto vertices = mesh.getVertices();
    const auto elements = mesh.getElements();
    result.vertices = std::vector<Vec3d>(vertices.begin(), vertices.end());
    result.elements = std::vector<int>(elements.begin(), elements.end());
    result.numElementVertices = mesh.getNumElementVertices();
    return result;
}

}  // namespace detail

Geometry geometry(const VolumetricMesh& mesh) {
    return detail::geometry_impl(mesh);
}

void geometry(const VolumetricMesh& mesh, std::vector<Vec3d>& verticesOut, std::vector<int>& elementsOut) {
    const auto vertices = mesh.getVertices();
    const auto elements = mesh.getElements();
    verticesOut.assign(vertices.begin(), vertices.end());
    elementsOut.assign(elements.begin(), elements.end());
}

void vertices(const VolumetricMesh& mesh, std::vector<Vec3d>& verticesOut) {
    const auto vertices = mesh.getVertices();
    verticesOut.assign(vertices.begin(), vertices.end());
}

}  // namespace ops
}  // namespace pgo::VolumetricMeshes
