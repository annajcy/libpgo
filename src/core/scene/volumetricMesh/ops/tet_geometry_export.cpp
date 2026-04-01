#include "ops/tet_geometry_export.h"

#include "tetMesh.h"
#include "tetMeshGeo.h"

namespace pgo::VolumetricMeshes::ops {

void tet_geometry(const TetMesh& mesh, std::vector<Vec3d>& verticesOut, std::vector<Vec4i>& tets) {
    const auto vertices = mesh.getVertices();
    verticesOut.assign(vertices.begin(), vertices.end());
    tets.resize(static_cast<size_t>(mesh.getNumElements()));
    for (int i = 0; i < mesh.getNumElements(); ++i) {
        const auto ii = mesh.getVertexIndices(i);
        tets[static_cast<size_t>(i)] = Vec4i(ii[0], ii[1], ii[2], ii[3]);
    }
}

void tet_geometry(const TetMesh& mesh, Mesh::TetMeshGeo& geo) {
    tet_geometry(mesh, geo.positions(), geo.tets());
}

}  // namespace pgo::VolumetricMeshes::ops
