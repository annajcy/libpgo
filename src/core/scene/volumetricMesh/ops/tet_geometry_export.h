#pragma once

#include "volumetricMesh.h"

namespace pgo::Mesh {
class TetMeshGeo;
}

namespace pgo::VolumetricMeshes {

class TetMesh;

namespace ops {

void tet_geometry(const TetMesh& mesh, std::vector<Vec3d>& vertices, std::vector<Vec4i>& tets);
void tet_geometry(const TetMesh& mesh, Mesh::TetMeshGeo& geo);

}  // namespace ops

}  // namespace pgo::VolumetricMeshes
