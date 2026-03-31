#pragma once

#include "volumetricMeshTypes.h"
#include "volumetricMesh.h"

namespace pgo::Mesh {
class TetMeshGeo;
}

namespace pgo::VolumetricMeshes {

class TetMesh;

namespace exporting {

Geometry geometry(const VolumetricMesh& mesh);
void geometry(const VolumetricMesh& mesh, std::vector<Vec3d>& vertices, std::vector<int>& elements);
void vertices(const VolumetricMesh& mesh, std::vector<Vec3d>& vertices);
int to_ele(const VolumetricMesh& mesh, const std::filesystem::path& baseFilename, bool includeRegions = false);

void tet_geometry(const TetMesh& mesh, std::vector<Vec3d>& vertices, std::vector<Vec4i>& tets);
void tet_geometry(const TetMesh& mesh, Mesh::TetMeshGeo& geo);

}  // namespace exporting

}  // namespace pgo::VolumetricMeshes
