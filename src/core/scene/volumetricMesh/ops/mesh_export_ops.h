#pragma once

#include "ops/mesh_export_types.h"
#include "volumetricMesh.h"

namespace pgo::VolumetricMeshes {

namespace ops {

Geometry geometry(const VolumetricMesh& mesh);
void geometry(const VolumetricMesh& mesh, std::vector<Vec3d>& vertices, std::vector<int>& elements);
void vertices(const VolumetricMesh& mesh, std::vector<Vec3d>& vertices);

}  // namespace ops

}  // namespace pgo::VolumetricMeshes
