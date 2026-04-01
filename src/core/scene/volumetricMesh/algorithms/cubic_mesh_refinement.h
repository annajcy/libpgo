#pragma once

namespace pgo::VolumetricMeshes {
class CubicMesh;
}

namespace pgo::VolumetricMeshes::algorithms {

void subdivide_cubic_mesh(CubicMesh& mesh);

}  // namespace pgo::VolumetricMeshes::algorithms
