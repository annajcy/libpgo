#pragma once

#include "meshLinearAlgebra.h"

#include <vector>

namespace pgo::VolumetricMeshes::ops {

struct Geometry {
    std::vector<Vec3d> vertices;
    std::vector<int>   elements;
    int                numElementVertices = 0;
};

}  // namespace pgo::VolumetricMeshes::ops
