#pragma once

#include "meshLinearAlgebra.h"

#include <array>
#include <vector>

namespace pgo::VolumetricMeshes {

using TetElement = std::array<int, 4>;
using CubicElement = std::array<int, 8>;

struct Geometry {
    std::vector<Vec3d> vertices;
    std::vector<int> elements;
    int numElementVertices = 0;
};

struct InterpolationWeights {
    int numElementVertices = 0;
    std::vector<int> indices;
    std::vector<double> weights;
    std::vector<int> elements;
};

}  // namespace pgo::VolumetricMeshes
