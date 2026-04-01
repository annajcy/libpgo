#pragma once

#include <vector>

namespace pgo::VolumetricMeshes {

struct InterpolationWeights {
    int                 numElementVertices = 0;
    std::vector<int>    indices;
    std::vector<double> weights;
    std::vector<int>    elements;
};

}  // namespace pgo::VolumetricMeshes
