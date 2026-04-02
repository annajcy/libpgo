#pragma once

#include "cubicMesh.h"

#include <array>
#include <memory>

namespace cubic_mesher {

struct UniformCubicMeshOptions {
    int                   resolution = 0;
    double                size       = 1.0;
    std::array<double, 3> offset     = {0.0, 0.0, 0.0};
    double                E          = 1e6;
    double                nu         = 0.45;
    double                density    = 1000.0;
};

std::unique_ptr<pgo::VolumetricMeshes::CubicMesh> createUniformCubicMesh(const UniformCubicMeshOptions& options);

}  // namespace cubic_mesher
