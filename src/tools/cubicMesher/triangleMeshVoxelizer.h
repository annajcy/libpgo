#pragma once

#include "cubicMesh.h"

#include <array>
#include <memory>
#include <string>

namespace cubic_mesher {

enum class TriangleMeshClassifyMode { Center, Conservative };

struct TriangleMeshVoxelizerOptions {
    std::string                   inputMesh;
    int                           resolution    = 0;
    int                           paddingVoxels = 1;
    TriangleMeshClassifyMode      classifyMode  = TriangleMeshClassifyMode::Conservative;
    double                        scale         = 1.0;
    std::array<double, 3>         offset        = {0.0, 0.0, 0.0};
    double                        E             = 1e6;
    double                        nu            = 0.45;
    double                        density       = 1000.0;
};

const char* toString(TriangleMeshClassifyMode mode);

std::unique_ptr<pgo::VolumetricMeshes::CubicMesh> createTriangleMeshCubicMesh(
    const TriangleMeshVoxelizerOptions& options);

}  // namespace cubic_mesher
